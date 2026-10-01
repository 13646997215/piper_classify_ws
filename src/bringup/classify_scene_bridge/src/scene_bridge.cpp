#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <array>
#include <chrono>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <tuple>

#include <classify_interfaces/msg/detected_blocks.hpp>
#include <classify_interfaces/msg/scene_status.hpp>
#include <classify_interfaces/msg/task_status.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include "scene_tracker.hpp"

namespace
{
using CollisionObject = moveit_msgs::msg::CollisionObject;
using ObjectColor = moveit_msgs::msg::ObjectColor;

geometry_msgs::msg::Pose pose(double x, double y, double z)
{
  geometry_msgs::msg::Pose result;
  result.position.x = x;
  result.position.y = y;
  result.position.z = z;
  result.orientation.w = 1.0;
  return result;
}

void add_box(CollisionObject& object, double sx, double sy, double sz,
             double x, double y, double z)
{
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {sx, sy, sz};
  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose(x, y, z));
}

CollisionObject object(const std::string& id, const std::string& frame)
{
  CollisionObject result;
  result.id = id;
  result.header.frame_id = frame;
  result.operation = CollisionObject::ADD;
  return result;
}

ObjectColor color(const std::string& id, float r, float g, float b)
{
  ObjectColor result;
  result.id = id;
  result.color.r = r;
  result.color.g = g;
  result.color.b = b;
  result.color.a = 1.0f;
  return result;
}

std::vector<CollisionObject> fixed_geometry(const std::map<std::string, std::vector<double>>& geometry)
{
  std::vector<CollisionObject> objects;
  for (const auto& [name, rows] : geometry) {
    if (rows.empty() || rows.size() % 7) throw std::runtime_error("Invalid fixed geometry for " + name);
    auto result = object(name, "world");
    for (std::size_t i = 0; i < rows.size(); i += 7) {
      add_box(result, rows[i], rows[i+1], rows[i+2], rows[i+3], rows[i+4], rows[i+5]);
      result.primitive_poses.back().orientation.z = std::sin(rows[i+6]/2);
      result.primitive_poses.back().orientation.w = std::cos(rows[i+6]/2);
    }
    objects.push_back(result);
  }
  return objects;
}

std::vector<ObjectColor> fixed_colors()
{
  return {color("table_top", 0.72f, 0.60f, 0.46f),
          color("table_legs", 0.26f, 0.26f, 0.28f),
          color("bin_red", 0.80f, 0.20f, 0.20f),
          color("bin_blue", 0.20f, 0.35f, 0.80f)};
}

ObjectColor block_color(const std::string& name, const std::string& id)
{
  if (name == "red") return color(id, 0.92f, 0.16f, 0.16f);
  if (name == "green") return color(id, 0.15f, 0.82f, 0.22f);
  return color(id, 0.16f, 0.35f, 0.92f);
}
}  // namespace

class SceneBridge : public rclcpp::Node
{
public:
  SceneBridge() : Node("scene_bridge")
  {
    xmin_ = declare_parameter("pickup_x_min", .30); xmax_ = declare_parameter("pickup_x_max", .45);
    ymax_ = declare_parameter("pickup_y_abs_max", .20); cube_size_ = declare_parameter("cube_size", .05);
    table_z_ = declare_parameter("table_top_z", .003);
    for (const auto& c : names_) {
      expected_[c] = declare_parameter<int>("expected_"+c+"_count",1);
      if (expected_[c]<1 || expected_[c]>8) throw std::runtime_error("Expected counts must be 1..8");
    }
    std::map<std::string, std::vector<double>> geometry;
    for (const auto& name : {"table_top", "table_legs", "bin_red", "bin_blue"})
      geometry[name] = declare_parameter<std::vector<double>>(std::string(name)+"_boxes", std::vector<double>{});
    const auto obstacles=declare_parameter<std::vector<std::string>>("obstacle_names", std::vector<std::string>{});
    for (const auto& name:obstacles) {
      if (name.rfind("obstacle_",0)!=0) throw std::runtime_error("Invalid obstacle name");
      geometry[name]=declare_parameter<std::vector<double>>(name+"_boxes", std::vector<double>{});
    }
    status_pub_ = create_publisher<classify_interfaces::msg::SceneStatus>(
        "/classify/scene_status", rclcpp::QoS(1).transient_local());
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/classify/scene_markers", rclcpp::QoS(1).transient_local());
    ready_service_ = create_service<std_srvs::srv::Trigger>(
        "/classify/scene_ready", [this](std_srvs::srv::Trigger::Request::SharedPtr,
                                       std_srvs::srv::Trigger::Response::SharedPtr response) {
          const auto status = current_status();
          response->success = status.ready;
          response->message = status.detail;
        });
    detections_ = create_subscription<classify_interfaces::msg::DetectedBlocks>(
        "/classify/detections", 10,
        [this](classify_interfaces::msg::DetectedBlocks::ConstSharedPtr msg) { update_blocks(*msg); });
    task_sub_ = create_subscription<classify_interfaces::msg::TaskStatus>(
        "/classify/status", rclcpp::QoS(1).transient_local(),
        [this](classify_interfaces::msg::TaskStatus::ConstSharedPtr msg) {
          task_object_ = (msg->state == "PLANNING" || msg->state == "PICKING" || msg->state == "PLACING" ||
                          msg->state == "FAILED" || msg->state == "CANCELLED") ? msg->current_object : "";
          if (msg->placed_objects.size() != msg->placed_poses.size()) return;
          for (size_t i=0; i<msg->placed_objects.size(); ++i) {
            placed_[msg->placed_objects[i]]=msg->placed_poses[i];
          }
        });

    // PlanningSceneInterface waits for move_group's apply_planning_scene service.
    // This launch starts both nodes together; the service may appear later.
    scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    if (!scene_->applyCollisionObjects(fixed_geometry(geometry), fixed_colors())) {
      throw std::runtime_error("Failed to add Gazebo workcell to MoveIt planning scene");
    }
    // The bolted base intentionally touches the table. Keep all moving links checked.
    moveit_msgs::msg::PlanningScene mounting;
    mounting.is_diff = true;
    // MoveIt replaces an incoming ACM even for a scene diff. Preserve the SRDF
    // entries instead of sending a two-name matrix that loses adjacent links.
    auto reader = std::make_shared<rclcpp::Node>("classify_mounting_scene_reader");
    auto client = reader->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!client->wait_for_service(std::chrono::seconds(5))) throw std::runtime_error("Scene reader unavailable");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(reader, future, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
      throw std::runtime_error("Failed to read existing collision matrix");
    collision_detection::AllowedCollisionMatrix matrix(future.get()->scene.allowed_collision_matrix);
    matrix.setEntry("base_link", "table_top", true);
    matrix.getMessage(mounting.allowed_collision_matrix);
    if (!scene_->applyPlanningScene(mounting)) throw std::runtime_error("Failed to register the bolted base/table contact");
    RCLCPP_INFO(get_logger(), "Planning scene ready: table top, four legs, red and blue bins");
    timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() { publish_status(); });
  }

private:
  void update_blocks(const classify_interfaces::msg::DetectedBlocks& detections)
  {
    if (detections.header.frame_id != "base_link") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Expected base_link detections; got '%s'", detections.header.frame_id.c_str());
      return;
    }
    const double stamp = rclcpp::Time(detections.header.stamp).seconds();
    const double age = now().seconds() - stamp;
    if (stamp <= last_message_stamp_ || age < -.05 || age > .8) return;
    last_message_stamp_ = stamp;
    last_message_wall_ = std::chrono::steady_clock::now();
    camera_stable_ = detections.camera_stable;
    detector_status_ = detections.status;
    const auto attached = scene_->getAttachedObjects();
    std::map<std::string, bool> seen;
    std::vector<CollisionObject> updates;
    std::vector<ObjectColor> colors;
    std::vector<classify_interfaces::msg::DetectedBlock> valid;
    if (camera_stable_) for (const auto& block : detections.blocks) {
      if (!expected_.count(block.color)) continue;
      const auto& p = block.center;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(block.yaw) || !std::isfinite(block.top_z) || !std::isfinite(block.confidence) ||
          !std::isfinite(block.top_width) || !std::isfinite(block.top_depth) ||
          !std::isfinite(block.plane_error) || p.x <= xmin_ || p.x >= xmax_ ||
          std::abs(p.y) >= ymax_ || std::abs(p.z - table_z_ - cube_size_/2) > .007 ||
          std::abs(block.top_z - p.z - cube_size_/2) > .002 || block.confidence < .75 ||
          block.top_width < cube_size_*.88 || block.top_width > cube_size_*1.12 ||
          block.top_depth < cube_size_*.88 || block.top_depth > cube_size_*1.12 ||
          block.plane_error < 0 || block.plane_error > .003 || block.visible_points < 150) continue;
      valid.push_back(block);
    }
    excess_inventory_=false;
    for (const auto& c:names_) {
      const auto count=std::count_if(valid.begin(),valid.end(),[&](const auto& b){return b.color==c;});
      if (count>expected_.at(c)) excess_inventory_=true;
    }
    // Contour order is not an identity. Sort only births; existing objects are
    // associated by spatial continuity, including temporarily frozen targets.
    std::sort(valid.begin(),valid.end(),[](const auto& a,const auto& b) {
      return std::tie(a.color,a.center.x,a.center.y)<std::tie(b.color,b.center.x,b.center.y);
    });
    std::vector<classify_scene_bridge::TrackCandidate> candidates, observations;
    std::vector<std::string> ids;
    for (const auto& [id,track]:tracks_) {
      ids.push_back(id);candidates.push_back({object_colors_.at(id),track.known ? track.accepted : track.pending});
    }
    for (const auto& b:valid) observations.push_back({b.color,b.center});
    const auto matches=classify_scene_bridge::associate(observations,candidates);
    for (size_t i=0;i<valid.size();++i) {
      const auto& block=valid[i]; std::string id;
      if (matches[i]==-2) continue;
      if (matches[i]>=0) id=ids[matches[i]];
      else {
        const auto count=std::count_if(object_colors_.begin(),object_colors_.end(),[&](const auto& item){return item.second==block.color;});
        if (count>=expected_.at(block.color)) continue; // No identity replacement after ambiguous motion.
        const auto number=++next_id_[block.color];
        id=block.color+"_block"+(number==1 ? "" : "_"+std::to_string(number));
        object_colors_[id]=block.color;
      }
      if (placed_.count(id) || task_object_==id || attached.count(id)) continue;
      seen[id]=true;
      auto& track=tracks_[id];
      if (!track.observe(block.center,stamp,block.yaw)) continue;
      auto cube=object(id,"base_link");
      add_box(cube,cube_size_,cube_size_,cube_size_,track.accepted.x,track.accepted.y,track.accepted.z);
      cube.primitive_poses.back().orientation.z=std::sin(track.accepted_yaw/2);
      cube.primitive_poses.back().orientation.w=std::cos(track.accepted_yaw/2);
      updates.push_back(cube);colors.push_back(block_color(block.color,id));
    }
    for (auto& [id,track]:tracks_) {
      attached_[id]=attached.count(id)!=0;
      if (!seen[id]) track.missing();
    }
    if (!updates.empty() && !scene_->applyCollisionObjects(updates,colors)) {
      RCLCPP_ERROR(get_logger(),"Failed to update confirmed objects");
      for (const auto& item:updates) tracks_[item.id]=classify_scene_bridge::ObservationTrack();
    }
    publish_status();
  }

  classify_interfaces::msg::SceneStatus current_status() const
  {
    classify_interfaces::msg::SceneStatus status;
    status.header.stamp = now(); status.header.frame_id = "base_link";
    status.camera_stable = camera_stable_ && now().seconds() - last_message_stamp_ < .8 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_message_wall_).count() < .8;
    status.ready=status.camera_stable && !excess_inventory_;
    std::map<std::string,int> counts, available, placed_counts;
    for (const auto& [id,track]:tracks_) {
      const auto& c=object_colors_.at(id);++counts[c];
      status.object_ids.push_back(id);status.object_colors.push_back(c);
      const auto attached=attached_.find(id);
      const bool fresh=!placed_.count(id) && status.camera_stable && track.fresh(now().seconds()) &&
        (attached==attached_.end() || !attached->second);
      if (fresh) { status.fresh_objects.push_back(id);++available[c]; }
      else status.stale_objects.push_back(id);
      if (placed_.count(id)) ++placed_counts[c];
      else if (!fresh) status.ready=false;
    }
    for (const auto& c:names_) {
      if (available[c]==expected_.at(c) && counts[c]==expected_.at(c)) status.fresh_colors.push_back(c);
      else status.stale_colors.push_back(c);
      if (counts[c]!=expected_.at(c) || available[c]+placed_counts[c]!=expected_.at(c)) status.ready=false;
    }
    std::ostringstream detail;
    detail << (status.ready ? "Vision scene ready" : "Vision scene unavailable; stale/unconfirmed:");
    if (!status.ready) for (const auto& name : status.stale_colors) detail << ' ' << name;
    if (excess_inventory_) detail << " unexpected extra workpieces";
    detail << "; " << detector_status_;
    status.detail = detail.str();
    return status;
  }

  void publish_status()
  {
    const auto status = current_status();
    status_pub_->publish(status);
    visualization_msgs::msg::MarkerArray markers;
    moveit_msgs::msg::PlanningScene color_update;
    color_update.is_diff = true;
    int index = 0;
    for (auto& [name,track] : tracks_) {
      if (!track.known || attached_[name]) {
        visualization_msgs::msg::Marker remove;
        remove.header = status.header; remove.ns = "confirmed_blocks"; remove.id = index++;
        remove.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(remove);
        continue;
      }
      const bool placed = placed_.count(name);
      const bool fresh = placed || (status.camera_stable && track.fresh(now().seconds()));
      if (!rendered_fresh_[name] || *rendered_fresh_[name] != fresh) {
        auto tint = fresh ? block_color(object_colors_.at(name),name) : color(name, .5f, .5f, .5f);
        if (!fresh) tint.color.a = .55f;
        color_update.object_colors.push_back(tint);
      }
      visualization_msgs::msg::Marker label;
      label.header = status.header; label.ns = "confirmed_blocks"; label.id = index++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = placed ? placed_.at(name).position : track.accepted; label.pose.position.z += .07;
      label.pose.orientation.w = 1;
      label.scale.z = .028;
      label.color.r = label.color.g = label.color.b = fresh ? 1.f : .65f;
      label.color.a = 1;
      label.text = name + (placed ? "\nPLACED" : (fresh ? "" : "\nLAST_SEEN"));
      markers.markers.push_back(label);
    }
    if (!color_update.object_colors.empty() && scene_->applyPlanningScene(color_update)) {
      for (const auto& item : color_update.object_colors) {
        const auto& name = item.id;
        rendered_fresh_[name] = placed_.count(name) || (status.camera_stable && tracks_[name].fresh(now().seconds()));
      }
    }
    visualization_msgs::msg::Marker state;
    state.header = status.header; state.ns = "vision_status"; state.id = 0;
    state.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    state.action = visualization_msgs::msg::Marker::ADD;
    state.pose = pose(.30, .30, .23); state.scale.z = .032;
    state.color = color("", status.ready ? .25f : 1.f, status.ready ? 1.f : .65f, .25f).color;
    state.text = status.ready ? "VISION_READY" : "VISION_UNAVAILABLE";
    markers.markers.push_back(state);
    markers_pub_->publish(markers);
  }

  rclcpp::Subscription<classify_interfaces::msg::DetectedBlocks>::SharedPtr detections_;
  rclcpp::Subscription<classify_interfaces::msg::TaskStatus>::SharedPtr task_sub_;
  std::string task_object_;
  std::map<std::string, geometry_msgs::msg::Pose> placed_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;
  const std::array<std::string, 3> names_{{"red", "green", "blue"}};
  std::map<std::string, classify_scene_bridge::ObservationTrack> tracks_;
  std::map<std::string,std::string> object_colors_;
  std::map<std::string,int> expected_,next_id_;
  std::map<std::string, bool> attached_;
  std::map<std::string, std::optional<bool>> rendered_fresh_;
  std::chrono::steady_clock::time_point last_message_wall_{};
  double xmin_, xmax_, ymax_, cube_size_, table_z_;
  double last_message_stamp_{-1};
  bool camera_stable_{false}, excess_inventory_{false};
  std::string detector_status_{"Waiting for perception"};
  rclcpp::Publisher<classify_interfaces::msg::SceneStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SceneBridge>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("scene_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
