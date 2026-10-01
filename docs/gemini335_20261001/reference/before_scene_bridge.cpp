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

#include <classify_interfaces/msg/detected_blocks.hpp>
#include <classify_interfaces/msg/scene_status.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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

CollisionObject bin(const std::string& id, double x, double y)
{
  auto result = object(id, "world");
  // Match the five separate collision boxes in classify.world exactly.
  // The Gazebo model yaw is -pi/2, so the side dimensions swap in world XY.
  add_box(result, 0.13, 0.13, 0.012, x, y, 0.356);
  add_box(result, 0.012, 0.13, 0.09, x - 0.059, y, 0.401);
  add_box(result, 0.012, 0.13, 0.09, x + 0.059, y, 0.401);
  add_box(result, 0.13, 0.012, 0.09, x, y - 0.059, 0.401);
  add_box(result, 0.13, 0.012, 0.09, x, y + 0.059, 0.401);
  return result;
}

std::vector<CollisionObject> fixed_geometry()
{
  // These are the world-frame collision dimensions and poses from
  // classify_description/worlds/classify.world after its -pi/2 table rotation.
  auto top = object("table_top", "world");
  add_box(top, 0.72, 0.84, 0.04, 0.26, 0.0, 0.33);

  auto legs = object("table_legs", "world");
  for (double x : {-0.06, 0.56}) {
    for (double y : {-0.36, 0.36}) {
      add_box(legs, 0.045, 0.045, 0.31, x, y, 0.155);
    }
  }
  return {top, legs, bin("bin_red", 0.52, -0.22), bin("bin_blue", 0.52, 0.22)};
}

std::vector<ObjectColor> fixed_colors()
{
  return {color("table_top", 0.72f, 0.60f, 0.46f),
          color("table_legs", 0.26f, 0.26f, 0.28f),
          color("bin_red", 0.80f, 0.20f, 0.20f),
          color("bin_blue", 0.20f, 0.35f, 0.80f)};
}

ObjectColor block_color(const std::string& name)
{
  if (name == "red") return color("red_block", 0.92f, 0.16f, 0.16f);
  if (name == "green") return color("green_block", 0.15f, 0.82f, 0.22f);
  return color("blue_block", 0.16f, 0.35f, 0.92f);
}
}  // namespace

class SceneBridge : public rclcpp::Node
{
public:
  SceneBridge() : Node("scene_bridge")
  {
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

    // PlanningSceneInterface waits for move_group's apply_planning_scene service.
    // This launch starts both nodes together; the service may appear later.
    scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    if (!scene_->applyCollisionObjects(fixed_geometry(), fixed_colors())) {
      throw std::runtime_error("Failed to add Gazebo workcell to MoveIt planning scene");
    }
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
    if (camera_stable_) for (const auto& block : detections.blocks) {
      if (block.color != "red" && block.color != "green" && block.color != "blue") continue;
      const auto& p = block.center;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(block.top_z) || !std::isfinite(block.confidence) ||
          !std::isfinite(block.top_width) || !std::isfinite(block.top_depth) ||
          !std::isfinite(block.plane_error) || p.x <= .20 || p.x >= .35 ||
          std::abs(p.y) >= .20 || std::abs(p.z - .028) > .007 ||
          std::abs(block.top_z - p.z - .025) > .002 || block.confidence < .75 ||
          block.top_width < .044 || block.top_width > .056 ||
          block.top_depth < .044 || block.top_depth > .056 ||
          block.plane_error < 0 || block.plane_error > .003 || block.visible_points < 150) continue;
      const std::string id = block.color + "_block";
      // A grasped block becomes an attached object. Do not reintroduce it as
      // a world obstacle while the gripper carries it.
      if (attached.count(id)) { attached_[block.color] = true; continue; }
      attached_[block.color] = false;
      seen[block.color] = true;
      auto& track = tracks_[block.color];
      if (!track.observe(p, stamp)) continue;
      auto cube = object(id, "base_link");
      add_box(cube, 0.05, 0.05, 0.05, track.accepted.x, track.accepted.y, track.accepted.z);
      updates.push_back(cube);
      colors.push_back(block_color(block.color));
    }
    for (const auto& name : names_) {
      attached_[name] = attached.count(name + "_block") != 0;
      if (!seen[name]) tracks_[name].missing();
    }
    if (!updates.empty() && !scene_->applyCollisionObjects(updates, colors)) {
      RCLCPP_ERROR(get_logger(), "Failed to update detected blocks in MoveIt planning scene");
      for (const auto& item : updates) tracks_[item.id.substr(0, item.id.size() - 6)] = classify_scene_bridge::ObservationTrack();
    }
    publish_status();
  }

  classify_interfaces::msg::SceneStatus current_status() const
  {
    classify_interfaces::msg::SceneStatus status;
    status.header.stamp = now(); status.header.frame_id = "base_link";
    status.camera_stable = camera_stable_ && now().seconds() - last_message_stamp_ < .8 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_message_wall_).count() < .8;
    for (const auto& name : names_) {
      const auto found = tracks_.find(name);
      const auto attached = attached_.find(name);
      if (status.camera_stable && found != tracks_.end() && found->second.fresh(now().seconds()) &&
          (attached == attached_.end() || !attached->second)) status.fresh_colors.push_back(name);
      else status.stale_colors.push_back(name);
    }
    status.ready = status.fresh_colors.size() == names_.size();
    std::ostringstream detail;
    detail << (status.ready ? "Vision scene ready" : "Vision scene unavailable; stale/unconfirmed:");
    if (!status.ready) for (const auto& name : status.stale_colors) detail << ' ' << name;
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
    for (const auto& name : names_) {
      auto& track = tracks_[name];
      if (!track.known || attached_[name]) {
        visualization_msgs::msg::Marker remove;
        remove.header = status.header; remove.ns = "confirmed_blocks"; remove.id = index++;
        remove.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(remove);
        continue;
      }
      const bool fresh = status.camera_stable && track.fresh(now().seconds());
      if (!rendered_fresh_[name] || *rendered_fresh_[name] != fresh) {
        auto tint = fresh ? block_color(name) : color(name + "_block", .5f, .5f, .5f);
        if (!fresh) tint.color.a = .55f;
        color_update.object_colors.push_back(tint);
      }
      visualization_msgs::msg::Marker label;
      label.header = status.header; label.ns = "confirmed_blocks"; label.id = index++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = track.accepted; label.pose.position.z += .07;
      label.pose.orientation.w = 1;
      label.scale.z = .028;
      label.color.r = label.color.g = label.color.b = fresh ? 1.f : .65f;
      label.color.a = 1;
      label.text = name + (fresh ? "" : "\nLAST_SEEN");
      markers.markers.push_back(label);
    }
    if (!color_update.object_colors.empty() && scene_->applyPlanningScene(color_update)) {
      for (const auto& item : color_update.object_colors) {
        const auto name = item.id.substr(0, item.id.size() - 6);
        rendered_fresh_[name] = status.camera_stable && tracks_[name].fresh(now().seconds());
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
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;
  const std::array<std::string, 3> names_{{"red", "green", "blue"}};
  std::map<std::string, classify_scene_bridge::ObservationTrack> tracks_;
  std::map<std::string, bool> attached_;
  std::map<std::string, std::optional<bool>> rendered_fresh_;
  std::chrono::steady_clock::time_point last_message_wall_{};
  double last_message_stamp_{-1};
  bool camera_stable_{false};
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
