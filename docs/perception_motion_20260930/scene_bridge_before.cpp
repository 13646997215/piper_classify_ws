#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <classify_interfaces/msg/detected_blocks.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

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
  return {top, legs, bin("bin_red", 0.42, -0.22), bin("bin_blue", 0.42, 0.22)};
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
  }

private:
  void update_blocks(const classify_interfaces::msg::DetectedBlocks& detections)
  {
    if (detections.header.frame_id != "base_link") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Expected base_link detections; got '%s'", detections.header.frame_id.c_str());
      return;
    }
    std::vector<CollisionObject> updates;
    std::vector<ObjectColor> colors;
    for (const auto& block : detections.blocks) {
      if (block.color != "red" && block.color != "green" && block.color != "blue") continue;
      const auto& p = block.center;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      const std::string id = block.color + "_block";
      // A grasped block becomes an attached object. Do not reintroduce it as
      // a world obstacle while the gripper carries it.
      if (!scene_->getAttachedObjects({id}).empty()) continue;
      const auto last = last_positions_.find(id);
      if (last != last_positions_.end() &&
          std::hypot(std::hypot(p.x - last->second.x, p.y - last->second.y),
                     p.z - last->second.z) < 0.004) continue;
      auto cube = object(id, "base_link");
      add_box(cube, 0.05, 0.05, 0.05, p.x, p.y, p.z);
      updates.push_back(cube);
      colors.push_back(block_color(block.color));
      last_positions_[id] = p;
    }
    if (!updates.empty() && !scene_->applyCollisionObjects(updates, colors)) {
      RCLCPP_ERROR(get_logger(), "Failed to update detected blocks in MoveIt planning scene");
      for (const auto& item : updates) last_positions_.erase(item.id);
    }
  }

  rclcpp::Subscription<classify_interfaces::msg::DetectedBlocks>::SharedPtr detections_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;
  std::map<std::string, geometry_msgs::msg::Point> last_positions_;
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
