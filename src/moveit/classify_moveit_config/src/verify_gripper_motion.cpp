#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("verify_gripper_motion");
  const bool execute = node->declare_parameter<bool>("execute", false);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int result = 1;
  try {
    moveit::planning_interface::MoveGroupInterface gripper(node, "gripper");
    gripper.setPlanningTime(8.0);
    gripper.setNumPlanningAttempts(5);
    gripper.setMaxVelocityScalingFactor(0.25);
    gripper.setMaxAccelerationScalingFactor(0.25);

    const auto names = gripper.getJointNames();
    if (names.size() != 2 || names.at(0) != "joint7" || names.at(1) != "joint8") {
      RCLCPP_ERROR(node->get_logger(), "Unexpected gripper joint order");
    } else {
      const auto check_position = [&](const char* label, double joint7, double joint8) {
        const auto actual = gripper.getCurrentJointValues();
        if (actual.size() != 2 || std::abs(actual.at(0) - joint7) > 0.002 ||
            std::abs(actual.at(1) - joint8) > 0.002) {
          RCLCPP_ERROR(node->get_logger(), "%s: measured gripper joints outside 2 mm tolerance", label);
          return false;
        }
        RCLCPP_INFO(node->get_logger(), "%s: measured joint7=%.5f m joint8=%.5f m",
                    label, actual.at(0), actual.at(1));
        return true;
      };

      gripper.setStartStateToCurrentState();
      gripper.setNamedTarget("open");
      moveit::planning_interface::MoveGroupInterface::Plan open_plan;
      if (gripper.plan(open_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Could not plan gripper opening");
      } else if (!execute) {
        RCLCPP_INFO(node->get_logger(), "GRIPPER_PLAN_OK: %zu points; execute=false",
                    open_plan.trajectory_.joint_trajectory.points.size());
        result = 0;
      } else if (gripper.execute(open_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Could not execute gripper opening");
      } else if (check_position("GRIPPER_OPEN_OK", 0.035, -0.035)) {
        gripper.setStartStateToCurrentState();
        gripper.setNamedTarget("closed");
        moveit::planning_interface::MoveGroupInterface::Plan close_plan;
        if (gripper.plan(close_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Could not plan gripper closing");
        } else if (gripper.execute(close_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Could not execute gripper closing");
        } else if (check_position("GRIPPER_CLOSED_OK", 0.0005, -0.0005)) {
          result = 0;
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Gripper verification failed: %s", e.what());
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result;
}
