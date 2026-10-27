#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("verify_joint_motion");
  const bool execute = node->declare_parameter<bool>("execute", false);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int result = 1;
  try {
    moveit::planning_interface::MoveGroupInterface arm(node, "arm");
    arm.setPlanningTime(8.0);
    arm.setNumPlanningAttempts(5);
    arm.setMaxVelocityScalingFactor(0.10);
    arm.setMaxAccelerationScalingFactor(0.10);

    // Start the monitor before recording the return target. A newly created
    // monitor can otherwise expose the robot model's zero defaults before
    // /clock and the first joint-state sample have arrived.
    if (!arm.startStateMonitor(5.0)) {
      throw std::runtime_error("No complete joint state received; refusing to move");
    }
    const auto measured_start = arm.getCurrentState(5.0);
    if (!measured_start) {
      throw std::runtime_error("Current robot state unavailable; refusing to move");
    }
    std::vector<double> start;
    measured_start->copyJointGroupPositions("arm", start);
    if (start.size() != 6) {
      RCLCPP_ERROR(node->get_logger(), "Expected 6 arm joint states, got %zu", start.size());
    } else {
      auto target = start;
      target.at(0) = std::clamp(start.at(0) + 0.08, -2.4, 2.4);
      RCLCPP_INFO(node->get_logger(), "Current joint1 %.5f rad; target %.5f rad", start.at(0), target.at(0));
      arm.setStartStateToCurrentState();
      if (!arm.setJointValueTarget(target)) {
        RCLCPP_ERROR(node->get_logger(), "Joint target rejected");
      } else {
        moveit::planning_interface::MoveGroupInterface::Plan forward;
        const auto planned = arm.plan(forward);
        if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "MoveIt failed to plan the small joint motion");
        } else if (!execute) {
          RCLCPP_INFO(node->get_logger(), "PLAN_OK: trajectory has %zu points; execute=false, robot did not move",
                      forward.trajectory_.joint_trajectory.points.size());
          result = 0;
        } else if (arm.execute(forward) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Controller did not execute forward trajectory");
        } else {
          const auto reached = arm.getCurrentJointValues();
          if (reached.size() != 6 || std::abs(reached.at(0) - target.at(0)) > 0.02) {
            RCLCPP_ERROR(node->get_logger(), "Measured joint1 did not reach target");
          } else {
            RCLCPP_INFO(node->get_logger(), "EXECUTE_OK: measured joint1 %.5f rad", reached.at(0));
            arm.setStartStateToCurrentState();
            arm.setJointValueTarget(start);
            moveit::planning_interface::MoveGroupInterface::Plan back;
            if (arm.plan(back) == moveit::core::MoveItErrorCode::SUCCESS &&
                arm.execute(back) == moveit::core::MoveItErrorCode::SUCCESS) {
              const auto restored = arm.getCurrentJointValues();
              if (restored.size() == 6 && std::equal(restored.begin(), restored.end(), start.begin(),
                    [](double actual, double initial) { return std::abs(actual - initial) < 0.02; })) {
                RCLCPP_INFO(node->get_logger(), "RESTORE_OK: all 6 joints returned; joint1 %.5f rad", restored.at(0));
                result = 0;
              } else {
                RCLCPP_ERROR(node->get_logger(), "Return trajectory ended outside tolerance");
              }
            } else {
              RCLCPP_ERROR(node->get_logger(), "Could not plan/execute return to initial pose");
            }
          }
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Verification failed: %s", e.what());
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result;
}
