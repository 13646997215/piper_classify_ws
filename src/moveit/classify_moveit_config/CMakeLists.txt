cmake_minimum_required(VERSION 3.8)
project(classify_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(verify_joint_motion src/verify_joint_motion.cpp)
ament_target_dependencies(verify_joint_motion rclcpp moveit_ros_planning_interface)
target_compile_features(verify_joint_motion PUBLIC cxx_std_17)

add_executable(verify_gripper_motion src/verify_gripper_motion.cpp)
ament_target_dependencies(verify_gripper_motion rclcpp moveit_ros_planning_interface)
target_compile_features(verify_gripper_motion PUBLIC cxx_std_17)

install(TARGETS verify_joint_motion verify_gripper_motion DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
