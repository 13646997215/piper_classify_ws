cmake_minimum_required(VERSION 3.8)
project(classify_scene_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(classify_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(scene_bridge src/scene_bridge.cpp)
ament_target_dependencies(scene_bridge
  rclcpp classify_interfaces geometry_msgs moveit_ros_planning_interface moveit_msgs shape_msgs std_srvs visualization_msgs)
target_compile_features(scene_bridge PUBLIC cxx_std_17)
install(TARGETS scene_bridge DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scene_tracker test/test_scene_tracker.cpp)
  target_include_directories(test_scene_tracker PRIVATE src)
  ament_target_dependencies(test_scene_tracker geometry_msgs)
  target_compile_features(test_scene_tracker PUBLIC cxx_std_17)
endif()

ament_package()
