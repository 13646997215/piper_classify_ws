cmake_minimum_required(VERSION 3.8)
project(classify_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(classify_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)

add_executable(detect_blocks src/detect_blocks.cpp)
ament_target_dependencies(detect_blocks
  rclcpp sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros
  cv_bridge classify_interfaces message_filters)
target_link_libraries(detect_blocks ${OpenCV_LIBS})
target_compile_features(detect_blocks PUBLIC cxx_std_17)
install(TARGETS detect_blocks DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_block_geometry test/test_block_geometry.cpp)
  target_include_directories(test_block_geometry PRIVATE src ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_block_geometry ${OpenCV_LIBS})
  target_compile_features(test_block_geometry PUBLIC cxx_std_17)
endif()

ament_package()
