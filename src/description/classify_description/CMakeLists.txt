cmake_minimum_required(VERSION 3.8)
project(classify_description)

find_package(ament_cmake REQUIRED)

# ------------------------------------------------------------------------------
# 机械臂本体 URDF
#   base_link / link1..link6 / gripper_base / link7 / link8
#   joint1..joint6 (revolute) + joint7,joint8 (prismatic 镜像夹爪)
#   collision 已全部改为 box，visual 保留官方 STL
# ------------------------------------------------------------------------------
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# 控制器配置
#   ros2_controllers.yaml            3 个控制器（手臂 / 夹爪 / 关节广播）
#   gazebo_initial_positions.yaml    Gazebo 初始位姿（非全零，避免奇异位形）
# ------------------------------------------------------------------------------
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Gazebo world：地面 / 桌面 / 3 彩色工件 / 2 料盒 / 俯视 RGB-D 相机
# ------------------------------------------------------------------------------
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# STL 网格（10 个，官方原版，二进制格式，米制，无 scale）
#   注意 link2.STL 3.6MB / 73166 面，只用于 visual，不可用于 collision
# ------------------------------------------------------------------------------
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# launch / rviz 配置
# ------------------------------------------------------------------------------
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
