#!/usr/bin/env python3
"""Query the live controller manager without ROS CLI daemon cache."""
import rclpy
from controller_manager_msgs.srv import ListControllers

rclpy.init()
node = rclpy.create_node('classify_controller_check')
client = node.create_client(ListControllers, '/controller_manager/list_controllers')
ok = False
try:
    if client.wait_for_service(timeout_sec=2):
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        if future.done() and future.result() is not None:
            rows = future.result().controller
            for row in rows:
                print(row.name, row.type, row.state)
            required = {'piper_joint_state_broadcaster', 'piper_arm_controller', 'piper_gripper_controller'}
            ok = required <= {row.name for row in rows if row.state == 'active'}
finally:
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if ok else 1)
