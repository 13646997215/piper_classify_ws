#!/usr/bin/env python3
"""Read-only IK feasibility probe for task development (no trajectories sent)."""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation

rclpy.init()
n=Node('task_ik_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
states=[]
sub=n.create_subscription(JointState,'/joint_states',states.append,10)
ik=n.create_client(GetPositionIK,'/compute_ik')
valid=n.create_client(GetStateValidity,'/check_state_validity')
assert ik.wait_for_service(timeout_sec=10)
while not states:rclpy.spin_once(n,timeout_sec=.2)
for x,y,z in [(.4,0,.028),(.4,0,.15),(.4,0,.23),(.4,-.14,.028),(.52,-.22,.13),(.52,.22,.13),(.52,-.22,.23)]:
    found=False
    for yaw in [0,1.5707963,-1.5707963,3.1415926]:
        req=GetPositionIK.Request();q=req.ik_request
        q.group_name='arm';q.ik_link_name='link6';q.robot_state.joint_state=states[-1]
        q.pose_stamped.header.frame_id='base_link'
        p=q.pose_stamped.pose;p.position.x=float(x);p.position.y=float(y);p.position.z=float(z+.118)
        rot=Rotation.from_euler('zyx',[yaw,0,3.141592653589793]).as_quat()
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,rot)
        q.timeout.sec=2;q.avoid_collisions=True
        f=ik.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=4)
        res=f.result()
        if res and res.error_code.val==1:
            print((x,y,z),'yaw',yaw,'IK_OK',dict(zip(res.solution.joint_state.name,res.solution.joint_state.position)),flush=True)
            found=True;break
    if not found:print((x,y,z),'NO_COLLISION_FREE_IK',flush=True)
n.destroy_node();rclpy.shutdown()
