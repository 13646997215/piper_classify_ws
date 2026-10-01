#!/usr/bin/env python3
"""Reject visually observed extra inventory rather than silently truncating IDs."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict as convert
from classify_interfaces.msg import DetectedBlocks, SceneStatus, TaskStatus, TaskResult
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
rclpy.init();n=Node('verify_extra_inventory',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
msgs={};goals=[]
subs=[n.create_subscription(DetectedBlocks,'/classify/detections',lambda m:msgs.update(d=m),10),
      n.create_subscription(SceneStatus,'/classify/scene_status',lambda m:msgs.update(s=m),qos),
      n.create_subscription(TaskStatus,'/classify/status',lambda m:msgs.update(t=m),qos),
      n.create_subscription(TaskResult,'/classify/result',lambda m:msgs.update(r=m),qos),
      n.create_subscription(JointState,'/joint_states',lambda m:msgs.update(j=m),10),
      n.create_subscription(GoalStatusArray,'/piper_arm_controller/follow_joint_trajectory/_action/status',goals.append,10)]
def wait(predicate,seconds=30):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.1)
        if predicate():return
    raise AssertionError('Condition timed out')
def call(name):
    c=n.create_client(Trigger,name);assert c.wait_for_service(timeout_sec=5)
    f=c.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5)
    assert f.done() and f.result();return f.result()
record={};failure=None
try:
    wait(lambda:'d' in msgs and 's' in msgs and 'j' in msgs and 't' in msgs and
         len(msgs['d'].blocks)==4 and 'unexpected extra' in msgs['s'].detail and msgs['s'].camera_stable)
    assert not msgs['s'].ready;record['scene']=convert(msgs['s']);record['detections']=convert(msgs['d'])
    assert msgs['t'].state=='IDLE';before=dict(zip(msgs['j'].name,msgs['j'].position))
    assert call('/classify/start').success;wait(lambda:msgs['t'].state=='DETECTING',10)
    end=time.monotonic()+3
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
    assert msgs['t'].state=='DETECTING' and not msgs['s'].ready
    assert not any(m.status_list for m in goals),'Arm goal sent for inconsistent inventory'
    after=dict(zip(msgs['j'].name,msgs['j'].position));change=max(abs(after[k]-v) for k,v in before.items())
    assert change<.005,change;record['max_joint_change_rad']=change
    assert call('/classify/cancel').success;wait(lambda:'r' in msgs,10)
    assert msgs['t'].state=='CANCELLED' and not msgs['r'].success and msgs['r'].failed==0
    assert not call('/classify/start').success
    record.update(passed=True,result=convert(msgs['r']),no_arm_goals=True)
    print('VERIFY_INVENTORY_OK: extra visible cube rejected; no motion; cancellation locked',flush=True)
except Exception as e:
    failure=str(e);record.update(passed=False,failure=failure);print('VERIFY_INVENTORY_FAILED:',failure,flush=True)
finally:
    (args.output/'record.json').write_text(json.dumps(record,indent=2));n.destroy_node();rclpy.shutdown()
if failure:raise SystemExit(1)
