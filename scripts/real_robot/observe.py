#!/usr/bin/env python3
"""Domain-43 read-only observation and haptic preview. Cannot command the robot."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import Joy, JointState
from rclpy.qos import QoSProfile, DurabilityPolicy
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from controller_manager_msgs.srv import ListControllers
from real_robot.preview import Preview

class Observer(Node):
    def __init__(self,path):
        if os.environ.get('ROS_DOMAIN_ID')!='43':raise RuntimeError('Real observation requires domain 43')
        super().__init__('ur5e_readonly_observer')
        manifest=Path(__file__).resolve().parents[2]/'config/real_robot_commissioning.json'
        metadata=json.loads(manifest.read_text())
        metadata['recording_mode']='preview_only'
        metadata['recording_started_ns']=time.time_ns()
        with open(str(path)+'.metadata.json','x') as meta:
            json.dump(metadata,meta,indent=2,allow_nan=False)
        self.file=open(path,'x',buffering=1)
        self.preview=Preview();self.tcp=None;self.touch=None;self.tcp_rx=0;self.touch_rx=0
        self.last_tcp_stamp=-1;self.last_touch_stamp=-1
        self.joints=None;self.joints_rx=0;self.wrench=None;self.wrench_rx=0
        self.robot_mode=None;self.safety_mode=None;self.active_controllers=[];self.controllers_rx=0
        self.last_reason=None;self.request=None
        self.client=self.create_client(ListControllers,'/controller_manager/list_controllers')
        self.create_subscription(PoseStamped,'/tcp_pose_broadcaster/pose',self.on_tcp,1)
        self.create_subscription(Joy,'/touch_x/state',self.on_touch,1)
        self.create_subscription(JointState,'/joint_states',self.on_joints,1)
        self.create_subscription(WrenchStamped,'/force_torque_sensor_broadcaster/wrench',self.on_wrench,1)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RobotMode,'/io_and_status_controller/robot_mode',lambda m:setattr(self,'robot_mode',m.mode),latched)
        self.create_subscription(SafetyMode,'/io_and_status_controller/safety_mode',lambda m:setattr(self,'safety_mode',m.mode),latched)
        # Deliberately no publisher, action client, controller-switch or dashboard client.
        self.create_timer(.02,self.tick);self.create_timer(1.,self.query_controllers)
        self.get_logger().info('READ ONLY: recording state and preview; no robot commands or haptic forces')

    def stamp(self,header):
        value=header.stamp.sec*10**9+header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-value)*1e-9
        return value if 0<=age<.15 else None

    def on_tcp(self,msg):
        stamp=self.stamp(msg.header);p=msg.pose.position;q=msg.pose.orientation
        pose=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if stamp is None or stamp<=self.last_tcp_stamp or msg.header.frame_id!='base' or not np.all(np.isfinite(pose)) or abs(np.linalg.norm(pose[3:])-1)>.02:
            self.tcp=None;return
        self.last_tcp_stamp=stamp;self.tcp=pose;self.tcp_rx=time.monotonic()

    def on_touch(self,msg):
        stamp=self.stamp(msg.header)
        if stamp is None or stamp<=self.last_touch_stamp or msg.header.frame_id!='touch_x_base' or len(msg.axes)!=7 or len(msg.buttons)!=4 or not np.all(np.isfinite(msg.axes)) or any(b not in (0,1) for b in msg.buttons[:3]):
            self.touch=None;self.preview.clutch.reset();return
        self.last_touch_stamp=stamp;self.touch=msg;self.touch_rx=time.monotonic()
        if not msg.buttons[0] or msg.buttons[1] or not msg.buttons[2]:
            self.preview.clutch.active=False
            self.preview.clutch.released=not msg.buttons[0] and not msg.buttons[1] and bool(msg.buttons[2])

    def on_joints(self,msg):
        if len(msg.name)==len(msg.position) and np.all(np.isfinite(msg.position)):
            self.joints=dict(zip(msg.name,msg.position));self.joints_rx=time.monotonic()

    def on_wrench(self,msg):
        f,t=msg.wrench.force,msg.wrench.torque;v=[f.x,f.y,f.z,t.x,t.y,t.z]
        if self.stamp(msg.header) is not None and np.all(np.isfinite(v)):
            self.wrench={'frame':msg.header.frame_id,'values':v};self.wrench_rx=time.monotonic()

    def query_controllers(self):
        if self.request is not None:
            if not self.request.done():return
            result=self.request.result();self.request=None
            if result is not None:
                self.active_controllers=[{'name':c.name,'type':c.type,'claims':list(c.claimed_interfaces)} for c in result.controller if c.state=='active']
                self.controllers_rx=time.monotonic()
        if self.client.service_is_ready():self.request=self.client.call_async(ListControllers.Request())

    def tick(self):
        now=time.monotonic()
        reason='ready'
        if self.tcp is None or now-self.tcp_rx>=.15:reason='missing/stale robot TCP in base frame'
        elif self.touch is None or now-self.touch_rx>=.15:reason='missing/stale Touch state'
        elif now-self.controllers_rx>2.5:reason='controller inventory unavailable'
        elif any(i.rsplit('/',1)[-1] in ('position','velocity','effort') for c in self.active_controllers for i in c['claims']):reason='a command interface is claimed; deactivate motion controllers for observation'
        target=None
        if self.tcp is not None and self.touch is not None:
            m=self.touch
            target=self.preview.update(now,m.axes[:3],m.buttons[:2],m.buttons[2],m.buttons[3],self.tcp[:3],fresh=reason=='ready')
        else:self.preview.clutch.reset()
        if reason!=self.last_reason:self.get_logger().info(reason);self.last_reason=reason
        row={'mode':'preview_only','monotonic_s':now,'wall_time_ns':time.time_ns(),'reason':reason,
             'tcp':self.tcp if now-self.tcp_rx<.15 else None,
             'preview_xyz':target.tolist() if target is not None else None,
             'touch_xyz':list(self.touch.axes[:3]) if self.touch and now-self.touch_rx<.15 else None,
             'button':self.touch.buttons[0] if self.touch and now-self.touch_rx<.15 else None,
             'joints':self.joints if now-self.joints_rx<.15 else None,
             'wrench':self.wrench if now-self.wrench_rx<.15 else None,
             'robot_mode':self.robot_mode,'safety_mode':self.safety_mode,
             'active_controller_names':[c['name'] for c in self.active_controllers]}
        self.file.write(json.dumps(row,allow_nan=False)+'\n')

    def close(self):self.file.close()

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='43':parser.error('Requires domain 43')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Observer(args.output)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
