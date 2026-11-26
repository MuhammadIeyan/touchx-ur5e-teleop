"""Guarded rolling-horizon position teleoperation; no physical RCM or forces.

Refresh trajectories without waiting for completion. Every replacement has a
finite zero-velocity endpoint; retain handles so release cancels late acceptance.
"""
import argparse
import os
import time
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance,JointTrajectoryControllerState
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters
from trajectory_msgs.msg import JointTrajectoryPoint
import rclpy

from real_robot.bench_step import BenchStep,CONTROLLER,main as run_step
from real_robot import planning
from real_robot.motion_profile import MotionProfile,duration_nanoseconds
from real_robot.smooth_control import HoldGesture,ActionWindow


class Continuous(BenchStep):
    speed_mm_s=12.0
    workspace_m=.1
    motion_scale=2

    def __init__(self,*args,**kwargs):
        if 'UR5E_GUARD_FD' not in os.environ:raise RuntimeError('Independent guard required')
        self.window=ActionWindow()
        self.gesture=HoldGesture()
        self.geometry=None
        self.planner=None
        self.planner_initializing=None
        self.planning=None
        self.plan_generation=0
        self.expired_plans=0
        self.origin=None
        self.reference=None
        self.reference_at=0.
        self.reference_stamp=-1
        self.last_send=0.
        self.cancelled=set()
        self.last_epoch=None
        self.activation_at=None
        self.last_status=None
        self.at_boundary=False
        self.blending_verified=False
        self.blending_future=None
        self.blending_started=None
        super().__init__(*args,**kwargs)
        self.planner_notify=self.create_guard_condition(self.tick)
        self.parameter_client=self.create_client(GetParameters,f'/{CONTROLLER}/get_parameters')
        self.clutch.mode='position'
        self.clutch.scale=self.motion_scale
        self.status_pub=self.create_publisher(String,'/real_teleop/status',1)
        for topic in ('controller_state','state'):
            self.create_subscription(JointTrajectoryControllerState,f'/{CONTROLLER}/{topic}',self.controller_state,1)
        self.log('smooth_configuration',workspace_radius_m=self.workspace_m,scale=self.motion_scale,
                 refresh_s=.05,horizon_s=MotionProfile(self.speed_mm_s).duration_s,
                 speed_limit_mm_s=self.speed_mm_s,max_outstanding_displacement_m=.002,rcm_enabled=False)

    def status(self,text):
        if text!=self.last_status:
            self.last_status=text
            self.get_logger().info(text)
            self.status_pub.publish(String(data=text))
            self.log('status',text=text)

    def planner_finished(self,_):
        if not self.closing:
            try:self.planner_notify.trigger()
            except Exception:
                # A late worker result may race node shutdown; the normal
                # control timer also polls while the node remains alive.
                pass

    def verify_blending(self):
        # Never silently run the new command format against the old controller setup.
        if self.blending_verified:return True
        for controller in self.controllers or []:
            if controller.name==CONTROLLER and controller.type!='ur5e_clock_fixed_controller/ScaledJointTrajectoryController':
                self.fail('Installed controller has an activation clock bug; restart driver with run_ur5e_observe_driver.sh to load the local fix')
                return False
        if self.blending_started is None:self.blending_started=time.monotonic()
        if time.monotonic()-self.blending_started>5.:
            self.fail('Controller parameter check timed out: scaled controller may have failed to load; inspect driver terminal')
            return False
        if self.blending_future is None:
            if self.parameter_client.service_is_ready():
                request=GetParameters.Request(names=['open_loop_control','interpolation_method','command_interfaces'])
                self.blending_future=self.parameter_client.call_async(request)
            self.status('Checking controller interpolation settings')
            return False
        if not self.blending_future.done():return False
        try:
            values=self.blending_future.result().values
            valid=(len(values)==3 and values[0].bool_value
                   and values[1].string_value=='splines'
                   and list(values[2].string_array_value)==['position'])
        except Exception:
            valid=False
        if not valid:
            self.fail('Controller interpolation configuration mismatch; restart driver with run_ur5e_observe_driver.sh')
            return False
        self.blending_verified=True
        self.log('controller_blending_verified',open_loop_control=True,interpolation_method='splines')
        return True

    def controller_state(self,msg):
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp<=self.reference_stamp:return
        age=(self.get_clock().now().nanoseconds-stamp)/1e9
        if not 0<=age<.1:
            self.reference=None;return
        ref=msg.reference if len(msg.reference.positions)==6 else msg.desired
        if self.model is None or len(ref.positions)!=6 or len(ref.velocities)!=6:return
        try:
            order=[msg.joint_names.index(n) for n in self.model.names]
            q=np.array(ref.positions)[order];v=np.array(ref.velocities)[order]
            a=np.array(ref.accelerations)[order] if len(ref.accelerations)==6 else np.zeros(6)
        except (ValueError,IndexError):return
        if not np.all(np.isfinite(np.r_[q,v,a])):self.reference=None;return
        self.reference=(q,v,a)
        self.reference_at=time.monotonic()-age
        self.reference_stamp=stamp

    def haptic(self,msg):
        super().haptic(msg)
        valid=(self.touch is msg and bool(msg.buttons[2]) and not msg.buttons[1])
        if not valid:
            self.gesture.update(False,time.monotonic(),valid=False)
            self.stop_stream('Invalid or uncalibrated Touch input')
            return
        epoch=msg.buttons[3]
        if self.last_epoch is not None and epoch!=self.last_epoch:
            self.gesture.update(False,time.monotonic(),valid=False)
            self.clutch.reset()
            self.stop_stream('Calibration changed; release to re-arm')
        self.last_epoch=epoch
        if not msg.buttons[0]:self.stop_stream('Button released')

    def stop_stream(self,reason):
        # A running process cannot be cancelled reliably. Invalidate its result;
        # the worker has no ROS clients or hardware handles.
        self.plan_generation+=1
        self.expired_plans=0
        if not self.window.drained():
            if not self.window.stopping:self.log('stream_stop',reason=reason)
            self.window.stopping=True
            self.cancel_all()
        self.clutch.active=False

    def cancel_all(self):
        for seq,handle in list(self.window.handles.items()):
            if seq not in self.cancelled:
                self.cancelled.add(seq)
                handle.cancel_goal_async()
                self.log('cancel_requested',sequence=seq)

    def fail(self,reason):
        super().fail(reason)
        self.stop_stream(reason)

    def accept_stream(self,seq,future):
        try:handle=future.result()
        except Exception as error:
            self.window.pending=None;self.fail(f'Goal acceptance failed: {error}');return
        if not handle.accepted:
            self.window.pending=None;self.fail('Controller rejected rolling trajectory');return
        cancel=self.window.accept(seq,handle)
        handle.get_result_async().add_done_callback(lambda f:self.finish_stream(seq,f))
        self.log('accepted',sequence=seq)
        if cancel:self.cancel_all()

    def finish_stream(self,seq,future):
        try:result=future.result()
        except Exception as error:
            self.fail(f'Trajectory result unavailable: {error}');return
        expected_cancel=(seq in self.cancelled or seq<self.window.sequence or self.window.stopping)
        success=result.status==4 and result.result.error_code==0
        if not success and not (result.status==5 and expected_cancel):
            self.fail(f'Trajectory failed: status={result.status}, code={result.result.error_code}')
        self.window.terminal(seq);self.cancelled.discard(seq)
        self.log('trajectory_terminal',sequence=seq,status=result.status)

    def tick(self):
        if self.closing or not self.heartbeat():return
        now=time.monotonic()
        reason=self.ready()
        if reason:
            if self.origin is not None:self.fail(reason)
            self.gesture.update(False,now,valid=False)
            self.clutch.reset();self.status(reason);return
        if not self.verify_blending():return
        pose=self.model.fk(self.q)
        if self.geometry is None:
            if self.planner is None:
                self.planner=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn'))
                self.planner_initial_q=self.q.copy()
                self.planner_initializing=self.planner.submit(planning.initialize,self.model_xml,
                    self.manifest['expected_calibration_hash'],self.manifest['robot_ipv4'],self.planner_initial_q)
                self.planner_initializing.add_done_callback(self.planner_finished)
                self.planner_started=now
                self.status('Preparing collision geometry in planner worker')
            elif self.planner_initializing.done():
                try:
                    self.planner_initializing.result()
                    if np.max(np.abs(self.q-self.planner_initial_q))>.001:
                        raise ValueError('Robot moved during geometry initialization; restart from a stationary pose')
                    self.geometry=True
                except Exception as error:self.fail(f'Planner initialization failed: {error}')
            elif now-self.planner_started>10:self.fail('Planner initialization timeout')
            return
        active=any(c.name==CONTROLLER and c.state=='active' for c in self.controllers)
        if not active:
            if self.activated_here:
                if self.origin is not None or now-self.activation_at>3:self.fail('Controller activation lost/failed')
                return
            if self.touch.buttons[0]:self.status('Release button before activation');return
            if not self.switch.service_is_ready() or not self.heartbeat('A'):return
            request=SwitchController.Request();request.activate_controllers=[CONTROLLER]
            request.strictness=SwitchController.Request.STRICT;request.timeout.sec=2
            self.switch_pending=self.switch.call_async(request)
            self.activated_here=True;self.activation_at=now
            return
        if not self.activated_here:self.fail('Controller already active; refusing takeover');return
        if self.reference is None or now-self.reference_at>.1:
            if self.origin is not None:self.fail('Controller reference feedback stale')
            else:self.status('Waiting for controller reference feedback')
            return
        if self.origin is None:
            self.origin=pose.copy()
            self.q_origin=self.q.copy()
            self.log('fixed_session_origin',pose=pose.tolist(),q=self.q.tolist())
        if np.linalg.norm(pose[:3,3]-self.origin[:3,3])>self.workspace_m+.002:
            self.fail('Measured position left fixed workspace');return
        if np.sum(np.abs(self.q-self.q_origin))>1.:
            self.fail('Session joint envelope exceeded');return
        held,mode_request=self.gesture.update(bool(self.touch.buttons[0]),now)
        if mode_request:
            self.status('RCM not configured: free-space mode retained')
            self.log('mode_request_rejected',requested='rcm',reason='Measured pivot/tool geometry unavailable')
        if not held:
            self.stop_stream('Not holding motion enable')
            if self.window.stopping and self.window.drained():self.window.stopping=False
            self.clutch.update(self.touch.axes[:3],[0,0],True,self.touch.buttons[3],pose[:3,3])
            if not mode_request:self.status('Hold button to track; release to reposition')
            return
        if self.window.stopping:
            if not self.window.drained():return
            self.window.stopping=False
            self.clutch.reset()
            # The gesture has already observed a release and a fresh hold. Do
            # not demand a second release merely because cancellation was slow.
            self.clutch.released=True
            self.status('Re-engaging from current pose');return
        # Consume device movement on EVERY tick, including during goal acceptance.
        target=self.clutch.update(self.touch.axes[:3],[1,0],True,self.touch.buttons[3],pose[:3,3])
        if target is None:return
        offset=target-self.origin[:3,3]
        boundary=np.linalg.norm(offset)>self.workspace_m-.002
        if boundary:target=self.origin[:3,3]+offset/np.linalg.norm(offset)*(self.workspace_m-.002)
        # Hysteresis affects the message only; the position bound is unchanged.
        if boundary:self.at_boundary=True
        elif np.linalg.norm(offset)<self.workspace_m-.0025:self.at_boundary=False
        self.status('Workspace boundary; move back inward' if self.at_boundary else 'Free-space tracking')
        if self.window.pending is not None:
            if now-self.last_send>.2:self.fail('Goal acceptance timeout')
            return
        if len(self.window.handles)>8:self.fail('Old action results not retiring');return
        if self.planning is None:
            if now-self.last_send<.05:return
            q,v,a=self.reference
            if np.max(np.abs(q-self.q))>.01:self.fail('Reference/actual tracking error');return
            delta=target-self.model.fk(q)[:3,3]
            if np.linalg.norm(delta)<.00002 and np.max(np.abs(v))<.0001:return
            request=dict(reference=tuple(x.copy() for x in self.reference),
                reference_at=self.reference_at,reference_stamp=self.reference_stamp,
                target=target.copy(),origin=self.origin.copy(),workspace_m=self.workspace_m,speed_mm_s=self.speed_mm_s,
                generation=self.plan_generation,submitted_at=now)
            try:self.planning=(self.planner.submit(planning.plan,request),request)
            except Exception as error:self.fail(f'Planner worker unavailable: {error}');return
            self.planning[0].add_done_callback(self.planner_finished)
            return
        future,request=self.planning
        if not future.done():
            if now-request['submitted_at']>.5:self.fail('Planner worker stalled (>500 ms)')
            return
        self.planning=None
        reason=planning.rejection(request,self.plan_generation,self.reference_at,target,self.q,now)
        if reason:
            self.log('plan_discarded',reason=reason,snapshot_age_s=now-request['reference_at'])
            if reason=='plan expired':
                self.expired_plans+=1
                if self.expired_plans>=3:self.fail('Three planning deadlines missed; controller stopped')
            return
        try:result=future.result()
        except Exception as error:self.fail(f'Planning failed: {error}');return
        self.expired_plans=0
        q,v,a=request['reference'];end=result['end']
        self.log('planning_timing',worker_s=result['planning_s'],
                 snapshot_age_s=now-request['reference_at'])
        # Feedback, release and heartbeat processing continued during planning.
        # A fresh callback never makes an OLD planning snapshot fresh again.
        if self.ready() is not None or time.monotonic()-request['reference_at']>=.1:
            self.log('plan_discarded',reason='state changed before send');return
        if not self.action.server_is_ready():self.fail('Trajectory server unavailable');return
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=self.model.names
        # A t=0 waypoint forces stale feedback onto the controller immediately.
        # Supply only the future endpoint: open_loop_control + splines blends
        # from the controller's last commanded q/v/a at the real handoff time.
        point=JointTrajectoryPoint(positions=end.tolist(),velocities=[0.]*6,accelerations=[0.]*6)
        point.time_from_start.nanosec=duration_nanoseconds(result['duration_s'])
        goal.trajectory.points.append(point)
        goal.path_tolerance=[JointTolerance(name=n,position=.01) for n in self.model.names]
        goal.goal_tolerance=[JointTolerance(name=n,position=.001,velocity=.01) for n in self.model.names]
        goal.goal_time_tolerance.sec=1
        seq=self.window.begin();self.last_send=time.monotonic()
        future=self.action.send_goal_async(goal)
        future.add_done_callback(lambda f:self.accept_stream(seq,f))
        self.log('rolling_goal',sequence=seq,start=q.tolist(),velocity=v.tolist(),acceleration=a.tolist(),
                 reference_stamp_ns=request['reference_stamp'],reference_age_s=time.monotonic()-request['reference_at'],
                 end=end.tolist(),target=request['target'].tolist(),controller_blended=True,
                 duration_s=result['duration_s'],speed_limit_mm_s=request['speed_mm_s'])

    def stop_and_wait(self):
        self.closing=True
        self.stop_stream('Session shutdown')
        until=time.monotonic()+3
        while not self.window.drained() and time.monotonic()<until and rclpy.ok():
            rclpy.spin_once(self,timeout_sec=.02);self.cancel_all()
        if not self.window.drained():self.log('STREAM_CANCEL_UNCONFIRMED')
        super().stop_and_wait()
        if self.planner is not None:
            self.planner.shutdown(wait=False,cancel_futures=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed-mm-s',type=float,default=12.,help='Physical speed setting, 1–50 mm/s (default 12)')
    parser.add_argument('--session',help='Deprecated; continuous control uses the commissioned setup')
    parser.add_argument('--budget-mm',type=float,default=100.,help='Fixed workspace radius, never auto-rebased')
    parser.add_argument('--scale',type=float,default=1.0,help='Robot/hand displacement ratio (0.1–100.0)')
    parser.add_argument('--step-mm',type=float,default=None,help='Legacy argument; fixed-size steps are no longer used')
    parser.add_argument('--mode',choices=['position'],default='position')
    parser.add_argument('--timeout-min',type=float,default=10.)
    args=parser.parse_args()
    try:MotionProfile(args.speed_mm_s)
    except ValueError as error:parser.error(str(error))
    Continuous.speed_mm_s=args.speed_mm_s
    if args.step_mm is not None:
        print('Ignoring legacy --step-mm: smooth tracking follows the requested displacement.',flush=True)
    if not np.isfinite(args.budget_mm) or not 10<=args.budget_mm<=5000:parser.error('Workspace must be 10–5000 mm')
    if not np.isfinite(args.timeout_min) or not 0<args.timeout_min<=60:parser.error('Timeout must be 0–60 minutes')
    if args.session:print('Using commissioned setup; no daily session record is required.',flush=True)
    if not np.isfinite(args.scale) or not .1<=args.scale<=100.:parser.error('Scale must be 0.1–100.0')
    Continuous.motion_scale=args.scale
    Continuous.workspace_m=args.budget_mm/1000
    run_step(['--execute-one-step'],node_type=Continuous,timeout_s=args.timeout_min*60,commissioned_setup=True)


if __name__=='__main__':main()
