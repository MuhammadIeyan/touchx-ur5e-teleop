"""Guard-required, fixed-orientation continuous haptic teleoperation session.

One 2 mm goal at a time; no trajectory queue. Release cancels and re-anchors.
Budget checkpoints re-arm from current pose every 50 mm so the session is
unbounded in total travel. Faults terminate the session. No force feedback,
RCM or automatic Home.
"""
import os
import time

import numpy as np
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectoryPoint

from real_robot.bench_step import BenchStep, CONTROLLER, main as run_step
from real_robot.bench_motion import GoalLease
from real_robot.travel_budget import TravelBudget
from real_robot.path_geometry import PathGeometry


class Segmented(BenchStep):
    step_mm = 5.0
    budget_mm = 200.0
    mode = 'rate'

    def __init__(self,*args,**kwargs):
        if 'UR5E_GUARD_FD' not in os.environ:
            raise RuntimeError('Segmented motion requires the independent guard')
        self.budget=None
        self.geometry=None
        self.settle_until=0.
        self.is_continuous=False
        self.step_m = max(0.0001, min(0.030, float(getattr(self, 'step_mm', 5.0)) / 1000.0))
        self.budget_m = max(0.05, min(0.60, float(getattr(self, 'budget_mm', 200.0)) / 1000.0))
        super().__init__(*args,**kwargs)
        mode = getattr(self, 'mode', 'rate')
        self.clutch.scale = 1.0
        self.clutch.mode = mode
        if mode == 'rate':
            self.clutch.deadband = 0.003
            self.clutch.max_linear_speed = 0.05
            self.clutch.rate_gain = 2.0
        self.duration = max(0.15, self.step_m / 0.03)
        self.log('segmented_configuration',total_budget_m=self.budget_m,segment_m=self.step_m,
                 duration_s=self.duration,translation_scale=1.0,rotation_enabled=False,mode=mode)

    def finished(self,future):
        result=future.result()
        success=result.status==GoalStatus.STATUS_SUCCEEDED and result.result.error_code==0
        cancelled=result.status==GoalStatus.STATUS_CANCELED and self.lease.stop_requested
        self.log('segment_terminal',status=result.status,error_code=result.result.error_code,
                 commanded_m=self.budget.commanded if self.budget else 0.)
        self.lease.terminal=True
        if not success and not cancelled:
            self.fail('Segment failed; session stopped')
            self.done=True
            return
        if self.fault:
            self.done=True
            return
        if cancelled:
            self.clutch.reset()
            self.is_continuous=False
        self.lease=GoalLease()
        self.pending=self.result_future=self.cancel_future=None

        # Seamless checkpoint re-arm on segment completion when nearing budget
        total_budget = getattr(self.budget, 'total_budget_m', None)
        if total_budget is not None and getattr(self.budget, 'commanded', 0) >= total_budget * 0.75 and hasattr(self, 'model'):
            pose = self.model.fk(self.q)
            self.budget.reset_origin(self.q, pose)
            if getattr(self, 'geometry', None) is not None:
                self.geometry.initial = self.geometry.transforms(self.q)
            if hasattr(self, 'get_logger'):
                self.get_logger().info(f'Checkpoint {self.budget.checkpoints}: re-armed origin at {np.round(pose[:3,3]*1000, 1)} mm; continuing motion')

        # If user is still holding the button in continuous mode, do not pause or wait to settle!
        touch = getattr(self, 'touch', None)
        if touch is not None and len(getattr(touch, 'buttons', [])) >= 2 and touch.buttons[0] and not touch.buttons[1]:
            self.settle_until = 0.0
            self.is_continuous = True
        else:
            self.settle_until = time.monotonic() + 0.05
            self.is_continuous = False

    def tick(self):
        if self.closing or not self.heartbeat():
            return
        now=time.monotonic()
        reason=self.ready()
        self.log('sample',q=self.q.tolist() if self.q is not None else None,
                 tcp=self.tcp[:3,3].tolist() if self.tcp is not None else None,
                 button=self.touch.buttons[0] if self.touch is not None else None,
                 reason=reason,commanded_m=self.budget.commanded if self.budget else 0.)
        if reason:
            if self.lease.sent or self.budget is not None:
                self.fail(reason)
            self.clutch.reset()
            if reason!=self.last_reason:
                self.get_logger().info(reason)
                self.last_reason=reason
            return
        pose=self.model.fk(self.q)
        if self.budget is not None:
            try:self.budget.observe(self.q,pose)
            except ValueError as error:
                self.fail(str(error));return
        if self.lease.sent:
            deadline = max(4.0, getattr(self, 'duration', 0.2) * 4.0 + 1.0)
            if now-self.sent_at > deadline:
                self.fail(f'Segment exceeded {deadline:.1f} s; stopping instead of resuming stale motion')
            return
        if now<self.settle_until:
            return
        if self.geometry is None:
            try:
                self.geometry=PathGeometry(self.model,self.model_xml,self.q)
                self.geometry.check(self.q)
            except ValueError as error:
                self.fail(str(error));return
            return  # Process fresh feedback before any controller activation.
        active=any(c.name==CONTROLLER and c.state=='active' for c in self.controllers)
        if not active:
            if self.activated_here:
                # During initial activation, wait for inventory acknowledgement.
                if self.budget is not None:
                    self.fail('Motion controller became inactive')
                return
            self.clutch.reset()
            if self.touch.buttons[0] or not self.switch.service_is_ready():return
            if not self.heartbeat('A'):return
            request=SwitchController.Request()
            request.activate_controllers=[CONTROLLER]
            request.strictness=SwitchController.Request.STRICT
            request.timeout.sec=2
            self.switch_pending=self.switch.call_async(request)
            self.activated_here=True
            self.log('activation_requested')
            return
        if not self.activated_here:
            self.fail('Controller already active; refusing another session');return
        budget_m = getattr(self, 'budget_m', 0.20)
        if self.budget is None:
            self.budget = TravelBudget(self.q, pose, total_budget_m=budget_m, max_step_m=self.step_m)
            self.log('session_origin', q=self.q.tolist(), flange=pose.tolist())
            self.get_logger().info(f'Continuous haptic session ready ({self.step_m*1000:.1f} mm/step at {self.step_m/self.duration*1000:.0f} mm/s, checkpoint {budget_m*1000:.0f} mm); hold to move')
        m=self.touch
        target=self.clutch.update(m.axes[:3],m.buttons[:2],m.buttons[2],m.buttons[3],pose[:3,3])
        if target is None:
            self.is_continuous = False
            return

        direction=target-pose[:3,3]
        if np.linalg.norm(direction)<.0005:
            self.is_continuous = False
            return

        # Stationary check: only required when starting from rest, NOT while continuing continuous motion!
        if not self.stationary and not getattr(self, 'is_continuous', False):
            return

        step_len = getattr(self, 'step_m', self.budget.max_step_m if self.budget else 0.0001)

        # Seamless checkpoint re-arm before next step exceeds budget
        if self.budget.commanded + step_len > self.budget.total_budget_m - 1e-7:
            cur_pose = self.model.fk(self.q)
            self.budget.reset_origin(self.q, cur_pose)
            if self.geometry is not None:
                self.geometry.initial = self.geometry.transforms(self.q)
            self.get_logger().info(f'Checkpoint {self.budget.checkpoints}: re-armed origin at {np.round(cur_pose[:3,3]*1000, 1)} mm; continuing motion')
            self.log('budget_checkpoint', checkpoints=self.budget.checkpoints, q=self.q.tolist(), flange=cur_pose.tolist())

        # Clamp desired position to the current checkpoint sphere
        delta=target-self.budget.pose0[:3,3]
        if np.linalg.norm(delta) > self.budget.total_budget_m:
            target = self.budget.pose0[:3,3] + self.budget.total_budget_m * delta / np.linalg.norm(delta)
            direction = target - pose[:3,3]
            if np.linalg.norm(direction) < 0.0005:
                return

        try:
            end,planned=self.model.plan(self.q,direction,orientation=self.budget.pose0[:3,:3],step_m=step_len)
            self.geometry.segment(self.q,end)
            self.budget.reserve(end,planned,length=step_len)
        except ValueError as error:
            self.fail(str(error));return
        if self.ready() is not None or not self.action.server_is_ready():
            self.fail('State/action unavailable after planning');return

        dur = getattr(self, 'duration', 0.2)
        v_cruise = [0.0] * 6
        if getattr(self, 'is_continuous', False) and dur > 0:
            v_cruise = ((end - self.q) / dur).tolist()

        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=self.model.names
        for stamp, q_pt, v_pt in ((0., self.q, v_cruise if getattr(self, 'is_continuous', False) else [0.]*6),
                                  (dur, end, v_cruise if getattr(self, 'is_continuous', False) else [0.]*6)):
            point=JointTrajectoryPoint(positions=q_pt.tolist() if isinstance(q_pt, np.ndarray) else q_pt,
                                       velocities=v_pt, accelerations=[0.]*6)
            point.time_from_start.nanosec=round(stamp*1e9)
            goal.trajectory.points.append(point)
        goal.path_tolerance=[JointTolerance(name=n,position=.005) for n in self.model.names]
        goal.goal_tolerance=[JointTolerance(name=n,position=.001,velocity=.01) for n in self.model.names]
        goal.goal_time_tolerance.sec=1
        self.lease.reserve();self.sent_at=time.monotonic()
        self.pending=self.action.send_goal_async(goal)
        self.pending.add_done_callback(self.accepted)
        self.log('goal_sent',q_end=end.tolist(),commanded_m=self.budget.commanded)


def main():
    import argparse,json
    from pathlib import Path
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',required=True)
    parser.add_argument('--mode',choices=['rate','position'],default='rate',help='Teleoperation mode: rate (continuous motion while holding button) or position (clutched step)')
    parser.add_argument('--step-mm',type=float,default=5.0,help='Step size in mm for visible motion (default: 5.0 mm)')
    parser.add_argument('--budget-mm',type=float,default=200.0,help='Checkpoint budget sphere radius in mm (default: 200.0 mm; auto-re-arms)')
    parser.add_argument('--timeout-min',type=float,default=60.0,help='Session hard timeout in minutes (default: 60)')
    args,extra=parser.parse_known_args()
    session=json.loads(Path(args.session).read_text())
    if session.get('minimum_clearance_mm',0)<660:
        parser.error('Larger session requires the recorded 660 mm clearance')
    Segmented.step_mm=args.step_mm
    Segmented.budget_mm=args.budget_mm
    Segmented.mode=args.mode
    run_step(['--execute-one-step','--session',args.session]+extra,node_type=Segmented,timeout_s=int(args.timeout_min*60))


if __name__=='__main__':
    # Compatibility entry point: physical users get the rolling controller.
    # The old class remains importable for historical regression tests only.
    from real_robot.continuous import main as continuous_main
    continuous_main()
