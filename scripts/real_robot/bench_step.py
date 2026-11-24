#!/usr/bin/env python3
"""One Touch-directed 0.1 mm physical step, or default command-free preview.

Explicit execution requires a recent operator session record. No dashboard,
URScript, power/brake commands, force feedback, RCM or continuous streaming.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, SwitchController
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from touch_clutch import Clutch
from real_robot.bench_motion import Model, GoalLease, DURATION_S, validate_session, validate_payload, validate_commissioning
from real_robot.rtde_snapshot import snapshot

ROOT = Path(__file__).resolve().parents[2]
CONTROLLER = 'scaled_joint_trajectory_controller'


class BenchStep(Node):
    def __init__(self, execute, record, manifest, telemetry, release_test=False):
        if os.environ.get('ROS_DOMAIN_ID') != '43' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Bench adapter requires domain 43, localhost only')
        super().__init__('ur5e_bench_single_step')
        self.execute = execute
        self.guard_fd = int(os.environ['UR5E_GUARD_FD']) if 'UR5E_GUARD_FD' in os.environ else None
        if self.guard_fd is not None:
            os.set_blocking(self.guard_fd, False)
        self.duration = 5.0 if release_test else DURATION_S
        self.manifest = manifest
        self.record = record
        self.offset = np.eye(4)
        self.offset[:3, 3] = telemetry['tcp_offset'][:3]
        self.offset[:3, :3] = Rotation.from_rotvec(telemetry['tcp_offset'][3:]).as_matrix()
        self.model = None
        self.model_xml = None
        self.q = self.tcp = self.touch = None
        self.rx, self.stamps = {}, {}
        self.mode = self.safety = self.program = None
        self.clutch = Clutch(scale=.1, rot_scale=0)
        self.lease = GoalLease()
        self.cancel_future = self.result_future = self.pending = None
        self.fault = None
        self.done = False
        self.closing = False
        self.started = time.monotonic()
        self.sent_at = None
        self.last_reason = None
        self.controllers = None
        self.inventory_at = 0
        self.target = None
        self.start_q = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', self.description, latched)
        self.create_subscription(JointState, '/joint_states', self.joints, 1)
        self.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self.pose, 1)
        self.create_subscription(Joy, '/touch_x/state', self.haptic, 1)
        self.create_subscription(RobotMode, '/io_and_status_controller/robot_mode',
                                 lambda m: setattr(self, 'mode', m.mode), latched)
        self.create_subscription(SafetyMode, '/io_and_status_controller/safety_mode',
                                 lambda m: setattr(self, 'safety', m.mode), latched)
        self.create_subscription(Bool, '/io_and_status_controller/robot_program_running',
                                 lambda m: setattr(self, 'program', m.data), latched)
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.hw_client = self.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.list_pending = self.hw_pending = None
        self.hardware_ok = False
        self.switch_pending = None
        self.activated_here = False
        # Preview mode has neither an action client nor controller-switch client.
        self.action = self.switch = None
        if execute:
            self.action = ActionClient(self, FollowJointTrajectory, f'/{CONTROLLER}/follow_joint_trajectory')
            self.switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.create_timer(.25, self.inventory)
        self.create_timer(.02, self.tick)
        self.log('startup', execute=execute, telemetry=telemetry, duration_s=self.duration)

    def log(self, event, **data):
        self.record.write(json.dumps(dict(event=event, monotonic_s=time.monotonic(),
                                         wall_time_ns=time.time_ns(), **data), allow_nan=False) + '\n')
        self.record.flush()

    def heartbeat(self, kind='H'):
        if self.guard_fd is not None:
            try:
                os.write(self.guard_fd, f'{kind} {time.monotonic():.9f}\n'.encode())
            except (BlockingIOError, BrokenPipeError, OSError):
                self.fail('Independent guard heartbeat pipe unavailable')
                return False
        return True

    def fail(self, reason):
        if self.fault is None:
            self.fault = reason
            self.log('fault', reason=reason)
            self.get_logger().error(reason)
        self.clutch.reset()
        if self.lease.stop():
            self.cancel()
        if not self.lease.sent:
            self.done = True

    def fresh(self, name, header):
        stamp = header.stamp.sec * 10**9 + header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if not 0 <= age < .1 or stamp <= self.stamps.get(name, -1):
            self.rx.pop(name, None)
            return False
        self.stamps[name] = stamp
        self.rx[name] = time.monotonic()
        return True

    def description(self, msg):
        try:
            model = Model(msg.data, self.manifest['expected_calibration_hash'], self.manifest['robot_ipv4'])
            if self.model is None:
                self.model = model
                self.model_xml = msg.data
                self.log('model_verified', hash=self.manifest['expected_calibration_hash'])
            elif self.model_xml != msg.data:
                self.fail('Robot description changed during commissioning')
        except (ValueError, KeyError) as error:
            self.fail(str(error))

    def joints(self, msg):
        if self.model is None:
            return
        if not self.fresh('joints', msg.header):
            self.q = None
            return
        values = dict(zip(msg.name, msg.position))
        try:
            q = np.array([values[n] for n in self.model.names])
        except KeyError:
            self.q = None
            return
        if not np.all(np.isfinite(q)):
            self.q = None
            return
        self.q = q
        velocity = dict(zip(msg.name, msg.velocity))
        self.stationary = all(n in velocity and np.isfinite(velocity[n]) and abs(velocity[n]) < .005
                              for n in self.model.names)

    def pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        if (not self.fresh('tcp', msg.header) or msg.header.frame_id != 'base'
                or not np.all(np.isfinite(values)) or abs(np.linalg.norm(values[3:])-1) > .01):
            self.tcp = None
            return
        self.tcp = np.eye(4)
        self.tcp[:3, 3] = values[:3]
        self.tcp[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()

    def haptic(self, msg):
        valid = (self.fresh('touch', msg.header) and msg.header.frame_id == 'touch_x_base'
                 and len(msg.axes) == 7 and len(msg.buttons) == 4
                 and np.all(np.isfinite(msg.axes)) and all(b in (0, 1) for b in msg.buttons[:3]))
        if not valid:
            self.touch = None
            self.clutch.reset()
            if self.lease.sent:
                self.fail('Invalid haptic data during step')
            return
        self.touch = msg
        if self.lease.sent and not self.lease.terminal and not self.lease.stop_requested and (not msg.buttons[0] or msg.buttons[1] or not msg.buttons[2]
                               or msg.buttons[3] != self.clutch.epoch):
            reason = ('button_release' if not msg.buttons[0] else 'reserved_button' if msg.buttons[1]
                      else 'calibration_invalid' if not msg.buttons[2] else 'calibration_epoch_changed')
            self.log('button_or_calibration_stop', buttons=list(msg.buttons), reason=reason,
                     engagement_epoch=self.clutch.epoch)
            self.lease.stop()
            self.cancel()

    def inventory(self):
        if self.list_pending is not None and self.list_pending.done():
            result = self.list_pending.result()
            self.controllers = result.controller if result else None
            self.inventory_at = time.monotonic()
            self.list_pending = None
        if self.list_pending is None and self.list_client.service_is_ready():
            self.list_pending = self.list_client.call_async(ListControllers.Request())
        if self.hw_pending is None and self.hw_client.service_is_ready():
            self.hw_pending = self.hw_client.call_async(ListHardwareComponents.Request())
        if self.hw_pending is not None and self.hw_pending.done():
            result = self.hw_pending.result()
            self.hardware_ok = bool(result and len(result.component) == 1 and
                result.component[0].class_type == 'ur_robot_driver/URPositionHardwareInterface')

    def ready(self):
        now = time.monotonic()
        if self.fault:
            return self.fault
        if self.model is None or not self.hardware_ok:
            return 'Waiting for calibrated real robot description/hardware'
        if self.q is None or self.tcp is None or self.touch is None:
            return 'Waiting for valid joints, TCP and Touch measurements'
        stale = [n for n in ('joints', 'tcp', 'touch') if now-self.rx.get(n, 0) >= .1]
        if stale:
            return 'Stale input or robot feedback: ' + ', '.join(stale)
        if self.mode != 7 or self.safety != 1:
            return 'Robot must be RUNNING with NORMAL safety state'
        if self.execute and self.program is not True:
            return 'External Control program is not running'
        if self.controllers is None or now-self.inventory_at > 1:
            return 'Controller inventory unavailable/stale'
        owners = [c.name for c in self.controllers if c.state == 'active' and
                  any(i.rsplit('/', 1)[-1] in ('position', 'velocity', 'effort') for i in c.claimed_interfaces)]
        if any(name != CONTROLLER for name in owners):
            return 'Another controller owns motion interfaces'
        predicted = self.model.fk(self.q) @ self.offset
        if np.linalg.norm(predicted[:3, 3]-self.tcp[:3, 3]) > .002:
            return 'Calibrated FK / measured TCP position disagree (>2 mm)'
        if np.linalg.norm(Rotation.from_matrix(predicted[:3, :3] @ self.tcp[:3, :3].T).as_rotvec()) > .005:
            return 'Calibrated FK / measured TCP orientation disagree (>0.005 rad)'
        return None

    def cancel(self):
        if self.lease.handle is not None and not self.lease.terminal and self.cancel_future is None:
            self.log('cancel_requested')
            self.cancel_future = self.lease.handle.cancel_goal_async()

    def accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.fail('Controller rejected step')
            self.lease.terminal = self.done = True
            return
        must_cancel = self.lease.accepted(handle)
        self.log('accepted')
        self.result_future = handle.get_result_async()
        self.result_future.add_done_callback(self.finished)
        if must_cancel:
            self.cancel()

    def finished(self, future):
        result = future.result()
        self.lease.terminal = True
        self.log('terminal', status=result.status, error_code=result.result.error_code,
                 error_string=result.result.error_string,
                 joints=self.q.tolist() if self.q is not None else None)
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            self.get_logger().warn('Step did not finish successfully; inspect log and pendant')
        self.get_logger().info('Goal terminal; observing stationary feedback for 2 seconds')
        self.terminal_at = time.monotonic()

    def tick(self):
        if self.closing:
            return
        if not self.heartbeat():
            return
        now = time.monotonic()
        self.log('sample', q=self.q.tolist() if self.q is not None else None,
                 tcp=self.tcp[:3, 3].tolist() if self.tcp is not None else None,
                 button=self.touch.buttons[0] if self.touch is not None else None,
                 mode=self.mode, safety=self.safety, program=self.program,
                 feedback_age_s={k: now-v for k,v in self.rx.items()})
        if self.lease.terminal:
            if now-getattr(self, 'terminal_at', now) > 2:
                self.done = True
            self.log('post_step', q=self.q.tolist() if self.q is not None else None,
                     tcp=self.tcp[:3, 3].tolist() if self.tcp is not None else None)
            return
        reason = self.ready()
        if reason != self.last_reason:
            self.get_logger().info(reason or 'Measurements ready; release button, then hold and move gently')
            self.last_reason = reason
        if self.lease.sent:
            if reason or now-self.sent_at > max(3.,self.duration+1.):
                self.fail(reason or 'Step deadline exceeded; cancelling rather than resuming later')
            return
        if reason:
            self.clutch.reset()
            return
        if self.execute:
            active = any(c.name == CONTROLLER and c.state == 'active' for c in self.controllers)
            if active and not self.activated_here:
                self.fail('Motion controller was already active; refusing to take over another session')
                return
            if not active:
                self.clutch.reset()
                if self.touch.buttons[0]:
                    return
                if self.switch_pending is None and self.switch.service_is_ready():
                    if not self.heartbeat('A'):
                        return
                    request = SwitchController.Request()
                    request.activate_controllers = [CONTROLLER]
                    request.strictness = SwitchController.Request.STRICT
                    request.timeout.sec = 2
                    self.switch_pending = self.switch.call_async(request)
                    self.activated_here = True
                    self.log('activation_requested')
                elif self.switch_pending is not None and self.switch_pending.done():
                    if not self.switch_pending.result().ok:
                        self.fail('Controller activation rejected')
                return
        m = self.touch
        flange = self.model.fk(self.q)
        target = self.clutch.update(m.axes[:3], m.buttons[:2], m.buttons[2], m.buttons[3], flange[:3, 3])
        if target is None or np.linalg.norm(target-flange[:3, 3]) < .0005:
            return
        if not self.stationary:
            self.fail('Robot not stationary before single step')
            return
        try:
            end, pose = self.model.plan(self.q, target-flange[:3, 3])
        except ValueError as error:
            self.fail(str(error))
            return
        self.log('planned', q_start=self.q.tolist(), q_end=end.tolist(),
                 start_flange=flange.tolist(), target_flange=pose.tolist(), execute=self.execute)
        if not self.execute:
            self.get_logger().info('Preview step validated; no goal was sent')
            self.done = True
            return
        if self.ready() is not None or not self.action.server_is_ready():
            self.fail('State/action unavailable after planning')
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.model.names
        for duration, q in ((0., self.q), (self.duration, end)):
            point = JointTrajectoryPoint(positions=q.tolist(), velocities=[0.]*6, accelerations=[0.]*6)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(duration * 1e9), 10**9)
            goal.trajectory.points.append(point)
        goal.path_tolerance = [JointTolerance(name=n, position=.001) for n in self.model.names]
        goal.goal_tolerance = [JointTolerance(name=n, position=.0001, velocity=.005) for n in self.model.names]
        goal.goal_time_tolerance.sec = 1
        self.lease.reserve()
        self.sent_at = time.monotonic()
        self.pending = self.action.send_goal_async(goal)
        self.pending.add_done_callback(self.accepted)
        self.log('goal_sent')

    def stop_and_wait(self):
        self.closing = True
        self.lease.stop()
        self.cancel()
        until = time.monotonic() + 3
        while self.lease.sent and not self.lease.terminal and time.monotonic() < until and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=.02)
            self.cancel()
        if self.lease.sent and not self.lease.terminal:
            self.log('STOP_UNCONFIRMED')
            self.get_logger().error('STOP UNCONFIRMED: use pendant Stop; inspect robot before restarting')
        if self.activated_here and self.switch.service_is_ready() and rclpy.ok():
            request = SwitchController.Request()
            request.deactivate_controllers = [CONTROLLER]
            request.strictness = SwitchController.Request.STRICT
            request.timeout.sec = 2
            future = self.switch.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3)
            confirmed = future.done() and future.exception() is None and future.result().ok
            self.log('deactivation_result', confirmed=confirmed)
            if not confirmed:
                self.get_logger().error('Controller deactivation unconfirmed; use pendant Stop')


def main(argv=None, node_type=BenchStep, timeout_s=120, commissioned_setup=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-one-step', action='store_true')
    parser.add_argument('--release-test', action='store_true', help='Slow the same bounded step to 5 s for release testing')
    parser.add_argument('--session', type=Path)
    args = parser.parse_args(argv)
    if os.environ.get('ROS_DOMAIN_ID') != '43' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Requires ROS_DOMAIN_ID=43 and ROS_LOCALHOST_ONLY=1')
    manifest_path = ROOT/'config/real_robot_commissioning.json'
    if not manifest_path.is_file():
        parser.error('Missing local commissioning record; see docs/REAL_ROBOT_COMMISSIONING.md')
    manifest = json.loads(manifest_path.read_text())
    session = None
    if args.execute_one_step and commissioned_setup:
        validate_commissioning(manifest)
        clearance = manifest.get('minimum_clearance_mm', 0)
        if not np.isfinite(clearance) or clearance < 660:
            parser.error('Continuous control requires commissioned clearance >=660 mm')
    elif args.execute_one_step:
        if args.session is None:
            parser.error('Execution requires --session with a recent measured-clearance/operator record')
        session = validate_session(args.session, manifest)
    telemetry = snapshot(manifest['robot_ipv4'])
    validate_payload(telemetry, manifest)
    if not np.allclose(telemetry['tcp_offset'], [0,0,.197,0,0,0], atol=.0001):
        raise RuntimeError('Active TCP changed; review geometry before proceeding')
    output = ROOT/'diagnostics/real-robot'/time.strftime('bench-step-%Y%m%d-%H%M%S.jsonl')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    interrupted = []
    signal.signal(signal.SIGINT, lambda *_: interrupted.append(True))
    signal.signal(signal.SIGTERM, lambda *_: interrupted.append(True))
    with output.open('x') as record:
        node = node_type(args.execute_one_step, record, manifest, telemetry, args.release_test)
        if args.execute_one_step:
            node.log('commissioned_setup' if commissioned_setup else 'operator_session',
                     confirmation=session, manifest=manifest)
        try:
            while rclpy.ok() and not node.done and not interrupted and time.monotonic()-node.started < timeout_s:
                rclpy.spin_once(node, timeout_sec=.02)
        finally:
            node.stop_and_wait()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    print(f'Bench record: {output}')


if __name__ == '__main__':
    main()
