#!/usr/bin/env python3
"""Read-only Touch X publisher; bounded force output requires --enable-force. Simulation domain 42; explicit real read-only mode on 43."""
import argparse
import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
from teleop_geometry import rotation_to_xyzw
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from force_feedback import validate_tuning

ROOT = Path(__file__).resolve().parents[1]


def validate_backend(domain, real_read_only=False, enable_force=False):
    expected = '43' if real_read_only else '42'
    if domain != expected or (real_read_only and enable_force):
        raise RuntimeError(f'Touch backend requires domain {expected}; real backend forbids motor output')
    return expected


def decode_sample(raw, now_ns):
    seq, stamp = int(raw['seq']), int(raw['monotonic_ns'])
    transport = int(raw['transport_ns'])
    xyz = np.asarray(raw['xyz_mm'], dtype=float) / 1000.0
    transform = np.asarray(raw['transform'], dtype=float).reshape(4, 4, order='F')
    if (seq <= 0 or not 0 <= (now_ns - stamp) / 1e9 < 0.1
            or not 0 <= (now_ns-transport)/1e9 < 0.1
            or xyz.shape != (3,) or not np.all(np.isfinite(xyz))
            or np.any(np.abs(xyz) > 1) or not np.all(np.isfinite(transform))):
        raise ValueError('invalid or stale Touch sample')
    rot = transform[:3, :3].copy()
    if not np.allclose(rot.T @ rot, np.eye(3), atol=0.02) or abs(np.linalg.det(rot)-1) > 0.02:
        raise ValueError('invalid Touch orientation')
    quaternion = rotation_to_xyzw(rot)
    force = np.asarray(raw.get('force_n', [0.,0.,0.]), dtype=float)
    if force.shape != (3,) or not np.all(np.isfinite(force)) or np.linalg.norm(force)>1.000001:
        raise ValueError('invalid servo force telemetry')
    buttons = int(raw['buttons'])
    if not 0 <= buttons <= 3:
        raise ValueError('unexpected button bits')
    return dict(seq=seq, stamp=stamp, xyz=xyz.tolist(), force=force.tolist(),
                quat=quaternion.tolist(), buttons=buttons,
                calibrated=int(raw['calibration']) == 20480,
                epoch=int(raw['calibration_epoch']))


class TouchPublisher(Node):
    def __init__(self, enable_force=False, force_limit=0.5, real_read_only=False):
        validate_tuning(force_limit=force_limit)
        self.force_limit = force_limit
        domain = validate_backend(os.environ.get('ROS_DOMAIN_ID'), real_read_only, enable_force)
        super().__init__('touch_x_publisher')
        qos = QoSProfile(depth=1)
        self.pose_pub = self.create_publisher(PoseStamped, '/touch_x/pose', qos)
        self.joy_pub = self.create_publisher(Joy, '/touch_x/state', qos)
        self.valid_pub = self.create_publisher(Bool, '/touch_x/valid', qos)
        self.enable_force = enable_force
        self.force_sequence = 0
        self.force_stamp = -1
        self.applied_pub = self.create_publisher(WrenchStamped, '/touch_x/applied_force', 1)
        if enable_force:
            self.force_sub = self.create_subscription(
                WrenchStamped, '/touch_x/force_feedback', self.on_force, 1)
        self.lock = threading.Lock()
        self.latest = None
        self.last_seq = 0
        self.failed = False
        self.start_time = time.monotonic()
        self.last_received = None
        self.epoch = 0
        env = dict(os.environ, TOUCH_STREAM_JSON='1', ROS_DOMAIN_ID=domain,
                   TOUCH_ENABLE_FORCE='1' if enable_force else '0',
                   TOUCH_FORCE_LIMIT_N=str(force_limit))
        self.child = subprocess.Popen([str(ROOT/'scripts/run_touch_x.sh'), '3600'],
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT, text=True, bufsize=1,
                                      env=env, start_new_session=True)
        os.set_blocking(self.child.stdin.fileno(), False)
        self.reader = threading.Thread(target=self.read_pipe, daemon=True)
        self.reader.start()
        self.create_timer(0.01, self.publish_latest)
        self.get_logger().info(f'Touch X tracking; force permission={enable_force}; domain {domain}')

    def send_force(self, force, source_ns):
        self.force_sequence += 1
        payload = f'{self.force_sequence} {source_ns} {force[0]:.9g} {force[1]:.9g} {force[2]:.9g}\n'
        try:
            # A short write (< PIPE_BUF) is atomic or EAGAIN; never stall ROS.
            os.write(self.child.stdin.fileno(), payload.encode())
        except (BrokenPipeError, BlockingIOError, OSError, ValueError):
            pass  # The servo independently expires the original timestamp.

    def on_force(self, msg):
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        age_ns = self.get_clock().now().nanoseconds-stamp
        f, t = msg.wrench.force, msg.wrench.torque
        force = np.array([f.x, f.y, f.z])
        publishers = self.get_publishers_info_by_topic('/touch_x/force_feedback')
        sender_ok = len(publishers)==1 and publishers[0].node_name=='touch_x_sim_teleop' and publishers[0].node_namespace=='/'
        if (self.failed or not sender_ok or msg.header.frame_id!='touch_x_base'
                or not 0 <= age_ns < 100_000_000 or stamp<=self.force_stamp
                or not np.all(np.isfinite(force)) or np.linalg.norm(force)>self.force_limit+1e-6
                or (t.x,t.y,t.z)!=(0.,0.,0.)):
            self.send_force([0.,0.,0.], 0)
            return
        self.force_stamp = stamp
        self.send_force(force, time.monotonic_ns()-age_ns)

    def read_pipe(self):
        for line in self.child.stdout:
            if line.startswith('{'):
                try:
                    value = json.loads(line)
                    with self.lock:
                        self.latest = value
                except (ValueError, TypeError):
                    self.get_logger().error('Malformed device stream')
                    self.failed = True
                    return
            else:
                self.get_logger().info('device: ' + line.strip())

    def invalidate(self, reason):
        if not self.failed:
            self.get_logger().error(reason)
        self.failed = True
        if self.enable_force:
            self.send_force([0.,0.,0.], 0)
        self.valid_pub.publish(Bool(data=False))
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'touch_x_base'
        msg.axes = [0., 0., 0., 0., 0., 0., 1.]
        msg.buttons = [0, 0, 0, self.epoch]
        self.joy_pub.publish(msg)

    def publish_latest(self):
        if self.failed or self.child.poll() is not None:
            self.invalidate('Touch reader exited or failed')
            return
        with self.lock:
            raw, self.latest = self.latest, None
        now = time.monotonic()
        if raw is None:
            if (self.last_received is not None and now-self.last_received > 0.15
                    or self.last_received is None and now-self.start_time > 15):
                self.invalidate('Touch samples stopped')
            return
        try:
            state = decode_sample(raw, time.monotonic_ns())
            if state['seq'] <= self.last_seq:
                raise ValueError('Touch sequence did not advance')
        except (ValueError, KeyError, TypeError, OverflowError) as exc:
            self.invalidate(str(exc))
            return
        self.last_seq = state['seq']
        self.last_received = now
        self.epoch = state['epoch']
        # Stamp the sample, not the later pipe-delivery time.
        age_ns = time.monotonic_ns() - state['stamp']
        stamp_ns = self.get_clock().now().nanoseconds - age_ns
        pose = PoseStamped()
        pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(stamp_ns, 10**9)
        pose.header.frame_id = 'touch_x_base'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = state['xyz']
        q = pose.pose.orientation
        q.x, q.y, q.z, q.w = state['quat']
        joy = Joy()
        joy.header = pose.header
        joy.axes = state['xyz'] + state['quat']
        joy.buttons = [state['buttons'] & 1, (state['buttons'] >> 1) & 1,
                       int(state['calibrated']), state['epoch']]
        applied = WrenchStamped()
        applied.header = pose.header
        applied.wrench.force.x, applied.wrench.force.y, applied.wrench.force.z = state['force']
        self.applied_pub.publish(applied)
        self.pose_pub.publish(pose)
        self.joy_pub.publish(joy)
        self.valid_pub.publish(Bool(data=state['calibrated']))

    def close(self):
        if rclpy.ok():
            self.invalidate('Publisher stopped')
        if getattr(self, 'child', None) and self.child.stdin and not self.child.stdin.closed:
            try:
                self.send_force([0.,0.,0.], 0)
                self.child.stdin.close()
            except (BrokenPipeError, OSError):
                pass
        if self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGTERM)
            try:
                self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid, signal.SIGKILL)
                self.child.wait(timeout=2)
        self.reader.join(timeout=1)
        self.child.stdout.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--force-limit', type=float, default=0.5)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--enable-force', action='store_true')
    group.add_argument('--read-only', action='store_true')
    group.add_argument('--real-read-only', action='store_true', help='Tracking only on domain 43; motor output prohibited')
    args = parser.parse_args()
    validate_backend(os.environ.get('ROS_DOMAIN_ID'), args.real_read_only, args.enable_force)
    stopping = False
    def stop_signal(_signum, _frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TouchPublisher(args.enable_force, args.force_limit, args.real_read_only)
    failed = False
    try:
        while rclpy.ok() and not node.failed and not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
        failed = node.failed
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
