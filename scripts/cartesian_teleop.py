#!/usr/bin/env python3
"""Simulation-only UR5e bowel-grasper Cartesian keyboard teleoperation.

Requires ROS_DOMAIN_ID=42, MuJoCo ur5e_grasper.xml, and the ROS command router.
Default mode is read-only. --execute enables bounded simulated motion.
Tkinter: W/S world +/-X, A/D +/-Y, R/F +/-Z, Space hold, Escape exit.
"""
import argparse
import os
import threading
import time
from pathlib import Path
from xml.etree import ElementTree

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import tkinter as tk

MODEL_PATH = (Path.home() / 'ur5e_teleop/models/mujoco_menagerie/'
              'universal_robots_ur5e/ur5e_grasper.xml')
JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
HOME_TARGETS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
DT = 0.05
MAX_LINEAR_SPEED = 0.15        # m/s (150 mm/s for surgical responsiveness)
MAX_JOINT_SPEED = 1.50         # rad/s (~86 deg/s for agile wrist movement)
DAMPING = 0.01
KP_ORIENTATION = 4.0
MAX_AXIS_TRAVEL = 0.20          # meters from initial tip pose (200 mm)
MAX_TOTAL_TRAVEL = 0.30         # meters from initial tip pose (300 mm)
MAX_COMMAND_TRAVEL = np.array([0.90, 0.90, 0.90, 2.50, 2.50, 3.14])  # rad: full surgical wrist range
MAX_UNEXPECTED_TRACKING = 0.050  # rad beyond gravity holding offset
JOINT_FRESHNESS = 0.20         # seconds
ROUTER_FRESHNESS = 0.25
HEARTBEAT_FRESHNESS = 2.5
HEARTBEAT_TOPIC = '/heart_beat_plugin/mujoco_heartbeat'
TELEOP_TOPIC = '/teleop/joint_commands'
CONTROLLER_TOPIC = '/position_controller/commands'


class KeyboardState:
    """Key state safely shared between the Tk and ROS threads."""
    def __init__(self):
        self.lock = threading.Lock()
        self.pressed = set()
        self.has_focus = False
        self.closed = False

    def update_key(self, key, is_pressed):
        with self.lock:
            if is_pressed:
                self.pressed.add(key)
            else:
                self.pressed.discard(key)

    def clear(self):
        with self.lock:
            self.pressed.clear()
            self.has_focus = False

    def read(self):
        with self.lock:
            return set(self.pressed), self.has_focus, self.closed

    def set_focus(self, focused):
        with self.lock:
            self.has_focus = focused
            if not focused:
                self.pressed.clear()

    def close(self):
        with self.lock:
            self.closed = True
            self.pressed.clear()
            self.has_focus = False


class CartesianKeyboardTeleop(Node):
    def __init__(self, keys, execute, node_name='cartesian_keyboard_teleop'):
        super().__init__(node_name)
        self.keys = keys
        self.execute = execute
        self.finished = False
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.tip_id = self.model.site('instrument_tip').id
        self.qpos_idx = [self.model.joint(n).qposadr[0] for n in JOINT_NAMES]
        self.dof_idx = [self.model.joint(n).dofadr[0] for n in JOINT_NAMES]
        actuator_ids = [self.model.actuator(n.removesuffix('_joint')).id
                        for n in JOINT_NAMES]
        self.gains = self.model.actuator_gainprm[actuator_ids, 0]
        self.ctrl_ranges = self.model.actuator_ctrlrange[actuator_ids]
        self.force_ranges = self.model.actuator_forcerange[actuator_ids]
        if np.any(self.gains <= 0):
            raise ValueError('Expected positive position-actuator gains')
        self.q = None
        self.dq = None
        self.state_time = None
        self.router_output = None
        self.router_time = None
        self.heartbeat_time = None
        self.model_verified = False
        self.initial_position = None
        self.initial_rotation = None
        self.initial_hold = None
        self.last_log = 0.0
        self.start_wait = time.monotonic()
        self.publisher = None  # read-only mode never creates this
        self.first_publish_time = None

        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(String, HEARTBEAT_TOPIC, self.on_heartbeat, 10)
        self.create_subscription(
            String, '/robot_description', self.on_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Float64MultiArray, CONTROLLER_TOPIC,
                                 self.on_router, 10)
        self.create_timer(DT, self.update)
        self.get_logger().info('Cartesian keyboard teleop initialized: ' +
                               ('SIMULATION MOTION ENABLED' if execute
                                else 'READ-ONLY (no command publisher)'))

    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity))
        if not all(n in pos and n in vel for n in JOINT_NAMES):
            return
        q = np.array([pos[n] for n in JOINT_NAMES], dtype=float)
        dq = np.array([vel[n] for n in JOINT_NAMES], dtype=float)
        if np.all(np.isfinite(q)) and np.all(np.isfinite(dq)):
            self.q, self.dq = q, dq
            self.state_time = time.monotonic()

    def on_heartbeat(self, msg):
        if msg.data.startswith('MuJoCo ROS2 Control Heartbeat #'):
            self.heartbeat_time = time.monotonic()

    def on_description(self, msg):
        try:
            root = ElementTree.fromstring(msg.data)
            hardware = root.find("./ros2_control[@name='UR5eMujocoSystem']/hardware")
            self.model_verified = bool(
                hardware is not None
                and hardware.findtext('plugin') ==
                'mujoco_ros2_control/MujocoSystemInterface'
                and Path(hardware.findtext("param[@name='mujoco_model']", '')).resolve()
                == MODEL_PATH.resolve())
        except (ElementTree.ParseError, ValueError, OSError):
            self.model_verified = False

    def on_router(self, msg):
        value = np.asarray(msg.data, dtype=float)
        if value.shape == (6,) and np.all(np.isfinite(value)):
            self.router_output = value
            self.router_time = time.monotonic()

    def fail(self, reason):
        if not self.finished:
            self.get_logger().error(reason + '; stop publishing (router holds last target)')
            self.finished = True

    def graph_ok(self):
        expected = {
            HEARTBEAT_TOPIC: '/mujoco_ros2_control_node',
            '/robot_description': '/robot_state_publisher',
            '/joint_states': '/joint_state_broadcaster',
            CONTROLLER_TOPIC: '/ur5e_command_router',
        }
        for topic, expected_node in expected.items():
            pubs = self.get_publishers_info_by_topic(topic)
            names = [p.node_namespace.rstrip('/') + '/' + p.node_name for p in pubs]
            if names != [expected_node]:
                return False
        incoming = self.get_subscriptions_info_by_topic(TELEOP_TOPIC)
        if [(s.node_namespace, s.node_name) for s in incoming] != [
                ('/', 'ur5e_command_router')]:
            return False
        publishes = self.get_publishers_info_by_topic(TELEOP_TOPIC)
        if self.publisher is None:
            return len(publishes) == 0
        return [(p.node_namespace, p.node_name) for p in publishes] == [
            ('/', self.get_name())]

    def update(self):
        if self.finished:
            return
        keys, focused, closed = self.keys.read()
        if closed:
            self.finished = True
            return
        now = time.monotonic()
        ready = (self.model_verified and self.q is not None
                 and self.state_time is not None
                 and now - self.state_time < JOINT_FRESHNESS
                 and self.heartbeat_time is not None
                 and now - self.heartbeat_time < HEARTBEAT_FRESHNESS
                 and self.graph_ok())
        if not ready:
            if self.initial_position is not None or now - self.start_wait > 8.0:
                self.fail('ROS endpoints, robot model, joint states, or heartbeat invalid')
            return

        self.data.qpos[self.qpos_idx] = self.q
        mujoco.mj_forward(self.model, self.data)
        if self.data.ncon:
            self.fail('Modeled contact detected')
            return
        position = self.data.site_xpos[self.tip_id].copy()
        rotation = self.data.site_xmat[self.tip_id].reshape(3, 3).copy()
        gravity = self.data.qfrc_bias[self.dof_idx]
        hold = self.q + gravity / self.gains
        if (not np.all(np.isfinite(hold))
                or np.any(gravity < self.force_ranges[:, 0])
                or np.any(gravity > self.force_ranges[:, 1])):
            self.fail('Gravity holding target / force invalid')
            return

        if self.initial_position is None:
            if np.max(np.abs(self.dq)) >= 0.002:
                return  # only capture the reference while stationary
            if np.max(np.abs(hold - HOME_TARGETS)) > 0.04:
                self.fail('Initial holding targets differ from home; restart simulation at home')
                return
            self.initial_position = position.copy()
            self.initial_rotation = rotation.copy()
            self.initial_hold = hold.copy()
            self.get_logger().info('Reference tip XYZ (mm) ' +
                                   str(np.round(position * 1000, 3)))
            if self.execute:
                self.publisher = self.create_publisher(
                    Float64MultiArray, TELEOP_TOPIC, 10)
            return

        delta = position - self.initial_position
        if (np.any(np.abs(delta) > MAX_AXIS_TRAVEL)
                or np.linalg.norm(delta) > MAX_TOTAL_TRAVEL):
            self.fail('Tip left the bounded local workspace')
            return

        # A *normal* gravity-support offset is not actuator tracking error.
        if self.router_time is not None and now - self.router_time < ROUTER_FRESHNESS:
            error_excess = self.router_output - hold
            if np.max(np.abs(error_excess)) > MAX_UNEXPECTED_TRACKING:
                self.fail('Excessive actuator tracking mismatch after gravity compensation')
                return
        elif (self.first_publish_time is not None
              and now - self.first_publish_time > 0.5):
            # Before first command there is no router output; after publishing it must arrive.
            self.fail('Router output is stale or missing')
            return

        if not self.execute:
            if now - self.last_log >= 2.0:
                self.get_logger().info('READ ONLY: tip XYZ (mm) ' +
                                       str(np.round(position * 1000, 3)))
                self.last_log = now
            return

        # Keyboard focus loss and key release mean zero Cartesian velocity.
        v = np.zeros(3)
        if focused and 'space' not in keys:
            v = np.array([
                float('w' in keys) - float('s' in keys),
                float('a' in keys) - float('d' in keys),
                float('r' in keys) - float('f' in keys),
            ])
            norm = np.linalg.norm(v)
            if norm > 0:
                v *= MAX_LINEAR_SPEED / norm

        # Refuse to command velocity farther out of the permitted local box.
        if np.any(np.abs(delta + v * DT) > MAX_AXIS_TRAVEL):
            v[:] = 0
        orientation_error = 0.5 * sum(
            np.cross(rotation[:, i], self.initial_rotation[:, i])
            for i in range(3))
        desired_twist = np.concatenate([v, KP_ORIENTATION * orientation_error])
        jp = np.zeros((3, self.model.nv))
        jr = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jp, jr, self.tip_id)
        jac = np.vstack([jp, jr])[:, self.dof_idx]
        try:
            qdot = jac.T @ np.linalg.solve(
                jac @ jac.T + DAMPING ** 2 * np.eye(6), desired_twist)
        except np.linalg.LinAlgError:
            self.fail('IK linear solve failed')
            return
        speed = np.max(np.abs(qdot))
        if speed > MAX_JOINT_SPEED:
            qdot *= MAX_JOINT_SPEED / speed
        command = hold + qdot * DT
        if (not np.all(np.isfinite(command))
                or np.any(np.abs(command - self.initial_hold) > MAX_COMMAND_TRAVEL)
                or np.any(command < self.ctrl_ranges[:, 0])
                or np.any(command > self.ctrl_ranges[:, 1])):
            self.fail('Command exceeded joint/actuator bounds')
            return
        msg = Float64MultiArray()
        msg.data = command.tolist()
        self.publisher.publish(msg)
        if self.first_publish_time is None:
            self.first_publish_time = now
        if now - self.last_log >= 2.0:
            self.get_logger().info(
                'Tip displacement (mm) ' + str(np.round(delta * 1000, 3)) +
                ' | orientation error (deg) ' +
                f'{np.degrees(np.linalg.norm(orientation_error)):.5f}')
            self.last_log = now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='Enable simulation-only joint commands (default: read-only)')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        parser.error('This program only runs with ROS_DOMAIN_ID=42')

    rclpy.init()
    state = KeyboardState()
    node = CartesianKeyboardTeleop(state, args.execute)
    worker = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    worker.start()

    root = tk.Tk()
    root.title('UR5e MuJoCo Cartesian keyboard teleop')
    root.geometry('510x230')
    header = ('SIMULATION MOTION ENABLED' if args.execute else
              'READ-ONLY DIAGNOSTIC — no robot commands')
    tk.Label(root, text=header, font=('Arial', 13, 'bold')).pack(pady=14)
    tk.Label(root, text='Click here to focus. Hold W/S: ±X, A/D: ±Y, R/F: ±Z.').pack()
    tk.Label(root, text='Space: hold  |  Escape: quit  |  Focus lost: hold').pack(pady=7)
    tk.Label(root, text='Speed: 80 mm/s  |  Local limit: ±120 mm per axis').pack()
    status = tk.Label(root, text='Waiting for MuJoCo...', font=('Arial', 11))
    status.pack(pady=15)

    valid = {'w', 's', 'a', 'd', 'r', 'f', 'space'}

    def key_down(event):
        key = event.keysym.lower()
        if key == 'escape':
            quit_gui()
        elif key in valid:
            state.update_key(key, True)

    def key_up(event):
        key = event.keysym.lower()
        if key in valid:
            state.update_key(key, False)

    def quit_gui():
        state.close()
        root.quit()

    def update_status():
        if node.finished:
            status.configure(text='Controller stopped — inspect terminal logs')
            return
        keys, focused, _ = state.read()
        label = 'FOCUSED' if focused else 'NOT FOCUSED (holding)'
        status.configure(text=f'{label} — keys: {" ".join(sorted(keys)) or "none"}')
        root.after(100, update_status)

    root.bind('<KeyPress>', key_down)
    root.bind('<KeyRelease>', key_up)
    root.bind('<FocusIn>', lambda e: state.set_focus(True))
    root.bind('<FocusOut>', lambda e: state.set_focus(False))
    root.protocol('WM_DELETE_WINDOW', quit_gui)
    root.after(100, update_status)
    try:
        root.mainloop()
    finally:
        state.close()
        node.finished = True
        root.destroy()
        if rclpy.ok():
            rclpy.shutdown()
        worker.join(timeout=2)
        node.destroy_node()


if __name__ == '__main__':
    main()
