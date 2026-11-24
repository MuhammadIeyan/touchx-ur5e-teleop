#!/usr/bin/env python3

import argparse
import os
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


MODEL_PATH = (
    Path.home()
    / "ur5e_teleop/models/mujoco_menagerie/"
    / "universal_robots_ur5e/ur5e_grasper.xml"
)

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

DT = 0.05
DAMPING = 0.01

MAX_CARTESIAN_SPEED = 0.01  # m/s
MAX_JOINT_SPEED = 0.10      # rad/s

KP_POSITION = 2.0
KP_ORIENTATION = 2.0

MAX_TIP_TRAVEL = 0.020  # m
MAX_ERROR_GROWTH = 0.002  # m above the best error observed
MAX_COMMAND_TRAVEL = 0.040  # rad from the initial holding reference
MAX_TRACKING_ERROR = 0.020  # rad from router output to measured position
HEARTBEAT_TOPIC = "/heart_beat_plugin/mujoco_heartbeat"
TELEOP_TOPIC = "/teleop/joint_commands"
CONTROLLER_TOPIC = "/position_controller/commands"

EXPECTED_PUBLISHERS = {
    HEARTBEAT_TOPIC: "/mujoco_ros2_control_node",
    "/robot_description": "/robot_state_publisher",
    "/joint_states": "/joint_state_broadcaster",
    CONTROLLER_TOPIC: "/ur5e_command_router",
}


class CartesianIKTest(Node):

    def __init__(self, execute, distance_mm, axis):
        super().__init__("cartesian_ik_test")

        self.model = mujoco.MjModel.from_xml_path(
            str(MODEL_PATH)
        )
        self.data = mujoco.MjData(self.model)

        self.tip_id = self.model.site("instrument_tip").id

        self.joints = [
            self.model.joint(name)
            for name in JOINT_NAMES
        ]
        self.addresses = [
            joint.qposadr[0]
            for joint in self.joints
        ]

        self.dof_addresses = [joint.dofadr[0] for joint in self.joints]
        actuator_ids = [
            self.model.actuator(name.removesuffix("_joint")).id
            for name in JOINT_NAMES
        ]
        # The grasper model has one unit-gear affine position servo per joint.
        self.position_gains = self.model.actuator_gainprm[actuator_ids, 0]
        self.ctrl_ranges = self.model.actuator_ctrlrange[actuator_ids]
        self.force_ranges = self.model.actuator_forcerange[actuator_ids]
        self.execute = execute

        axis_vectors = {
            "x": np.array([1.0, 0.0, 0.0]),
            "y": np.array([0.0, 1.0, 0.0]),
            "z": np.array([0.0, 0.0, 1.0]),
        }

        self.movement = axis_vectors[axis] * (distance_mm / 1000.0)
        self.q = None
        self.stationary = False
        self.heartbeat_time = None
        self.model_verified = False
        self.router_output = None
        self.router_output_time = None
        self.start_position = None
        self.start_control = None
        self.best_error = None
        self.wait_started = time.monotonic()
        self.success = False
        self.last_state_time = None
        self.target_position = None
        self.target_rotation = None
        self.start_time = None
        self.last_log_time = 0.0
        self.finished = False

        self.create_subscription(JointState, "/joint_states", self.joint_callback, 10)
        self.create_subscription(String, HEARTBEAT_TOPIC, self.heartbeat_callback, 10)
        self.create_subscription(
            String, "/robot_description", self.description_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            Float64MultiArray, CONTROLLER_TOPIC, self.router_callback, 10,
        )
        self.publisher = None
        self.create_timer(DT, self.control_loop)
        mode = "bounded motion test" if execute else "read-only diagnostic"
        self.get_logger().info(f"Waiting for MuJoCo on domain 42 ({mode}).")

    def heartbeat_callback(self, msg):
        if msg.data.startswith("MuJoCo ROS2 Control Heartbeat #"):
            self.heartbeat_time = time.monotonic()

    def description_callback(self, msg):
        self.model_verified = False
        try:
            hardware = ElementTree.fromstring(msg.data).find(
                "./ros2_control[@name='UR5eMujocoSystem']/hardware"
            )
        except ElementTree.ParseError:
            return
        if hardware is None:
            return
        model = hardware.findtext("param[@name='mujoco_model']", "")
        self.model_verified = (
            hardware.findtext("plugin") == "mujoco_ros2_control/MujocoSystemInterface"
            and Path(model).resolve() == MODEL_PATH.resolve()
        )

    def router_callback(self, msg):
        output = np.asarray(msg.data)
        if output.shape == (6,) and np.all(np.isfinite(output)):
            self.router_output = output
            self.router_output_time = time.monotonic()

    def graph_matches(self):
        for topic, expected in EXPECTED_PUBLISHERS.items():
            endpoints = self.get_publishers_info_by_topic(topic)
            if [ep.node_namespace.rstrip("/") + "/" + ep.node_name for ep in endpoints] != [expected]:
                return False
        subscribers = self.get_subscriptions_info_by_topic(TELEOP_TOPIC)
        if [(ep.node_namespace, ep.node_name) for ep in subscribers] != [("/", "ur5e_command_router")]:
            return False
        subscribers = self.get_subscriptions_info_by_topic(CONTROLLER_TOPIC)
        downstream = sorted((ep.node_namespace, ep.node_name) for ep in subscribers)
        if downstream != [("/", "cartesian_ik_test"), ("/", "position_controller")]:
            return False
        publishers = self.get_publishers_info_by_topic(TELEOP_TOPIC)
        if self.publisher is None:
            return not publishers
        return [(ep.node_namespace, ep.node_name) for ep in publishers] == [("/", self.get_name())]

    def joint_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))

        if not all(name in positions for name in JOINT_NAMES):
            return

        q = np.array(
            [positions[name] for name in JOINT_NAMES],
            dtype=float
        )

        if not np.all(np.isfinite(q)):
            return

        velocities = dict(zip(msg.name, msg.velocity))
        self.stationary = all(abs(velocities.get(name, np.inf)) < 0.002 for name in JOINT_NAMES)
        self.q = q
        self.last_state_time = time.monotonic()

    def update_kinematics(self):
        for address, value in zip(self.addresses, self.q):
            self.data.qpos[address] = value

        mujoco.mj_forward(self.model, self.data)

        position = self.data.site_xpos[self.tip_id].copy()
        rotation = self.data.site_xmat[
            self.tip_id
        ].reshape(3, 3).copy()

        return position, rotation

    def stop(self, message, success=False):
        self.finished = True
        self.success = success
        log = self.get_logger().info if success else self.get_logger().error
        log(message)

    def control_loop(self):
        if self.finished:
            return
        
        now = time.monotonic()
        ready = (
            self.model_verified
            and self.heartbeat_time is not None
            and now - self.heartbeat_time < 2.5
            and self.last_state_time is not None
            and now - self.last_state_time < 0.2
            and self.graph_matches()
        )
        if not ready:
            if (
                self.start_time is not None and now - self.start_time > 0.3
                or self.start_time is None and now - self.wait_started > 8.0
            ):
                self.stop("Simulator model, ROS endpoints, heartbeat, or joint states failed verification.")
            return

        position, rotation = self.update_kinematics()
        gravity = self.data.qfrc_bias[self.dof_addresses]
        holding_control = self.q + gravity / self.position_gains
        if not np.all(np.isfinite(holding_control)) or self.data.ncon:
            self.stop("Invalid holding target or modeled contact detected.")
            return

        if self.start_time is None:
            if not self.execute:
                if now - self.wait_started < 3.0:
                    return
                self.log_tracking(now, holding_control)
                self.stop("Read-only diagnostic complete.", success=True)
                return
            if not self.stationary:
                self.stop("Wait for the simulator joints to settle before the motion test.")
                return
            self.start_time = now
            self.start_position = position
            self.start_control = holding_control
            self.target_position = position + self.movement
            self.target_rotation = rotation
            self.best_error = np.linalg.norm(self.movement)
            self.publisher = self.create_publisher(Float64MultiArray, TELEOP_TOPIC, 10)
            self.log_tracking(now, holding_control)
            return

        error = self.target_position - position
        distance = np.linalg.norm(error)
        travel = np.linalg.norm(position - self.start_position)
        if travel > MAX_TIP_TRAVEL or distance > self.best_error + MAX_ERROR_GROWTH:
            self.stop(f"Tip bound exceeded. Travel {travel * 1000:.2f} mm, error {distance * 1000:.2f} mm.")
            return
        self.best_error = min(self.best_error, distance)

        if self.router_output_time is not None and now - self.router_output_time < 0.2:
            if np.max(np.abs(self.router_output - self.q)) > MAX_TRACKING_ERROR:
                self.stop("Router output differs from measured joints by more than 0.020 rad.")
                return
        elif now - self.start_time > 0.5:
            self.stop("Router output is stale or missing.")
            return

        if distance < 0.0005:
            self.stop(f"Target reached. Position error {distance * 1000:.3f} mm.", success=True)
            return
        if now - self.start_time > 15.0:
            self.stop(f"Test timed out with {distance * 1000:.3f} mm of position error.")
            return

        q_command = self.calculate_command(error, rotation, holding_control)
        if (
            not np.all(np.isfinite(q_command))
            or np.any(np.abs(q_command - self.start_control) > MAX_COMMAND_TRAVEL)
            or np.any(q_command < self.ctrl_ranges[:, 0])
            or np.any(q_command > self.ctrl_ranges[:, 1])
            or np.any(gravity < self.force_ranges[:, 0])
            or np.any(gravity > self.force_ranges[:, 1])
        ):
            self.stop("IK command exceeds the joint displacement or actuator limits.")
            return

        msg = Float64MultiArray()
        msg.data = q_command.tolist()
        self.publisher.publish(msg)

        if now - self.last_log_time >= 1.0:
            self.get_logger().info(f"Tip error {distance * 1000:.3f} mm, travel {travel * 1000:.3f} mm.")
            self.log_tracking(now, holding_control, q_command)
            self.last_log_time = now

    def log_tracking(self, now, holding_control, request=None):
        self.get_logger().info(f"Measured joints (rad) {np.round(self.q, 5)}")
        self.get_logger().info(f"Estimated holding targets (rad) {np.round(holding_control, 5)}")
        if self.router_output_time is not None and now - self.router_output_time < 0.2:
            self.get_logger().info(f"Router output (rad) {np.round(self.router_output, 5)}")
            self.get_logger().info(f"Router minus measured (rad) {np.round(self.router_output - self.q, 5)}")
        else:
            self.get_logger().info("No recent router output. The router is silent before its first command.")
        if request is not None:
            self.get_logger().info(f"IK request (rad) {np.round(request, 5)}")

    def calculate_command(self, error, rotation, holding_control):
        # Desired translational velocity.
        linear_velocity = KP_POSITION * error

        speed = np.linalg.norm(linear_velocity)
        if speed > MAX_CARTESIAN_SPEED:
            linear_velocity *= MAX_CARTESIAN_SPEED / speed

        # Orientation-holding error in world coordinates.
        orientation_error = 0.5 * sum(
            np.cross(
                rotation[:, i],
                self.target_rotation[:, i]
            )
            for i in range(3)
        )

        angular_velocity = (
            KP_ORIENTATION * orientation_error
        )

        # Calculate current Jacobian.
        Jp = np.zeros((3, self.model.nv))
        Jr = np.zeros((3, self.model.nv))

        mujoco.mj_jacSite(
            self.model,
            self.data,
            Jp,
            Jr,
            self.tip_id
        )

        J = np.vstack((Jp, Jr))[:, self.dof_addresses]

        desired_twist = np.concatenate([
            linear_velocity,
            angular_velocity
        ])

        # Damped least-squares differential IK.
        q_dot = J.T @ np.linalg.solve(
            J @ J.T + DAMPING**2 * np.eye(6),
            desired_twist
        )

        max_speed = np.max(np.abs(q_dot))
        if max_speed > MAX_JOINT_SPEED:
            q_dot *= MAX_JOINT_SPEED / max_speed

        # A target equal to the measured angle produces no static servo
        # torque. Keep gravity support in the target before adding the IK step.
        return holding_control + q_dot * DT


def main():
    parser = argparse.ArgumentParser(description="Diagnose the UR5e grasper in MuJoCo on domain 42.")
    parser.add_argument("--execute", action="store_true", help="Allow a bounded simulated motion test.")
    parser.add_argument(
        "--axis",
        choices=["x", "y", "z"],
        default="x",
        help="Cartesian movement axis in the MuJoCo world frame.",
    )
    parser.add_argument("--distance-mm", type=float, default=2.0, help="Positive X displacement, 0.5–10 mm.")
    args = parser.parse_args()
    if not 0.5 <= args.distance_mm <= 10:
        parser.error("--distance-mm must be between 0.5 and 10.")
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        parser.error("This diagnostic requires ROS_DOMAIN_ID=42.")

    rclpy.init()
    node = None
    try:
        node = CartesianIKTest(args.execute, args.distance_mm, args.axis)
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        return 0 if node.success else 1
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())