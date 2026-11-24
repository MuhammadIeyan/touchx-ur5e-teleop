#!/usr/bin/env python3

import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_LIMITS = [
    (-6.2832, 6.2832),
    (-6.2832, 6.2832),
    (-3.1415, 3.1415),
    (-6.2832, 6.2832),
    (-6.2832, 6.2832),
    (-6.2832, 6.2832),
]

SIM_HOME_TARGETS = [
    -1.5708,
    -1.5708,
     1.5708,
    -1.5708,
    -1.5708,
     0.0,
]

# Router settings
PUBLISH_RATE = 100.0       # Hz
MAX_TARGET_VELOCITY = 2.5  # rad/s
COMMAND_TIMEOUT = 0.5      # seconds


class UR5eCommandRouter(Node):

    def __init__(self):
        if os.environ.get('ROS_DOMAIN_ID') != '42':
            raise RuntimeError('Simulation router requires ROS_DOMAIN_ID=42')
        super().__init__("ur5e_command_router")
        self.require_deadman = self.declare_parameter('require_deadman', False).value
        self.deadman = False
        self.deadman_time = None
        self.joint_time = None
        self.create_subscription(Bool, '/teleop/deadman', self.on_deadman, 1)

        self.teleop_sub = self.create_subscription(
            Float64MultiArray,
            "/teleop/joint_commands",
            self.teleop_callback,
            10,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.robot_pub = self.create_publisher(
            Float64MultiArray,
            "/position_controller/commands",
            10,
        )

        self.actual_positions = None
        self.target_positions = None
        self.output_positions = None

        self.last_command_time = None
        self.command_received = False
        self.last_update_time = time.monotonic()

        self.timeout_active = False
        self.simulation_domain = os.environ.get("ROS_DOMAIN_ID") == "42"
        self.rejected_commands = 0

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE,
            self.update,
        )

        self.get_logger().info("UR5e command router started.")
        self.get_logger().info(
            f"Command timeout: {COMMAND_TIMEOUT:.2f} seconds"
        )

    def joint_state_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))

        if not all(name in positions for name in JOINT_NAMES):
            return

        self.actual_positions = [
            positions[name] for name in JOINT_NAMES
        ]
        if not all(math.isfinite(v) for v in self.actual_positions):
            self.actual_positions = None
            return
        self.joint_time = time.monotonic()

        if self.output_positions is None:
            if self.simulation_domain:
                self.output_positions = SIM_HOME_TARGETS.copy()
                self.target_positions = SIM_HOME_TARGETS.copy()
            else:
                self.output_positions = self.actual_positions.copy()
                self.target_positions = self.actual_positions.copy()

    def on_deadman(self, msg):
        self.deadman, self.deadman_time = bool(msg.data), time.monotonic()
        if self.require_deadman and not self.deadman and self.output_positions is not None:
            self.target_positions = self.output_positions.copy()

    def teleop_callback(self, msg):
        if self.require_deadman and (not self.deadman or self.deadman_time is None
                or time.monotonic()-self.deadman_time > 0.15):
            return

        if len(msg.data) != 6:
            self.rejected_commands += 1
            self.get_logger().warning(
                "Rejected command: expected exactly 6 joint positions."
            )
            return

        if not all(math.isfinite(value) for value in msg.data):
            self.rejected_commands += 1
            self.get_logger().warning(
                "Rejected command: non-finite joint position."
            )
            return

        if self.actual_positions is None:
            return

        new_target = []

        for i, value in enumerate(msg.data):
            lower, upper = JOINT_LIMITS[i]

            # Reject commands outside the configured limits.
            if value < lower or value > upper:
                self.rejected_commands += 1
                self.get_logger().warning(
                    f"Rejected command: {JOINT_NAMES[i]} "
                    f"outside joint limits."
                )
                return

            new_target.append(value)

        self.target_positions = new_target
        self.last_command_time = time.monotonic()
        self.command_received = True

        if self.timeout_active:
            self.timeout_active = False
            self.get_logger().info("Teleoperation command stream restored.")

    def update(self):

        if self.actual_positions is None:
            return

        now = time.monotonic()
        if self.joint_time is None or now-self.joint_time > 0.2:
            return
        if self.require_deadman and (not self.deadman or self.deadman_time is None
                or now-self.deadman_time > 0.15):
            self.target_positions = self.output_positions.copy()
        dt = min(now - self.last_update_time, 0.05)
        self.last_update_time = now

        # Watchdog: capture a hold target if commands stop.
        command_expired = (
            self.last_command_time is None
            or now - self.last_command_time > COMMAND_TIMEOUT
        )

        # Do not override MuJoCo's actuator targets before
        # receiving the first valid teleoperation command.
        if not self.command_received:
            return

        # Capture the hold position only once on timeout.
        if command_expired and not self.timeout_active:
            if self.simulation_domain:
                # The simulated position servos need the existing target
                # offset from the measured joints to keep supporting gravity.
                self.target_positions = self.output_positions.copy()
                hold_source = "the last published targets"
            else:
                self.target_positions = self.actual_positions.copy()
                hold_source = "measured positions"
            self.timeout_active = True
            self.get_logger().warning(f"Command timeout. Holding {hold_source}.")

        max_change = MAX_TARGET_VELOCITY * dt

        # Apply a velocity limit to the outgoing position target.
        for i in range(6):

            error = (
                self.target_positions[i] - self.output_positions[i]
            )

            change = max(
                -max_change,
                min(max_change, error),
            )

            self.output_positions[i] += change

        command = Float64MultiArray()
        command.data = self.output_positions.copy()

        self.robot_pub.publish(command)


def main(args=None):
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise SystemExit('Simulation router requires ROS_DOMAIN_ID=42')
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = UR5eCommandRouter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
