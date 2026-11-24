#!/usr/bin/env python3

import sys
import time
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Home configuration matches the MuJoCo Menagerie "home" keyframe.
HOME = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

# MuJoCo model joint limits, in radians.
JOINT_LIMITS = [
    (-6.2832, 6.2832),
    (-6.2832, 6.2832),
    (-3.1415, 3.1415),
    (-6.2832, 6.2832),
    (-6.2832, 6.2832),
    (-6.2832, 6.2832),
]

# Each key press changes the desired target by this amount.
STEP_SIZE = 0.0174533  # 1 degree

# Maximum rate at which the published target can change.
# This limits target changes, not the robot's measured joint velocity.
TARGET_RATE = 0.15  # radians/second

PUBLISH_RATE = 20.0  # Hz


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__("ur5e_keyboard_teleop")

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/teleop/joint_commands",
            10,
        )

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.actual_positions = None
        self.desired_positions = None
        self.command_positions = None

        self.last_time = time.monotonic()

    def joint_state_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))

        if not all(name in positions for name in JOINT_NAMES):
            return

        self.actual_positions = [
            positions[name] for name in JOINT_NAMES
        ]

        # Initialize targets from the actual robot state.
        if self.desired_positions is None:
            self.desired_positions = self.actual_positions.copy()
            self.command_positions = self.actual_positions.copy()

    def change_target(self, joint_index, direction):
        self.desired_positions[joint_index] += direction * STEP_SIZE

        lower, upper = JOINT_LIMITS[joint_index]

        self.desired_positions[joint_index] = max(
            lower,
            min(upper, self.desired_positions[joint_index]),
        )

    def publish_command(self):
        now = time.monotonic()
        dt = min(now - self.last_time, 0.1)
        self.last_time = now

        max_change = TARGET_RATE * dt

        # Gradually move the commanded target towards the desired target.
        for i in range(6):
            error = self.desired_positions[i] - self.command_positions[i]

            change = max(-max_change, min(max_change, error))
            self.command_positions[i] += change

        msg = Float64MultiArray()
        msg.data = self.command_positions.copy()

        self.publisher.publish(msg)

    def hold_current_position(self):
        if self.actual_positions is not None:
            self.desired_positions = self.actual_positions.copy()
            print("\nHolding the latest measured joint positions.")


def print_controls():
    print("\n========== UR5e KEYBOARD TELEOPERATION ==========")
    print("Q / A : Shoulder pan       + / -")
    print("W / S : Shoulder lift      + / -")
    print("E / D : Elbow              + / -")
    print("R / F : Wrist 1            + / -")
    print("T / G : Wrist 2            + / -")
    print("Y / H : Wrist 3            + / -")
    print("SPACE : Hold current measured position")
    print("0     : Return to home configuration")
    print("+ / - : Increase / decrease step size")
    print("ESC   : Exit")
    print("=================================================\n")


def main():
    global STEP_SIZE

    if not sys.stdin.isatty():
        print("Error: Run this program directly in a terminal.")
        return

    original_terminal_settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = KeyboardTeleop()

    try:
        print("Waiting for joint states...")

        while rclpy.ok() and node.actual_positions is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.actual_positions is None:
            print("No joint states received. Exiting.")
            return

        print("Joint states received. Teleoperation ready.")
        print_controls()

        tty.setcbreak(sys.stdin.fileno())

        loop_period = 1.0 / PUBLISH_RATE
        last_print = time.monotonic()

        while rclpy.ok():
            loop_start = time.monotonic()

            rclpy.spin_once(node, timeout_sec=0.001)

            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)

                if key == "\x1b":
                    print("\nExiting keyboard teleoperation.")
                    break

                elif key == " ":
                    node.hold_current_position()

                elif key == "0":
                    node.desired_positions = HOME.copy()
                    print("\nHome position requested.")

                elif key == "+" or key == "=":
                    STEP_SIZE = min(STEP_SIZE * 2.0, 0.174533)
                    print(f"\nStep size: {STEP_SIZE:.4f} rad")

                elif key == "-":
                    STEP_SIZE = max(STEP_SIZE / 2.0, 0.004363)
                    print(f"\nStep size: {STEP_SIZE:.4f} rad")

                else:
                    key_map = {
                        "q": (0, 1), "a": (0, -1),
                        "w": (1, 1), "s": (1, -1),
                        "e": (2, 1), "d": (2, -1),
                        "r": (3, 1), "f": (3, -1),
                        "t": (4, 1), "g": (4, -1),
                        "y": (5, 1), "h": (5, -1),
                    }

                    if key.lower() in key_map:
                        index, direction = key_map[key.lower()]
                        node.change_target(index, direction)

            node.publish_command()

            if time.monotonic() - last_print > 2.0:
                positions_deg = [
                    round(p * 180.0 / 3.141592653589793, 1)
                    for p in node.actual_positions
                ]
                print(f"\rMeasured joints (degrees): {positions_deg}   ",
                      end="", flush=True)
                last_print = time.monotonic()

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, loop_period - elapsed))

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            original_terminal_settings,
        )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()