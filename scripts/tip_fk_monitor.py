#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import mujoco
import numpy as np

from pathlib import Path


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


class TipFKMonitor(Node):

    def __init__(self):
        super().__init__("tip_fk_monitor")

        self.model = mujoco.MjModel.from_xml_path(
            str(MODEL_PATH)
        )

        self.data = mujoco.MjData(self.model)

        self.tip_id = self.model.site("instrument_tip").id

        self.joint_qpos_addresses = [
            self.model.joint(name).qposadr[0]
            for name in JOINT_NAMES
        ]

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.last_print_time = 0.0
        # Reference pose captured from the first valid joint-state message.
        self.reference_position = None
        self.reference_rotation = None

        self.get_logger().info(
            "Tip FK monitor started. Waiting for joint states..."
        )

    def joint_state_callback(self, msg):

        # Match measurements by name, not array order.
        positions = dict(zip(msg.name, msg.position))

        if not all(name in positions for name in JOINT_NAMES):
            return

        q = np.array([
            positions[name] for name in JOINT_NAMES
        ])

        if not np.all(np.isfinite(q)):
            return

        # Update the kinematic model using measured joints.
        for address, value in zip(
            self.joint_qpos_addresses, q
        ):
            self.data.qpos[address] = value

        mujoco.mj_forward(self.model, self.data)

        tip_position = self.data.site_xpos[
            self.tip_id
        ].copy()

        # Orientation of the instrument-tip site in world coordinates.
        tip_rotation = self.data.site_xmat[
            self.tip_id
        ].reshape(3, 3).copy()

        # Capture the reference pose once.
        if self.reference_rotation is None:
            self.reference_position = tip_position.copy()
            self.reference_rotation = tip_rotation.copy()

        # Relative rotation from the initial orientation to the current one.
        relative_rotation = self.reference_rotation.T @ tip_rotation

        # Convert relative rotation to a single angular deviation.
        cos_angle = (np.trace(relative_rotation) - 1.0) / 2.0
        cos_angle = np.clip(cos_angle, -1.0, 1.0)

        orientation_error_deg = np.degrees(np.arccos(cos_angle))

        # Cartesian displacement relative to the initial pose.
        position_change_mm = (
            tip_position - self.reference_position
        ) * 1000.0

        # Quaternion convention: [w, x, y, z].
        quaternion = np.zeros(4)
        mujoco.mju_mat2Quat(
            quaternion,
            tip_rotation.flatten()
        )

        # Print approximately once per second.
        current_time = self.get_clock().now().nanoseconds / 1e9

        if current_time - self.last_print_time >= 1.0:

            self.get_logger().info(
                f"Tip XYZ (mm): {np.round(tip_position * 1000, 3)}"
            )

            self.get_logger().info(
                f"Tip displacement (mm): {np.round(position_change_mm, 3)}"
            )

            self.get_logger().info(
                f"Tip quaternion [w,x,y,z]: {np.round(quaternion, 6)}"
            )

            self.get_logger().info(
                f"Orientation deviation: {orientation_error_deg:.5f} deg"
            )

            self.last_print_time = current_time


def main():

    rclpy.init()

    node = TipFKMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()