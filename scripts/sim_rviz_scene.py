#!/usr/bin/env python3
"""Read-only RViz scene from the same MuJoCo model and simulated joint states."""
import os
from pathlib import Path
import time

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'models/mujoco_menagerie/universal_robots_ur5e/ur5e_grasper.xml'


def scene_markers(model, data):
    """Use MuJoCo world poses; never infer the tool pose from the pendant TCP."""
    result = MarkerArray()
    environment = model.body('surgical_environment').id
    for gid in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        if model.geom_bodyid[gid] != environment and name not in ('bowel_grasper_visual', 'instrument_tip_collision'):
            continue
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.ns = 'simulation_scene'
        marker.id = gid
        marker.action = Marker.ADD
        marker.lifetime.nanosec = 500000000
        size = model.geom_size[gid]
        position = data.geom_xpos[gid]
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, data.geom_xmat[gid])
        kind = model.geom_type[gid]
        if name == 'bowel_grasper_visual':
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = (MODEL.parent / 'assets/bowel_grasper_v1.stl').as_uri()
            # The source mesh has no geom offset. MuJoCo internally recenters
            # mesh vertices; use the original body frame for the STL in RViz.
            body = model.body('bowel_grasper_body').id
            position, quat = data.xpos[body], data.xquat[body]
            scales = np.ones(3)
        elif kind == mujoco.mjtGeom.mjGEOM_BOX:
            marker.type, scales = Marker.CUBE, 2*size
        elif kind == mujoco.mjtGeom.mjGEOM_ELLIPSOID:
            marker.type, scales = Marker.SPHERE, 2*size
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            marker.type, scales = Marker.SPHERE, np.repeat(2*size[0], 3)
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            marker.type, scales = Marker.CYLINDER, np.array([2*size[0], 2*size[0], 2*size[1]])
        else:
            continue
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, position)
        marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z = map(float, quat)
        marker.scale.x, marker.scale.y, marker.scale.z = map(float, scales)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = map(float, model.geom_rgba[gid])
        result.markers.append(marker)
    return result


class Scene(Node):
    def __init__(self):
        super().__init__('sim_rviz_scene')
        self.model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.data = mujoco.MjData(self.model)
        self.names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)]
        self.received = None
        self.create_subscription(JointState, '/joint_states', self.joints, 1)
        self.publisher = self.create_publisher(MarkerArray, '/simulation/scene', 1)
        self.create_timer(.05, self.publish)

    def joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(name in values and np.isfinite(values[name]) for name in self.names):
            return
        for i, name in enumerate(self.names):
            self.data.qpos[self.model.jnt_qposadr[i]] = values[name]
        mujoco.mj_forward(self.model, self.data)
        self.received = time.monotonic()

    def publish(self):
        if self.received is not None and time.monotonic()-self.received < .2:
            self.publisher.publish(scene_markers(self.model, self.data))


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise RuntimeError('RViz simulation scene requires ROS_DOMAIN_ID=42')
    rclpy.init()
    node = Scene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
