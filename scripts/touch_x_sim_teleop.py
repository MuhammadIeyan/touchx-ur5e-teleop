#!/usr/bin/env python3
"""Touch X 6-DOF simulation teleoperation with optional experimental RCM and contact feedback.
Domain 42 only; physical robot domain 43 prohibited.
"""
import argparse
import os
import time
import numpy as np
import mujoco
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, Float64MultiArray
from rcl_interfaces.srv import GetParameters
from cartesian_teleop import (
    CartesianKeyboardTeleop, KeyboardState, DT, JOINT_FRESHNESS,
    HEARTBEAT_FRESHNESS, MAX_AXIS_TRAVEL, MAX_TOTAL_TRAVEL, HOME_TARGETS,
    MAX_COMMAND_TRAVEL, DAMPING,
    KP_ORIENTATION, TELEOP_TOPIC, CONTROLLER_TOPIC,
)
from touch_clutch import Clutch
from force_feedback import reflected_force, validate_tuning

# Commissioning limits for this Touch interface, independent of keyboard settings.
MAX_JOINT_SPEED = 0.5
MAX_LINEAR_SPEED = 0.06


class TouchTeleop(CartesianKeyboardTeleop):
    def __init__(self, execute, scale=0.2, rot_scale=1.0, rcm=False,
                 force_gain=1.0, force_limit=0.5, max_linear_speed=MAX_LINEAR_SPEED,
                 max_joint_speed=MAX_JOINT_SPEED, mode='rate',
                 deadband=0.003, rate_gain=2.5):
        validate_tuning(force_gain, force_limit, max_linear_speed, max_joint_speed)
        self.force_gain, self.force_limit = force_gain, force_limit
        self.max_linear_speed, self.max_joint_speed = max_linear_speed, max_joint_speed
        self.mode = mode
        if os.environ.get('ROS_DOMAIN_ID') != '42':
            raise RuntimeError('Touch simulation requires ROS_DOMAIN_ID=42')
        super().__init__(KeyboardState(), execute, 'touch_x_sim_teleop')
        self.clutch = Clutch(scale=scale, rot_scale=rot_scale, mode=mode,
                             deadband=deadband, max_linear_speed=max_linear_speed,
                             rate_gain=rate_gain)
        self.rcm = rcm
        # Trocar incision port location in world frame (abdominal wall RCM point)
        self.p_rcm = np.array([-0.134, 0.492, 0.280])
        self.attach_site_id = self.model.site('attachment_site').id
        self.contact = None
        self.contact_received = 0.0
        self.last_contact_stamp = -1
        self.last_rendered_stamp = -1
        self.last_safe_command = 0.0
        self.create_subscription(WrenchStamped, '/simulation/tool_contact', self.on_contact, 1)
        self.create_timer(0.01, self.render_force)
        self.touch = None
        self.touch_time = None
        self.last_stamp = -1
        self.last_command = None
        self.deadman_pub = self.create_publisher(Bool, '/teleop/deadman', 1) if execute else None
        self.force_pub = self.create_publisher(WrenchStamped, '/touch_x/force_feedback', 1) if execute else None
        self.create_subscription(Joy, '/touch_x/state', self.on_touch, 1)
        self.was_active = False
        self.router_parameters = self.create_client(GetParameters, '/ur5e_command_router/get_parameters')
        self.router_verified = False
        self.router_request = None
        self.mapping = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)

    def zero_force(self):
        if self.force_pub:
            msg = WrenchStamped()
            msg.header.frame_id = 'touch_x_base'
            msg.header.stamp = self.get_clock().now().to_msg()
            self.force_pub.publish(msg)

    def on_contact(self, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        f = msg.wrench.force
        if (msg.header.frame_id!='world' or not 0<=age<0.1 or stamp<=self.last_contact_stamp
                or not np.all(np.isfinite([f.x,f.y,f.z]))):
            self.contact = None
            return
        self.last_contact_stamp = stamp
        self.contact_received = time.monotonic()
        self.contact = msg

    def render_force(self):
        if not self.force_pub:
            return
        now = time.monotonic()
        source = self.contact
        if (not self.was_active or self.finished
                or now-self.last_safe_command>=0.1
                or self.touch_time is None or now-self.touch_time>=0.1
                or not self.touch.buttons[0] or not self.touch.buttons[2]):
            self.zero_force()
            return

        # 1. Environmental contact force reflection
        if source is not None and now-self.contact_received < 0.1:
            age = (self.get_clock().now().nanoseconds-self.last_contact_stamp)/1e9
            if 0 <= age < 0.1:
                if self.last_rendered_stamp != self.last_contact_stamp:
                    self.last_rendered_stamp = self.last_contact_stamp
                    f = source.wrench.force
                    result = reflected_force([f.x,f.y,f.z], self.clutch.scale, self.mapping,
                                             gain=self.force_gain, limit=self.force_limit)
                    msg = WrenchStamped()
                    msg.header = source.header
                    msg.header.frame_id = 'touch_x_base'
                    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = result.tolist()
                    self.force_pub.publish(msg)
                    return

        # 2. Tactile centering spring in rate mode when not in contact
        if self.mode == 'rate' and self.clutch.anchor_touch is not None:
            disp = np.asarray(self.touch.axes[:3], dtype=float) - self.clutch.anchor_touch
            k_spring = 25.0  # N/m centering stiffness
            f_spring = -k_spring * disp
            mag = np.linalg.norm(f_spring)
            if mag > self.force_limit:
                f_spring *= self.force_limit / mag
            msg = WrenchStamped()
            msg.header.frame_id = 'touch_x_base'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = f_spring.tolist()
            self.force_pub.publish(msg)
            return

        self.zero_force()

    def on_touch(self, msg):
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if (msg.header.frame_id != 'touch_x_base' or len(msg.axes) != 7
                or len(msg.buttons) != 4 or not np.all(np.isfinite(msg.axes))
                or any(b not in (0, 1) for b in msg.buttons[:3])
                or not 0 <= age < 0.15 or stamp <= self.last_stamp):
            self.hold('invalid/stale Touch state')
            self.touch_time = None
            return
        self.last_stamp = stamp
        self.touch = msg
        self.touch_time = time.monotonic()
        if not msg.buttons[0] or msg.buttons[1] or not msg.buttons[2]:
            # Observe even a short release between controller timer ticks.
            self.clutch.active = False
            self.clutch.released = not msg.buttons[0] and not msg.buttons[1] and bool(msg.buttons[2])
            self.clutch.previous = np.asarray(msg.axes[:3], dtype=float)
            if self.deadman_pub:
                self.deadman_pub.publish(Bool(data=False))
            if self.force_pub:
                self.zero_force()

    def hold(self, reason):
        if hasattr(self, 'clutch'):
            self.clutch.reset()
        if getattr(self, 'deadman_pub', None):
            self.deadman_pub.publish(Bool(data=False))
        if getattr(self, 'force_pub', None):
            self.zero_force()
        if getattr(self, 'was_active', False):
            self.get_logger().info('HOLD: '+reason+'; release button 1 before re-engaging')
        self.was_active = False

    def fail(self, reason):
        self.hold(reason)
        super().fail(reason)

    def touch_graph_ok(self):
        def names(topic, publisher=True):
            endpoints = (self.get_publishers_info_by_topic(topic) if publisher
                         else self.get_subscriptions_info_by_topic(topic))
            return sorted(e.node_namespace.rstrip('/')+'/'+e.node_name for e in endpoints)
        return (names('/touch_x/state') == ['/touch_x_publisher']
                and (not self.execute or names('/teleop/deadman') == ['/touch_x_sim_teleop'])
                and (not self.execute or names('/teleop/deadman', False) == ['/ur5e_command_router'])
                and (not self.execute or names('/touch_x/force_feedback') == ['/touch_x_sim_teleop'])
                and names('/simulation/tool_contact') == ['/mujoco_ros2_control_node']
                and names(CONTROLLER_TOPIC, False) == ['/position_controller', '/touch_x_sim_teleop'])

    def update(self):
        if self.finished:
            return
        now = time.monotonic()
        if not self.router_verified:
            if self.router_request is None and self.router_parameters.service_is_ready():
                self.router_request = self.router_parameters.call_async(GetParameters.Request(names=['require_deadman']))
            if self.router_request is not None and self.router_request.done():
                result = self.router_request.result()
                if not result or not result.values or not result.values[0].bool_value:
                    self.fail('Router must have require_deadman:=true')
                    return
                self.router_verified = True
            return
        ready = (self.model_verified and self.q is not None
                 and self.state_time is not None and now-self.state_time < JOINT_FRESHNESS
                 and self.heartbeat_time is not None and now-self.heartbeat_time < HEARTBEAT_FRESHNESS
                 and self.graph_ok() and self.touch_graph_ok())
        if not ready:
            if now-self.last_log > 2.0 and self.initial_position is None:
                self.get_logger().info(f'Waiting: model={self.model_verified}, joints={self.state_time is not None}, heartbeat={self.heartbeat_time is not None}, graph={self.graph_ok()}, touch_graph={self.touch_graph_ok()}')
                self.last_log=now
            self.hold('simulation graph, model, heartbeat, or joints not ready')
            if self.initial_position is not None:
                self.fail('Simulation verification lost')
            return
        self.data.qpos[self.qpos_idx] = self.q
        self.data.qvel[self.dof_idx] = self.dq
        mujoco.mj_forward(self.model, self.data)

        # Permit tip/environment contact; reject robot-link environment contact
        # and non-adjacent link self-collision in the local model.
        env_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'surgical_environment')
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            b1 = self.model.geom_bodyid[c.geom1]
            b2 = self.model.geom_bodyid[c.geom2]
            tip_geom = self.model.geom('instrument_tip_collision').id
            if (b1 == env_body_id or b2 == env_body_id) and c.geom1 != tip_geom and c.geom2 != tip_geom:
                self.fail('Robot link contact with environment')
                return
            if b1 != 0 and b2 != 0 and b1 != env_body_id and b2 != env_body_id and self.model.body_parentid[b1] != b2 and self.model.body_parentid[b2] != b1:
                self.fail('Robot link self-collision detected')
                return

        position = self.data.site_xpos[self.tip_id].copy()
        rotation = self.data.site_xmat[self.tip_id].reshape(3, 3).copy()
        flange_pos = self.data.site_xpos[self.attach_site_id].copy()
        shaft_vec = position - flange_pos
        shaft_len = np.linalg.norm(shaft_vec)
        u_shaft = shaft_vec / shaft_len if shaft_len > 0.01 else np.array([0, 0, -1.0])

        # RCM geometric kinematics
        s_proj = np.dot(self.p_rcm - flange_pos, u_shaft)
        p_closest = flange_pos + s_proj * u_shaft
        e_rcm = p_closest - self.p_rcm
        rcm_dist = np.linalg.norm(e_rcm)

        if self.rcm and rcm_dist > 0.025:
            self.fail('RCM incision constraint exceeded safe tolerance (> 25 mm)')
            return

        gravity = self.data.qfrc_bias[self.dof_idx]
        hold_target = self.q + gravity/self.gains
        if (not np.all(np.isfinite(hold_target)) or np.any(gravity < self.force_ranges[:, 0])
                or np.any(gravity > self.force_ranges[:, 1])):
            self.fail('Invalid gravity holding target')
            return
        if self.initial_position is None:
            if np.max(np.abs(self.dq)) > 0.002:
                return
            if np.max(np.abs(hold_target-HOME_TARGETS)) > 0.04:
                self.fail('Simulation must start at home')
                return
            self.initial_position, self.initial_rotation = position.copy(), rotation.copy()
            self.initial_hold = hold_target.copy()
            self.last_command = hold_target.copy()
            if self.execute:
                self.publisher = self.create_publisher(Float64MultiArray, TELEOP_TOPIC, 1)
            rcm_mode = "RCM ENFORCED" if self.rcm else "FREE 6-DOF"
            self.get_logger().info(f'READY: Simulation teleop active [{rcm_mode}]. Scale {self.clutch.scale}. Contact feedback published; device motors require explicit opt-in.')
            return
        delta = position-self.initial_position
        if np.any(np.abs(delta) > MAX_AXIS_TRAVEL+0.003) or np.linalg.norm(delta) > MAX_TOTAL_TRAVEL+0.003:
            self.fail('Tip exceeded local workspace')
            return
        if self.first_publish_time is not None:
            if self.router_time is None or now-self.router_time > 0.25:
                self.fail('Router output stale')
                return
            if np.max(np.abs(self.router_output-hold_target)) > 0.080:
                self.fail('Excessive actuator tracking error')
                return
        if self.touch_time is None or now-self.touch_time > 0.15:
            self.hold('Touch stream stale')
            return
        msg = self.touch
        target_res = self.clutch.update(
            msg.axes[:3], msg.buttons[:2], msg.buttons[2], msg.buttons[3], position,
            device_quat=msg.axes[3:7], tip_rot=rotation,
        )
        if target_res is None:
            if self.deadman_pub:
                self.deadman_pub.publish(Bool(data=False))
            if self.force_pub:
                self.zero_force()
            if self.was_active:
                self.get_logger().info('HOLD: clutch/deadman released (forces zeroed)')
            self.was_active = False
            return
        if isinstance(target_res, tuple):
            target_pos, target_rot = target_res
        else:
            target_pos, target_rot = target_res, None
        if not self.was_active:
            self.get_logger().info('ENGAGED: relative reference captured')
        self.was_active = True
        if not self.execute:
            return

        # -------------------------------------------------------------
        # INVERSE KINEMATICS (RCM-CONSTRAINED OR FREE 6-DOF)
        # -------------------------------------------------------------
        if self.mode == 'rate':
            velocity = self.clutch.current_velocity.copy()
            delta = position - self.initial_position
            for ax in range(3):
                if delta[ax] >= MAX_AXIS_TRAVEL and velocity[ax] > 0:
                    velocity[ax] = 0.0
                elif delta[ax] <= -MAX_AXIS_TRAVEL and velocity[ax] < 0:
                    velocity[ax] = 0.0
            norm_delta = np.linalg.norm(delta)
            if norm_delta >= MAX_TOTAL_TRAVEL and np.dot(velocity, delta) > 0:
                velocity -= (np.dot(velocity, delta) / (norm_delta**2)) * delta
            if np.linalg.norm(velocity) > self.max_linear_speed:
                velocity *= self.max_linear_speed / np.linalg.norm(velocity)
        else:
            travel = np.clip(target_pos-self.initial_position, -MAX_AXIS_TRAVEL, MAX_AXIS_TRAVEL)
            if np.linalg.norm(travel) > MAX_TOTAL_TRAVEL:
                travel *= MAX_TOTAL_TRAVEL/np.linalg.norm(travel)
            velocity = 4.0 * (self.initial_position+travel-position)
            if np.linalg.norm(velocity) > self.max_linear_speed:
                velocity *= self.max_linear_speed/np.linalg.norm(velocity)

        target_rotation = target_rot if target_rot is not None else self.initial_rotation
        orientation_error = 0.5*sum(np.cross(rotation[:, i], target_rotation[:, i]) for i in range(3))

        if self.rcm:
            # ---------------------------------------------------------
            # Augmented RCM Inverse Kinematics
            # ---------------------------------------------------------
            jp_tip, jr_tip = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jp_tip, jr_tip, self.tip_id)
            J_tip = jp_tip[:, self.dof_idx]

            jp_flange, jr_flange = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jp_flange, jr_flange, self.attach_site_id)
            Jv_flange = jp_flange[:, self.dof_idx]
            Jw_flange = jr_flange[:, self.dof_idx]

            u_cross = np.array([
                [0, -u_shaft[2], u_shaft[1]],
                [u_shaft[2], 0, -u_shaft[0]],
                [-u_shaft[1], u_shaft[0], 0]
            ])
            J_rcm_point = Jv_flange - s_proj * (u_cross @ Jw_flange)

            if abs(u_shaft[0]) < 0.9:
                n1 = np.cross(u_shaft, [1, 0, 0])
            else:
                n1 = np.cross(u_shaft, [0, 1, 0])
            n1 /= np.linalg.norm(n1)
            n2 = np.cross(u_shaft, n1)
            P_lat = np.vstack([n1, n2])
            J_rcm_lat = P_lat @ J_rcm_point  # 2x6 lateral incision constraint

            J_roll = u_shaft.reshape(1, 3) @ Jw_flange  # 1x6 tool shaft roll

            w_rcm, w_tip, w_roll = 6.0, 1.0, 1.0
            J_aug = np.vstack([w_rcm * J_rcm_lat, w_tip * J_tip, w_roll * J_roll])

            v_rcm_des = -w_rcm * 12.0 * (P_lat @ e_rcm)
            v_tip_des = w_tip * velocity
            e_roll = np.dot(u_shaft, orientation_error)
            w_roll_des = np.array([w_roll * KP_ORIENTATION * e_roll])

            desired = np.concatenate([v_rcm_des, v_tip_des, w_roll_des])
            try:
                qdot = J_aug.T @ np.linalg.solve(J_aug @ J_aug.T + DAMPING**2 * np.eye(6), desired)
            except np.linalg.LinAlgError:
                self.fail('RCM IK solve failed')
                return
        else:
            # Free 6-DOF (unconstrained) IK
            desired = np.r_[velocity, KP_ORIENTATION*orientation_error]
            jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jp, jr, self.tip_id)
            jac = np.vstack((jp, jr))[:, self.dof_idx]
            try:
                qdot = jac.T @ np.linalg.solve(jac@jac.T + DAMPING**2*np.eye(6), desired)
            except np.linalg.LinAlgError:
                self.fail('IK failed')
                return

        if np.max(np.abs(qdot)) > self.max_joint_speed:
            qdot *= self.max_joint_speed/np.max(np.abs(qdot))
        command = hold_target + qdot*DT
        if (not np.all(np.isfinite(command)) or np.any(command < self.ctrl_ranges[:, 0])
                or np.any(command > self.ctrl_ranges[:, 1])
                or np.any(np.abs(command-self.initial_hold) > MAX_COMMAND_TRAVEL)):
            self.fail('Joint target out of bounds')
            return
        self.last_safe_command = now
        self.deadman_pub.publish(Bool(data=True))
        self.publisher.publish(Float64MultiArray(data=command.tolist()))
        self.last_command = command
        if self.first_publish_time is None:
            self.first_publish_time = now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--mode', choices=['rate', 'position'], default='rate',
                        help='Teleop mode: rate (continuous motion while holding button) or position (clutched tracking)')
    parser.add_argument('--deadband', type=float, default=0.003,
                        help='Neutral deadband in meters for rate mode (default: 0.003 = 3 mm)')
    parser.add_argument('--rate-gain', type=float, default=2.5,
                        help='Velocity gain in rate mode (m/s per meter deflection, default: 2.5)')
    parser.add_argument('--scale', type=float, default=0.2, help='Translational scale factor for position mode (default: 0.2)')
    parser.add_argument('--rot-scale', type=float, default=1.0, help='Rotational scale factor (default: 1.0)')
    parser.add_argument('--rcm', action='store_true', default=False, help='Enable experimental RCM constraint (default: off)')
    parser.add_argument('--no-rcm', dest='rcm', action='store_false', help='Disable RCM (free 6-DOF motion)')
    parser.add_argument('--force-gain', type=float, default=1.0)
    parser.add_argument('--force-limit', type=float, default=0.5)
    parser.add_argument('--max-linear-speed', type=float, default=MAX_LINEAR_SPEED)
    parser.add_argument('--max-joint-speed', type=float, default=MAX_JOINT_SPEED)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        parser.error('Requires ROS_DOMAIN_ID=42; domain 43 prohibited')
    rclpy.init()
    node = TouchTeleop(args.execute, scale=args.scale, rot_scale=args.rot_scale, rcm=args.rcm,
                       force_gain=args.force_gain, force_limit=args.force_limit,
                       max_linear_speed=args.max_linear_speed, max_joint_speed=args.max_joint_speed,
                       mode=args.mode, deadband=args.deadband, rate_gain=args.rate_gain)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.hold('controller stopped')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
