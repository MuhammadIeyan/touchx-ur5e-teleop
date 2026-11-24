import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
from touch_clutch import Clutch
from touch_x_publisher import decode_sample
from command_router import UR5eCommandRouter


class SafetyTests(unittest.TestCase):
    def test_start_held_cannot_arm(self):
        c = Clutch()
        self.assertIsNone(c.update([0,0,0],[1,0],True,0,[1,2,3]))
        c.update([0,0,0],[0,0],True,0,[1,2,3])
        np.testing.assert_allclose(c.update([.01,0,0],[1,0],True,0,[1,2,3]),[1,2,3])
        np.testing.assert_allclose(c.update([.02,0,0],[1,0],True,0,[1,2,3]),[1.002,2,3])

    def test_release_rebases_without_jump(self):
        c = Clutch()
        c.update([0,0,0],[0,0],True,0,[0,0,0])
        c.update([0,0,0],[1,0],True,0,[0,0,0])
        self.assertIsNone(c.update([.01,0,0],[0,0],True,0,[.001,0,0]))
        np.testing.assert_allclose(c.update([.02,0,0],[1,0],True,0,[.001,0,0]),[.001,0,0])

    def test_rate_mode_continuous_motion_and_release(self):
        import time
        c = Clutch(mode='rate', deadband=0.003, max_linear_speed=0.08, rate_gain=2.5)
        # 1. Require release before first engagement
        self.assertIsNone(c.update([0,0,0], [1,0], True, 0, [0,0,0]))
        c.update([0,0,0], [0,0], True, 0, [0,0,0])
        # 2. Press deadman button: neutral anchor captured
        p0 = c.update([0,0,0], [1,0], True, 0, [0,0,0])
        np.testing.assert_allclose(p0, [0,0,0])
        self.assertTrue(c.active)
        # 3. Deflect stylus beyond 3mm deadband (15 mm in X)
        time.sleep(0.04)
        p1 = c.update([0.015, 0, 0], [1,0], True, 0, [0,0,0])
        self.assertIsNotNone(p1)
        self.assertGreater(p1[0], 0.0)
        self.assertGreater(c.current_velocity[0], 0.0)
        # 4. Hold deflection: motion continues smoothly
        time.sleep(0.04)
        p2 = c.update([0.015, 0, 0], [1,0], True, 0, [0,0,0])
        self.assertGreater(p2[0], p1[0])  # Continues moving without stopping!
        # 5. Release deadman button: stops immediately
        p_rel = c.update([0.015, 0, 0], [0,0], True, 0, [0,0,0])
        self.assertIsNone(p_rel)
        self.assertFalse(c.active)
        np.testing.assert_allclose(c.current_velocity, [0,0,0])

    def test_fault_requires_release(self):
        for fault in ('stale','epoch','jump','calibration','clutch'):
            c=Clutch(); c.update([0,0,0],[0,0],True,0,[0,0,0])
            c.update([0,0,0],[1,0],True,0,[0,0,0])
            kwargs=dict(xyz=[0,0,0], buttons=[1,0], calibrated=True, epoch=0, tip=[0,0,0])
            if fault=='stale': kwargs['valid']=False
            if fault=='epoch': kwargs['epoch']=1
            if fault=='jump': kwargs['xyz']=[.1,0,0]
            if fault=='calibration': kwargs['calibrated']=False
            if fault=='clutch': kwargs['buttons']=[1,1]
            self.assertIsNone(c.update(**kwargs))
            self.assertIsNone(c.update([0,0,0],[1,0],True,kwargs['epoch'],[0,0,0]))

    def test_sample_age_transform_and_units(self):
        raw=dict(seq=1,monotonic_ns=10**9,transport_ns=10**9,xyz_mm=[100,20,-30],transform=np.eye(4).reshape(16).tolist(),buttons=3,calibration=20480,calibration_epoch=0)
        sample=decode_sample(raw,10**9+1000)
        np.testing.assert_allclose(sample['xyz'],[.1,.02,-.03])
        self.assertEqual(sample['quat'],[0,0,0,1])
        self.assertTrue(all(type(f) is float for f in sample['force']))
        self.assertEqual(decode_sample(dict(raw,force_n=[0,0,0]),10**9+1000)['force'],[0.,0.,0.])
        for key,value in [('seq',0),('xyz_mm',[float('nan'),0,0]),('transform',[0]*16),('buttons',8)]:
            with self.assertRaises(ValueError): decode_sample(dict(raw,**{key:value}),10**9+1000)
        with self.assertRaises(ValueError): decode_sample(raw,2*10**9)
        with self.assertRaises(ValueError): decode_sample(dict(raw,transport_ns=0),10**9+1000)

    def test_router_release_cancels_queued_target(self):
        fake=SimpleNamespace(require_deadman=True, output_positions=[1.]*6,target_positions=[2.]*6)
        UR5eCommandRouter.on_deadman(fake,SimpleNamespace(data=False))
        self.assertEqual(fake.target_positions,[1.]*6)
        fake.actual_positions=[1.]*6
        UR5eCommandRouter.teleop_callback(fake,SimpleNamespace(data=[3.]*6))
        self.assertEqual(fake.target_positions,[1.]*6)

    def test_6dof_orientation_tracking_and_rebasing(self):
        c = Clutch(scale=0.6, rot_scale=1.0)
        c.update([0,0,0], [0,0], True, 0, [0,0,0], device_quat=[0,0,0,1], tip_rot=np.eye(3))
        res = c.update([0,0,0], [1,0], True, 0, [0,0,0], device_quat=[0,0,0,1], tip_rot=np.eye(3))
        self.assertIsNotNone(res)
        pos, rot = res
        np.testing.assert_allclose(pos, [0,0,0])
        np.testing.assert_allclose(rot, np.eye(3), atol=1e-7)

        # Rotate device by 90 deg about Z in device frame (qx=0, qy=0, qz=sin(pi/4), qw=cos(pi/4))
        qz, qw = np.sin(np.pi / 4), np.cos(np.pi / 4)
        res = c.update([0,0,0], [1,0], True, 0, [0,0,0], device_quat=[0,0,qz,qw], tip_rot=np.eye(3))
        pos, rot = res
        # Mapping maps device +Z to robot -Y, so 90 deg rotation is around robot -Y
        expected_rot = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
        np.testing.assert_allclose(rot, expected_rot, atol=1e-6)

        # Release deadman: must return None
        self.assertIsNone(c.update([0,0,0], [0,0], True, 0, [0,0,0], device_quat=[0,0,qz,qw], tip_rot=np.eye(3)))
        # Re-arm at new orientation: must rebase without jump
        res = c.update([0,0,0], [1,0], True, 0, [0,0,0], device_quat=[0,0,qz,qw], tip_rot=np.eye(3))
        pos, rot = res
        np.testing.assert_allclose(rot, np.eye(3), atol=1e-7)

    def test_laparoscopic_rcm_fulcrum_kinematics(self):
        import mujoco
        model_path = Path(__file__).resolve().parents[2] / 'models/mujoco_menagerie/universal_robots_ur5e/ur5e_grasper.xml'
        m = mujoco.MjModel.from_xml_path(str(model_path))
        d = mujoco.MjData(m)
        JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        qpos_idx = [m.joint(n).qposadr[0] for n in JOINT_NAMES]
        HOME_TARGETS = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
        d.qpos[qpos_idx] = HOME_TARGETS
        mujoco.mj_forward(m, d)

        tip_id = m.site('instrument_tip').id
        flange_id = m.site('attachment_site').id
        p_rcm = np.array([-0.134, 0.492, 0.280])

        p_tip = d.site_xpos[tip_id].copy()
        p_flange = d.site_xpos[flange_id].copy()
        u = (p_tip - p_flange) / np.linalg.norm(p_tip - p_flange)

        # Shaft passes directly through trocar at home
        s_proj = np.dot(p_rcm - p_flange, u)
        p_closest = p_flange + s_proj * u
        rcm_err = np.linalg.norm(p_closest - p_rcm)
        self.assertLess(rcm_err, 0.001)  # < 1 mm at home

        # Fulcrum effect: moving tip +30mm in X requires flange to move in -X
        p_target = p_tip + np.array([0.03, 0.0, 0.0])
        v_in = p_target - p_rcm
        u_des = v_in / np.linalg.norm(v_in)
        p_flange_des = p_target - 0.330 * u_des
        self.assertLess(p_flange_des[0], p_flange[0])  # Opposite lateral motion across fulcrum

    def test_domain_guard_before_ros_init(self):
        with patch.dict(os.environ,ROS_DOMAIN_ID='43'):
            with self.assertRaises(RuntimeError): UR5eCommandRouter()


if __name__=='__main__': unittest.main()
