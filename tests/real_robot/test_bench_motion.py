import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from scipy.spatial.transform import Rotation
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
NOMINAL_KINEMATICS=Path(get_package_share_directory("ur_description"))/"config/ur5e/default_kinematics.yaml"
NOMINAL_HASH=yaml.safe_load(NOMINAL_KINEMATICS.read_text())["kinematics"]["hash"]

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
from real_robot.bench_motion import Model, GoalLease, STEP_M, JOINT_STEP_RAD, validate_session, validate_payload
from real_robot.bench_step import BenchStep
from sensor_msgs.msg import Joy


class BenchTests(unittest.TestCase):
    def test_verified_payload_units_and_mismatch(self):
        manifest = dict(payload_verified=True, reported_payload_mass_kg=.14,
                        reported_payload_cog_mm=[-23,30,61])
        telemetry = dict(payload=.14, payload_cog=[-.023,.030,.061])
        validate_payload(telemetry, manifest)
        for change in ({'payload':.15}, {'payload_cog':[-.072,.023,.039]},
                       {'payload':float('nan')}, {'payload_cog':[-23,30,61]}):
            with self.assertRaises(RuntimeError):validate_payload(dict(telemetry,**change),manifest)
        with self.assertRaises(RuntimeError):
            validate_payload(telemetry,dict(manifest,payload_verified=False))

    @classmethod
    def setUpClass(cls):
        cls.xml = xacro.process_file('/opt/ros/humble/share/ur_description/urdf/ur.urdf.xacro', mappings={
            'ur_type': 'ur5e', 'name': 'ur',
            'kinematics_params': str(NOMINAL_KINEMATICS)}).toxml()
        cls.model = Model(cls.xml, NOMINAL_HASH)
        cls.q = np.array([.69559842348, -1.58299173931, 1.56357652346,
                          .00710694372, 1.57043409348, -.01063329378])

    def test_six_axis_steps_fixed_orientation_and_bounded(self):
        start = self.model.fk(self.q)
        for direction in [s*a for a in np.eye(3) for s in (-1,1)]:
            end, target = self.model.plan(self.q, direction)
            np.testing.assert_allclose(target[:3,3]-start[:3,3], STEP_M*direction, atol=1e-12)
            self.assertLessEqual(np.max(np.abs(end-self.q)), JOINT_STEP_RAD)
            np.testing.assert_allclose(self.model.fk(end), target, atol=1e-7)
            for u in np.linspace(0,1,101):
                s=10*u**3-15*u**4+6*u**5
                pose=self.model.fk(self.q+s*(end-self.q))
                self.assertLessEqual(np.linalg.norm(pose[:3,3]-start[:3,3]), STEP_M*1.01)
                angle=np.linalg.norm(Rotation.from_matrix(pose[:3,:3]@start[:3,:3].T).as_rotvec())
                self.assertLess(angle,1e-5)

    def test_bad_model_and_invalid_targets_fail_closed(self):
        with self.assertRaises(ValueError):Model(self.xml,'wrong_hash')
        with self.assertRaises(ValueError):Model(self.xml,NOMINAL_HASH,'192.168.1.200')
        for direction in ([0,0,0],[float('nan'),0,1],[1,2]):
            with self.assertRaises(ValueError):self.model.plan(self.q,direction)
        with self.assertRaises(ValueError):self.model.plan(self.model.upper,[1,0,0])

    def test_one_goal_only_and_late_acceptance_requires_cancel(self):
        lease=GoalLease();lease.reserve()
        with self.assertRaises(RuntimeError):lease.reserve()
        self.assertFalse(lease.stop())
        handle=object()
        self.assertTrue(lease.accepted(handle))
        self.assertTrue(lease.stop())
        lease.terminal=True
        self.assertFalse(lease.stop())
        stopped=GoalLease();stopped.stop()
        with self.assertRaises(RuntimeError):stopped.reserve()

    def test_session_missing_expired_clearance(self):
        manifest=dict(payload_verified=True,mounting_configuration_verified=True,calibration_match_verified=True,robot_ipv4='192.168.1.103',ros_domain_id=43)
        good=dict(confirmed_at_unix_s=1000,operator_at_pendant=True,workspace_empty=True,
                  external_control_host_confirmed=True,minimum_clearance_mm=100)
        def check(data,now=1010):
            return validate_session(SimpleNamespace(read_text=lambda:json.dumps(data)),manifest,now)
        check(good)
        for change in ({'minimum_clearance_mm':49},{'workspace_empty':False},
                       {'operator_at_pendant':False},{'external_control_host_confirmed':False}):
            with self.assertRaises(ValueError):check(dict(good,**change))
        with self.assertRaises(ValueError):check(good,9000)
        with self.assertRaises(ValueError):check(good,999)

    def test_button_release_cancels_and_does_not_send_replacement(self):
        lease=GoalLease();lease.reserve();lease.accepted(object())
        obj=SimpleNamespace(fresh=lambda *_:True, touch=None,clutch=SimpleNamespace(epoch=1),
                            lease=lease,log=Mock(),cancel=Mock(),fail=Mock())
        msg=Joy();msg.header.frame_id='touch_x_base';msg.axes=[0.]*7;msg.buttons=[0,0,1,1]
        BenchStep.haptic(obj,msg)
        self.assertTrue(lease.stop_requested)
        obj.cancel.assert_called_once()

    def test_late_goal_acceptance_after_stop_is_cancelled(self):
        lease=GoalLease();lease.reserve();lease.stop()
        result_future=SimpleNamespace(add_done_callback=Mock())
        handle=SimpleNamespace(accepted=True,get_result_async=lambda:result_future)
        obj=SimpleNamespace(lease=lease,log=Mock(),finished=Mock(),cancel=Mock())
        BenchStep.accepted(obj,SimpleNamespace(result=lambda:handle))
        obj.cancel.assert_called_once()

    def test_replayed_or_old_source_timestamp_rejected(self):
        clock=SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000_000))
        obj=SimpleNamespace(rx={},stamps={},get_clock=lambda:clock)
        header=SimpleNamespace(stamp=SimpleNamespace(sec=0,nanosec=950_000_000))
        self.assertTrue(BenchStep.fresh(obj,'joints',header))
        self.assertFalse(BenchStep.fresh(obj,'joints',header))
        self.assertNotIn('joints',obj.rx)
        header.stamp.nanosec=500_000_000
        self.assertFalse(BenchStep.fresh(obj,'touch',header))

    def test_execute_sends_one_zero_terminal_velocity_segment(self):
        self.check_segment(.4)

    def test_slow_release_test_duration_is_valid_ros_duration(self):
        self.check_segment(5.0)

    def check_segment(self,duration):
        from touch_clutch import Clutch
        from real_robot.bench_step import CONTROLLER
        clutch=Clutch(scale=.1,rot_scale=0)
        flange=self.model.fk(self.q)[:3,3]
        clutch.update([0,0,0],[0,0],True,1,flange)
        clutch.update([0,0,0],[1,0],True,1,flange)
        pending=SimpleNamespace(add_done_callback=Mock())
        action=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=Mock(return_value=pending))
        obj=SimpleNamespace(closing=False,heartbeat=lambda *args:True,duration=duration,log=Mock(),q=self.q.copy(),tcp=np.eye(4),
            touch=SimpleNamespace(axes=[.01,0,0,0,0,0,1],buttons=[1,0,1,1]),
            mode=7,safety=1,program=True,rx={},lease=GoalLease(),ready=lambda:None,
            last_reason=None,execute=True,activated_here=True,controllers=[SimpleNamespace(name=CONTROLLER,state='active')],
            clutch=clutch,model=self.model,stationary=True,action=action,accepted=Mock(),fail=Mock(),
            get_logger=lambda:Mock())
        BenchStep.tick(obj)
        action.send_goal_async.assert_called_once()
        goal=action.send_goal_async.call_args.args[0]
        self.assertEqual(list(goal.trajectory.points[-1].velocities),[0.]*6)
        self.assertEqual(list(goal.trajectory.points[-1].accelerations),[0.]*6)
        stamp=goal.trajectory.points[-1].time_from_start
        self.assertEqual((stamp.sec,stamp.nanosec),divmod(round(duration*1e9),10**9))
        BenchStep.tick(obj)
        action.send_goal_async.assert_called_once()
        obj.ready=lambda:'stale'
        BenchStep.tick(obj)
        obj.fail.assert_called_once_with('stale')


if __name__=='__main__':unittest.main()
