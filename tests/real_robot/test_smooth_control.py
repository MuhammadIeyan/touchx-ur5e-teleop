import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.smooth_control import HoldGesture,Quintic,ActionWindow

class SmoothTests(unittest.TestCase):
    def test_hold_stays_enabled_without_repress(self):
        g=HoldGesture();g.update(False,0.)
        self.assertFalse(g.update(True,.01)[0])
        for t in np.arange(.27,5.,.02):self.assertTrue(g.update(True,t)[0])
        self.assertFalse(g.update(False,5.1)[0])
        g.update(True,5.2);self.assertTrue(g.update(True,5.5)[0])

    def test_startup_held_and_invalid_input_require_release(self):
        g=HoldGesture();g.update(True,0.)
        self.assertFalse(g.update(True,1.)[0])
        g.update(False,1.1);g.update(True,1.2);self.assertTrue(g.update(True,1.5)[0])
        g.update(True,1.6,valid=False);self.assertFalse(g.update(True,2.)[0])

    def test_three_taps_never_enable_motion_and_request_only_on_release(self):
        g=HoldGesture();g.update(False,0.)
        for i in range(3):
            self.assertEqual(g.update(True,.1+i*.2),(False,False))
            self.assertEqual(g.update(False,.2+i*.2),(False,i==2))

    def test_two_taps_and_separated_taps_do_not_request_rcm(self):
        g=HoldGesture();g.update(False,0)
        for t in (1.,1.2,3.):
            self.assertFalse(g.update(True,t)[1]);self.assertFalse(g.update(False,t+.1)[1])

    def test_replacement_preserves_position_velocity_acceleration(self):
        q=np.zeros(6);end=np.ones(6)*.001
        first=Quintic(q,q,q,end)
        start=first.sample(.05)
        second=Quintic(*start,np.ones(6)*.002)
        for expected,actual in zip(start,second.sample(0)):np.testing.assert_allclose(actual,expected,atol=1e-12)
        final=second.sample(.3)
        np.testing.assert_allclose(final[0],.002,atol=1e-12)
        np.testing.assert_allclose(final[1:],0,atol=1e-12)

    def test_old_zero_time_waypoint_rewinds_delayed_feedback(self):
        zero=np.zeros(6)
        old=Quintic(zero,zero,zero,np.ones(6)*.0015)
        feedback=old.sample(.06)
        at_delivery=old.sample(.10)
        stale=Quintic(*feedback,np.ones(6)*.002)
        # Reproduce the original bug: replacing at t=0 resets to older q/v/a.
        self.assertGreater(np.max(at_delivery[0]-stale.sample(0)[0]),.0001)
        blended=Quintic(*at_delivery,np.ones(6)*.002)
        for expected,actual in zip(at_delivery,blended.sample(0)):
            np.testing.assert_allclose(actual,expected,atol=1e-12)

    def test_geometry_subdivides_without_losing_reversal_travel(self):
        from real_robot.path_geometry import PathGeometry
        from types import SimpleNamespace
        zero=np.zeros(6)
        curve=Quintic(zero,np.ones(6)*.01,zero,zero)
        # Same endpoints do not mean zero motion: the curve reverses internally.
        travel=curve.joint_travel(0,.3)
        dense=np.array([curve.sample(t)[0] for t in np.linspace(0,.3,2001)])
        self.assertGreater(travel,.001)
        self.assertAlmostEqual(travel,np.abs(np.diff(dense,axis=0)).sum(),places=7)
        checks=[]
        obj=SimpleNamespace(check=lambda q,padding=.003:checks.append(padding))
        PathGeometry.curve(obj,curve)
        self.assertGreater(len(checks),2)
        self.assertLessEqual(max(checks),.0030000001)

    def test_constant_target_is_not_fixed_size_jump(self):
        q=np.zeros(6);curve=Quintic(q,q,q,np.ones(6)*.00001)
        samples=np.array([curve.sample(t)[0] for t in np.linspace(0,.3,100)])
        self.assertLessEqual(samples.max(),.000010000001)
        self.assertGreaterEqual(samples.min(),0.)

    def test_full_curve_bounds_reject_oversized_motion(self):
        from types import SimpleNamespace
        def fk(q):
            pose=np.eye(4);pose[:3,3]=q[:3];return pose
        model=SimpleNamespace(lower=np.ones(6)*-3,upper=np.ones(6)*3,fk=fk)
        q=np.zeros(6);end=q.copy();end[0]=.001
        Quintic(q,q,q,end).validate(model,np.zeros(3),np.eye(3),.05)
        end[0]=.003
        with self.assertRaises(ValueError):Quintic(q,q,q,end).validate(model,np.zeros(3),np.eye(3),.05)
        end[0]=.001
        with self.assertRaises(ValueError):Quintic(q,q,q,end).validate(model,np.zeros(3),np.eye(3),.0005)

    def test_stop_catches_pending_late_acceptance(self):
        w=ActionWindow();seq=w.begin();w.stopping=True
        self.assertTrue(w.accept(seq,object()))
        self.assertFalse(w.drained());w.terminal(seq);self.assertTrue(w.drained())
        with self.assertRaises(RuntimeError):w.begin()

    def test_new_goal_does_not_wait_for_old_completion(self):
        w=ActionWindow();first=w.begin();w.accept(first,object())
        second=w.begin();self.assertEqual(second,first+1)
        with self.assertRaises(RuntimeError):w.begin()
        w.accept(second,object());w.terminal(first)
        self.assertIn(second,w.handles)

if __name__=='__main__':unittest.main()
