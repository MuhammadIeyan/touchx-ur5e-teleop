import sys, unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from force_feedback import reflected_force, validate_tuning
from touch_clutch import Clutch, _scale_rotation
class ForceTests(unittest.TestCase):
    def test_power_conjugate_mapping(self):
        m=Clutch().mapping; scale=.2; v=np.array([.01,-.02,.03]); f=np.array([.4,.5,.6])
        actual=reflected_force(f,scale,m)
        self.assertAlmostEqual(actual@v, f@(scale*m@v))
        np.testing.assert_allclose(reflected_force([0,0,1],scale,m),[0,.2,0])
    def test_clamp_and_invalid(self):
        m=Clutch().mapping
        self.assertAlmostEqual(np.linalg.norm(reflected_force([20,30,40],.2,m)),.5)
        for f in ([np.nan,0,0],[np.inf,0,0],[0,0]):
            with self.assertRaises(ValueError):reflected_force(f,.2,m)
    def test_responsive_profile_and_limits(self):
        m=Clutch().mapping
        np.testing.assert_allclose(reflected_force([0,0,1],.35,m,gain=2,limit=.8),[0,.7,0])
        self.assertAlmostEqual(np.linalg.norm(reflected_force([0,0,10],.35,m,gain=2,limit=.8)),.8)
        for name in ('force_gain','force_limit','max_linear_speed','max_joint_speed'):
            for value in (float('nan'),float('inf'),-1,100):
                with self.assertRaises(ValueError):validate_tuning(**{name:value})
        validate_tuning(2,.8,.03,.2)

    def test_pi_rotation(self):
        r=_scale_rotation(np.diag([1,-1,-1]),.5)
        np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(r),1)
        self.assertAlmostEqual(np.trace(r),1)
    def test_bad_orientation_disarms(self):
        c=Clutch();c.update([0]*3,[0,0],True,0,[0]*3)
        self.assertIsNone(c.update([0]*3,[1,0],True,0,[0]*3,device_quat=[0]*4,tip_rot=np.eye(3)))
        self.assertFalse(c.released)
