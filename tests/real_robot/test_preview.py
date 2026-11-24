import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.preview import Preview
from teleop_geometry import rotation_to_xyzw
from touch_clutch import _quat_to_mat
from touch_x_publisher import validate_backend
class PreviewTests(unittest.TestCase):
    def test_shared_rotation_round_trip(self):
        rng=np.random.default_rng(7)
        for q in ([1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.],*rng.normal(size=(100,4))):
            q=np.asarray(q);q=q/np.linalg.norm(q);r=_quat_to_mat(q)
            np.testing.assert_allclose(_quat_to_mat(rotation_to_xyzw(r)),r,atol=1e-12)

    def test_backend_isolation(self):
        self.assertEqual(validate_backend('43',True,False),'43')
        for domain,real,force in [('43',False,False),('42',True,False),('43',True,True),('44',False,False)]:
            with self.assertRaises(RuntimeError):validate_backend(domain,real,force)
        self.assertEqual(validate_backend('42',False,True),'42')
    def test_release_speed_workspace_and_stale(self):
        p=Preview();tcp=[.4,0,.5]
        self.assertIsNone(p.update(0,[0]*3,[1,0],True,0,tcp))
        p.update(.02,[0]*3,[0,0],True,0,tcp)
        np.testing.assert_allclose(p.update(.04,[0]*3,[1,0],True,0,tcp),tcp)
        old=np.array(tcp)
        for i in range(1,401):
            x=min(.3,i*.005)
            target=p.update(.04+i*.02,[x,0,0],[1,0],True,0,tcp)
            self.assertLessEqual(np.linalg.norm(target-old),.000100001)
            self.assertLessEqual(np.linalg.norm(target-tcp),.02000001)
            old=target.copy()
        self.assertIsNone(p.update(9,[.3,0,0],[1,0],True,0,tcp,fresh=False))
        self.assertIsNone(p.update(9.02,[.3,0,0],[1,0],True,0,tcp))
    def test_reengagement_anchors_at_measured_tcp(self):
        p=Preview();p.update(0,[0]*3,[0,0],True,0,[0]*3)
        p.update(.02,[0]*3,[1,0],True,0,[0]*3)
        p.update(.04,[.01,0,0],[0,0],True,0,[.5,0,0])
        np.testing.assert_allclose(p.update(.06,[.02,0,0],[1,0],True,0,[.5,0,0]),[.5,0,0])
