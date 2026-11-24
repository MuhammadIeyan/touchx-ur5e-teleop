import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.travel_budget import TravelBudget

class BudgetTests(unittest.TestCase):
    def test_total_budget_cannot_be_reset_by_round_trips(self):
        b=TravelBudget(np.zeros(6),np.eye(4))
        for i in range(500):
            p=np.eye(4);p[0,3]=.0001*(i%2)
            b.reserve(np.zeros(6),p)
        with self.assertRaises(ValueError):b.reserve(np.zeros(6),np.eye(4))

    def test_workspace_joint_and_orientation_bounds(self):
        b=TravelBudget(np.zeros(6),np.eye(4))
        p=np.eye(4);p[0,3]=.051
        with self.assertRaises(ValueError):b.check(np.zeros(6),p)
        # 1.0 rad session-wide joint-displacement limit (6 joints × ~0.17 rad each = 1.0 rad).
        with self.assertRaises(ValueError):b.check(np.ones(6)*.18,np.eye(4))
        # 0.6 rad total (6 × 0.1) must pass with new 1.0 rad limit.
        b.check(np.ones(6)*.1,np.eye(4))
        p=np.eye(4);p[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        with self.assertRaises(ValueError):b.check(np.zeros(6),p)

    def test_invalid_or_large_segment_rejected_without_budget_use(self):
        b=TravelBudget(np.zeros(6),np.eye(4))
        for length in (0,-1,.001,float('nan')):
            with self.assertRaises(ValueError):b.reserve(np.zeros(6),np.eye(4),length)
        self.assertEqual(b.commanded,0)

    def test_measured_return_trip_limit(self):
        b=TravelBudget(np.zeros(6),np.eye(4))
        for x in (.02,0,.02):
            p=np.eye(4);p[0,3]=x
            if x==.02 and b.measured>=.04:
                with self.assertRaises(ValueError):b.observe(np.zeros(6),p)
            else:b.observe(np.zeros(6),p)

    def test_reset_origin_continues_after_checkpoint(self):
        """reset_origin() re-arms the per-checkpoint budget but preserves
        the session-wide joint-displacement check (anchored to q0_session)."""
        q0=np.zeros(6)
        b=TravelBudget(q0,np.eye(4),total_budget_m=.05,max_step_m=.002)
        # Fill the first checkpoint.
        for _ in range(25):
            b.reserve(np.zeros(6),np.eye(4),length=.002)
        self.assertAlmostEqual(b.commanded,.05)
        # Re-arm from a small displacement.
        q1=np.ones(6)*.02;p1=np.eye(4);p1[0,3]=.01
        b.reset_origin(q1,p1)
        self.assertEqual(b.checkpoints,1)
        self.assertEqual(b.commanded,0.)
        # Second checkpoint should work.
        b.reserve(q1,p1,length=.002)
        self.assertAlmostEqual(b.commanded,.002)
        # Session joint envelope (1.0 rad) still enforced after re-arm.
        big_q=np.ones(6)*.18  # 1.08 rad total > 1.0 rad limit
        with self.assertRaises(ValueError):b.check(big_q,p1)

if __name__=='__main__':unittest.main()
