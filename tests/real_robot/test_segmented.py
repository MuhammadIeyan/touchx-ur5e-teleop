import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.segmented import Segmented,CONTROLLER
from real_robot.bench_motion import GoalLease
from real_robot.travel_budget import TravelBudget
from touch_clutch import Clutch

class SegmentedTests(unittest.TestCase):
    def test_only_one_goal_outstanding(self):
        p=np.eye(4);planned=p.copy();planned[0,3]=.0001
        clutch=Clutch(scale=.5,rot_scale=0)
        clutch.update([0,0,0],[0,0],True,1,[0,0,0])
        clutch.update([0,0,0],[1,0],True,1,[0,0,0])
        pending=SimpleNamespace(add_done_callback=Mock())
        action=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=Mock(return_value=pending))
        obj=SimpleNamespace(closing=False,heartbeat=lambda:True,ready=lambda:None,log=Mock(),
            q=np.zeros(6),tcp=p,touch=SimpleNamespace(buttons=[1,0,1,1],axes=[.01,0,0,0,0,0,1]),
            budget=TravelBudget(np.zeros(6),p),model=SimpleNamespace(fk=lambda q:p,plan=lambda *a,**k:(np.ones(6)*.0001,planned),names=['a','b','c','d','e','f']),
            geometry=SimpleNamespace(segment=Mock()),lease=GoalLease(),settle_until=0.,
            controllers=[SimpleNamespace(name=CONTROLLER,state='active')],activated_here=True,
            clutch=clutch,stationary=True,action=action,accepted=Mock(),fail=Mock())
        Segmented.tick(obj);Segmented.tick(obj)
        action.send_goal_async.assert_called_once()
        self.assertAlmostEqual(obj.budget.commanded,.0001)
        self.assertEqual(list(action.send_goal_async.call_args.args[0].trajectory.points[-1].velocities),[0.]*6)

    def test_cancel_preserves_budget_and_requires_new_release(self):
        clutch=Clutch();clutch.active=True;clutch.released=True
        lease=GoalLease();lease.reserve();lease.stop()
        obj=SimpleNamespace(lease=lease,budget=SimpleNamespace(commanded=.023),clutch=clutch,
                            fault=None,log=Mock(),fail=Mock())
        result=SimpleNamespace(status=5,result=SimpleNamespace(error_code=0))
        Segmented.finished(obj,SimpleNamespace(result=lambda:result))
        self.assertEqual(obj.budget.commanded,.023)
        self.assertFalse(obj.clutch.released)
        self.assertFalse(obj.lease.sent)

    def test_faulted_cancel_does_not_rearm(self):
        lease=GoalLease();lease.reserve();lease.stop()
        obj=SimpleNamespace(lease=lease,budget=SimpleNamespace(commanded=.001),fault='stale',log=Mock())
        Segmented.finished(obj,SimpleNamespace(result=lambda:SimpleNamespace(status=5,result=SimpleNamespace(error_code=0))))
        self.assertTrue(obj.done);self.assertTrue(obj.lease.terminal)

if __name__=='__main__':unittest.main()
