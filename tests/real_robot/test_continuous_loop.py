"""Offline closed-loop action replacement; no DDS or hardware connection."""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace,MethodType
from unittest.mock import Mock,patch
import sys
import unittest
import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
NOMINAL_KINEMATICS=Path(get_package_share_directory("ur_description"))/"config/ur5e/default_kinematics.yaml"
NOMINAL_HASH=yaml.safe_load(NOMINAL_KINEMATICS.read_text())["kinematics"]["hash"]
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.continuous import Continuous,CONTROLLER
from real_robot.planning import build_plan
from real_robot.smooth_control import Quintic,ActionWindow,HoldGesture
from real_robot.bench_motion import Model
from touch_clutch import Clutch


class LoopTests(unittest.TestCase):
    def test_one_hold_tracks_many_small_movements_and_release_stops(self):
        self.run_tracking(0.,0.,1.)

    def test_delayed_feedback_delivery_and_speed_scaling(self):
        for scaling in (1.,.2):
            with self.subTest(scaling=scaling):self.run_tracking(.04,.02,scaling)

    def test_worker_delay_does_not_block_release_or_reengagement(self):
        self.run_tracking(0.,0.,1.,planner_delay=.06)

    def test_expired_worker_results_never_command_motion(self):
        self.run_tracking(0.,0.,1.,planner_delay=.12,expected_fault='Three planning deadlines missed')

    def test_hung_worker_stops_without_commanding_motion(self):
        self.run_tracking(0.,0.,1.,planner_delay=.6,expected_fault='Planner worker stalled')

    def run_tracking(self,feedback_delay,delivery_delay,scaling,planner_delay=0.,expected_fault=None):
        root=Path(__file__).resolve().parents[2]
        xml=xacro.process_file('/opt/ros/humble/share/ur_description/urdf/ur.urdf.xacro',mappings={
            'ur_type':'ur5e','name':'ur','kinematics_params':str(NOMINAL_KINEMATICS)}).toxml()
        model=Model(xml,NOMINAL_HASH)
        q=np.array([.0073,-1.566,-1.4766,-1.5676,-1.3114,-.631])
        origin=model.fk(q);clock=[0.];world=SimpleNamespace(curve=None,start=0.,handle=None)
        sent=[];faults=[];continuity=[];queued=[];history=[];planned=[]
        class Handle:
            accepted=True
            def __init__(self):self.result=Future()
            def get_result_async(self):return self.result
            def cancel_goal_async(self):
                if not self.result.done():
                    self.result.set_result(SimpleNamespace(status=5,result=SimpleNamespace(error_code=0)))
                if world.handle is self:
                    world.curve=None
                    obj.reference=(obj.q.copy(),np.zeros(6),np.zeros(6))
                f=Future();f.set_result(None);return f
        def deliver(goal,future):
            self.assertEqual(len(goal.trajectory.points),1)
            endpoint=goal.trajectory.points[0]
            self.assertGreater(endpoint.time_from_start.nanosec,0)
            # The controller anchors at handoff, not the publisher's older feedback.
            prior=(world.curve.sample((clock[0]-world.start)*scaling) if world.curve is not None
                   else (obj.q.copy(),np.zeros(6),np.zeros(6)))
            if world.handle is not None and not world.handle.result.done():
                world.handle.result.set_result(SimpleNamespace(status=5,result=SimpleNamespace(error_code=0)))
            world.curve=Quintic(*prior,endpoint.positions)
            continuity.append(max(np.max(np.abs(prior[i]-world.curve.sample(0)[i])) for i in range(3)))
            world.start=clock[0];world.handle=Handle()
            future.set_result(world.handle)
        def send(goal):
            sent.append(goal);future=Future()
            if delivery_delay:queued.append((clock[0]+delivery_delay,goal,future))
            else:deliver(goal,future)
            return future
        obj=SimpleNamespace(closing=False,heartbeat=lambda:True,ready=lambda:None,log=Mock(),status=Mock(),
            model=model,q=q.copy(),tcp=origin.copy(),geometry=SimpleNamespace(check=Mock(),curve=Mock()),
            verify_blending=lambda:True,at_boundary=False,origin=origin.copy(),q_origin=q.copy(),workspace_m=.05,window=ActionWindow(),gesture=HoldGesture(),
            last_send=-1.,cancelled=set(),reference=(q.copy(),np.zeros(6),np.zeros(6)),reference_at=0.,
            reference_stamp=-1,touch=SimpleNamespace(axes=[0.,0.,0.,0.,0.,0.,1.],buttons=[0,0,1,0]),
            controllers=[SimpleNamespace(name=CONTROLLER,state='active')],activated_here=True,
            clutch=Clutch(scale=.5,rot_scale=0),action=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=send),
            plan_generation=0,expired_plans=0,planning=None,planner_notify=SimpleNamespace(trigger=Mock()),
            fault=None,fail=lambda reason:faults.append(reason))
        def submit(function,request):
            future=Future()
            try:result=build_plan(model,obj.geometry,request)
            except Exception as error:future.set_exception(error);return future
            if planner_delay:planned.append((clock[0]+planner_delay,future,result))
            else:future.set_result(result)
            return future
        obj.planner=SimpleNamespace(submit=submit)
        for name in ('planner_finished','stop_stream','cancel_all','accept_stream','finish_stream'):
            setattr(obj,name,MethodType(getattr(Continuous,name),obj))
        motion=[];second_hold_goals=0;released_goals=0
        with patch('time.monotonic',side_effect=lambda:clock[0]):
            for i in range(400):
                clock[0]=i*.02
                if world.curve is not None:
                    current=world.curve.sample((clock[0]-world.start)*scaling)
                    obj.q=current[0].copy()
                else:current=(obj.q.copy(),np.zeros(6),np.zeros(6))
                history.append((clock[0],current))
                old=[entry for entry in history if entry[0]<=clock[0]-feedback_delay+1e-9]
                obj.reference=(old[-1][1] if old else history[0][1])
                while queued and queued[0][0]<=clock[0]+1e-9:
                    _,goal,future=queued.pop(0);deliver(goal,future)
                obj.reference_at=clock[0]
                obj.touch.buttons[0]=int(.1<=clock[0]<4.5 or 5.2<=clock[0]<7.5)
                obj.touch.axes[0]=(max(0.,min(.012,(clock[0]-.5)*.003)) if clock[0]<4.5
                                   else max(0.,(clock[0]-5.6)*.003))
                while planned and planned[0][0]<=clock[0]+1e-9:
                    _,future,result=planned.pop(0);future.set_result(result)
                before=len(sent)
                Continuous.tick(obj)
                if 5.5<clock[0]<7.5:second_hold_goals+=len(sent)-before
                if not obj.touch.buttons[0]:released_goals+=len(sent)-before
                motion.append(model.fk(obj.q)[:3,3])
                if faults:break
        if expected_fault:
            self.assertTrue(faults)
            self.assertIn(expected_fault,faults[0])
            self.assertEqual(sent,[])
            return
        self.assertEqual(faults,[])
        self.assertGreater(len(sent),40)  # Many replacements during ONE uninterrupted hold.
        self.assertGreater(second_hold_goals,15)
        self.assertEqual(released_goals,0)
        self.assertTrue(obj.window.drained())
        self.assertLess(max(continuity),1e-9)
        displacement=np.linalg.norm(motion[-1]-origin[:3,3])
        self.assertGreater(displacement,.003 if scaling==1. else .0001)
        self.assertLess(displacement,.012)
        for goal in sent:
            self.assertEqual(list(goal.trajectory.points[-1].velocities),[0.]*6)


if __name__=='__main__':unittest.main()
