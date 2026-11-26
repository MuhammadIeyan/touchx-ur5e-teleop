import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.motion_profile import MotionProfile,duration_nanoseconds
from real_robot.planning import build_plan
from real_robot.smooth_control import Quintic


@pytest.mark.parametrize('speed',[0,-1,50.1,120,float('nan'),float('inf')])
def test_invalid_or_unreviewed_speed_rejected(speed):
    with pytest.raises(ValueError):MotionProfile(speed)


def test_baseline_and_units():
    baseline=MotionProfile()
    assert baseline.duration_s==.3
    assert baseline.endpoint_step_m==.0015
    assert MotionProfile(20).duration_s==.18
    assert MotionProfile(50).speed_m_s==.05
    assert MotionProfile(50).duration_s==.15
    assert MotionProfile(50).endpoint_step_m==.0015
    assert duration_nanoseconds(.18)==180000000
    assert MotionProfile(1).endpoint_step_m < .0015


def model():
    def fk(q):
        pose=np.eye(4);pose[:3,3]=q[:3];return pose
    def plan(q,direction,orientation,step_m):
        end=q.copy();end[:3]+=step_m*direction/np.linalg.norm(direction)
        return end,fk(end)
    return SimpleNamespace(fk=fk,plan=plan,lower=np.ones(6)*-3,upper=np.ones(6)*3)


@pytest.mark.parametrize('speed',[1.,6.,12.,15.,20.,50.])
def test_planned_and_sent_horizons_match_with_bounded_motion(speed):
    robot=model();geometry=SimpleNamespace(curve=lambda curve:None)
    zero=np.zeros(6)
    request=dict(reference=(zero,zero,zero),origin=np.eye(4),target=np.array([.02,0,0]),workspace_m=.05,speed_mm_s=speed)
    result=build_plan(robot,geometry,request)
    duration=result['duration_s']
    assert duration_nanoseconds(duration)/1e9==pytest.approx(duration)
    curve=Quintic(zero,zero,zero,result['end'],duration=duration)
    states=curve.validate(robot,np.zeros(3),np.eye(3),.05,max_cartesian_speed=speed/1000)
    assert max(np.linalg.norm(s[0][:3]) for s in states)<=.002
    assert max(np.linalg.norm(s[1][:3]) for s in states)<=speed/1000+1e-9
    np.testing.assert_allclose(curve.sample(duration)[1:],0,atol=1e-12)


def test_higher_setting_advances_faster_without_larger_endpoints():
    distances=[]
    for speed in (12.,20.):
        robot=model();state=(np.zeros(6),np.zeros(6),np.zeros(6))
        for _ in range(40):
            request=dict(reference=state,origin=np.eye(4),target=np.array([.03,0,0]),workspace_m=.05,speed_mm_s=speed)
            result=build_plan(robot,SimpleNamespace(curve=lambda curve:None),request)
            assert np.linalg.norm(result['end']-state[0])<=.001500001
            state=Quintic(*state,result['end'],duration=result['duration_s']).sample(.05)
        distances.append(state[0][0])
    assert distances[1]>distances[0]*1.2


def high_joint_motion_model():
    robot=model()
    def fk(q):
        pose=np.eye(4);pose[:3,3]=q[:3]/4;return pose
    def plan(q,direction,orientation,step_m):
        end=q.copy();end[:3]+=4*step_m*direction/np.linalg.norm(direction)
        return end,fk(end)
    robot.fk=fk;robot.plan=plan
    return robot


def test_high_speed_retimes_joint_limited_pose_without_relaxing_bounds():
    robot=high_joint_motion_model();zero=np.zeros(6)
    request=dict(reference=(zero,zero,zero),origin=np.eye(4),target=np.array([.02,0,0]),workspace_m=.05,speed_mm_s=50.)
    checked=[]
    result=build_plan(robot,SimpleNamespace(curve=checked.append),request)
    assert result['duration_s'] > MotionProfile(50).duration_s
    assert result['duration_s'] <= .5
    assert len(checked)==1
    curve=checked[0]
    assert curve.duration==result['duration_s']
    curve.validate(robot,np.zeros(3),np.eye(3),.05,max_cartesian_speed=.05)
    np.testing.assert_allclose(robot.fk(result['end'])[:3,3],[.0015,0,0])


def test_unrecoverable_joint_dynamics_still_fail_closed():
    robot=high_joint_motion_model();zero=np.zeros(6)
    request=dict(reference=(zero,np.ones(6)*.09,zero),origin=np.eye(4),target=np.array([.02,0,0]),workspace_m=.05,speed_mm_s=50.)
    with pytest.raises(ValueError,match='Joint rate/acceleration limit'):
        build_plan(robot,SimpleNamespace(curve=lambda _:pytest.fail('Invalid curve reached geometry')),request)


def test_retimed_plan_still_checks_collision_geometry():
    robot=high_joint_motion_model();zero=np.zeros(6)
    request=dict(reference=(zero,zero,zero),origin=np.eye(4),target=np.array([.02,0,0]),workspace_m=.05,speed_mm_s=50.)
    def collision(curve):raise ValueError('collision')
    with pytest.raises(ValueError,match='collision'):
        build_plan(robot,SimpleNamespace(curve=collision),request)
