import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.planning import rejection


def request():
    return dict(reference=(np.zeros(6),np.zeros(6),np.zeros(6)),reference_at=10.,
                generation=3,target=np.zeros(3))


def test_new_feedback_cannot_make_old_plan_fresh():
    assert rejection(request(),3,10.11,np.zeros(3),np.zeros(6),10.11)=='plan expired'


def test_release_and_reengage_never_reuses_previous_hold_plan():
    assert rejection(request(),4,10.02,np.zeros(3),np.zeros(6),10.03)=='engagement changed'


def test_fresh_plan_needs_current_robot_and_hand_agreement():
    assert rejection(request(),3,10.02,np.zeros(3),np.zeros(6),10.03) is None
    assert rejection(request(),3,10.02,np.ones(3)*.01,np.zeros(6),10.03)=='hand target changed'
    assert rejection(request(),3,10.02,np.zeros(3),np.ones(6)*.02,10.03)=='robot moved beyond planned reference'
