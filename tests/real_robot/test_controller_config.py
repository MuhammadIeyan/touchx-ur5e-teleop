from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.controller_config import blended_config
from real_robot.continuous import Continuous
from real_robot.bench_motion import validate_commissioning


def test_only_scaled_position_interpolation_changes():
    source={'scaled_joint_trajectory_controller':{'ros__parameters':{'command_interfaces':['position'], 'constraints':{'goal_time':0.}}},
            'other_controller':{'ros__parameters':{'arbitrary':1}}}
    result=blended_config(source)
    assert result['other_controller']==source['other_controller']
    assert 'open_loop_control' not in source['scaled_joint_trajectory_controller']['ros__parameters']
    assert result['scaled_joint_trajectory_controller']['ros__parameters']['constraints']=={'goal_time':0.}
    assert result['scaled_joint_trajectory_controller']['ros__parameters']['open_loop_control'] is True


@pytest.mark.parametrize('enabled,method,interfaces,valid',[
    (True,'splines',['position'],True), (False,'splines',['position'],False),
    (True,'none',['position'],False), (True,'splines',['velocity'],False)])
def test_refuses_incompatible_live_controller(enabled,method,interfaces,valid):
    future=Future()
    future.set_result(SimpleNamespace(values=[SimpleNamespace(bool_value=enabled),
        SimpleNamespace(string_value=method),SimpleNamespace(string_array_value=interfaces)]))
    obj=SimpleNamespace(controllers=[],blending_verified=False,blending_started=None,blending_future=future,fail=Mock(),log=Mock())
    assert Continuous.verify_blending(obj)==valid
    assert obj.fail.called != valid


def test_persistent_setup_retains_identity_and_commissioning_checks():
    config=dict(payload_verified=True,mounting_configuration_verified=True,
                calibration_match_verified=True,robot_ipv4='192.168.1.103',ros_domain_id=43)
    validate_commissioning(config)  # No timestamp or fabricated operator assertion.
    with pytest.raises(ValueError):validate_commissioning(dict(config,ros_domain_id=42))
    with pytest.raises(ValueError):validate_commissioning(dict(config,payload_verified=False))


def test_rendered_launch_yaml_preserves_numeric_rate_and_empty_prefix(tmp_path):
    import yaml
    from launch import LaunchContext
    from launch_ros.parameter_descriptions import ParameterFile
    from real_robot.controller_config import QuotedDumper
    data={'scaled_joint_trajectory_controller':{'ros__parameters':{
        'update_rate':'$(var update_rate)','tf_prefix':'$(var tf_prefix)',
        'joints':['$(var tf_prefix)shoulder_pan_joint']}}}
    source=tmp_path/'controllers.yaml'
    source.write_text(yaml.dump(data,Dumper=QuotedDumper))
    context=LaunchContext()
    context.launch_configurations.update(update_rate='500',tf_prefix='')
    parameter_file=ParameterFile(str(source),allow_substs=True)
    try:
        rendered=yaml.safe_load(Path(parameter_file.evaluate(context)).read_text())
        params=rendered['scaled_joint_trajectory_controller']['ros__parameters']
        assert type(params['update_rate']) is int
        assert params['update_rate']==500
        assert params['tf_prefix']==''
        assert params['joints']==['shoulder_pan_joint']
    finally:parameter_file.cleanup()


@pytest.mark.parametrize('pending',[False,True])
def test_parameter_check_times_out_without_activation(pending):
    import time
    obj=SimpleNamespace(controllers=[],blending_verified=False,blending_started=time.monotonic()-6.,
                        blending_future=Future() if pending else None,fail=Mock())
    assert Continuous.verify_blending(obj) is False
    assert 'timed out' in obj.fail.call_args.args[0]


def test_vendor_controller_refused_before_activation():
    obj=SimpleNamespace(blending_verified=False,controllers=[SimpleNamespace(
        name='scaled_joint_trajectory_controller',type='ur_controllers/ScaledJointTrajectoryController')],fail=Mock())
    assert Continuous.verify_blending(obj) is False
    assert 'clock bug' in obj.fail.call_args.args[0]
