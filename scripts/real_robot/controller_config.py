"""Generate the physical driver's config from its installed vendor defaults."""
import copy
from pathlib import Path
import sys
import yaml
from ament_index_python.packages import get_package_share_directory


class QuotedDumper(yaml.SafeDumper):
    pass


def str_representer(dumper, data):
    # Launch substitutes before ROS parses YAML. This placeholder must become
    # an integer (500), whereas an empty tf_prefix must remain a string ("").
    if data == '$(var update_rate)':
        return dumper.represent_scalar('tag:yaml.org,2002:str', data)
    if '$(' in data:
        return dumper.represent_scalar('tag:yaml.org,2002:str', data, style='"')
    return dumper.represent_scalar('tag:yaml.org,2002:str', data)


QuotedDumper.add_representer(str, str_representer)


def blended_config(vendor):
    config = copy.deepcopy(vendor)
    params = config['scaled_joint_trajectory_controller']['ros__parameters']
    if params['command_interfaces'] != ['position']:
        raise ValueError('Expected position-only UR trajectory controller')
    # For a position-only controller this selects the last commanded q/v/a as
    # the interpolation anchor. Robot servo feedback and tracking checks remain.
    params['open_loop_control'] = True
    params['interpolation_method'] = 'splines'
    if 'controller_manager' in config:
        config['controller_manager']['ros__parameters']['scaled_joint_trajectory_controller']['type'] = (
            'ur5e_clock_fixed_controller/ScaledJointTrajectoryController')
    return config


if __name__ == '__main__':
    source = Path(get_package_share_directory('ur_robot_driver'))/'config/ur_controllers.yaml'
    output = Path(sys.argv[1])
    output.write_text(yaml.dump(blended_config(yaml.safe_load(source.read_text())), Dumper=QuotedDumper, sort_keys=False))

