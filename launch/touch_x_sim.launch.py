"""Simulation-only Touch X stack; domain 42 is mandatory."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise RuntimeError('Touch X simulation requires ROS_DOMAIN_ID=42')
    root = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false', description='Show RViz simulation scene'),
        DeclareLaunchArgument('execute', default_value='false', description='Enable simulated clutch control'),
        DeclareLaunchArgument('device', default_value='true', description='Start Touch hardware bridge (forces require opt-in)'),
        DeclareLaunchArgument('force_feedback', default_value='false', description='Permit bounded Touch motor output while deadman held'),
        DeclareLaunchArgument('mode', default_value='rate', description='Teleoperation mode: rate (continuous velocity) or position'),
        DeclareLaunchArgument('force_gain', default_value='1.0'),
        DeclareLaunchArgument('force_limit', default_value='0.5'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.06'),
        DeclareLaunchArgument('max_joint_speed', default_value='0.5'),
        DeclareLaunchArgument('rcm', default_value='false', description='Experimental RCM constraint'),
        DeclareLaunchArgument('scale', default_value='0.2', description='Translation scale factor'),
        DeclareLaunchArgument('rot_scale', default_value='1.0', description='Rotation scale factor'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/ur5e_mujoco.launch.py')),
                                 launch_arguments={'mujoco_model': 'ur5e_grasper.xml',
                                                   'headless': LaunchConfiguration('headless'),
                                                   'plugins_file': str(root/'config/touch_x_plugins.yaml')}.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', str(root/'config/touch_x_sim.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
        ExecuteProcess(cmd=['python3', str(root/'scripts/sim_rviz_scene.py')],
                       condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
        ExecuteProcess(cmd=['python3', str(root/'scripts/command_router.py'), '--ros-args',
                            '-p', 'require_deadman:=true'], output='screen'),
        ExecuteProcess(cmd=['python3', str(root/'scripts/touch_x_publisher.py'),
                            '--force-limit', LaunchConfiguration('force_limit'),
                            PythonExpression(["'--enable-force' if '", LaunchConfiguration('force_feedback'), "'.lower() == 'true' else '--read-only'"])],
                        condition=IfCondition(LaunchConfiguration('device')), output='screen'),
        ExecuteProcess(cmd=['python3', str(root/'scripts/touch_x_sim_teleop.py'), '--execute',
                            '--mode', LaunchConfiguration('mode'),
                            '--scale', LaunchConfiguration('scale'),
                            '--force-gain', LaunchConfiguration('force_gain'),
                            '--force-limit', LaunchConfiguration('force_limit'),
                            '--max-linear-speed', LaunchConfiguration('max_linear_speed'),
                            '--max-joint-speed', LaunchConfiguration('max_joint_speed'),
                            '--rot-scale', LaunchConfiguration('rot_scale'),
                            PythonExpression(["'--rcm' if '", LaunchConfiguration('rcm'), "'.lower() == 'true' else '--no-rcm'"])],
                       condition=IfCondition(LaunchConfiguration('execute')), output='screen'),
    ])
