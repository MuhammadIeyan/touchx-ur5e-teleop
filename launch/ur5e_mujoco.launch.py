import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable


def generate_launch_description():
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise RuntimeError('MuJoCo simulation requires ROS_DOMAIN_ID=42')
    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    mujoco_model_arg = DeclareLaunchArgument(
        "mujoco_model",
        default_value="ur5e.xml",
        description="MuJoCo XML model filename"
    )

    mujoco_model = LaunchConfiguration("mujoco_model")


    root = Path(__file__).resolve().parents[1]
    urdf_file = str(root / 'description/ur5e_mujoco.urdf.xacro')
    controllers_file = str(root / 'config/controllers.yaml')
    plugins_file = str(root / 'config/mujoco_ros2_control_plugins.yaml')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
            " ",
            "mujoco_model:=",
            str(root / 'models/mujoco_menagerie/universal_robots_ur5e') + '/' ,
            mujoco_model,
            ' headless:=', LaunchConfiguration('headless')
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]
    )

    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controllers_file,
            LaunchConfiguration('plugins_file'),
            {"use_sim_time": True}
        ],
        remappings=[
            ("~/robot_description", "/robot_description")
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file
        ],
        output="screen"
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_file
        ],
        output="screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument('plugins_file', default_value=plugins_file),
        headless_arg,
        mujoco_model_arg,
        robot_state_publisher,
        mujoco_node,
        joint_state_broadcaster_spawner,
        position_controller_spawner
    ])
