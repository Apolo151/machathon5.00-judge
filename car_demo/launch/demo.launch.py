#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    urdf_file_name = "prius.urdf"

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(
        get_package_share_directory('car_demo'), "models")
    world_file_name = 'MachathonTrack.world'
    world = os.path.join(get_package_share_directory(
        "car_demo"), 'worlds', world_file_name)
    urdf = os.path.join(
        get_package_share_directory("prius_description"),
        "urdf", urdf_file_name
    )
    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    rviz_path = os.path.join(
        get_package_share_directory("car_demo"),
        "rviz", "ros2.rviz"
    )
    print(f"{rviz_path=}")

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'frame',
            default_value='base_link',
            description='The fixed frame to be used in RViz'),


        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace="prius",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
            arguments=[urdf]
        ),



        Node(
            package="car_demo",
            executable="prius_teleop_keyboard.py",
            name="prius_teleop",
            prefix=["xterm -hold -e"],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world, 'verbose': "true",
                              'extra_gazebo_args': 'verbose'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            launch_arguments={'verbose': "true"}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("car_demo"), "launch", "spawn_prius.launch.py",
                             )
            ),
            launch_arguments={"pose": "1"}.items()
        )
    ])
