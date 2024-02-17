import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    urdf_file_name = "prius.urdf"
    urdf_path = os.path.join(
        get_package_share_directory("prius_description"),
        "urdf", urdf_file_name
    )

    poses = {
        "0": {"x": 0.0, "y": 0.0, "z": 0.5, "Z": 0.0},
        "1": {"x": 73.66, "y": -470.31, "z": -4, "Z": 0.5},
    }
    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    pose = LaunchConfiguration("pose", default="0")
    
    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_pose_cmd = DeclareLaunchArgument(
        "pose", default_value="0", description="Spawn Pose"
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf_path,
            '-x', TextSubstitution(text=str(poses["1"]["x"])),
            '-y', TextSubstitution(text=str(poses["1"]["y"])),
            '-z', TextSubstitution(text=str(poses["1"]["z"])),
            # '-x', x_pose,
            # '-y', y_pose,
            # '-z', 0.5,
            "-entity", "prius"
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options for file
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_pose_cmd)
    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld