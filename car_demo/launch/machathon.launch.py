from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_demo',
            executable='solution.py'
        ),
        Node(
            package='car_demo',
            executable='judge.py'
        )
    ])