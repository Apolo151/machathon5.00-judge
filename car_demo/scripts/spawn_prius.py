"""
spawn_prius.py

Script used to spawn a prius in a generic position
"""

import rclpy
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory

def spawn(x,y,z,x_angle,y_angle,z_angle):
    node = rclpy.create_node("entity_spawner")
    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Get path to the prius 
    package_share_directory = get_package_share_directory('prius_description')
    sdf_file_path = package_share_directory + "/urdf/prius.urdf"
    # print(package_share_directory)
    # sdf_file_path = "../../prius_description/urdf/prius.urdf"

    # Set data for request
    request = SpawnEntity.Request()
    request.name = "prius"
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = "prius"
    request.initial_pose.position.x = float(x)
    request.initial_pose.position.y = float(y)
    request.initial_pose.position.z = float(z)
    request.initial_pose.orientation.x = float(x_angle)
    request.initial_pose.orientation.y = float(y_angle)
    request.initial_pose.orientation.z = float(z_angle)


    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.destroy_node()
