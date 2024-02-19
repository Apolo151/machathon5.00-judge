"""
spawn_prius.py

Script used to spawn a prius in a generic position
"""

import rclpy
from gazebo_msgs.srv import DeleteEntity

def despawn():
    # Start node
    node = rclpy.create_node("entity_despawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(DeleteEntity, "/delete_entity")

    node.get_logger().info("Connecting to `/delete_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")


    # Set data for request
    request = DeleteEntity.Request()
    request.name = "prius"

    node.get_logger().info("Sending service request to `/delete_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    print("Done")
