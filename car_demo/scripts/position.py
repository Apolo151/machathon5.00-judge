
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class MyNode(Node):

    def __init__(self):
        super().__init__("subscriber_node")
        self.get_logger().info("Oi mate i'm ros2")
        self.pose_subscriber = self.create_subscription(Odometry,"/prius/odom",self.callback,10)

    def callback(self,msg:Odometry):
        # self.get_logger().info(str(msg.data))
        print(msg.pose.pose.position.x)
        print(msg.pose.pose.position.y)


def main():
    rclpy.init()


    node = MyNode()
    rclpy.spin(node)


    rclpy.shutdown()

if __name__ == '__main__':
    main()