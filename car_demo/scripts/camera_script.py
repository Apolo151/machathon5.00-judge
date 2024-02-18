#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class MyNode(Node):

    def __init__(self):
        super().__init__("subscriber_node")
        self.pose_subscriber = self.create_subscription(Image,"/prius/front_camera/image_raw",self.callback,10)
        self.bridge = CvBridge()

    def callback(self,msg:Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("prius_front",cv_image)
        cv2.waitKey(5)


def main():
    rclpy.init()


    node = MyNode()
    rclpy.spin(node)


    rclpy.shutdown()

if __name__ == '__main__':
    main()