#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time

class FPSCounter:
    def __init__(self):
        self.frames = []

    def step(self):
        self.frames.append(time.monotonic())

    def get_fps(self):
        n_seconds = 5

        count = 0
        cur_time = time.monotonic()
        for f in self.frames:
            if cur_time - f < n_seconds:  # Count frames in the past n_seconds
                count += 1

        return count / n_seconds

class Camera(Node):

    def __init__(self):
        super().__init__("subscriber_node")
        self.pose_subscriber = self.create_subscription(Image,"/prius/front_camera/image_raw",self.callback,10)
        self.bridge = CvBridge()
        self.fps_counter = FPSCounter()

    def callback(self,msg:Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
        self.fps_counter.step()
        fps = self.fps_counter.get_fps()
        # draw fps on image
        cv2.putText(
            cv_image,
            f"FPS: {fps:.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("prius_front",cv_image)
        cv2.waitKey(5)


def main():
    rclpy.init()

    node = Camera()
    rclpy.spin(node)


    rclpy.shutdown()

if __name__ == '__main__':
    main()