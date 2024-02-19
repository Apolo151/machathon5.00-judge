#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from prius_msgs.msg import Control

import keyboard
import sys

"""
Reading from the keyboard and Publishing to Prius Control!
---------------------------
Moving around:
         w     
    a    s    d

anything else : neutral gear
CTRL-C to quit
"""

class PriusTeleop(Node):
    def __init__(self, node_name, name_space) -> None:
        super().__init__(node_name, namespace=name_space)
        qos = QoSReliabilityPolicy(1)
        self.pub = self.create_publisher(Control, "control", qos_profile=qos)
        self.status = 0
        self.forward = False
        self.backward = False
        self.left = False
        self.right = False
        self.command = Control()
        self.teleop()

    def vels(self, speed, turn):
        return "currently:\tthrottle %s\tsteering %s " % (speed, turn)

    def teleop(self):
        try:
            while(1):
                print(self.vels(self.command.throttle, self.command.steer))
                if keyboard.is_pressed('w'):
                    self.forward = True
                    self.backward = False
                if keyboard.is_pressed('s'):
                    self.forward = False
                    self.backward = True
                if keyboard.is_pressed('d'):
                    self.right = True
                    self.left = False
                if keyboard.is_pressed('a'):
                    self.left = True
                    self.right = False

                if self.forward:
                    self.command.throttle += 1.0
                    self.command.shift_gears = Control.FORWARD
                    self.command.brake = 0.0
                    self.forward = False
                else:
                    if self.backward:
                        self.command.throttle += 1.0
                        self.command.shift_gears = Control.REVERSE
                        self.command.brake = 0.0
                        self.backward = False
                    else:
                        self.command.throttle = 0.0
                if self.right:
                    self.command.steer -= 0.01
                    self.right = False
                else:
                    if self.left:
                        self.command.steer += 0.01
                        self.left = False
                    else:
                        self.command.steer = 0.0

                if self.command.throttle > 10.0:
                    self.command.throttle = 10.0
                if self.command.steer > 30.0:
                    self.command.steer = 30.0
                if self.command.steer < -30.0:
                    self.command.steer = -30.0
                
                if keyboard.is_pressed('space'):
                    self.command.throttle = 0.0
                    self.command.brake = 5.0
                    
                self.pub.publish(self.command)

        except Exception as e:
            print(e)

        finally:
            self.command.throttle = 0.0
            self.command.steer = 0.0
            self.command.shift_gears = Control.NO_COMMAND
            self.command.brake = 0.0
            self.pub.publish(self.command)

def main(args=None):
    rclpy.init(args=args)
    node = PriusTeleop(node_name="teleop_prius", name_space="prius")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)