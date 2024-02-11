#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from prius_msgs.msg import Control

import sys
import select
import termios
import tty

msg = """
Reading from the keyboard and Publishing to Prius Control!
---------------------------
Moving around:
   u    i    o
        k    
   m    ,    .

anything else : neutral gear

q/z : increase/decrease throttle & steering by 10%
w/x : increase/decrease only throttle by 10%
e/c : increase/decrease only steering by 10%

CTRL-C to quit
"""


class PriusTeleop(Node):
    def __init__(self, node_name, name_space) -> None:
        super().__init__(node_name, namespace=name_space)
        qos = QoSReliabilityPolicy(1)
        self.pub = self.create_publisher(Control, "control", qos_profile=qos)
        self.settings = termios.tcgetattr(sys.stdin)

        self.speed = self.get_parameter_or("speed", 0.5)
        self.turn = self.get_parameter_or("turn", 1.0)
        self.status = 0
        self.command = Control()

        self.speedBindings = {
            'q': (1.1, 1.1),
            'z': (.9, .9),
            'w': (1.1, 1),
            'x': (.9, 1),
            'e': (1, 1.1),
            'c': (1, .9),
        }
        self.teleop()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tthrottle %s\tsteering %s " % (speed, turn)

    def teleop(self):
        global msg
        try:
            
            print(self.vels(self.speed, self.turn))
            while(1):
                self.get_logger().info(str(self.speed))
                # command = Control()
                key = self.getKey()
                # key == 'i': throttle +, steering 0, gear FORWARD, braking 0
                if key == 'i':
                    self.command.throttle = self.speed
                    self.command.steer = 0.0
                    self.command.shift_gears = Control.FORWARD
                    self.command.brake = 0.0
                # key == 'u': throttle +, steering +, gear FORWARD, braking 0
                elif key == 'u':
                    self.command.throttle = self.speed
                    self.command.steer = self.turn
                    self.command.shift_gears = Control.FORWARD
                    self.command.brake = 0.0
                # key == 'o': throttle +, steering -, gear FORWARD, braking 0
                elif key == 'o':
                    self.command.throttle = self.speed
                    self.command.steer = -self.turn
                    self.command.shift_gears = Control.FORWARD
                    self.command.brake = 0.0
                # key == 'k': throttle 0, steering 0, gear NEUTRAL, braking +
                elif key == 'k':
                    self.command.throttle = 0.0
                    self.command.steer = 0.0
                    self.command.shift_gears = Control.NEUTRAL
                    self.command.brake = 5.0
                # key == ',': throttle +, steering 0, gear REVERSE, braking 0
                elif key == ',':
                    self.command.throttle = self.speed
                    self.command.steer = 0.0
                    self.command.shift_gears = Control.REVERSE
                    self.command.brake = 0.0
                # key == 'm': throttle +, steering +, gear REVERSE, braking 0
                elif key == 'm':
                    self.command.throttle = self.speed
                    self.command.steer = self.turn
                    self.command.shift_gears = Control.REVERSE
                    self.command.brake = 0.0
                # key == '.': throttle +, steering -, gear REVERSE, braking 0
                elif key == '.':
                    self.command.throttle = self.speed
                    self.command.steer = -self.turn
                    self.command.shift_gears = Control.REVERSE
                    self.command.brake = 0.0
                # key == 'q'/'z'/'w'/'x'/'e'/'c'
                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]
                    print(self.vels(self.speed, self.turn))
                    if (self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15
                # else: : throttle 0, steering 0, gear NEUTRAL, braking 0
                else:
                    self.command.throttle = 0.0
                    self.command.steer = 0.0
                    self.command.shift_gears = Control.NO_COMMAND
                    self.command.brake = 0.0
                    if (key == '\x03'):
                        break
                self.pub.publish(self.command)

        except Exception as e:
            print(e)

        finally:
            self.command.throttle = 0.0
            self.command.steer = 0.0
            self.command.shift_gears = Control.NO_COMMAND
            self.command.brake = 0.0
            self.pub.publish(self.command)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    node = PriusTeleop(node_name="teleop_prius", name_space="prius")

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
