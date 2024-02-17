
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time



class MyNode(Node):

    def start_timer(self):
        self.start_time = time.time()
        self.timer_started = True
        print("Timer started!")
        pass

    def stop_timer(self):
        if self.timer_started:
            end_time = time.time()
            elapsed_time = end_time - self.start_time
            print(f"Elapsed time: {elapsed_time} seconds")
            self.timer_started = False


    def __init__(self):
        self.timer_started = False 
        self.start_time = 0.0
        self.start_timer()
        self.xF = 70.508037
        self.yF= -469.2935
        self.zF= -4.42
        self.checkpoints = [False] * 3
        self.checkpointIndex = 0
        #To do (not me)
        self.checkpointpositions = [[73,-523,-5],[22,-499,-5.2],[-41.45,-467,-5.7]]
        super().__init__("subscriber_node")
        self.get_logger().info("Oi mate i'm ros2")
        self.pose_subscriber = self.create_subscription(Odometry,"/prius/odom",self.callback,10)

    def callback(self,msg:Odometry):
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        z=msg.pose.pose.position.z

        if(self.checkpointIndex <3):
            if((abs(x - self.checkpointpositions[self.checkpointIndex][0])<=6) and (abs(y - self.checkpointpositions[self.checkpointIndex][1])<=6) and (abs(z - self.checkpointpositions[self.checkpointIndex][2])<=2)):
                self.checkpointIndex += 1
                print("GOOD JOOOOOOOOOOOOOOOOOOOOOOOOOB")




        if((abs(x-self.xF)<=6) and (abs(y-self.yF)<=6) and (abs(z-self.zF)<=1) and self.checkpointIndex == 3):
            print("HEEEEEEYOOOOOOOOOOOO")
            self.stop_timer()
            while 1:
                pass





def main():
    rclpy.init()


    node = MyNode()
    rclpy.spin(node)


    rclpy.shutdown()

if __name__ == '__main__':
    main()