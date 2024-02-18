import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

FINAL_X, FINAL_Y, FINAL_Z = 70.508037, -469.2935 , -4.42
# TODO: add submission JSON

# TODO: send request to API

class Checkpoints():
    def __init__(self):
        self.checkpoints_passed = [False] * 3
        self.curr_checkpoint_idx = 0
        # TODO: remove hardcoded values
        self.checkpoints_pos = [[73,-523,-5],[22,-499,-5.2],[-41.45,-467,-5.7]]

    def checkPassedCheckpoints(self, x, y, z):
        if(self.checkpoint_idx < 3):
            if((abs(x - self.checkpoints_pos[self.curr_checkpoint_idx][0])<=6) 
                and (abs(y - self.checkpoints_pos[self.curr_checkpoint_idx][1])<=6) 
                and (abs(z - self.checkpoints_pos[self.curr_checkpoint_idx][2])<=2)):
                self.curr_checkpoint_idx += 1
                print("You passed a checkpoint, GOOD JOB!")

class Timer():
    def __init__(self):
        self.timer_started = False
        self.start_time = 0.0

    def start_timer(self):
        self.start_time = time.time()
        self.timer_started = True
        print("Timer started!")

    def stop_timer(self):
        if self.timer_started:
            end_time = time.time()
            elapsed_time = end_time - self.start_time
            print(f"Elapsed time: {elapsed_time} seconds")
            self.timer_started = False

class Judge(Node):
    def __init__(self):
        self.final_pos = {"x": FINAL_X,"y": FINAL_Y,"z": FINAL_Z}
        self.timer = Timer()
        self.checkpoints = Checkpoints()
        ####
        super().__init__("subscriber_node")
        self.pose_subscriber = self.create_subscription(Odometry,"/prius/odom",self.callback,10)
        self.get_logger().info("Oi mate i'm ros2")
        self.timer.start_timer()


    def callback(self,msg:Odometry):
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        z=msg.pose.pose.position.z
        self.checkpoints.checkPassedCheckpoints(x,y,z)
        ####
        ## check if the car passed all checkpoints, and just passed the finish line
        if((self.checkpoints.checkpoint_idx == 3 and
            (abs(y-self.final_pos['y'])<=6) and
              (abs(z-self.final_pos['z'])<=1) and
                abs(x-self.final_pos['x'])<=6)):
            print("You finished your race!!!!!")
            self.timer.stop_timer()
            # TODO: save first lap time
            # TODO: repwan for second lap
            # TODO: alter to check for second lap finish time
            while 1:
                pass

######
def main():
    rclpy.init()
    node = Judge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()