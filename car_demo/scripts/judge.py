import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
import spawn_prius
import despawn_prius
import json

FINAL_X, FINAL_Y, FINAL_Z = 70.508037, -469.2935 , -4.42
# TODO: add submission JSON

# TODO: send request to API

class Checkpoints():
    def __init__(self):
        self.checkpoints_passed = [False] * 3
        self.curr_checkpoint_idx = 0
        # TODO: remove hardcoded values
        self.checkpoints_pos = [[73,-523,-5],[22,-499,-5.2],[-41.45,-467,-5.7]]

    def checkPassedCheckpoints(self, x, y, z,lap_completed):    
     if lap_completed == 0 :
        if(self.curr_checkpoint_idx < 3):
            if((abs(x - self.checkpoints_pos[self.curr_checkpoint_idx][0])<=6) 
                and (abs(y - self.checkpoints_pos[self.curr_checkpoint_idx][1])<=6) 
                and (abs(z - self.checkpoints_pos[self.curr_checkpoint_idx][2])<=2)):
                self.curr_checkpoint_idx += 1
                print("You passed a checkpoint, GOOD JOB!")
     if lap_completed == 1:
         
         if(self.curr_checkpoint_idx >= 0):
            if((abs(x - self.checkpoints_pos[self.curr_checkpoint_idx][0])<=6) 
                and (abs(y - self.checkpoints_pos[self.curr_checkpoint_idx][1])<=6) 
                and (abs(z - self.checkpoints_pos[self.curr_checkpoint_idx][2])<=2)):
                self.curr_checkpoint_idx -= 1
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
            #Will return Elapsed
            return elapsed_time

class Judge(Node):
    def __init__(self,Team_Name:str):
        despawn_prius.despawn()
        spawn_prius.spawn(72.71,-470.1,-5,-0.007621,0.025542,-0.135)
        self.final_pos = {"x": FINAL_X,"y": FINAL_Y,"z": FINAL_Z}
        self.timer = Timer()
        self.checkpoints = Checkpoints()
        ####
        super().__init__("Judge")
        self.pose_subscriber = self.create_subscription(Odometry,"/prius/odom",self.callback,10)
        self.timer.start_timer()
        #New Added By Jannah
        self.lap_completed = 0
        self.lapTime =[0,0]
        self.submission_data = {
            "team_name": Team_Name,
            "lap1_time": None,
            "lap2_time": None,
            "time_of_submission": None
        }


        



    def callback(self,msg:Odometry):
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        z=msg.pose.pose.position.z
        self.checkpoints.checkPassedCheckpoints(x,y,z,self.lap_completed)
        ####
        ## check if the car passed all checkpoints, and just passed the finish line
        if(((self.checkpoints.curr_checkpoint_idx == 3 or self.checkpoints.curr_checkpoint_idx == -1 ) and
            (abs(y-self.final_pos['y'])<=6) and
              (abs(z-self.final_pos['z'])<=1) and
                abs(x-self.final_pos['x'])<=6)) and self.lap_completed < 2:
            
            print("You finished your race!!!!!")
            self.lapTime[self.lap_completed] = self.timer.stop_timer()
            self.lap_completed += 1
            if self.lap_completed == 1 :
                #remove the car from the scene and then add it again in the new start position
                despawn_prius.despawn()
                self.pose_subscriber.destroy()
                spawn_prius.spawn(67.167259,-468.920073,-4.9,-0.000198,0.00,12)
                #restart the subscriber to prevent glitches
                self.pose_subscriber = self.create_subscription(Odometry,"/prius/odom",self.callback,10)
                self.timer.start_timer()
                self.checkpoints.curr_checkpoint_idx -=1



            if self.lap_completed == 2:
             self.submission_data["lap1_time"] = self.lapTime[0]
             self.submission_data["lap2_time"] = self.lapTime[1]
             self.submission_data["time_of_submission"] = time.strftime("%Y-%m-%d %H:%M:%S")
            # Convert submission data to JSON format
             submission_json = json.dumps(self.submission_data)
             print(submission_json)


######
def main():
    rclpy.init()
    node = Judge("Team Name")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()