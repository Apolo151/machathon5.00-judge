The Machathon 5.00 repository provides all the required tools and utilities for the Machathon 5.00 simulation phase

# Requirements

 a-Using ROS2 humble(Ubuntu 22.04) installation: 
     * [Install ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
 OR
 b-Using ROS2 foxy(Ubuntu 20.04) installation:
     * [Install ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

## Install Gazebo and ROS-Gazebo packages

```bash
$ sudo apt update
$ sudo apt -y install gazebo
$ sudo apt install ros-humble-gazebo-ros-pkgs
$ sudo apt install ros-humble-xacro
```
## Setting up and Building
```bash
$ cd ~/ros2_ws/src
$ git clone https://github.com/Apolo151/machathon5.00 # (if you don’t have git, install it: sudo apt install git)
$ cd ..
$ colcon build --symlink-install
```
## Running
#### To run the scene:
```bash
$ source /usr/share/gazebo/setup.bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch car_demo demo.launch.py
```
#### To control the car via keyboard
##### Open another terminal and run:
```bash
$ sudo su
$ source .bashrc 
$ source /opt/ros/humble/setup.bash
$ source /home/<user_name>/ros2_ws/install/setup.bash
$ ros2 run car_demo prius_teleop_keyboard.py
```
