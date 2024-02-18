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
## Setting up the environment
#### 1. Create a workspace
 ```bash
$ mkdir workspace_name
```
#### 2. Create src folder inside workspace
 ```bash
$ cd workspace_name
$ mkdir src
```
#### 3. Clone the repository inside the src folder
 ```bash
# cd src
$ git clone https://github.com/Apolo151/machathon5.00.git
```
#### 4. Build the workspace
```bash
$ cd ..
$ colcon build --symlink-install
```
## To run the scene 
#### 1. Open Gazebo Scene
```bash
$ cd workspace_name
$ source install/setup.bash
$ source ~/.bashrc
$ ros2 launch car_demo demo.launch.py
```
#### 2. Run ROS2 python code to control the car via the keyboard
1. open another terminal and run the following commands
 ```bash
$ sudo su
$ source ~/.bashrc
$ source install/setup.bash
$ ros2 run car_demo prius_teleop_keyboard.py
```
