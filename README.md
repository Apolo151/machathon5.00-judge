# Machathon 5.00
<p align="center">
    <img src="https://github.com/Apolo151/machathon5.00/assets/110634473/90477a3a-5e30-4b0c-a62b-65722f0271ca"
      alt="Picture" 
        width="400" 
        height="200" 
        style="display: block; margin: 0 auto" 
     >
</p>

This repository provides all the required tools and utilities for the Machathon 5.00 simulation phase. A gazebo world that contains the competition vehicle and the track. The car navigates in one direction, followed by navigating in the opposite direction. The time taken to complete each run, known as the lap time, is recorded. <br> After completing the track in both directions, the total lap time is send to the leaderboard if chosen to do so.

## Requirements
* ROS2 installation using any of the following options
  * Using ROS2 humble (Ubuntu 22.04) installation: [Install ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  * Using ROS2 foxy (Ubuntu 20.04) installation: [Install ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)


* Gazebo and ROS-Gazebo packages Installation
```bash
sudo apt update
sudo apt -y install gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro
```
* Python version 3.8 (humble) or 3.10 (foxy)
```bash
## check python version in termainl
python3 --version
```
* keyboard package
```bash
sudo apt install pip
sudo pip install keyboard
```
## Setting up the environment
#### 1. Create a workspace
 ```bash
mkdir workspace_name
```
#### 2. Create src folder inside workspace
 ```bash
cd workspace_name
mkdir src
```
#### 3. Clone the repository inside the src folder
 ```bash
cd src
git clone https://github.com/Apolo151/machathon5.00.git
```
#### 4. Build the workspace
```bash
cd ..
colcon build --symlink-install
```
## To run the scene
- Open two terminal windows
#### 1. Open Gazebo Scene
```bash
cd workspace_name
sudo su
source ~/.bashrc
source install/setup.bash
ros2 launch car_demo demo.launch.py
```
#### 2. Run the Judge and Solution launch file
```bash
sudo su
source ~/.bashrc
source install/setup.bash
ros2 launch car_demo machathon.launch.py
```
#### Running the telep node to control the car via the keyboard (for testing)
- open another terminal and run the following commands
 ```bash
sudo su
source ~/.bashrc
source install/setup.bash
ros2 run car_demo prius_teleop_keyboard.py
```
