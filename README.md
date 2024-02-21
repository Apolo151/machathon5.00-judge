# Machathon 5.00
<p align="center">
  <a style="text-decoration:none" >
    <img src="https://img.shields.io/badge/Code-Python-blue?logo=python" alt="Website" />
  </a>
  <a style="text-decoration:none" >
    <img src="https://img.shields.io/badge/Track Design-Blender-orange?logo=Blender" alt="Website" />
  </a>
  <a style="text-decoration:none" >
    <img src="https://img.shields.io/badge/Simulator-Gazebo-red" alt="Website" />
  </a>
</p>

<p align="center">
    <img src="https://github.com/Apolo151/machathon5.00/assets/110634473/90477a3a-5e30-4b0c-a62b-65722f0271ca"
      alt="Picture" 
        width="400" 
        height="200" 
        style="display: block; margin: 0 auto" 
     >
</p>

This repository provides all the required tools and utilities for the Machathon 5.00 simulation phase. A gazebo world that contains the competition vehicle and the track. The car navigates in one direction, followed by navigating in the opposite direction. The time taken to complete each run, known as the lap time, is recorded. <br> After completing the track in both directions, the total lap time is send to the leaderboard if chosen to do so.

#### Links
- [Rules Book](https://drive.google.com/file/d/1XvMcuaJ13R88KsatR51eGoJwgwkqPXrs/view?usp=sharing)
- [Leaderboard](https://stp-frontend-leaderboard.onrender.com/)

## Requirements
* ROS2 installation using any of the following options
  * Using ROS2 humble (Ubuntu 22.04) installation: [Install ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  * Using ROS2 foxy (Ubuntu 20.04) installation: [Install ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)


* Gazebo and ROS-Gazebo packages Installation
```bash
sudo apt update
sudo apt -y install gazebo
## if you are using ros2-humble
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro
## if you are using ros2-foxy
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-xacro
```
* Python version 3.8 (humble) or 3.10 (foxy)
```bash
## check python version in terminal
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

## Project Hierarchy

```bash
machathon5.00-judge
├── car_demo
│   ├── car_demo
│   ├── CMakeLists.txt
│   ├── env-hooks
│   ├── launch
│   │   ├── demo.launch.py # launches gazebo and track scene
│   │   ├── machathon.launch.py # launches judge and solution nodes
│   │   └── spawn_prius.launch.py
│   ├── models
│   ├── package.xml
│   ├── plugins
│   ├── scripts
│   │   ├── data.py # teams data config
│   │   ├── despawn_prius.py
│   │   ├── judge.py # judge node
│   │   ├── prius_teleop_keyboard.py # keyboard telep node
│   │   ├── solution.py # your solution file
│   │   └── spawn_prius.py
│   └── worlds
│       └── MachathonTrack.world # The Gazebo track scene
├── prius_description # car's description
├── prius_msgs
└── README.md

```

## Attribution
<p align="center">
  <img src="https://user-images.githubusercontent.com/59095993/218258481-82b37fcf-10ad-4a2f-99d0-555e5610b6f2.png" width=100 height=100 alt="STP logo">
</p>
This code was developed and maintained by Step Towards Progress STP, a non-profit organization focuses on developing youth in various fields personally and technically through academic programs, projects, and events. You can find more information about STP at https://stp-org.com/

----
This code was heavily based on [osrf car_demo](https://github.com/osrf/car_demo) and [NovoG93 car_demo](https://github.com/NovoG93/car_demo), we wholeheartedly thank them for their invaluable contributions and open source code.

