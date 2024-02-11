The Machathon 5.00 repository provides all the required tools and utilities for the Machathon 5.00 simulation phase.

# Requirements

 a-Using ROS2 humble(Ubuntu 22.04) installation: 
     * [Install ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
 OR
 b-Using ROS2 foxy(Ubuntu 20.04) installation:
     * [Install ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

# Building

```
$ cd ~/ros2_ws/src
$ git clone https://github.com/Apolo151/machathon5.00
$ cd ..
$ colcon build  --packages-up-to car_demo  --symlink-install 
```

# Running

```
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch car_demo demo.launch.py
```
