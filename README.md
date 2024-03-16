# Aubo-Robot-ROS

This repository contains experimental drivers for running Aubo i5 with ROS using Aubo's experimental servoj function. Please contact Aubo for the servoj C++ binaries.

**WARNING:** This implementation is still unstable. **PLEASE TEST IN SIMULATION MODE FIRST**. TO SWITCH TO SIMULATION MODE, NAVIGATE TO TEACH/CONTROL WINDOW ON AUBO TEACH PENDANT AND SWITCH TO SIMULATED CONTROL. Always have an emergency stop ready to push. When running servoj, the robot frequently hits a joint acceleration limit and throws an error. Work is ongoing to apply joint acceleration limits to the hardware interface.

## Requirements
- Aubo software v4.5.57-a14
- Firmware v3.4.46
- ROS Noetic

## Features
- Updated Aubo i5 URDF with color and improved accuracy
- MoveIt configuration package included

## Issues and Workarounds
- There are issues with the servoj C++ binaries, causing a seg fault when loading URDF objects in C++.
- Modified ROS controllers are available in the provided forked repositories.

## Installation
1. Clone this repository into your ROS Noetic workspace:
   
   ```bash
   cd ~/your_ws/src
   git clone https://github.com/ian-chuang/Aubo-Robot-ROS.git
   ```

2. Add the lib files to the following directories:
   - `aubo_ros_control/dependent/robotsdk/lib/linux_x64`
   - `aubo_ros_control/dependent/servojsdk/lib/linux_x64`

3. If you want to use Cartesian control or joint trajectory controller, clone these as well:
    ```bash
    git clone -b aubo https://github.com/ian-chuang/cartesian_controllers.git 
    git clone -b aubo https://github.com/ian-chuang/ros_controllers 
    ```

4. Install required dependencies for building:
    ```bash
    sudo apt install ros-noetic-catkin python3-catkin-tools
    cd ~/your_ws/src
    rosdep install -y --from-paths . --ignore-src --rosdistro noetic
    ```

## Usage
To launch the robot, use the following command with your robot's IP address:
```bash
roslaunch aubo_ros_control aubo_bringup.launch robot_ip:=<your robot ip>
```

Try out different controllers by modifying the launch file at `aubo_ros_control/launch/aubo_bringup.launch`.

### MoveIt Integration
Switch your controller to use MoveIt:
```xml
<arg name="controllers" default="joint_state_controller pos_joint_traj_controller"/>
```
Then enable the MoveIt plugin within RViz.

### Cartesian Motion Control
Switch your controller to use Cartesian motion control:
```xml
<arg name="controllers" default="joint_state_controller cartesian_motion_controller motion_control_handle"/>
```
Then enable the Robot Model and Interactive Marker plugin within RViz.