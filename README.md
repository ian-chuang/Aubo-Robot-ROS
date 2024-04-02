# Aubo-Robot-ROS

This repository contains experimental drivers for running the Aubo i5 robot with ROS 2 using Aubo's experimental servoj function.

**Important**: This driver requires libraries provided exclusively by Aubo.

**WARNING:** This implementation is still unstable. **PLEASE TEST IN SIMULATION MODE FIRST**. TO SWITCH TO SIMULATION MODE, NAVIGATE TO THE TEACH/CONTROL WINDOW ON THE AUBO TEACH PENDANT AND SWITCH TO SIMULATED CONTROL. Always have an emergency stop ready to push.

## Tested On:
- Aubo software v4.5.57-a14
- Firmware v3.4.46
- ROS2 Humble
- Ubuntu 22.04

## Installation

1. Source ROS2 Humble:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Clone the repository into your ROS2 workspace:
   ```bash
   cd /path/to/your_ros2_ws/src
   git clone -b humble https://github.com/ian-chuang/dh_ag95_gripper_ros2.git
   ```

3. Add the lib files (.so files) provided by Aubo to the following directories:
   - `aubo_driver/dependent/robotsdk/lib/linux_x64`
   - `aubo_driver/dependent/servojsdk/lib/linux_x64`

4. Install dependencies using rosdep:
   ```bash
   rosdep install -i --from-path . --rosdistro humble -y
   ```

5. Build the package:
   ```bash
   colcon build
   ```

6. Source the setup files:
   ```bash
   source /path/to/your_ros2_ws/install/local_setup.bash
   ```

## Usage
To launch the robot, use the following command with your robot's IP address (XXX.XXX.XXX.XXX). You can find your robot's IP on the teach pendant by going to Settings -> System -> Network and clicking the 'ifconfig' button. This will start the joint trajectory controller, which you can view in `aubo_driver/config`.

NOTE: Without MoveIt, it is difficult to control the robot using the joint trajectory controller. I suggest trying the Cartesian motion controller available at [cartesian_controllers](https://github.com/Soltanilara/cartesian_controllers).

```bash
ros2 launch aubo_driver aubo_i5_control.launch.py robot_ip:=<your robot ip>
```

View the URDF with this command:
```bash
ros2 launch aubo_i5_description view_aubo_i5.launch.py
```

### MoveIt Integration
To control the robot with MoveIt2, try out the `aubo_i5_moveit_config` package:
```bash
ros2 launch aubo_i5_moveit_config aubo_i5_moveit.launch.py robot_ip:=<your robot ip>
```