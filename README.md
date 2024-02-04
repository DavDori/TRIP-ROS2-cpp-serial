# TRIP-ROS2-interface

This repository contains a ROS2 node designed to interface with an Arduino board as part of a larger project. The node facilitates communication between the ROS 2 framework and the Arduino microcontroller, enabling seamless integration of sensor data, control commands, or any other relevant information between the two systems.

## Requirements
- ROS2 Foxy or later

## Installation

Clone this repository into your ROS2 workspace:

```bash
cd /path/to/your/ros2_workspace/src
git clone https://github.com/DavDori/TRIP-ROS2-interface.git
```

 ## Build the ROS2 workspace

```bash
cd /path/to/your/ros2_workspace
colcon build
```
and source
```bash
source install/setup.bash
```

## How to use it

Depending on the application, you can choose between controlling the motors directly, or take advantage of the unicycle model.

### Direct Motor Control

You can control the velocity set-point for each motor via a ROS message of type `sensor_msgs/JointState` that can be published on the
`/joints_cmd` topic. The command to launch the node is

```
ros2 run trip_interface trip_base
```

### Unicycle Model

To control the robot with forward speed and angular velocity use 

```
ros2 run trip_interface trip_unicycle
```
or
```
ros2 launch trip_interface trip.launch.py
```

In this case, you have to publish a message of type `geometry_msgs/Twist` on the `/cmd_vel` topic to set the deisred velocities.

