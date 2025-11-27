#!/bin/bash
source /opt/ros/humble/setup.bash
# Source ROS 2 setup file
source /home/wl/autorun/iiri-ros/install/setup.bash

# Launch the ROS 2 node
ros2 launch bringup qr_raspi.launch.py
