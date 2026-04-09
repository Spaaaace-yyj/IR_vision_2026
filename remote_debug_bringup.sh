#!/bin/bash

source /opt/ros/humble/setup.bash
source ./install/setup.bash
export GDK_BACKEND=x11
export DISPLAY=$DISPLAY
ros2 launch car_bringup car_bringup.launch.py