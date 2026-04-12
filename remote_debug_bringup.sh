#!/bin/bash

source /opt/ros/humble/setup.bash
source ./install/setup.bash
#export GDK_BACKEND=x11
#export DISPLAY=$DISPLAY
#export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

ros2 launch car_bringup car_bringup.launch.py