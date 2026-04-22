#!/bin/bash
# Start Cartesian Servo + Haptic Teleop
# Ustawia ROS_IP i ROS_MASTER_URI, potem odpala launch

export ROS_MASTER_URI=http://192.42.0.1:11311
export ROS_IP=192.42.0.1

echo "[start_teleop] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[start_teleop] ROS_IP=$ROS_IP"

source ~/ws_easy/underlay/devel/setup.bash

roslaunch cartesian_servo haptic_teleop.launch "$@"
