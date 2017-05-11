#!/bin/bash

export ROS_MASTER_URI=http://192.168.0.130:11311
export ROS_IP=192.168.0.130
source ./devel_isolated/setup.bash
roslaunch ./src/launch/apr_mocap_flea_mav_stamp.launch  
