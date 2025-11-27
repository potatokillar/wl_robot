#!/bin/bash

source devel/setup.bash

if [ $# == 1 ];then
    if [ "$1" == 'arm' ]; then
        roslaunch simulation_all gazebo_arm.launch
    elif [ "$1" == 'qr' ]; then
        roslaunch simulation_all gazebo_qr.launch
    elif [ "$1" == 'human' ]; then
        roslaunch simulation_all gazebo_gh.launch
    fi
elif [ $# == 2 ]; then
    if [ "$1" == 'qr' ]; then
        roslaunch simulation_all gazebo_qr.launch rname:=$2
    elif [ "$1" == 'human' ]; then
        roslaunch simulation_all gazebo_gh.launch rname:=$2
    elif [ "$1" == 'arm' ]; then
        roslaunch simulation_all gazebo_arm.launch rname:=$2
    fi
elif [ $# == 3 ]; then
    if [ "$1" == 'qr' ]; then
        if [ "$3" == 'user' ]; then
            roslaunch simulation_all gazebo_qr.launch rname:=$2
        elif [ "$3" == 'debug' ]; then
            roslaunch simulation_all gazebo_qr.launch rname:=$2 user_debug:=true
        fi
    elif [ "$1" == 'human' ]; then
        if [ "$3" == 'user' ]; then
            roslaunch simulation_all gazebo_gh.launch rname:=$2
        elif [ "$3" == 'debug' ]; then
            roslaunch simulation_all gazebo_gh.launch rname:=$2 user_debug:=true
        fi
    fi
else
    echo "please input bootargs!!"
fi