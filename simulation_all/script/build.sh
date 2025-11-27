#!/bin/bash

rm -r devel/ build/

cd src/

rm CMakeLists.txt

source /opt/ros/noetic/setup.bash

catkin_init_workspace

cd ..

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
