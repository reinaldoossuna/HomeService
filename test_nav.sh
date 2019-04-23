#!/bin/bash

x-terminal-emulator -x roslaunch turtlebot_gazebo turtlebot_world.launch &

sleep 5 &&

x-terminal-emulator -x roslaunch turtlebot_gazebo amcl.launch &

sleep 5 &&

x-terminal-emulator -x roslaunch turtlebot_rviz_launchers view_navigation.launch &

