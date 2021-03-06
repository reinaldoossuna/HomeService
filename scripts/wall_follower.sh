#!/bin/bash

x-terminal-emulator -x roslaunch turtlebot_gazebo turtlebot_world.launch &

sleep 5 &&

x-terminal-emulator -x rosrun gmapping slam_gmapping  &

sleep 5 &&

x-terminal-emulator -x roslaunch turtlebot_rviz_launchers view_navigation.launch &

sleep 5 &&

x-terminal-emulator -x rosrun wall_follower wall_follower
