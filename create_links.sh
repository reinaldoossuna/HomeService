#!/bin/bash

ln -fs launch/amcl_demo.launch turtlebot_simulator/turtlebot_gazebo/launch/

ln -fs launch/slam_gmapping.launch slam_gmapping/gmapping/launch/

ln -fs launch/turtlebot_world.launch turtlebot_simulator/turtlebot_gazebo/launch/

ln -fs launch/homeservice.launch turtlebot_interactions/turtlebot_rviz_launchers/launch/

ln -fs launch/homeservice.rviz turtlebot_interactions/turtlebot_rviz_launchers/rviz/

for filename in worlds/*; do
  ln -fs $filename turtlebot_simulator/turtlebot_gazebo/worlds
done
