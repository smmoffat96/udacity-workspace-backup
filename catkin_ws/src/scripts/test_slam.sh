#!/bin/sh
xterm -e " source /home/udacity-workspace-backup/catkin_ws/devel/setup.bash " &
sleep 5
xterm -e " roslaunch turtlebot_simulator turtlebot_world.launch " &
sleep 5
xterm -e " roslaunch turtlebot_navigation gmapping_demo.launch " &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch "