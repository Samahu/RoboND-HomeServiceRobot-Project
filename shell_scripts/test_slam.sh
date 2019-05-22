#!/bin/sh
xterm -e ' ROBOT_INITIAL_POSE="-x 3 -y -4" roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/robond/catkin_ws/src/world/u-world.world' & 
sleep 5
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=/home/robond/catkin_ws/src/shell_scripts/asus_xtion_pro_gmapping_customized.launch.xml" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch"
