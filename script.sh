#!/bin/bash
#gnome-terminal --tab  --command="bash -c 'roscore; $SHELL'" --tab --#command="bash -c 'pwd; $SHELL'" --tab --command="bash -c 'ls; $SHELL'"


echo -e "Choose the algorithm that you want to use: \n \t-gmapping\n \t-hector \n \t-karto\n and hit enter "
read algo

echo -e "Choose the map that you want to use: \n \t-playpen\n \t-willowgarrage \n \t-myworld\n and hit enter "
read world




gnome-terminal --tab  --command="bash -c 'rosparam set /joystick/dev /dev/input/js1 && sudo xboxdrv --device-by-id 2f24:0091 --type xbox360 --device-name 'EasySMX-9101' --silent; $SHELL'"

echo -e "waiting for password..."
read

gnome-terminal --tab  --command="bash -c 'roslaunch teleop_twist_joy teleop.launch; $SHELL'"




gnome-terminal --tab  --command="bash -c 'roslaunch husky_gazebo husky_gazebo.launch world:=$world; $SHELL'"


echo -e "waiting for gazebo ..."
read

gnome-terminal --tab  --command="bash -c 'roslaunch ira_laser_tools laserscan_multi_merger.launch; $SHELL'"

sleep 2

case $algo in
	"gmapping")
	gnome-terminal --tab  --command="bash -c 'roslaunch husky_navigation gmapping_demo.launch; $SHELL'" --tab  --command="bash -c 'roslaunch husky_viz view_robot.launch; $SHELL'"

	;;
	"hector")
	gnome-terminal --tab  --command="bash -c 'roslaunch husky_navigation hector_demo.launch; $SHELL'" --tab  --command="bash -c 'roslaunch husky_viz view_robot.launch; $SHELL'"
	
	;;
	"karto")
	gnome-terminal --tab  --command="bash -c 'roslaunch husky_navigation karto_slam.launch; $SHELL'" --tab  --command="bash -c 'roslaunch husky_viz view_robot.launch; $SHELL'"
	
	;;
esac	






