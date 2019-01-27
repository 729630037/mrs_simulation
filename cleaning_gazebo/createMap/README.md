These files intended to help creating a reference map of the environment.
1. Start the simulation - for instance 'roslaunch simulation_single_robot.launch'
	By default, a modified version of the turtlebot is used with highly unrealistic values that simplify the mapping process. If you want to use another robot or the realistic turtlebot model adapt the 'simulation_single_robot.launch' file
2. Start the corresponding launch file - 'roslaunch create_map.launch'
3. Control the robot with the keyboard (WASDX) by starting teleoperation: 'roslaunch teleop.launch'
4. When the map looks fine, save it with the command 'rosrun map_server map_saver -f mymap' (before shutting down the SLAM mapping node!)

We need to set the TURTLEBOT3_MODEL variable beforehand (in each terminal): 'export TURTLEBOT3_MODEL=burger'
Resolution or map size for other environments can be changed in the launch file.
