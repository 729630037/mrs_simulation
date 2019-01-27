The "simulation_single_robot.launch" launch file starts a Gazbeo simulation of a single Turtlebot 3 Burger robot in the given "demo_environment.world" map.
The "default_single_robot.launch" starts AMCL-based localization and the navigation stack with the default parameters provided in the turtlebot3 packages (more precisely in the turtlebot3_navigation package)


The `\*_simulation.launch` are used to launch the simulation in Gazebo. In 'createMap' a setup for creating a reference map from a provided simulation environment which is needed by AMCL is provided. 

There is a single robot scenario and a corresponding launch file for a single Turtlebot in the environment with configured AMCL-based localization and move base. Move goals have to be provided manually (via rviz's GUI) or by publishing it via `rostopic pub ...`

Follow these steps for the single robot:
1. Launch the simulation: `roslaunch simulation_single_robot.launch` (note: make sure that the required libraries are sourced and you specified the turtlebot model before: `export TURTLEBOT3_MODEL=burger`)
2. Start roslaunch: `roslaunch single_robot.launch` (see note above)
3. (optional) Start rviz with the corresponding configuration: `rviz -d single_robot.rviz`
4. Publish goals (via rviz GUI)

For multiple robots the steps are analogous. However, robots follow a predifined set of goals which can be changed in `multi_robot.launch`.
1. Launch the simulation: `roslaunch simulation_multi_robot.launch` (note: make sure that the required libraries are sourced and you specified the turtlebot model before: `export TURTLEBOT3_MODEL=burger`)
2. Start roslaunch: `roslaunch multi_robot.launch` (see note above)
3. Robots start moving - you may want to start up rviz (`rviz -d multi_robot.rviz`) or adapt the movement goals

Some notes:
The environment is defined in 'small_environment.world'. Another environment can be used by specifying it in the `\*_simulation.launch` file. The URDF files are contained in the turtlebot3 package. 
The configuration of the navigation stack for the single robot and multi robot scenario are similar but not identical.
Movement destinations (goals) are parsed differently depending on the format they are provided. There are three options:
1. [[x,y], [x,y], ...] eg: [[10.0,-10.0],...]
2. [[x,y,z], [x,y,z], ...] eg: [[10.0,-10.0,0],...]
3. [[x,y,z,w], [x,y,z,w], ...] eg: [[10.0,-10.0,0.0,1.0],...]
4. [[p.x,p.y,p.z,o.x,o.y,o,w], ...] eg: [[10.0,-10.0,0.0,0.5,0.3,0.1,1.0],...]

For option 1 and 2 no orientation for the goal is given which results in using the default values (0.0 for x,y,z and 1.0 for w).
For option 3, only the position of the goal and w of the orientation quaternion are defined.
Option 4 allows to define all elements of the goal. A set of goals may be comprised of any combination of the formats. The frame of the goals defaults to 'map'.
