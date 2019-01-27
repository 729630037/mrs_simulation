The `\*.py` files contain the builder scripts that launch the MORSE simulation. In 'createMap' a setup for creating a reference map from a provided simulation environment which is needed by AMCL is provided. 

There is a single robot scenario and a corresponding launch file for a single Pioneer3DX in the environment with configured AMCL-based localization and move base. Move goals have to be provided manually (via rviz's GUI) or by publishing it via 'rostopic pub ...'

Follow these steps for the single robot:
1. Launch the simulation: `morse run simulation_single_robot.py` (launch the ROS master with `roscore`)
2. Start roslaunch: `roslaunch single_robot.launch` (make sure that the required libraries are sourced)
3. (optional) Start rviz with the corresponding configuration: `rviz -d single_robot.rviz`
4. Publish goals (via rviz GUI)

For multiple robots the steps are analogous. However, robots follow a predifined set of goals which can be changed in `multi_robot.launch`.
1. Launch the simulation: `morse run simulation_multi_robot.py` (launch the ROS master with `roscore`)
2. Start roslaunch: `roslaunch multi_robot.launch` (make sure that the required libraries are sourced)
3. Robots start moving - you may want to start up rviz ('rviz -d multi_robot.rviz') or adapt the movement goals

Some notes:
The environmet is defined in the blender file which can be found under `../MORSE` and specified in the corresponding builder scripts.
The configuration of the navigation stack for the single robot and multi robot scenario are similar but not identical.
Movement destinations (goals) are parsed differently depending on the format they are provided. There are three options:
1. [[x,y], [x,y], ...] eg: [[10.0,-10.0],...]
2. [[x,y,z], [x,y,z], ...] eg: [[10.0,-10.0,0],...]
3. [[x,y,z,w], [x,y,z,w], ...] eg: [[10.0,-10.0,0.0,1.0],...]
4. [[p.x,p.y,p.z,o.x,o.y,o,w], ...] eg: [[10.0,-10.0,0.0,0.5,0.3,0.1,1.0],...]

For option 1 and 2 no orientation for the goal is given which results in using the default values (0.0 for x,y,z and 1.0 for w).
For option 3, only the position of the goal and w of the orientation quaternion are defined.
Option 4 allows to define all elements of the goal. A set of goals may be comprised of any combination of the formats. The frame of the goals defaults to 'map'.
