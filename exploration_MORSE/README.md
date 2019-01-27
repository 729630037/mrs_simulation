# Running the Multi-Robot Room Exploration
Assuming that all required packages are installed, run the following commands in a terminal each:
1. `roscore` - rather launch the ROS master before the other ROS nodes, so the simulator can initialize all components.  
2. `morse run simulation_multi_robot.py`  
3. `roslaunch multi_robot.launch` - In this terminal you need to source the needed files beforehand.

The MORSE resources (including other environments) can be found in the `../MORSE` directory. The implementation is based on the mentioned papers and some of the original implementations on ROS Groovy which are [publicly available](https://github.com/yzrobot/mrs_testbed) as well.
