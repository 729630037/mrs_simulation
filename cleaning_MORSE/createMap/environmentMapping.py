from morse.builder import *
import bpy
import os

p3dx = Pioneer3DX()
p3dx.add_default_interface('ros')
p3dx.translate(x=-18.0, y=18.0, z=0.0)

odom = Odometry()
odom.add_interface('ros')
#odom.alter('Noise', pos_std=0.022, rot_std=0.02)
odom.frequency(10)
p3dx.append(odom)

sick = Sick()
sick.translate(z=0.252)
#sick.properties(Visible_arc=True)
sick.properties(scan_window = 190)
sick.properties(laser_range = 26.0)
sick.add_interface('ros', frame_id='base_laser_link')
sick.frequency(25)
p3dx.append(sick)

keyboard = Keyboard()
keyboard.properties(ControlType="Differential",Speed=1.0)
p3dx.append(keyboard)

# we need to explicitly add a clock if we define
# <param name="/use_sim_time" value="true"/> in the launch file

#clock = Clock()
#clock.add_interface('ros', topic="/clock")
#p3dx0.append(clock)



env = Environment( os.path.abspath('../../MORSE/scenarios/demo_small'), fastmode=True)
#env.use_vsync('OFF')
#env.set_time_strategy(TimeStrategies.FixedSimulationStep)
env.set_camera_rotation([0, 0, 0])
env.set_camera_location([0, 0, 60])
env.set_camera_speed(20) #default: 2m/s
#env.show_debug_properties()
#env.show_framerate()
#env.show_physics()

