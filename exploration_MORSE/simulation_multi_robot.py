from morse.builder import *
import bpy
import os

#bpymorse.set_speed(60, 20, 20)
##################### ROBOT  0 ####################
p3dx0 = Pioneer3DX()
p3dx0.add_default_interface('ros')
p3dx0.translate(x=-18.0, y=18.0, z=0.0)

odom0 = Odometry()
odom0.add_interface('ros', frame_id='p3dx0_tf/odom', child_frame_id='p3dx0_tf/base_footprint')
odom0.alter('Noise', pos_std=0.022, rot_std=0.02)
odom0.frequency(10)
p3dx0.append(odom0)

sick0 = Sick()
sick0.translate(z=0.252)
#sick0.properties(Visible_arc=True)
sick0.properties(scan_window = 190)
sick0.properties(laser_range = 26.0)
sick0.add_interface('ros', frame_id='p3dx0_tf/base_laser_link')
sick0.frequency(25)
p3dx0.append(sick0)

motion0 = MotionVWDiff()
motion0.add_interface('ros')
p3dx0.append(motion0)

##################### ROBOT  1 ####################
p3dx1 = Pioneer3DX()
p3dx1.add_default_interface('ros')
p3dx1.translate(x=12.0, y=10.0, z=0.0)

odom1 = Odometry()
odom1.add_interface('ros', frame_id='p3dx1_tf/odom', child_frame_id='p3dx1_tf/base_footprint')
odom1.alter('Noise', pos_std=0.022, rot_std=0.02)
odom1.frequency(10)
p3dx1.append(odom1)

sick1 = Sick()
sick1.translate(z=0.252)
#sick1.properties(Visible_arc=True)
sick1.properties(scan_window = 190)
sick1.properties(laser_range = 26.0)
sick1.add_interface('ros', frame_id='p3dx1_tf/base_laser_link')
sick1.frequency(25)
p3dx1.append(sick1)

motion1 = MotionVWDiff()
motion1.add_interface('ros')
p3dx1.append(motion1)

# we need to explicitly add a clock if we define
# <param name="/use_sim_time" value="true"/> in the launch file

#clock = Clock()
#clock.add_interface('ros', topic="/clock")
#p3dx0.append(clock)


# Adapting physics properties of sensor, so robots detect each other
for bgeObj in bpy.data.objects.keys():
	if "SickMesh" in bgeObj: # We want the same behaviour for all scanner meshes
		bpy.data.objects[bgeObj].game.physics_type = 'DYNAMIC'
		bpy.data.objects[bgeObj].game.use_collision_bounds = True
		bpy.data.objects[bgeObj].game.collision_bounds_type = 'BOX' #default
		bpy.data.objects[bgeObj].game.mass = 0.0 #considered in the robot base


################ ENVIRONMENT ###############
env = Environment(os.path.abspath('../MORSE/scenarios/demo_small'), fastmode=True)
#env.use_vsync('OFF')
#env.set_time_strategy(TimeStrategies.FixedSimulationStep)
env.set_camera_rotation([0, 0, 0])
env.set_camera_location([0, 0, 60])
env.set_camera_speed(20) #default: 2m/s
#env.show_debug_properties()
env.show_framerate()
#env.show_physics()
