from morse.builder import *
import bpy
import os

#bpymorse.set_speed(60, 20, 20)
##################### ROBOT  0 ####################
p3dx = Pioneer3DX()
p3dx.add_default_interface('ros')
p3dx.translate(x=-18.0, y=18.0, z=0.0)

odom = Odometry()
odom.add_interface('ros', frame_id='odom', child_frame_id='base_footprint')
#odom0.alter('Noise', pos_std=0.022, rot_std=0.02)
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

motion = MotionVWDiff()
motion.add_interface('ros')
p3dx.append(motion)

#keyboard = Keyboard()
#keyboard.properties(ControlType="Differential",Speed=1.0)
#p3dx.append(keyboard)


clock = Clock()
clock.add_interface('ros', topic="/clock")
p3dx.append(clock)


# Adapting physics properties of sensor, so robots detect each other
#for bgeObj in bpy.data.objects.keys():
#	if "SickMesh" in bgeObj: # We want the same behaviour for all scanner meshes
#		bpy.data.objects[bgeObj].game.physics_type = 'DYNAMIC'
#		bpy.data.objects[bgeObj].game.use_collision_bounds = True
#		bpy.data.objects[bgeObj].game.collision_bounds_type = 'BOX' #default
#		bpy.data.objects[bgeObj].game.mass = 0.0 #considered in the robot base



################ ENVIRONMENT ###############
env = Environment( os.path.abspath('../MORSE/scenarios/demo_small'), fastmode=True)
#env.use_vsync('OFF')
#env.set_time_strategy(TimeStrategies.FixedSimulationStep)
env.set_camera_rotation([0, 0, 0])
env.set_camera_location([0, 0, 60])
env.set_camera_speed(20) #default: 2m/s
#env.show_debug_properties()
#env.show_framerate()
#env.show_physics()

