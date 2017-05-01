#!/usr/bin/env python
from __future__ import print_function

import rospy
#import message_filters

import std_msgs
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from remap import RemapXbox360

def callback(msg):
	global cmd_l,cmd_r,joy_data_remapped
	joy_data_remapped.update(msg)

	if joy_data_remapped.LB == 1:
		cmd_vel_joy.linear.x  = 0
		cmd_vel_joy.angular.z = 0

	elif joy_data_remapped.LB == 0:
		cmd_vel_joy.linear.x = joy_data_remapped.LS_V/2
		cmd_vel_joy.angular.z = joy_data_remapped.RS_H

	else:
		pass


	cmd_l = Int16()
	#cmd_l.data = int(99*joy_data_remapped.LS_V)
	cmd_r = Int16()
	#cmd_r.data = int(99*joy_data_remapped.RS_V)

	cmd_l = int(99*joy_data_remapped.LS_V)-int(50*joy_data_remapped.RS_H)
	cmd_r = int(99*joy_data_remapped.LS_V)+int(50*joy_data_remapped.RS_H)
	#pub_cmd_l.publish(cmd_l)
	#pub_cmd_r.publish(cmd_r)
	#pub_vel.publish(cmd_vel_joy)

#Main Part
rospy.init_node('joystick_controller_xbox360')

joy_data_remapped = RemapXbox360()
cmd_vel_joy = Twist()

cmd_l = Int16()
cmd_r = Int16()

#sub = message_filters.Subscriber('/joy', Joy)
#cache = message_filters.Cache(sub, 10)
#cache.registerCallback(callback)
sub = rospy.Subscriber('/joy', Joy, callback )

#pub = rospy.Publisher('/command_control', Command_msgs, queue_size=5 )
pub_vel = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=5 )
pub_cmd_l = rospy.Publisher('/cmd_vel/left', Int16, queue_size=5 )
pub_cmd_r = rospy.Publisher('/cmd_vel/right', Int16, queue_size=5 )

rospy.loginfo('Joystick Remapping Node Initialized ...')

rate = rospy.Rate(10)
while not rospy.is_shutdown():
	#pub_cmd_l.publish(cmd_l)
	#pub_cmd_r.publish(cmd_r)
	pub_vel.publish(cmd_vel_joy)
	rate.sleep()

rospy.spin()
