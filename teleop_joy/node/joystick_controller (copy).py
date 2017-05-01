#!/usr/bin/env python
from __future__ import print_function

import rospy
#import message_filters

import std_msgs
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from remap import RemapXbox360

def callback(msg):

	joy_data_remapped.update(msg)

	if joy_data_remapped.LB == 1:
		cmd_vel_joy.linear.x  = 0
		cmd_vel_joy.angular.z = 0

	elif joy_data_remapped.LB == 0:
		cmd_vel_joy.linear.x = joy_data_remapped.LS_V/2
		cmd_vel_joy.angular.z = joy_data_remapped.RS_H

	else:
		pass

	pub_vel.publish(cmd_vel_joy)

#Main Part
rospy.init_node('joystick_controller_xbox360')

joy_data_remapped = RemapXbox360()
cmd_vel_joy = Twist()

#sub = message_filters.Subscriber('/joy', Joy)
#cache = message_filters.Cache(sub, 10)
#cache.registerCallback(callback)
sub = rospy.Subscriber('/joy', Joy, callback )

#pub = rospy.Publisher('/command_control', Command_msgs, queue_size=5 )
pub_vel = rospy.Publisher('/cmd_vel_joy', Twist, queue_size=5 )

rospy.loginfo('Joystick Remapping Node Initialized ...')

rospy.spin()
