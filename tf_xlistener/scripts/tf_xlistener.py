#!/usr/bin/env python  
import roslib
roslib.load_manifest('tf_xlistener')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	rospy.init_node('tf_xlistener')
	listener = tf.TransformListener()
	pub = rospy.Publisher('/cmd_vel', Twist)
	rate = rospy.Rate(1.0)
	vel = geometry_msgs.msg.Twist()
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		print "!!!!!!!!!!!!!!!!!!!!!"
		vel.linear.x = 0.2
		pub.publish(vel)
		print trans[0]
		if trans[0] > 1.0:
			vel.linear.x = 0.0
			pub.publish(vel)
		rate.sleep()