#!/usr/bin/env python
import rospy
import tf
import time
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from math import radians

def tf_lookup(object_name):
	do_try = True
	listener = tf.TransformListener()

	while not rospy.is_shutdown() and do_try:
		try:
			(trans,rot) = listener.lookupTransform('base', object_name, rospy.Time(0))
			do_try = False
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print('Failed to get frame data, retrying...')
			time.sleep(0.1)
			continue

		object_angles = tf.transformations.euler_from_quaternion([rot[0] , rot[1] , rot[2], rot[3]])
		xangle = object_angles[0]#+radians(90)# + radians(77.878)
		xangle1 = object_angles[1]# - radians(90)
		xangle2 = object_angles[2]# - radians(22.231)
		object_angles = (xangle, xangle1, xangle2)
		print("Angles:")
		print(xangle)
		print(xangle1)
		print(xangle2)
		print()
		target_quat = tf.transformations.quaternion_from_euler(xangle, xangle1, xangle2)
		target_pose = Pose()
		target_pose.position.x = trans[0]
		target_pose.position.y = trans[1]
		target_pose.position.z = trans[2]
		target_pose.orientation.x = target_quat[0]
		target_pose.orientation.y = target_quat[1]
		target_pose.orientation.z = target_quat[2]
		target_pose.orientation.w = target_quat[3]

		return target_pose

if __name__ == "__main__":
	rospy.init_node('example_tf_listener')
	print(tf_lookup('c1'))


#      
#      TARGET TOP PICKING EULER ANGLES
#      LAYING MODIFIERS:
#      Pitch(0): +90 deg.
#      Roll(1):    0 deg.
#      Yaw(2):     0 deg.
#      
#      FINAL VALUES:
#      Pitch(0): +180 deg.
#      Roll(1):     0 deg.
#      Yaw(2):   -180 deg.
#      
#      INITIAL STANDING VALUES
#      0.211     12.112 + 77.878
#      1.57      90.000 - 90.000
#      -2.75   -157.769 - 22.231
#      
#      
#      
