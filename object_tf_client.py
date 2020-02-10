#!/usr/bin/env python
import rospy
import tf
import time
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)

def tf_lookup(object_name):
	do_try = True
	listener = tf.TransformListener()

	while not rospy.is_shutdown() and do_try:
		try:
			(trans,rot) = listener.lookupTransform(object_name, 'base', rospy.Time(0))
			do_try = False
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print('Failed to get frame data, retrying...')
			time.sleep(0.1)
			continue

		target_pose = Pose()
		target_pose.position.x = trans[0]
		target_pose.position.y = trans[1]
		target_pose.position.z = trans[2]
		target_pose.orientation.x = rot[0]
		target_pose.orientation.y = rot[1]
		target_pose.orientation.z = rot[2]
		target_pose.orientation.w = rot[3]
		return target_pose

if __name__ == "__main__":
	rospy.init_node('example_tf_listener')
	print(tf_lookup('c1'))