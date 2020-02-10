#!/usr/bin/env python
import rospy
import tf
if __name__ == '__main__':
	""" A simple example of tf listener """ 
	rospy.init_node('example_tf_listener')

	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('left_gripper', 'base', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		Translation = [trans.x , trans.y , trans.z]
		Quaternion = [rot.x , rot.y , rot.z, rot.w]
		Angles = tf.transformations.euler_from_quaternion([rot.x , rot.y ,
		rot.z, rot.w])
		print("Translation: ", Translation) 
		print("Quaternion: ", Quaternion)
		print("Angles: ", Angles)
		print("")
		rate.sleep()