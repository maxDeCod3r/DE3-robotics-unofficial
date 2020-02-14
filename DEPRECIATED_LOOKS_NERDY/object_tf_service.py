#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates
import bricks_etc as bet
global SIGKILL # Horrible hack
SIGKILL = False


class tf_service():
	def __init__(self):
		self._nodes = bet.getNodes()

	def _callback(self, data):
		# Get index
		if SIGKILL:
			self._kill()
		for object_name in self._nodes:
			'''HORRIBLE HACK BEGIN'''
			if object_name == 'baxter::base':
				spawn_name = 'base'
			else:
				spawn_name = object_name[:2]

			'''HORRIBLE HACK END'''
			# print(object_name)
			object_idx = data.name.index(object_name)
			# Get baxter_base pose and object pose w.r.t. gazebo world
			object_pose = data.pose[object_idx]
			# Broadcast base and object poses w.r.t. the gazebo world into tf tree
			br = tf.TransformBroadcaster()
			br.sendTransform((object_pose.position.x, object_pose.position.y, object_pose.position.z),
				(object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w),
				rospy.Time.now(), spawn_name,'gazebo_world')

	def _kill(self):
		rospy.signal_shutdown('User_requested') #SURPRISINGLY DIFFICULT

	def gazebo_link_subscriber(self):
		# rospy.init_node('gazebo_link_subscriber')
		rospy.Subscriber("/gazebo/link_states", LinkStates, self._callback)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

def init():
	x = tf_service()
	x.gazebo_link_subscriber()