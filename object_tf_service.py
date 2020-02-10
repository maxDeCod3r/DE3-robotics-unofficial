#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates

class tf_service():
	def __init__(self):
		self._nodes = ['t1::Table',
						'ia1::Brick',
						'ia2::Brick',
						'ia3::Brick',
						'ib1::Brick',
						'ib2::Brick',
						'ic1::Brick',
						'ic2::Brick',
						'id1::Brick',
						'ie1::Brick']



	def _callback(self, data):
		# Get index
		baxter_base_idx = data.name.index("baxter::base")
		object_idx = data.name.index("t1::Table")
		# Get baxter_base pose and object pose w.r.t. gazebo world
		baxter_base_pose = data.pose[baxter_base_idx]
		object_pose = data.pose[object_idx]
		# Broadcast base and object poses w.r.t. the gazebo world into tf tree
		br = tf.TransformBroadcaster()
		br.sendTransform((baxter_base_pose.position.x, baxter_base_pose.position.y, baxter_base_pose.position.z),
			(baxter_base_pose.orientation.x, baxter_base_pose.orientation.y, baxter_base_pose.orientation.z,
			baxter_base_pose.orientation.w), rospy.Time.now(),
			"base", "gazebo_world")
		# Broadcast base and object poses w.r.t. the gazebo world into tf tree
		br2 = tf.TransformBroadcaster()
		br2.sendTransform((object_pose.position.x, object_pose.position.y, object_pose.position.z),
			(object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w),
			rospy.Time.now(), "object","gazebo_world")


	def gazebo_link_subscriber(self):
		rospy.init_node('gazebo_link_subscriber')
		rospy.Subscriber("/gazebo/link_states", LinkStates, self._callback)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

x = tf_service()

x.gazebo_link_subscriber()