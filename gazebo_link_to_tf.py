#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates
def callback(data):
	# Get index
	baxter_base_idx = data.name.index("baxter::base")
	brick_idx = data.name.index("t1::Table")
	# Get baxter_base pose and Brick pose w.r.t. gazebo world
	baxter_base_pose = data.pose[baxter_base_idx]
	brick_pose = data.pose[brick_idx]
	# Broadcast base and brick poses w.r.t. the gazebo world into tf tree
	br = tf.TransformBroadcaster()
	br.sendTransform((baxter_base_pose.position.x, baxter_base_pose.position.y, baxter_base_pose.position.z),
		(baxter_base_pose.orientation.x, baxter_base_pose.orientation.y, baxter_base_pose.orientation.z,
		baxter_base_pose.orientation.w), rospy.Time.now(),
		"base", "gazebo_world")
	# Broadcast base and brick poses w.r.t. the gazebo world into tf tree
	br2 = tf.TransformBroadcaster()
	br2.sendTransform((brick_pose.position.x, brick_pose.position.y, brick_pose.position.z),
		(brick_pose.orientation.x, brick_pose.orientation.y, brick_pose.orientation.z, brick_pose.orientation.w),
		rospy.Time.now(), "brick","gazebo_world")


def gazebo_link_subscriber():
	rospy.init_node('gazebo_link_subscriber')
	rospy.Subscriber("/gazebo/link_states", LinkStates, callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__': gazebo_link_subscriber()