import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (SpawnModel, DeleteModel)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import (Header, Empty)

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)

import baxter_interface

import target_poses as tps

import tuck_arms

import target_angles as ta

brickstuff = tps.brick_directions_notf

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.10, verbose=True, speed=0.2, accuracy=baxter_interface.settings.JOINT_ANGLE_TOLERANCE):
        self._accuracy = accuracy
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.set_joint_position_speed(0.1)
            self._limb.move_to_joint_positions(joint_angles, timeout=20.0, threshold=self._accuracy)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(0.2)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(0.2)

    def send(self, angles):
        self._guarded_move_to_joint_position(angles)


brick_ids = ['b1','b2','b3','b4','b5','b6','b7','b8','b9']

with open ("brick/model.sdf", "r") as brick_file:brick_sdf=brick_file.read().replace('\n', '')

rospy.wait_for_service('/gazebo/spawn_sdf_model')
spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

def cleanup():
    for obj in brick_ids:
        delete_model(obj)

# LET THE SHITSTORM BEGIN
import numpy as np
import time

def etq(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def spawn_v_brick():
	brick_pose = Pose()
	brick_pose.position.x = 0.475
	brick_pose.position.y = 0.759
	brick_pose.position.z = 0.818
	QUATS = etq(0, 1.57, 1.57)
	brick_pose.orientation.x = QUATS[0]
	brick_pose.orientation.y = QUATS[1]
	brick_pose.orientation.z = QUATS[2]
	brick_pose.orientation.w = QUATS[3]
	brick_reference_frame = 'world'
	brick_id = brick_ids.pop()
	spawn_sdf(brick_id, brick_sdf, "/", brick_pose, brick_reference_frame)

def spawn_h_brick():
	brick_pose = Pose()
	brick_pose.position.x = 0.4664
	brick_pose.position.y = 0.8069
	brick_pose.position.z = 0.7533
	brick_pose.orientation.x = 0
	brick_pose.orientation.y = 0
	brick_pose.orientation.z = 0.707
	brick_pose.orientation.w = 0.707
	brick_reference_frame = 'world'
	brick_id = brick_ids.pop()
	spawn_sdf(brick_id, brick_sdf, "/", brick_pose, brick_reference_frame)


rospy.init_node("I_still_have_some_hope")  # Am I wrong??

cleanup()

tuck_arms.init_arms()

hover_distance = 0.2 
left_pnp = PickAndPlace('left', hover_distance)

left_pnp.gripper_open()

def V_Routine():

	spawn_v_brick()

	left_pnp.send(ta.V_approach)
	x = raw_input('Ready?')
	left_pnp.send(ta.V_pickup)
	left_pnp.gripper_close()
	left_pnp.send(ta.V_approach)



def H_Routine():

	spawn_h_brick()

	left_pnp.send(ta.H_approach)
	x = raw_input('Ready?')
	left_pnp.send(ta.H_pickup)
	left_pnp.gripper_close()
	left_pnp.send(ta.H_approach)



V_Routine()
left_pnp.send(ta.B_1_A)
left_pnp.send(ta.B_1_P)
left_pnp.gripper_open()
left_pnp.send(ta.B_1_A)

V_Routine()
left_pnp.send(ta.B_2_A)
left_pnp.send(ta.B_2_P)
left_pnp.gripper_open()
left_pnp.send(ta.B_2_A)

V_Routine()
left_pnp.send(ta.B_3_A)
left_pnp.send(ta.B_3_P)
left_pnp.gripper_open()
left_pnp.send(ta.B_3_A)


H_Routine()
left_pnp.send(ta.B_4_A)
left_pnp.send(ta.B_4_P)
left_pnp.gripper_open()
left_pnp.send(ta.B_4_A)

H_Routine()
left_pnp.send(ta.B_5_A)
left_pnp.send(ta.B_5_P)
left_pnp.gripper_open()
left_pnp.send(ta.B_5_A)


V_Routine()
left_pnp.send(ta.B_6_A)
left_pnp.send(ta.B_6_P)
left_pnp.gripper_open()
left_pnp.send(ta.B_6_A)

V_Routine()
left_pnp.send(ta.B_7_A)
left_pnp.send(ta.B_7_P)
left_pnp.gripper_open()
left_pnp.send(ta.B_7_A)


H_Routine()
left_pnp.send(ta.B_8_A)
left_pnp.send(ta.B_8_P)
left_pnp.gripper_open()
left_pnp.send(ta.B_8_A)


V_Routine()
left_pnp.send(ta.B_9_A)
left_pnp.send(ta.B_9_P)
left_pnp.gripper_open()
left_pnp.send(ta.B_9_A)


left_pnp.send(H_pickup)

