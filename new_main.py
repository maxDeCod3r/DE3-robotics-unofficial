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

brickstuff = tps.brick_directions_notf

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.10, verbose=True, speed=0.2, accuracy=baxter_interface.settings.JOINT_ANGLE_TOLERANCE):
        # self._accuracy = accuracy
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

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(0.2)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            print()
            print(ikreq.SEED_USER)
            print()
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            # self._limb.set_joint_position_speed(1.5)
            self._limb.move_to_joint_positions(joint_angles)#, timeout=20.0, threshold=self._accuracy)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(0.2)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(0.2)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        print('Approaching')
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        print('Ready to grip')
        # close gripper
        self.gripper_close()
        print('grip')
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()


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

## TODO: MAKE hover_distance higher for place part or solve IK collision error

spawn_v_brick()
left_pnp.pick(brickstuff[0]['pose'])
left_pnp.place(brickstuff[2]['pose'])
spawn_v_brick()
left_pnp.pick(brickstuff[0]['pose'])
left_pnp.place(brickstuff[3]['pose'])
spawn_v_brick()
left_pnp.pick(brickstuff[0]['pose'])
left_pnp.place(brickstuff[4]['pose'])
spawn_h_brick()
left_pnp.pick(brickstuff[1]['pose'])
left_pnp.place(brickstuff[5]['pose'])
spawn_h_brick()
left_pnp.pick(brickstuff[1]['pose'])
left_pnp.place(brickstuff[6]['pose'])
spawn_v_brick()
left_pnp.pick(brickstuff[0]['pose'])
left_pnp.place(brickstuff[7]['pose'])
spawn_v_brick()
left_pnp.pick(brickstuff[0]['pose'])
left_pnp.place(brickstuff[8]['pose'])
spawn_h_brick()
left_pnp.pick(brickstuff[1]['pose'])
left_pnp.place(brickstuff[9]['pose'])
spawn_v_brick()
# left_pnp.pick(brickstuff[0]['pose'])
# left_pnp.place(brickstuff[10]['pose'])
