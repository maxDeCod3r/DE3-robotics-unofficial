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

global sumulation

simulation = True
debug = True

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
                 # print("IK Joint Solution:\n{0}".format(limb_joints))
                 print("------------------")
         else:
             rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
             return False
         print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
         print()
         print()
         print('Linb Joints:')
         print(limb_joints)
         print()
         print()
         print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
         return limb_joints

    def gripperPosition(self):
        return self._gripper.position()

if simulation:
    brick_ids = ['b1','b2','b3','b4','b5','b6','b7','b8','b9']

    with open ("brick/model.sdf", "r") as brick_file:brick_sdf=brick_file.read().replace('\n', '')
    with open ("L3-table/model.sdf", "r") as table_file:table_sdf=table_file.read().replace('\n', '')
    with open ("brick/static-b.sdf", "r") as table_file:static_brick=table_file.read().replace('\n', '')

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

def cleanup():
    for obj in brick_ids:
        delete_model(obj)
    delete_model('t1')
    delete_model('t2')

# LET THE SHITSTORM BEGIN
import numpy as np
import time

def etq(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
#LET THE SHITSTORM END


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

def spawn_tables():
    table1 = Pose()
    table1.position.x = 1.160
    table1.position.y = 0.365
    table1.position.z = -0.359
    table1.orientation.x = 0
    table1.orientation.y = 0
    table1.orientation.z = 0
    table1.orientation.w = 0
    table2 = copy.deepcopy(table1)
    table2.position.x = 0.996
    table2.position.y = 1.018
    table2.position.z = -0.003
    table_reference_frame = 'world'
    table1_id = 't1'
    table2_id = 't2'
    spawn_sdf(table1_id, table_sdf, "/", table1, table_reference_frame)
    spawn_sdf(table2_id, table_sdf, "/", table2, table_reference_frame)


rospy.init_node("I_still_have_some_hope")  # Am I wrong??


#####
b9 = Pose()
b9.position.x = 0.635
b9.position.y = 0.349
b9.position.z = 0.406
b9.orientation.x = -0.7071067811865476
b9.orientation.y = -0.7071067811865475
b9.orientation.z = 4.329780281177466e-17
b9.orientation.w = 4.329780281177467e-17
brick_reference_frame = 'world'
brick_id = 'static-9'
spawn_sdf(brick_id, static_brick, "/", b9, brick_reference_frame)
#####

if simulation:
    cleanup()
    spawn_tables()

tuck_arms.init_arms()

hover_distance = 0.2 
left_pnp = PickAndPlace('left', hover_distance)


left_pnp.gripper_open()

def V_Routine():
    if simulation:
        spawn_v_brick()

    left_pnp.send(ta.V_approach)
    x = raw_input('Ready?')
    if x == 'n':exit(0)
    left_pnp.send(ta.V_pickup)


    left_pnp.gripper_close()
    gripper_state = left_pnp.gripperPosition()

    if gripper_state < 10:
        command = raw_input('\n \nPROBLEM DETECTED!!!\n(C)ontinue, (A)bort, (O)pen gripper\n>_ ')
        if command in {'C', 'c', 'continue'}:
            pass
        elif command in {'O', 'o', 'open'}:
            open_and_wait()
        else:
            print('Exiting')
            exit(0)

    time.sleep(0.5)
    left_pnp.send(ta.V_approach)


def open_and_wait():
    left_pnp.gripper_open()
    x = raw_input('Close?')
    left_pnp.gripper_close()
    time.sleep(0.5)


def H_Routine():
    if simulation:
        spawn_h_brick()
    left_pnp.send(ta.H_approach)
    x = raw_input('Ready?')
    if x == 'n':exit(0)
    left_pnp.send(ta.H_pickup)
    left_pnp.gripper_close()

    gripper_state = left_pnp.gripperPosition()

    if gripper_state < 10:
        command = raw_input('\n \nPROBLEM DETECTED!!!\nGripper has nothing in it...\n(C)ontinue, (A)bort, (O)pen gripper\n>_ ')
        if command == 'C':
            pass
        elif command == 'c':
            pass
        elif command == 'O':
            open_and_wait()
        elif command == 'o':
            open_and_wait()
        else:
            exit(0)

    time.sleep(0.5)
    left_pnp.send(ta.H_approach)


left_pnp.gripper_open()

# V_Routine()

# left_pnp.send(ta.B_1_A)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.send(ta.B_1_P)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.gripper_open()

# left_pnp.send(ta.B_1_A)

# V_Routine()

# left_pnp.send(ta.B_2_A)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.send(ta.B_2_P)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.gripper_open()
# left_pnp.send(ta.B_2_A)

# V_Routine()

# left_pnp.send(ta.B_3_A)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.send(ta.B_3_P)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.gripper_open()
# left_pnp.send(ta.B_3_A)


# H_Routine()
# left_pnp.send(ta.B_4_A)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.send(ta.B_4_P)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.gripper_open()
# left_pnp.send(ta.B_4_A)

# H_Routine()
# left_pnp.send(ta.B_5_A)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.send(ta.B_5_P)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.gripper_open()
# left_pnp.send(ta.B_5_A)


# V_Routine()
# left_pnp.send(ta.B_6_A)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.send(ta.B_6_P)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.gripper_open()
# left_pnp.send(ta.B_6_A)

# V_Routine()
# left_pnp.send(ta.B_7_A)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.send(ta.B_7_P)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.gripper_open()
# left_pnp.send(ta.B_7_A)


# H_Routine()
# left_pnp.send(ta.B_8_A)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.send(ta.B_8_P)
# if debug:
#     x = raw_input('Continue?: ')
# left_pnp.gripper_open()
# left_pnp.send(ta.B_8_A)


V_Routine()
left_pnp.send(ta.B_9_A)
if debug:
    x = raw_input('Continue?: ')
left_pnp.send(ta.B_9_P)
if debug:
    x = raw_input('Continue?: ')
left_pnp.gripper_open()

brick9after = Pose()
brick9after.position.x = 0.635
brick9after.position.y = 0.349
brick9after.position.z = 0.506
brick9after.orientation.x = -0.7071067811865476
brick9after.orientation.y = -0.7071067811865475
brick9after.orientation.z = 4.329780281177466e-17
brick9after.orientation.w = 4.329780281177467e-17

new_x = left_pnp.ik_request(brick9after)
left_pnp.send(new_x)
x = raw_input('Ready???')
brick9after2 = Pose()
brick9after2.position.x = 0.635
brick9after2.position.y = 0.8069
brick9after2.position.z = 0.4
brick9after2.orientation.x = 0
brick9after2.orientation.y = 0
brick9after2.orientation.z = 0.707
brick9after2.orientation.w = 0.707
newest = left_pnp.ik_request(brick9after2)
left_pnp.send(newest)


left_pnp.send(ta.V_pickup)

