#!/usr/bin/python
import argparse
import struct
import sys
import copy

import rospy
import rospkg

import numpy as np

from gazebo_msgs.srv import (SpawnModel, DeleteModel)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import (Header, Empty)

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)

import time

import baxter_interface

import tuck_arms

import target_angles as ta

#^Importing essential libraries for ROS and motion planning and running for the main script^

simulation = True #Flag for spawning objects in the gazebo if the script is running in a simulation
debug = False #Flag for asking confirmation before robot performs key maneuvers

class PickAndPlace(object): #class that handles moving the robot, gripper and IK
    def __init__(self, limb, hover_distance = 0.10, verbose=True, speed=0.2, accuracy=baxter_interface.settings.JOINT_ANGLE_TOLERANCE):
        '''Robot limb control class initializer'''
        self._accuracy = accuracy
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        self._gripper.calibrate() #Calibrating gripper
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
        '''Private class to move robot limbs'''
        if joint_angles:
            self._limb.set_joint_position_speed(0.3) #For safety purposes, the speed was lockes to 0.3
            self._limb.move_to_joint_positions(joint_angles, timeout=20.0, threshold=self._accuracy)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        '''Opens Gripper'''
        self._gripper.open()
        rospy.sleep(0.2)

    def gripper_close(self):
        '''Closes Gripper'''
        self._gripper.close()
        rospy.sleep(0.2)

    def goto(self, angles):
        '''Function to send limb to predefined joint angles without running IK service'''
        self._guarded_move_to_joint_position(angles)

    def ik_request(self, pose): 
        '''Function to generate joint angles from target pose, does not take into account obstacles that are not the robot'''
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')         #This function is only used wile in te simulation environment and only for generating valid joint angles, due to te random nature of te solver, it it not 'trusted' to run in real time
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
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
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
        print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@') #Visual statemet to easily find and copy calculated joint angles
        return limb_joints


    def gripperPosition(self):
        '''Returns the current gripper state, 0 (Fully closed) -> 100 (Fully open)'''
        return self._gripper.position() 

if simulation: 
    brick_ids = ['b1','b2','b3','b4','b5','b6','b7','b8','b9'] #Each brick gets known id for easy deletion

    with open ("brick/new-model.sdf", "r") as brick_file:brick_sdf=brick_file.read().replace('\n', '')
    with open ("L3-table/model.sdf", "r") as table_file:table_sdf=table_file.read().replace('\n', '')
    with open ("brick/static-b.sdf", "r") as table_file:static_brick=table_file.read().replace('\n', '')
    #^Loading sdf models into RAM^

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    #^Initializing spawn and delete model services^

def cleanup():
    '''Clears all (known) preexisting models in Gazebo'''
    for obj in brick_ids:
        delete_model(obj)
    delete_model('t1')
    delete_model('t2')

def etq(roll, pitch, yaw): #Euler To Quaternian
    '''calculates Quaternian from euler angles'''
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def spawn_v_brick():
    '''Spawns Vertical brick in Gazebo simulation'''
    brick_pose = Pose()
    brick_pose.position.x = 0.475
    brick_pose.position.y = 0.739
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
    '''Spawns Horizontal brick in Gazebo simulation'''
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
    '''Spawns Tables in Gazebo simulation'''
    table1 = Pose()
    table1.position.x = 1.160
    table1.position.y = 0.365
    table1.position.z = -0.379
    table1.orientation.x = 0
    table1.orientation.y = 0
    table1.orientation.z = 0
    table1.orientation.w = 0
    table2 = copy.deepcopy(table1)
    table2.position.x = 0.996
    table2.position.y = 1.068
    table2.position.z = -0.003
    table_reference_frame = 'world'
    table1_id = 't1'
    table2_id = 't2'
    spawn_sdf(table1_id, table_sdf, "/", table1, table_reference_frame)
    spawn_sdf(table2_id, table_sdf, "/", table2, table_reference_frame)

def open_and_wait(limb):
    '''Opens gripper and waits for uer input to close'''
    limb.gripper_open()
    x = raw_input('Close?')
    limb.gripper_close()
    time.sleep(0.5)

def gripperBrickChecker(limb):
    '''Checks if brick detected in gripper, asks what to do it not'''
    time.sleep(0.5) #Wait for gripper to finish closing before checking gripper state
    gripper_state = limb.gripperPosition()
    if gripper_state < 5:
        command = raw_input('\n \nPROBLEM DETECTED!!!\n(C)ontinue, (A)bort, (O)pen gripper\n>_ ')
        if command in {'C', 'c', 'continue'}:
            time.sleep(0.5)
            return
        elif command in {'O', 'o', 'open'}:
            open_and_wait(limb)
        else:
            print('Exiting')
            exit(0)

def V_Routine(limb): 
    '''Spawns vertical brick (if sim), sends gripper to brick position, asks if ready (if in debug), picks it up and checks that the brick is in the gripper'''
    if simulation: spawn_v_brick()
    limb.goto(ta.V_approach)
    if debug: x = raw_input('Pick? ')
    limb.goto(ta.V_pickup)
    if debug: x = raw_input('Close gripper? ')
    limb.gripper_close()
    gripperBrickChecker(limb)
    limb.goto(ta.V_approach)

def H_Routine(limb):
    '''Spawns horizontal brick (if sim), sends gripper to brick position, asks if ready (if in debug), picks it up and checks that the brick is in the gripper'''
    if simulation: spawn_h_brick()
    limb.goto(ta.H_approach)
    if debug: x = raw_input('Pick? ')
    limb.goto(ta.H_pickup)
    if debug: x = raw_input('Close gripper? ')
    limb.gripper_close()
    gripperBrickChecker(limb)
    limb.goto(ta.H_approach)

def process(limb, step):
    if step['vertical'] == True:
        V_Routine(limb)
    else:
        H_Routine(limb)

    limb.goto(step['hover'])
    if debug:x = raw_input('Place?: ')
    limb.goto(step['place'])
    if debug:x = raw_input('Open gripper?: ')
    limb.gripper_open()
    if not step['lastBrick']:
        limb.goto(step['hover'])
    else:
        limb.goto(step['last1'])
        limb.goto(step['last2'])
        limb.goto(step['last3'])




rospy.init_node("MOTHER_CLUCKER")  #Initializes rospy node

if simulation:
    #Clean up any (known) preexisting objects in Gazebo and spawns the tables
    cleanup()
    spawn_tables()

tuck_arms.init_arms() #Makes sure the arms are in known starting place
left_pnp = PickAndPlace('left') #Limb initializer

left_pnp.gripper_open() #Ensures gripper is open before grabbing brick

tower_instructions = ta.instructions

for step in tower_instructions:
    process(left_pnp, step)

time.sleep(1)

print("#################################################################################################################")
print("#...............................................................................................................#")
print("#...............................................................................................................#")
print("#...............................................................................................................#")
print("#.................. ____                 _   _         _   _            ____  _         _ _ _ _.................#")
print("#................../ ___|  ___ _ __   __| | (_)_ __   | |_| |__   ___  | __ )(_)_ __ __| | | | |................#")
print("#..................\\___ \\ / _ \\ '_ \\ / _` | | | '_ \\  | __| '_ \\ / _ \\ |  _ \\| | '__/ _` | | | |................#")
print("#.................. ___) |  __/ | | | (_| | | | | | | | |_| | | |  __/ | |_) | | | | (_| |_|_|_|................#")
print("#..................|____/ \\___|_| |_|\\__,_| |_|_| |_|  \\__|_| |_|\\___| |____/|_|_|  \\__,_(_|_|_)................#")
print("#...............................................................................................................#")
print("#...............................................................................................................#")
print("#...............................................................................................................#")
print("#################################################################################################################")


