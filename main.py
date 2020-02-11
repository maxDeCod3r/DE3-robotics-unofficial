import argparse
import struct
import sys
import copy

import rospy
import rospkg

from math import radians

from gazebo_msgs.srv import (SpawnModel, DeleteModel)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import (Header, Empty)

from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)

import baxter_interface

from tf.transformations import quaternion_from_euler

import threading
import object_tf_service as ots
import object_tf_client as otc
import time
import bricks_etc as bet

running = True

bricks = bet.getBuildable()
table = bet.getTable()
brick_targets = bet.getDirections()
built_bricks = bet.getTargets()



class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.10, verbose=True, speed=0.9, accuracy=baxter_interface.settings.JOINT_ANGLE_TOLERANCE):
        self._speed = speed
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
            self._limb.set_joint_position_speed(self._speed)
            self._limb.move_to_joint_positions(joint_angles, timeout=20.0, threshold=self._accuracy)
            self._limb.set_joint_position_speed(0.3)
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


def _tf_service_initializer():
    tf_service = ots.init()


def tf_service(init=True):
    global tf_service_thread
    if init:
        print('Starting tf service thread')
        tf_service_thread = threading.Thread(target=_tf_service_initializer)
        tf_service_thread.start()
    else:
        print('Killing tf service thread')
        ots.SIGKILL = True


def cleanup():
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    for obj in bet.getAll():
        delete_model(obj['id'])


def load_objects():
    # Load Table SDFs
    with open ("models/L3-table/model.sdf", "r") as table_file:table_xml=table_file.read().replace('\n', '')
    with open ("models/brick/model.sdf", "r") as brick_file:brick_xml=brick_file.read().replace('\n', '')
    with open ("models/empty/model.sdf", "r") as empty_file:empty_xml=empty_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    table_pose = Pose()
    table_pose.position = Point(x=table['x'], y=table['y'], z=table['z'])
    tqo = quaternion_from_euler(table['roll'], table['pitch'], table['yaw'])
    table_pose.orientation = Quaternion(tqo[0], tqo[1], tqo[2], tqo[3])
    table_reference_frame="world"
    spawn_sdf(table['id'], table_xml, "/", table_pose, table_reference_frame)
    brick_ents = []
    for brick in bricks:
        ber = [brick['roll'], brick['pitch'], brick['yaw']] #brick_euler_rotation
        brick_pose = Pose(position=Point(x=brick['x'], y=brick['y'], z=brick['z']))
        brick_pose.position = Point(x=brick['x'], y=brick['y'], z=brick['z'])
        bqo = quaternion_from_euler(brick['roll'], brick['pitch'], brick['yaw'])
        brick_pose.orientation = Quaternion(bqo[0], bqo[1], bqo[2], bqo[3])
        brick_reference_frame=brick['rframe']
        brick_id = brick['id']
        brick_ents.append(spawn_sdf(brick_id, brick_xml, "/", brick_pose, brick_reference_frame))
    
    for brick in built_bricks:
        ber = [brick['roll'], brick['pitch'], brick['yaw']] #brick_euler_rotation
        brick_pose = Pose(position=Point(x=brick['x'], y=brick['y'], z=brick['z']))
        brick_pose.position = Point(x=brick['x'], y=brick['y'], z=brick['z'])
        bqo = quaternion_from_euler(brick['roll'], brick['pitch'], brick['yaw'])
        brick_pose.orientation = Quaternion(bqo[0], bqo[1], bqo[2], bqo[3])
        brick_reference_frame=brick['rframe']
        brick_id = brick['id']
        brick_ents.append(spawn_sdf(brick_id, empty_xml, "/", brick_pose, brick_reference_frame))

#TODO: delete this line

if __name__ == "__main__":
    rospy.init_node("ik_pick_and_place_demo") #What is this?

    cleanup()

    hover_distance = 0.1 # meters
    # Starting Pose for left arm
    left_pose = Pose()
    left_pose.position.x = 0.579679836383
    left_pose.position.y = 0.283311769707
    left_pose.position.z = 0.213676720426
    left_pose.orientation.x = -0.0249590815779
    left_pose.orientation.y = 0.999649402929
    left_pose.orientation.z = 0.00737916180073
    left_pose.orientation.w = 0.00486450832011

    # Starting Pose for right arm
    right_pose = Pose()
    right_pose.position.x = 0.579679836383
    right_pose.position.y = -0.283311769707
    right_pose.position.z = 0.213676720426
    right_pose.orientation.x = -0.0249590815779
    right_pose.orientation.y = 0.999649402929
    right_pose.orientation.z = -0.00737916180073
    right_pose.orientation.w = 0.00486450832011

    left_pnp = PickAndPlace('left', hover_distance)
    right_pnp = PickAndPlace('right', hover_distance)

    # Go to initial position
    left_pnp.move_to_start(left_pnp.ik_request(left_pose))
    right_pnp.move_to_start(right_pnp.ik_request(right_pose))

    load_objects()

    print("loaded all objects, starting tf service thread")
    tf_service(init=True)

    ###################################HACKING BEGIN



    left_pick = otc.tf_lookup('a3')
    left_pnp.pick(left_pick)

    left_place_pos = otc.tf_lookup('h1')

    # xpos = built_bricks[0]
    # zqtr = quaternion_from_euler(xpos['roll'], xpos['pitch'], xpos['yaw'])

    # left_place_pos = Pose()
    # left_place_pos.position.x = left_place_pre.position.x + xpos['x']
    # left_place_pos.position.y = left_place_pre.position.y + xpos['y']
    # left_place_pos.position.z = left_place_pre.position.z + xpos['z']
    # left_place_pos.orientation.x = left_place_pre.orientation.x + zqtr[0]
    # left_place_pos.orientation.y = left_place_pre.orientation.y + zqtr[1]
    # left_place_pos.orientation.z = left_place_pre.orientation.z + zqtr[2]
    # left_place_pos.orientation.w = left_place_pre.orientation.w + zqtr[3]

    left_pnp.place(left_place_pos)




    ###################################HACKING END

    print('Done with task, enter x to kill tf service')
    while running:
        a = raw_input()
        if a == 'x':
            print("x has been caught, killing tf service")
            running = False
            tf_service(init=False)


    print("Done, exiting")
    exit(0)




