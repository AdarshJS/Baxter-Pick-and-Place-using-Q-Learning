#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

red_x = [0.6725, 0.6725, 0.6725] #Note Red is now yellow
red_y = [0.2, 0.05, -0.1]
blue_x = [0.6725, 0.6725]
blue_y = [0.275, 0.125]
green_x = 0.6725
green_y = -0.025
All_z = 0.7825
array = [0,0,0,0,0,0]
print red_x
class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
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
        rospy.sleep(1.0)
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
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

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
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
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

#-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose_red_1=Pose(position=Point(x=red_x[0], y=red_y[0], z=All_z)),
                       block_reference_frame_red_1="world",
                       block_pose_red_2=Pose(position=Point(x=red_x[1], y=red_y[1], z=All_z)),
                       block_reference_frame_red_2="world",
                       block_pose_red_3=Pose(position=Point(x=red_x[2], y=red_y[2], z=All_z)),
                       block_reference_frame_red_3="world",
                       block_pose_blue_1=Pose(position=Point(x=blue_x[0], y=blue_y[0], z=All_z)),
                       block_reference_frame_blue_1="world",
                       block_pose_blue_2=Pose(position=Point(x=blue_x[1], y=blue_y[1], z=All_z)),
                       block_reference_frame_blue_2="world",
                       block_pose_green=Pose(position=Point(x=green_x, y=green_y, z=All_z)),
                       block_reference_frame_green="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml_red = ''
    block_xml_blue = ''
    
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml_red=block_file.read().replace('\n', '')
    with open (model_path + "block/model_blue.urdf", "r") as block_file:
        block_xml_blue=block_file.read().replace('\n', '')
    with open (model_path + "block/model_green.urdf", "r") as block_file:
        block_xml_green=block_file.read().replace('\n', '')            
#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------        
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # Spawn Block URDF------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_red_1", block_xml_red, "/",
                               block_pose_red_1, block_reference_frame_red_1)
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_red_2", block_xml_red, "/",
                               block_pose_red_2, block_reference_frame_red_2)

        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_red_3", block_xml_red, "/",
                               block_pose_red_3, block_reference_frame_red_3)

        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_blue_1", block_xml_blue, "/",
                               block_pose_blue_1, block_reference_frame_blue_1)

        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_blue_2", block_xml_blue, "/",
                               block_pose_blue_2, block_reference_frame_blue_2)

        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block_green", block_xml_green, "/",
                               block_pose_green, block_reference_frame_green)

    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block_red_1")
        resp_delete = delete_model("block_red_2")
        resp_delete = delete_model("block_red_3")
        resp_delete = delete_model("block_blue_1")
        resp_delete = delete_model("block_blue_2")
        resp_delete = delete_model("block_green")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def callback(data):
    #print rospy.get_name(), "I heard %s"%str(data.data)
    global array
    array = data.data
    #print array

def main():
    """RSDK Inverse Kinematics Pick and Place Example
    
    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")
    rospy.Subscriber("floats", numpy_msg(Floats), callback)
    global array
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    pnp = PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
    block_poses_red = list()
    block_poses_blue = list()

    array = array.astype(int)
    print array
    
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.

    X_initial_array = [0.6725, 0.6725, 0.6725, 0.6725, 0.6725, 0.6725]
    Y_initial_array = [0.275,0.2,0.125,0.05,-0.025,-0.1]
    Z_initial_array = [0.7825,0.7825,0.7825,0.7825,0.7825,0.7825]

    X_final_array = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
    Y_final_array = [0.1265, 0.0515, -0.0235, 0.089, 0.016, 0.0515]
    Z_final_array = [-0.129,-0.129, -0.129, -0.079,-0.079,-0.029]
    j = 0
    for i in range(0,6):
        
        block_poses_red.append(Pose(
            position=Point(x=(X_initial_array[array[i]]+0.0275), y=(Y_initial_array[array[i]]+0.0235), z=(Z_initial_array[array[i]]-0.9115)), #0.7 0.15 -0.129
            orientation=overhead_orientation))

        block_poses_red.append(Pose(
            position=Point(x=(X_final_array[j]), y=(Y_final_array[j]), z=(Z_final_array[j])),
            orientation=overhead_orientation))

        j+=1


    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    idx = 0
    idx_2 = 0
    while idx<=12:
        print("\nPicking...")
        pnp.pick(block_poses_red[idx])
        print("\nPlacing...")
        idx = (idx+1) % len(block_poses_red)
        pnp.place(block_poses_red[idx])
        idx = (idx+1)
    
        
    return 0

if __name__ == '__main__':
    #listener()
    sys.exit(main())
