#!/usr/bin/env python

# Copyright (c) 2015-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Sawyer SDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy
from collections import OrderedDict

import rospy
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

import intera_interface
from sawyer_interface.moveit_interface import SawyerMoveitInterface


class SimulationObject(object):
    def __init__(self, name, model_filepath, model_type, pose,
                 reference_frame="world", hover_distance=.15,
                 overhead_orientation=Quaternion(
                             x=-0.011940598492311645,
                             y=0.9990539733303562,
                             z=-0.007206884011932187,
                             w=0.04119030593855785)):
        self.name = name
        self.filepath = model_filepath
        self.model_type = model_type
        self.pose = pose
        self.reference_frame = reference_frame
        self.hover_distance = .15
        self.overhead_orientation = overhead_orientation

    def load_gazebo_model(self):
        # Load Table SDF
        xml = ''
        with open(self.filepath, "r") as file:
            xml = file.read().replace('\n', '')
        if self.model_type == "sdf":
            # Spawn Table SDF
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            try:
                spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                resp_sdf = spawn_sdf(self.name, xml, "/", self.pose, self.reference_frame)
            except rospy.ServiceException, e:
                rospy.logerr("Spawn SDF service call failed: {0}".format(e))
        elif self.model_type == "urdf":
            # Spawn Red Block URDF
            rospy.wait_for_service('/gazebo/spawn_urdf_model')
            try:
                spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
                resp_urdf = spawn_urdf(self.name, xml, "/", self.pose, self.reference_frame)
            except rospy.ServiceException, e:
                rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    def get_current_pose(self):
        return copy.deepcopy(self.pose)


class PickAndPlace(object):
    def __init__(self, limb="right", tip_name="right_gripper_tip"):
        self._limb_name = limb  # string
        self._tip_name = tip_name  # string
        self._limb = intera_interface.Limb(limb)
        self._gripper = intera_interface.Gripper()

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        self._moveit_interface = SawyerMoveitInterface()
        self._moveit_interface.set_velocity_scaling(.5)
        self._moveit_interface.set_acceleration_scaling(.5)

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.loginfo("Current position: {}".format(self._limb.endpoint_pose()))

    def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        if rospy.is_shutdown():
            return
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, sim_object):
        rospy.loginfo("Approaching {}".format(sim_object.name))
        approach_pose = sim_object.get_current_pose()
        # approach with a pose the hover-distance above the requested pose
        approach_pose.position.z = approach_pose.position.z + sim_object.hover_distance
        approach_pose.orientation = sim_object.overhead_orientation
        rospy.loginfo("Approach pose: {}".format(approach_pose))
        self._moveit_interface.move_to_pose_target(approach_pose)

    def _retract(self, sim_object):
        target_pose = sim_object.get_current_pose()
        target_pose.position.z = target_pose.position.z + sim_object.hover_distance
        target_pose.orientation = sim_object.overhead_orientation
        rospy.loginfo("Retract pose: {}".format(target_pose))

        self._moveit_interface.move_to_pose_target(target_pose)

    def pick(self, sim_object):
        if rospy.is_shutdown():
            return

        self.gripper_open()
        self._approach(sim_object)
        target_pose = sim_object.get_current_pose()
        rospy.loginfo("Pick pose: {}".format(target_pose))
        self._moveit_interface.move_to_pose_target(target_pose)
        if rospy.is_shutdown():
            return
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract(sim_object)

    def place(self, sim_object, target_pose):
        # Need to set target pose.
        if rospy.is_shutdown():
            return
        # servo above pose
        self._approach(sim_object)
        # servo to pose

        self._moveit_interface.move_to_pose_target(target_pose)
        if rospy.is_shutdown():
            return
        # open the gripper
        self.gripper_open()
        sim_object.pose.position = target_pose.position
        # retract to clear object
        self._retract(sim_object)

def load_gazebo_models(sim_objects):
  for obj in sim_objects:
    obj.load_gazebo_model()

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("red_block")
        resp_delete = delete_model("green_block")
        resp_delete = delete_model("green_bowl")
        resp_delete = delete_model("red_bowl")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    """SDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Sawyer will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('planning_sim_demo')+"/models/"

    sim_objects = OrderedDict()
    sim_objects["cafe_table"] = SimulationObject("cafe_table", model_path + "cafe_table/model.sdf", model_type="sdf", pose=Pose(position=Point(x=0.75, y=0.0, z=0.0)))
    sim_objects["red_block"] = SimulationObject("red_block", model_path + "red_block/model.urdf", model_type="urdf", pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725)))
    sim_objects["green_block"] = SimulationObject("green_block", model_path + "green_block/model.urdf", model_type="urdf", pose=Pose(position=Point(x=0.4225, y=-0.1665, z=0.7725)))
    sim_objects["green_bowl"] = SimulationObject("green_bowl", model_path + "green_bowl/model.sdf", model_type="sdf", pose=Pose(position=Point(x=0.6000, y=0.1265, z=0.7725)))
    sim_objects["red_bowl"] = SimulationObject("red_bowl", model_path + "red_bowl/model.sdf", model_type="sdf", pose=Pose(position=Point(x=0.6000, y=-0.1265, z=0.7725)))

    # Load all simulation objects into Gazebo
    load_gazebo_models(sim_objects.values())

    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    limb = 'right'
    # Starting Joint angles for right arm
    starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4':  -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}
    pnp = PickAndPlace(limb)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.011945179403945185,
                             y=0.9990525677759885,
                             z=-0.007193759099379768,
                             w=0.04122532925175943)
    block_poses = list()
    # The Pose of the block in its initial location.
    # You may wish to replace these poses with estimates
    # from a perception node.
    block_poses.append(Pose(
        position=Point(x=0.45, y=0.155, z=-0.129),
        orientation=copy.deepcopy(overhead_orientation)))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
        position=Point(x=0.6, y=-0.1, z=-0.129),
        orientation=copy.deepcopy(overhead_orientation)))
    # Move to the desired starting angles
    print("Running. Ctrl-c to quit")
    pnp.move_to_start(starting_joint_angles)
    idx = 0
    while not rospy.is_shutdown():
        rospy.loginfo("\nPicking...")
        pnp.pick(sim_objects["red_block"])
        rospy.loginfo("\nPlacing...")
        idx = (idx+1) % len(block_poses)
        pnp.place(sim_objects["red_block"], block_poses[idx])
    return 0

if __name__ == '__main__':
    sys.exit(main())
