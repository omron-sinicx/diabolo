#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, OMRON SINIC X
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of OMRON SINIC X nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Felix von Drigalski, Devwrat Joshi

import sys
import copy
import rospy
import tf_conversions
import tf
from math import pi
import math
import thread
import os

import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseArray
import moveit_msgs.msg
import shape_msgs.msg
import visualization_msgs.msg
import diabolo_gazebo.msg
from diabolo_play.srv import SetInitialStickPositionsRequest, SetInitialStickPositions
import pandas as pd
import numpy as np
from gazebo_msgs.srv import (
    DeleteModel,
    DeleteModelRequest,
    SpawnModel,
    SpawnModelRequest,
)
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest
import rospkg
from diabolo_gazebo.msg import DiaboloState
from diabolo_play.diabolo_state_recorder import DiaboloSimRecorder


class ExperimentPlaybackClass:
    """
    Reads in experiment data and plays it back by publishing markers to Rviz.
    """

    def __init__(self):
        rospy.init_node("diabolo_play", anonymous=True)
        self._rospack = rospkg.RosPack()
        self.diabolo_urdf_pack = self._rospack.get_path("diabolo_gazebo")
        self.diabolo_urdf_file_path = os.path.join(
            self.diabolo_urdf_pack, "urdf", "diabolo.urdf"
        )
        self._package_directory = self._rospack.get_path("diabolo_play")
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.marker_count = 0
        self.marker_pub = rospy.Publisher(
            "visualization_markers", visualization_msgs.msg.Marker, queue_size=100
        )
        self.marker_array_pub = rospy.Publisher(
            "visualization_marker_array",
            visualization_msgs.msg.MarkerArray,
            queue_size=100,
        )
        self.pub_stick_poses = rospy.Publisher(
            "/diabolo_stick_poses",
            geometry_msgs.msg.PoseArray,
            queue_size=50,
            latch=True,
        )
        self.pub_diabolo_position = rospy.Publisher(
            "/experiment_diabolo_position", geometry_msgs.msg.Pose, queue_size=50
        )
        self.diabolo_state_pub = rospy.Publisher(
            "/experiment_diabolo_state", DiaboloState, queue_size=1
        )
        self.set_robots_initial_position_service = rospy.ServiceProxy(
            "/initialize_robots_from_stick_positions", SetInitialStickPositions
        )
        self.pause_gazebo_service = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause_gazebo_service = rospy.ServiceProxy(
            "/gazebo/unpause_physics", Empty
        )
        self.create_markers()
        self.sim_recorder = None  # This will be filled with a DiaboloSimRecorder type if running automated trials
        self.current_rot_velocity = 0.0  # TODO: Calculate rotational velocity from experiment data and store here
        # If this is true, the intermediate frames are displayed. Useful if the calibration seems off, or the rotations are not correct.
        self.DEBUG_DISPLAY = False
        # This parameter is set by the gazebo launch file if robots are being spawned in gazebo
        # If the parameter is true, the program will wait for the service to initialize robot positions
        self.using_robots = rospy.get_param("/using_gazebo_robots", False)
        if self.using_robots:
            print("Using robots in gazebo")
        else:
            print("Not using robots")
        self.trial_list = []
        self.filename = ""

        print("Started experiment playback class")

    def create_markers(self):
        # Create marker objects from the meshes
        self.diabolo_shell_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_shell.stl",
            color=(1, 0, 0),
            scale=[0.001, 0.001, 0.001],
            namespace="",
        )
        self.diabolo_fixator_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_fixators.stl",
            color=(0.1, 0.1, 0.1),
            scale=[0.001, 0.001, 0.001],
            namespace="",
        )
        self.diabolo_axis_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_axis.stl",
            color=(0.7, 0.7, 0.7),
            scale=[0.001, 0.001, 0.001],
            namespace="",
        )
        self.stick_left_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_stick.stl",
            color=(153 / 255.0, 75 / 255.0, 0.1),
            scale=[0.001, 0.001, 0.001],
            namespace="",
        )
        self.stick_right_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_stick.stl",
            color=(153 / 255.0, 75 / 255.0, 0.1),
            scale=[0.001, 0.001, 0.001],
            namespace="",
        )
        self.holder_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_mount.stl",
            color=(1, 1, 200 / 255.0),
            scale=[0.001, 0.001, 0.001],
            namespace="",
        )

        # Add the string
        self.line_segments_marker = self._make_marker_from_mesh(
            "", color=(204 / 255.0, 100 / 255.0, 0), namespace=""
        )
        self.line_segments_marker.type = visualization_msgs.msg.Marker.LINE_STRIP
        self.line_segments_marker.points.append(
            geometry_msgs.msg.Point(1, 1, 1)
        )  # The left stick tip
        self.line_segments_marker.points.append(
            geometry_msgs.msg.Point(0, 0, 0)
        )  # The diabolo center
        self.line_segments_marker.points.append(
            geometry_msgs.msg.Point(-1, -1, 1)
        )  # The right stick tip
        self.line_segments_marker.scale.x = 0.005  # line width

        self.sphere_marker_1 = self._make_marker_from_mesh(
            "",
            color=(204 / 255.0, 100 / 255.0, 0.5),
            scale=[0.05, 0.05, 0.05],
            namespace="",
        )
        self.sphere_marker_1.type = visualization_msgs.msg.Marker.SPHERE
        self.sphere_marker_2 = self._make_marker_from_mesh(
            "",
            color=(204 / 255.0, 100 / 255.0, 0.5),
            scale=[0.05, 0.05, 0.05],
            namespace="",
        )
        self.sphere_marker_2.type = visualization_msgs.msg.Marker.SPHERE
        self.sphere_marker_3 = self._make_marker_from_mesh(
            "",
            color=(204 / 255.0, 100 / 255.0, 0.5),
            scale=[0.05, 0.05, 0.05],
            namespace="",
        )
        self.sphere_marker_3.type = visualization_msgs.msg.Marker.SPHERE

    def read_transformed_experiment_data(
        self, filename="experiments/output/2020-04-05/Take_diabolo_red.csv"
    ):
        # This is a different function because the header is formatted differently in the transformed CSV file
        self.experiment_df = pd.read_csv(
            os.path.join(self._package_directory, filename), header=[0, 1, 2]
        )
        self.diabolo_data = self.experiment_df["diabolo_red"]
        self.stick_left_data = self.experiment_df["stick_left"]
        self.stick_right_data = self.experiment_df["stick_right"]
        self.rot_velocity_data = self.experiment_df["rot_velocity"]

        if self.DEBUG_DISPLAY:
            print("diabolo_data.head() = ")
            print(self.diabolo_data.head())
            print("stick_left.head() = ")
            print(self.stick_left_data.head())

    def _make_marker_from_mesh(
        self,
        mesh_filename="package://diabolo_play/meshes/diabolo_shell.stl",
        namespace="diabolo",
        scale=(1, 1, 1),
        color=(1, 1, 1),
    ):
        """
        Based on the 'makeMesh()' function from 'moveit_commander/planning_scene_interface.py'
        pose is a PoseStamped object.
        """
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = self.marker_count
        self.marker_count = self.marker_count + 1
        marker.type = visualization_msgs.msg.Marker.MESH_RESOURCE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.mesh_resource = mesh_filename
        return marker

    def flip_quaternion_y(self, q):
        """
        Returns a flipped geometry_msgs quaternion (rotated 180 degrees around y).
        """
        q0 = [q.x, q.y, q.z, q.w]
        q_flip = tf_conversions.transformations.quaternion_from_euler(0, math.pi, 0)
        q_res = tf_conversions.transformations.quaternion_multiply(q0, q_flip)
        return geometry_msgs.msg.Quaternion(*q_res)

    def publish_stick_and_diabolo_poses(self, poses):
        """
        poses needs to be a dict containing "diabolo", "stick_left", "stick_right" poses as geometry_msgs.msg.Pose
        """
        # Adding an empty pose as the robot controller requires a pose array msg
        # Check that the stick poses do not contain nan
        sl = np.array(
            [
                poses["stick_left"].position.x,
                poses["stick_left"].position.y,
                poses["stick_left"].position.z,
            ]
        )
        sr = np.array(
            [
                poses["stick_right"].position.x,
                poses["stick_right"].position.y,
                poses["stick_right"].position.z,
            ]
        )
        a = np.hstack((sl, sr))
        if not np.isnan(a).any():
            pose_array = PoseArray()
            pose_array.poses.append(poses["stick_left"])
            pose_array.poses.append(poses["stick_right"])
            self.pub_stick_poses.publish(pose_array)
        self.pub_diabolo_position.publish(poses["diabolo"])

    def update_and_publish_markers(self, poses):
        """
        poses needs to be a dict containing "diabolo", "stick_left", "stick_right" poses as geometry_msgs.msg.Pose
        """
        self.diabolo_shell_marker.pose = poses["diabolo"]
        self.diabolo_fixator_marker.pose = poses["diabolo"]
        self.diabolo_axis_marker.pose = poses["diabolo"]
        self.stick_left_marker.pose = poses["stick_left"]
        self.stick_right_marker.pose = poses["stick_right"]
        # Flip orientations for correct display of the sticks
        self.stick_left_marker.pose.orientation = self.flip_quaternion_y(
            self.stick_left_marker.pose.orientation
        )
        self.stick_right_marker.pose.orientation = self.flip_quaternion_y(
            self.stick_right_marker.pose.orientation
        )

        self.line_segments_marker.points[0] = poses["stick_left"].position
        self.line_segments_marker.points[1] = poses["diabolo"].position
        self.line_segments_marker.points[2] = poses["stick_right"].position

        marker_array = [
            self.diabolo_shell_marker,
            self.diabolo_fixator_marker,
            self.diabolo_axis_marker,
            self.stick_left_marker,
            self.stick_right_marker,
            self.line_segments_marker,
        ]
        self.marker_array_pub.publish(marker_array)

        # Publish TF frames
        for name in ["diabolo", "stick_left", "stick_right"]:
            p = poses[name]
            if not math.isnan(p.position.x):
                self.tf_broadcaster.sendTransform(
                    (p.position.x, p.position.y, p.position.z),
                    (
                        p.orientation.x,
                        p.orientation.y,
                        p.orientation.z,
                        p.orientation.w,
                    ),
                    rospy.Time.now(),
                    name,
                    "world",
                )

        if self.DEBUG_DISPLAY:
            for (p, name) in zip(
                [
                    self.diabolo_original_pose,
                    self.diabolo_intermediate_pose,
                    self.stick_left_original_pose,
                    self.stick_right_original_pose,
                ],
                [
                    "diabolo_original",
                    "diabolo_intermediate",
                    "stick_left_original",
                    "stick_right_original",
                ],
            ):
                if not math.isnan(p.position.x):
                    self.tf_broadcaster.sendTransform(
                        (p.position.x, p.position.y, p.position.z),
                        (
                            p.orientation.x,
                            p.orientation.y,
                            p.orientation.z,
                            p.orientation.w,
                        ),
                        rospy.Time.now(),
                        name,
                        "world",
                    )

    # To spawn the diabolo in gazebo. Automatically spawns diabolo at the pose defined at the first line of experiment data
    def _spawn_diabolo_in_gazebo(self):
        # Create service proxy
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        diabolo_pose = self.dataframe_line_to_pose(self.diabolo_data, 0)
        # Load URDF
        with open(self.diabolo_urdf_file_path, "r") as f:
            poses = dict()
            model_xml = f.read()
            # Spawn model
            req = SpawnModelRequest()
            req.model_name = "diabolo"
            # req.initial_pose = diabolo_pose
            pose = Pose()
            poses["diabolo"] = self.dataframe_line_to_pose(self.diabolo_data, 0)
            req.initial_pose.position = poses["diabolo"].position
            req.model_xml = model_xml
            req.robot_namespace = "/"
            req.reference_frame = "world"
            if spawn_model(req).success:
                print("Spawning diabolo at ")
                print(diabolo_pose)
            rospy.sleep(0.2)

    # Takes diabolo sim parameters as argument.
    # parameters[0] = /pv_pre_cap_scaling_factor
    # parameters[1] = /pv_cap_scaling_factor
    # parameters[2] = /pv_post_cap_scaling_factor
    # parameters[3] = /constrained_velocity_scaling_factor
    def initialize_sim_diabolo(self, parameters=(1.0, 1.0, 1.0, 1.0)):
        # Set the pull velocity parameters
        rospy.set_param("/pv_pre_cap_scaling_factor", parameters[0])
        rospy.set_param("/pv_cap_scaling_factor", parameters[1])
        rospy.set_param("/pv_post_cap_scaling_factor", parameters[2])
        rospy.set_param("/constrained_velocity_scaling_factor", parameters[3])
        # Delete existing diabolo if present
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        rospy.wait_for_service("/gazebo/delete_model")
        req = DeleteModelRequest()
        req.model_name = "diabolo"
        try:
            if not delete_model(req):
                print("There was no diabolo spawned")
        except:
            raise

        # Set initial postions as parameters on the parameter server
        poses = dict()
        poses["stick_left"] = self.dataframe_line_to_pose(self.stick_left_data, 0)
        left_pos = poses["stick_left"].position
        poses["stick_right"] = self.dataframe_line_to_pose(self.stick_right_data, 0)
        right_pos = poses["stick_right"].position
        print(left_pos)
        print(right_pos)
        rospy.set_param(
            "/right_stick_initial_position",
            [float(right_pos.x), float(right_pos.y), float(right_pos.z)],
        )
        rospy.set_param(
            "/left_stick_initial_position",
            [float(left_pos.x), float(left_pos.y), float(left_pos.z)],
        )
        # The initial rotational velocity of the diabolo
        rospy.set_param(
            "/diabolo_initial_rot_velocity", float(self.rot_velocity_data.iloc[0][0])
        )

        print("Done setting params")
        self._spawn_diabolo_in_gazebo()
        return True

    def store_diabolo_state(self, pose):
        ds = DiaboloState()
        ds.header.stamp = rospy.Time.now()
        ds.header.frame_id = "world"
        ds.pose = pose
        ds.rot_velocity = self.current_rot_velocity
        # TODO: Store and publish translation velocity (Not sure if required)
        self.sim_recorder.store_data_point(ds)
        self.diabolo_state_pub.publish(ds)

    def dataframe_line_to_pose(self, dataframe, line_number=1):
        """
        Takes a dataframe containing one rigid body as formatted by Motive (with rotation in Euler angles in degrees)
        (e.g. self.diabolo_data) and returns the pose at the index line_number.
        """
        quat = dataframe["Rotation"].iloc[line_number].tolist()
        translation = dataframe["Position"].iloc[line_number].values
        pose = geometry_msgs.msg.Pose()
        pose.position.x = translation[0]
        pose.position.y = translation[1]
        pose.position.z = translation[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def start_playback(self, loop=False, playback_speed_factor=1.0):
        print("Starting playback")
        thread.start_new_thread(self._run_playback, (loop, playback_speed_factor))

    def stop_playback(self):
        self.exit_playback_flag = True

    def initialize_robot_positions(self):
        poses = dict()
        poses["stick_left"] = self.dataframe_line_to_pose(self.stick_left_data, 0)
        poses["stick_right"] = self.dataframe_line_to_pose(self.stick_right_data, 0)

        print("Waiting for robot arm initialization service")
        rospy.wait_for_service("/initialize_robots_from_stick_positions")
        req = SetInitialStickPositionsRequest()
        print(type(poses["stick_left"]))
        print(poses["stick_right"])
        req.left_stick_position = poses["stick_left"].position
        req.right_stick_position = poses["stick_right"].position

        self.set_robots_initial_position_service(req)

    def _run_playback(self, loop=False, playback_speed_factor=1.0):
        "This function is meant to be called in a separate thread by play_experiment"
        rospy.sleep(
            0.1
        )  # This is to ensure the process will not start while sim time is stopped
        print("Starting playback 2")
        self.exit_playback_flag = False
        self.pause_playback_flag = False

        poses = dict()
        r = rospy.Rate(math.floor(120 * playback_speed_factor))
        for i in range(len(self.stick_left_data)):
            # print("Time of recording = " , self.experiment_df.loc[i,("Unnamed: 1_level_0", "Unnamed: 1_level_1", "Time (Seconds)")])
            # Read poses at new line
            poses["diabolo"] = self.dataframe_line_to_pose(self.diabolo_data, i)
            poses["stick_left"] = self.dataframe_line_to_pose(self.stick_left_data, i)
            poses["stick_right"] = self.dataframe_line_to_pose(self.stick_right_data, i)
            self.current_rot_velocity = self.rot_velocity_data.iloc[i][0]
            if self.DEBUG_DISPLAY:
                self.diabolo_original_pose = self.dataframe_line_to_pose(
                    self.diabolo_data_original, i
                )
                self.diabolo_intermediate_pose = self.dataframe_line_to_pose(
                    self.diabolo_data_intermediate, i
                )
                self.stick_left_original_pose = self.dataframe_line_to_pose(
                    self.stick_left_data_original, i
                )
                self.stick_right_original_pose = self.dataframe_line_to_pose(
                    self.stick_right_data_original, i
                )

            self.update_and_publish_markers(poses)  # Publish marker poses
            self.publish_stick_and_diabolo_poses(poses)
            if self.sim_recorder:
                self.store_diabolo_state(poses["diabolo"])
            if self.pause_playback_flag:
                rospy.loginfo("Playback is paused! Current frame is " + str(i))
                while self.pause_playback_flag:
                    rospy.sleep(0.2)
                rospy.loginfo("Playback is resumed! Current frame is " + str(i))
            if self.exit_playback_flag or rospy.is_shutdown():
                break
            r.sleep()
        rospy.loginfo("Playback has finished!")
        self.exit_playback_flag = True
        return

    def pause_playback(self):
        self.pause_playback_flag = True

    def unpause_playback(self):
        self.pause_playback_flag = False

    # This function will run sim trials with plugin parameter values starting with parameters[0], ending with parameters[1] and seperated
    # by parameters[2]
    def start_automated_trials(self, min_param, max_param, param_step):
        self.stop_playback()
        self.sim_recorder = DiaboloSimRecorder(self.filename)
        self.store_trial_list(min_param, max_param, param_step)
        if self.DEBUG_DISPLAY:
            print("Stored trial list. Length is " + str(len(self.trial_list)))
        rospy.sleep(0.2)
        self.exit_playback_flag = False
        thread.start_new_thread(self._run_automated_trials, ())

    def _run_automated_trials(self):
        # Run trials for each group of parameters

        for i in range(len(self.trial_list)):
            if not self.trial_list[i]["done"]:
                self.sim_recorder.start_recording_trial(self.trial_list[i]["params"])
                if self.using_robots:
                    self.initialize_robot_positions()
                req = EmptyRequest()
                # self.pause_gazebo_service(req)
                if self.initialize_sim_diabolo(parameters=self.trial_list[i]["params"]):
                    # print("///////////////" + str(type(self.trial_list[i]["params"])))
                    self.start_playback()
                # self.unpause_gazebo_service(req)
                while True:
                    if self.exit_playback_flag:
                        self.trial_list[i]["done"] = True
                        break
                    rospy.sleep(0.2)  # Wait for trial to end
                self.exit_playback_flag = False
                self.sim_recorder.end_recording_trial()

        # This will store a list of trials to conduct as a list of dicts.
        # Structure of the list:
        # [{params:(tuple of parameters), done: True/False}, {params:(different params), done: True/Fslse}, {}...]
        # parameter_resolution is the increment between parameter values
        # parameter_min and parameter_max are the min and max values of the parameters resp
        # IMP: Number of trials = ((max_param-min_param)/param_resolution)+1)^3. Can become very large

    def store_trial_list(self, min_param=0.0, max_param=1.0, param_step=0.5):
        self.trial_list = []
        p_list = []
        # pv_pre_cap = np.arange(min_param, max_param+param_step, param_step)
        # pv_cap = np.arange(min_param, max_param+param_step, param_step)
        # pv_post_cap = np.arange(min_param, max_param+param_step, param_step)
        # constrained_vel_cap = np.array((0.996, 0.997, 0.998))
        pv_pre_cap = np.array((0.15))
        pv_cap = np.array((0.15))
        pv_post_cap = np.array((0.1))
        constrained_vel_cap = np.array((0.999))
        p_list.extend((pv_pre_cap, pv_cap, pv_post_cap, constrained_vel_cap))
        # for i in p_list[0:3]:
        #   i[0] = 0.1 # We do not want the first scaling factor to be 0.0

        # Getting all the possible parameter permutations
        param_list = np.stack(
            np.meshgrid(pv_pre_cap, pv_cap, pv_post_cap, constrained_vel_cap), -1
        ).reshape(-1, len(p_list))

        # Store each parameter as a dict
        for i in range(param_list.shape[0]):
            d = {"params": tuple([round(a, 4) for a in param_list[i]]), "done": False}
            self.trial_list.append(d)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(
            "Usage: rosrun diabolo_play playback_for_diabolo_system.py <data file to use>"
        )

    else:
        file = sys.argv[1]
        rospack = rospkg.RosPack()
        filename = os.path.join(
            rospack.get_path("diabolo_play"),
            "experiments",
            "output",
            "2020-07-11_basic_physics",
            file,
        )
        if not os.path.exists(filename):
            print("Attempted to open: " + filename)
            print("This file does not exist")

        else:
            try:
                c = ExperimentPlaybackClass()
                c.filename = file
                i = 1
                while i and not rospy.is_shutdown():
                    rospy.loginfo(
                        "Enter 1 to load data (smoothed, with added offsets) of the experiment."
                    )
                    rospy.loginfo("Enter a to run automated simulation trials")
                    rospy.loginfo("Enter b to initialize robot positions.")
                    rospy.loginfo("Enter d to spawn the diabolo in gazebo")
                    rospy.loginfo(
                        "Enter s to start playback (ss, sss for 0.5x, 0.25x speed)."
                    )
                    rospy.loginfo("Enter p to pause playback.")
                    rospy.loginfo("Enter c to continue playback.")
                    rospy.loginfo("Enter t to stop playback.")
                    rospy.loginfo("Enter x to exit.")
                    i = raw_input()

                    if i == "a" or i == "A":
                        print(
                            "Enter values for minimum sim parameter, maximum sim parameter and parameter resolution, seperated by spaces"
                        )
                        p = raw_input().split()
                        if len(p) >= 3:
                            print(
                                "New parameters are: "
                                + str(float(p[0]))
                                + " "
                                + str(float(p[1]))
                                + " "
                                + str(float(p[2]))
                            )
                            par = (float(p[0]), float(p[1]), float(p[2]))
                            c.start_automated_trials(
                                min_param=par[0], max_param=par[1], param_step=par[2]
                            )
                        else:
                            print("Not enough parameters")

                    elif i == "1":
                        c.read_transformed_experiment_data(
                            filename=(
                                "experiments/output/2020-07-11_basic_physics/" + file
                            )
                        )
                    elif i == "b" or i == "B":
                        c.initialize_robot_positions()
                    elif i == "d" or i == "D":
                        print(
                            "Default parameters are (0.1, 0.1, 0.1, 1.0). Change? y/n"
                        )
                        a = raw_input()
                        if a == "y":
                            print("Enter the parameters, seperated by spaces")
                            p = raw_input().split()
                            if len(p) >= 4:
                                print(
                                    "New parameters are: "
                                    + p[0]
                                    + " "
                                    + p[1]
                                    + " "
                                    + p[2]
                                    + " "
                                    + p[3]
                                )
                                c.initialize_sim_diabolo(
                                    parameters=(
                                        float(p[0]),
                                        float(p[1]),
                                        float(p[2]),
                                        float(p[3]),
                                    )
                                )
                            else:
                                print("Not enough parameters")
                        else:
                            c.initialize_sim_diabolo(
                                parameters=(0.1, 0.1, 0.1, 1.0)
                            )  # Set the diabolo plugin parameters and spawn the diabolo

                    elif i == "S" or i == "s":
                        c.start_playback()
                    elif i == "SS" or i == "ss":
                        c.start_playback(playback_speed_factor=0.5)
                    elif i == "SSS" or i == "sss":
                        c.start_playback(playback_speed_factor=0.25)
                    elif i == "SSSS" or i == "ssss":
                        c.start_playback(playback_speed_factor=0.125)
                    elif i == "P" or i == "p":
                        c.pause_playback()
                    elif i == "C" or i == "c":
                        c.unpause_playback()
                    elif i == "T" or i == "t":
                        c.stop_playback()
                    elif i == "x":
                        c.stop_playback()
                        break
                    elif i == "":
                        continue
            except rospy.ROSInterruptException:
                pass
