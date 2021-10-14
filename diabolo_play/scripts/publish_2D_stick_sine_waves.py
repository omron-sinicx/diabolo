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
import time
from std_srvs.srv import Empty
import moveit_msgs.msg
import shape_msgs.msg
import visualization_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray

import pandas as pd
import numpy as np

import rospkg
from diabolo_play.srv import SetInitialStickPositionsRequest, SetInitialStickPositions


class StickPosePublishClass:
    """
    Reads in experiment data and plays it back by publishing markers to Rviz.
    """

    def __init__(self):
        rospy.init_node("diabolo_play", anonymous=True)
        self._rospack = rospkg.RosPack()
        self._package_directory = self._rospack.get_path("diabolo_play")
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.init_stick_marker()
        self.dir_1_amplitude = 0.3
        self.dir_2_amplitude = 0.3
        self.dir_1_ang_freq = 0.3
        self.dir_2_ang_freq = 0.3
        self.right_stick_amp_scaling_factor = 1.0
        self.left_stick_amp_scaling_factor = 1.0
        self.angular_frequency = 0.1
        self.axis_of_vibration = "x"  # The axis the stick positions turn around
        self.sticks_in_phase = False
        self.both_sticks_moving = True
        self.string_length = 1.46
        self.left_stick_center_coords = (
            0.5,
            0.5,
            1.2,
        )  # TODO: Set these to suitable values
        self.right_stick_center_coords = (
            0.5,
            -0.5,
            1.2,
        )  # TODO: Set these to suitable values
        self.left_stick_initial_coords = (
            0.5,
            0.4,
            1.0,
        )  # TODO: Set these to suitable values
        self.right_stick_initial_coords = (
            0.5,
            -0.4,
            1.0,
        )  # TODO: Set these to suitable values
        self.left_stick_current_coords = [a for a in self.left_stick_initial_coords]
        self.right_stick_current_coords = [a for a in self.right_stick_initial_coords]
        self.use_simulated_diabolo = rospy.get_param(
            "/use_simulated_diabolo", False
        )  # Need to reset model poses in gazebo if this is true
        self.use_robot_arms = rospy.get_param("/use_robot_arms", True)
        if (
            self.use_simulated_diabolo
        ):  # Only reset gazebo if using diabolo plugin. Otherwise use moveit to move robots to initial positions
            self.gazebo_pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
            self.gazebo_unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
            self.gazebo_reset = rospy.ServiceProxy("/gazebo/reset_world", Empty)
            self.changed_parameter_flag = False  # If the user changed any parameters, restart the simulation. Else, unpause it

        if (
            self.use_robot_arms
        ):  # Service proxy to return robot arms to new initial positions
            self.set_robots_initial_position = rospy.ServiceProxy(
                "/initialize_robots_from_stick_positions", SetInitialStickPositions
            )

        self.right_stick_marker_pub = rospy.Publisher(
            "/right_stick_marker", Marker, queue_size=1
        )
        self.left_stick_marker_pub = rospy.Publisher(
            "/left_stick_marker", Marker, queue_size=1
        )
        # self.marker_count = 0
        # self.marker_pub = rospy.Publisher("visualization_markers", visualization_msgs.msg.Marker, queue_size = 100)
        # self.marker_array_pub = rospy.Publisher("visualization_marker_array", visualization_msgs.msg.MarkerArray, queue_size = 100)
        # self.create_markers()

        # If this is true, the intermediate frames are displayed. Useful if the calibration seems off, or the rotations are not correct.
        self.DEBUG_DISPLAY = False

        # self.pub_stick_left = rospy.Publisher("/diabolo_gazebo_node/diabolo_stick_pose_left", Point, queue_size=50)
        # self.pub_stick_right = rospy.Publisher("/diabolo_gazebo_node/diabolo_stick_pose_right", Point, queue_size=50)
        # Convention: First left stick, then right stick
        self.pub_stick_poses = rospy.Publisher(
            "/diabolo_stick_poses", PoseArray, queue_size=50
        )
        print("Started experiment publish class")

    def print_current_parameters(self):
        print("Amplitude of vibration dir_1 = " + str(self.dir_1_amplitude))
        print("Amplitude of vibration dir_2 = " + str(self.dir_2_amplitude))
        print("Angular frequency of vibration dir_1 = " + str(self.dir_1_ang_freq))
        print("Angular frequency of vibration dir_2 = " + str(self.dir_2_ang_freq))

        print("Axis of vibration = " + str(self.axis_of_vibration))

    def start_publish(self, loop=False, publish_speed_factor=1.0):
        print("Starting publish")
        thread.start_new_thread(self._run_publish, (loop,))

    def stop_publish(self):
        self.exit_publish_flag = True

    def _run_publish(self, loop=False, publish_speed_factor=1.0):
        "This function is meant to be called in a separate thread by play_experiment"
        print("Starting publish 2")
        self.exit_publish_flag = False
        self.pause_publish_flag = False

        r = rospy.Rate(100)
        initial_time = rospy.get_time()
        while True:

            left_stick_pos = Point()
            right_stick_pos = Point()
            if self.axis_of_vibration == "x":
                left_offset_x = 0.0
                right_offset_x = 0.0
                left_offset_y = self.dir_1_amplitude * math.sin(
                    2
                    * math.pi
                    * (rospy.get_time() - initial_time)
                    * self.dir_1_ang_freq
                )
                right_offset_z = self.dir_2_amplitude * math.cos(
                    2
                    * math.pi
                    * (rospy.get_time() - initial_time)
                    * self.dir_2_ang_freq
                )
                right_offset_y = self.dir_1_amplitude * math.sin(
                    2
                    * math.pi
                    * -(rospy.get_time() - initial_time)
                    * self.dir_1_ang_freq
                )
                left_offset_z = self.dir_2_amplitude * math.cos(
                    2
                    * math.pi
                    * (rospy.get_time() - initial_time)
                    * self.dir_2_ang_freq
                )
            elif self.axis_of_vibration == "y":
                left_offset_x = self.dir_1_amplitude * math.sin(
                    2
                    * math.pi
                    * (rospy.get_time() - initial_time)
                    * self.dir_1_ang_freq
                )
                right_offset_x = self.dir_1_amplitude * math.sin(
                    2
                    * math.pi
                    * (rospy.get_time() - initial_time)
                    * self.dir_1_ang_freq
                )
                left_offset_y = 0.0
                right_offset_y = 0.0
                left_offset_z = self.dir_2_amplitude * math.cos(
                    2
                    * math.pi
                    * -(rospy.get_time() - initial_time)
                    * self.dir_2_ang_freq
                )
                right_offset_z = self.dir_2_amplitude * math.cos(
                    2
                    * math.pi
                    * -(rospy.get_time() - initial_time)
                    * self.dir_2_ang_freq
                )
            elif self.axis_of_vibration == "z":
                left_offset_x = self.dir_1_amplitude * math.sin(
                    2
                    * math.pi
                    * (rospy.get_time() - initial_time)
                    * self.dir_1_ang_freq
                )
                right_offset_x = self.dir_1_amplitude * math.sin(
                    2
                    * math.pi
                    * (rospy.get_time() - initial_time)
                    * self.dir_1_ang_freq
                )
                left_offset_y = self.dir_2_amplitude * math.cos(
                    2
                    * math.pi
                    * -(rospy.get_time() - initial_time)
                    * self.dir_2_ang_freq
                )
                right_offset_y = self.dir_2_amplitude * math.cos(
                    2
                    * math.pi
                    * -(rospy.get_time() - initial_time)
                    * self.dir_2_ang_freq
                )
                left_offset_z = 0.0
                right_offset_z = 0.0

            right_stick_pos.x = self.right_stick_initial_coords[0] + (
                right_offset_x * self.right_stick_amp_scaling_factor
            )
            right_stick_pos.y = self.right_stick_initial_coords[1] + (
                right_offset_y * self.right_stick_amp_scaling_factor
            )
            right_stick_pos.z = self.right_stick_initial_coords[2] + (
                right_offset_z * self.right_stick_amp_scaling_factor
            )

            left_stick_pos.x = self.left_stick_initial_coords[0] + (
                left_offset_x * self.left_stick_amp_scaling_factor
            )
            left_stick_pos.y = self.left_stick_initial_coords[1] + (
                left_offset_y * self.left_stick_amp_scaling_factor
            )
            left_stick_pos.z = self.left_stick_initial_coords[2] + (
                left_offset_z * self.left_stick_amp_scaling_factor
            )

            # Adding an empty pose as the robot controller requires a pose array msg
            pose_array = PoseArray()
            pose_l = Pose()
            pose_r = Pose()
            pose_l.position = left_stick_pos
            pose_r.position = right_stick_pos
            pose_array.poses.append(pose_l)
            pose_array.poses.append(pose_r)
            self.pub_stick_poses.publish(pose_array)
            self.publish_stick_marker(left_stick_pos, right_stick_pos)
            if self.pause_publish_flag:
                rospy.loginfo("Publishing stick poses is paused!")
                while self.pause_publish_flag:
                    rospy.sleep(0.2)
                rospy.loginfo("Publishing stick poses is resumed!")
            if self.exit_publish_flag or rospy.is_shutdown():
                print("Done with thread while loop")
                break
            r.sleep()
        rospy.loginfo("Stopping...")
        return

    def pause_publish(self):
        self.pause_publish_flag = True

    def unpause_publish(self):
        self.pause_publish_flag = False

    def set_initial_stick_pos(
        self,
    ):  # The initial position depends on the axis. The stick will be placed at (dir_1_amplitude, 0.)
        initial_coords_left = [0.0, 0.0, 0.0]
        initial_coords_right = [0.0, 0.0, 0.0]
        if self.axis_of_vibration == "x":
            initial_coords_left[
                1
            ] = self.dir_1_amplitude  # y is dir_1 if rotation around x axis
            initial_coords_right[
                1
            ] = -self.dir_1_amplitude  # y is dir_1 if rotation around x axis
        elif self.axis_of_vibration == "y":
            initial_coords_left[
                0
            ] = self.dir_1_amplitude  # y is dir_1 if rotation around x axis
            initial_coords_right[
                0
            ] = self.dir_1_amplitude  # y is dir_1 if rotation around x axis
        elif self.axis_of_vibration == "z":
            initial_coords_left[
                0
            ] = self.dir_1_amplitude  # y is dir_1 if rotation around x axis
            initial_coords_right[
                0
            ] = self.dir_1_amplitude  # y is dir_1 if rotation around x axis
        self.left_stick_initial_coords = [
            i + j for i, j in zip(self.left_stick_center_coords, initial_coords_left)
        ]
        self.right_stick_initial_coords = [
            i + j for i, j in zip(self.right_stick_center_coords, initial_coords_right)
        ]

    def init_stick_marker(self):
        marker_scale = 0.1
        # right stick
        self.right_stick_marker = Marker()
        self.right_stick_marker.header.frame_id = "world"
        self.right_stick_marker.header.stamp = rospy.Time.now()

        self.right_stick_marker.ns = "right_stick"
        self.right_stick_marker.id = 0

        self.right_stick_marker.action = Marker.ADD

        self.right_stick_marker.pose.orientation.x = 0.0
        self.right_stick_marker.pose.orientation.y = 0.0
        self.right_stick_marker.pose.orientation.z = 1.0
        self.right_stick_marker.pose.orientation.w = 0.0

        self.right_stick_marker.color.r = 0
        self.right_stick_marker.color.g = 0
        self.right_stick_marker.color.b = 1
        self.right_stick_marker.color.a = 1.0

        self.right_stick_marker.scale.x = marker_scale
        self.right_stick_marker.scale.y = marker_scale
        self.right_stick_marker.scale.z = marker_scale

        self.right_stick_marker.lifetime = rospy.Duration()

        self.right_stick_marker.type = 2

        # left stick
        self.left_stick_marker = Marker()
        self.left_stick_marker.header.frame_id = "world"
        self.left_stick_marker.header.stamp = rospy.Time.now()

        self.left_stick_marker.ns = "left_stick"
        self.left_stick_marker.id = 0

        self.left_stick_marker.action = Marker.ADD

        self.left_stick_marker.pose.orientation.x = 0.0
        self.left_stick_marker.pose.orientation.y = 0.0
        self.left_stick_marker.pose.orientation.z = 1.0
        self.left_stick_marker.pose.orientation.w = 0.0

        self.left_stick_marker.color.r = 0
        self.left_stick_marker.color.g = 0
        self.left_stick_marker.color.b = 1
        self.left_stick_marker.color.a = 1.0

        self.left_stick_marker.scale.x = marker_scale
        self.left_stick_marker.scale.y = marker_scale
        self.left_stick_marker.scale.z = marker_scale

        self.left_stick_marker.lifetime = rospy.Duration()

        self.left_stick_marker.type = 2

    def publish_stick_marker(self, left_pos, right_pos):
        # right stick
        self.right_stick_marker.pose.position = left_pos
        self.right_stick_marker_pub.publish(self.right_stick_marker)

        # left stick
        self.left_stick_marker.pose.position = right_pos
        self.left_stick_marker_pub.publish(self.left_stick_marker)

    def reset(
        self,
    ):  # Currently only capable of resetting robot arms initial positions.
        self.stop_publish()
        self.set_initial_stick_pos()

        if self.use_robot_arms:  # Reset arms to initial positions
            print("Waiting for robot arm initialization service")
            rospy.wait_for_service("/initialize_robots_from_stick_positions")
            req = SetInitialStickPositionsRequest()
            req.left_stick_position.x = self.left_stick_initial_coords[0]
            req.left_stick_position.y = self.left_stick_initial_coords[1]
            req.left_stick_position.z = self.left_stick_initial_coords[2]

            req.right_stick_position.x = self.right_stick_initial_coords[0]
            req.right_stick_position.y = self.right_stick_initial_coords[1]
            req.right_stick_position.z = self.right_stick_initial_coords[2]
            self.set_robots_initial_position(req)


if __name__ == "__main__":
    try:
        c = StickPosePublishClass()

        i = 1
        while i and not rospy.is_shutdown():

            rospy.loginfo("Enter i to initialize the simulation.")
            rospy.loginfo("Enter s to start publish (ss, sss for 0.5x, 0.25x speed).")
            rospy.loginfo("Enter p to pause publish.")
            rospy.loginfo("Enter c to continue publish.")
            rospy.loginfo("Enter t to stop publish.")
            rospy.loginfo("Enter r to check or change the simulation parameters")
            rospy.loginfo("Enter e to exit.")
            i = raw_input()
            if i == "i" or i == "I":
                c.reset()
            elif i == "S" or i == "s":
                c.start_publish()
            elif i == "SS" or i == "ss":
                c.start_publish(0.5)
            elif i == "SSS" or i == "sss":
                c.start_publish(0.25)
            elif i == "SSSS" or i == "ssss":
                c.start_publish(0.125)
            elif i == "SSSSS" or i == "sssss":
                c.start_publish(1.0 / 16.0)
            elif i == "P" or i == "p":
                c.pause_publish()
            elif i == "C" or i == "c":
                c.unpause_publish()
            elif i == "T" or i == "t":
                c.stop_publish()
            elif i == "e":
                c.stop_publish()
                break
            elif i == "r" or i == "R":
                # c.gazebo_pause()
                i = "y"
                c.print_current_parameters()
                # Change to true if any parameters are changed, and restart the simulation
                c.changed_parameter_flag = False
                while i == "y":
                    print("Change parameters? y for yes, n for no")
                    i = raw_input()
                    if i == "n" or i == "N":
                        print("Done changing parameters")

                        break
                    elif i == "y" or "Y":
                        print("Enter a to change amplitude")
                        print("Enter w to change angular frequency")
                        print("Enter x to change between roll, pitch and yaw")
                        print("Enter s to change stick amplitude scaling factors")
                        print("Enter r to restart the current simulation")
                        print("Enter q to quit")

                        i = raw_input()

                        if i == "a" or i == "A":
                            print("Please enter amplitude in dir_1")
                            i = raw_input()
                            if i == "":
                                print("Leaving unchanged")
                            else:
                                try:
                                    f = float(i)
                                    c.dir_1_amplitude = f
                                    c.changed_parameter_flag = True
                                except TypeError:
                                    print(
                                        "Cannot change this to float! Keeping value unchanged"
                                    )

                            print("Please enter amplitude in dir_2")
                            i = raw_input()
                            if i == "":
                                print("Leaving unchanged")
                            else:
                                try:
                                    f = float(i)
                                    c.dir_2_amplitude = f
                                    c.changed_parameter_flag = True
                                except TypeError:
                                    print(
                                        "Cannot change this to float! Keeping value unchanged"
                                    )

                        elif i == "w" or i == "W":
                            print("Please enter angular frequency in dir_1")
                            i = raw_input()
                            if i == "":
                                print("Leaving unchanged")
                            else:
                                try:
                                    f = float(i)
                                    c.dir_1_ang_freq = f
                                    c.changed_parameter_flag = True
                                except TypeError:
                                    print(
                                        "Cannot change this to float! Keeping value unchanged"
                                    )

                            print("Please enter angular frequency in dir_2")
                            i = raw_input()
                            if i == "":
                                print("Leaving unchanged")
                            else:
                                try:
                                    f = float(i)
                                    c.dir_2_ang_freq = f
                                    c.changed_parameter_flag = True
                                except TypeError:
                                    print(
                                        "Cannot change this to float! Keeping value unchanged"
                                    )

                        elif i == "x" or i == "X":
                            print("Enter the axis of the sine wave ")
                            i = ""
                            while not i in ["x", "X", "y", "Y", "z", "Z"]:
                                print("Please enter x, y or z")
                                i = raw_input()

                                if i == "x" or i == "X":
                                    c.axis_of_vibration = "x"

                                elif i == "y" or i == "Y":
                                    c.axis_of_vibration = "y"

                                elif i == "z" or i == "Z":
                                    c.axis_of_vibration = "z"
                            c.changed_parameter_flag = True

                        elif i == "s" or i == "S":
                            print(
                                "Please enter the new left stick parameter, or press enter to leave unchanged"
                            )
                            i = raw_input()
                            if i == "":
                                print("Leaving unchanged")
                            else:
                                try:
                                    f = float(i)
                                    if f > 1.0:
                                        print(
                                            "Amplitude factors must be less than 1. Setting to 1.0"
                                        )
                                        f = 1.0
                                    c.left_stick_amp_scaling_factor = f
                                    c.changed_parameter_flag = True
                                except TypeError:
                                    print(
                                        "Cannot change this to float! Keeping value unchanged"
                                    )

                            print(
                                "Please enter the new right stick parameter, or press enter to leave unchanged"
                            )
                            i = raw_input()
                            if i == "":
                                print("Leaving unchanged")
                                break
                            else:
                                try:
                                    f = float(i)
                                    if f > 1.0:
                                        print(
                                            "Amplitude factors must be less than 1. Setting to 1.0 \n"
                                        )
                                        f = 1.0
                                    c.right_stick_amp_scaling_factor = f
                                    c.changed_parameter_flag = True
                                except TypeError:
                                    print(
                                        "Cannot change this to float! Keeping value unchanged"
                                    )

                        elif i == "q" or i == "Q":
                            pass

                        elif i == "r" or i == "R":
                            c.changed_parameter_flag = True
                            break

                        else:  # End of changing a particular parameter
                            break

                        i = "y"

                    else:  # End of if you want to change parameters
                        i = "y"

                    c.print_current_parameters()

                if c.changed_parameter_flag:
                    # c.stop_publish()
                    rospy.logwarn("Will reset simulation on startup.")
                    # c.reset()
                    # c.gazebo_reset()

                    # rospy.loginfo("Reset gazebo")
                    # c.gazebo_unpause()
                    # rospy.loginfo("Unpaused gazebo")
                    # c.set_initial_stick_pos()

                    # If gazebo simulation is paused, rospy sleep cannot be used
                    # c.gazebo_pause()
                    # Cannot sleep here, as using sim time. Will wait for clock to be published
                    rospy.loginfo("Starting publish again")

            elif i == "":
                continue
    except rospy.ROSInterruptException:
        pass
