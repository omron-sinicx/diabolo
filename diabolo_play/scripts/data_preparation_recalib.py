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

# For ROS
try:
    import tf_conversions
    import tf
    from tf_conversions import transformations as tftf
    import rospkg
except:
    # For the notebooks (copy from src/diabolo_play if not using ROS)
    import transformations as tftf

from math import pi
import math
import os

import pandas as pd
import numpy as np


class DataPreparationClass:
    """
    Reads in experiment data and cleans it up for further use
    """

    def __init__(self, package_directory):
        # If this is true, the intermediate frames are displayed. Useful if the calibration seems off, or the rotations are not correct.
        self.DEBUG_DISPLAY = False
        self.GET_AVG_STRING_LENGTH = False  # Calculates the average distance stick-diabolo-stick in the dataset (seems buggy)
        print("Started experiment playback class")
        self._package_directory = package_directory
        self.GLOBAL_X_OFFSET = 0.1
        self.GLOBAL_Y_OFFSET = -0.2
        self.GLOBAL_Z_OFFSET = 0.3
        self._ignore_diabolo = False  # Leaves diabolo transforms unchanged if True
        self._diabolo_name = "diabolo_red"
        self._stick_left = "stick_left"
        self._stick_right = "stick_right"
        self.recordings_dir = self._package_directory + "experiments/recordings"
        # Precalculated diabolo axis in diabolo rigid body frame
        self.diabolo_axis_vector_rb = np.array([-0.70691625, -0.01031466, 0.01276811])
        self.diabolo_axis_vector_rb = self.diabolo_axis_vector_rb / np.linalg.norm(
            self.diabolo_axis_vector_rb
        )

        ## IMP: This point is pre-calculated as the center of rotation of the diabolo rigid body frame (in the rigid body frame)
        ## We do not use this for calculating the diabolo center, as the new calibration data has markers at the diabolo center
        ## This point can be used if the extra diabolo center markers are not available
        ## As of now, it is used to estimate the rotational velocity of the diabolo
        self.diabolo_rb_center_of_rotation = np.array(
            [-0.14900837, -0.0071794, -0.01134134]
        )

    def read_raw_files(
        self, filename="experiments/recordings/2020-04-05/Take_diabolo_red.csv"
    ):
        print(
            "Reading experiment file " + os.path.join(self._package_directory, filename)
        )
        self.experiment_df = pd.read_csv(
            os.path.join(self._package_directory, filename), header=[2, 4, 5]
        )
        self.diabolo_data = self.experiment_df[self._diabolo_name]
        self.stick_left_data = self.experiment_df[self._stick_left]
        self.stick_right_data = self.experiment_df[self._stick_right]
        if self.DEBUG_DISPLAY:
            print("diabolo_data.head() = ")
            print(self.diabolo_data.head())
            print("stick_left.head() = ")
            print(self.stick_left_data.head())

    def apply_offsets(self):
        self.get_RB_to_frame_transformations()

        if self.DEBUG_DISPLAY:
            if not self._ignore_diabolo:
                self.diabolo_data_original = copy.deepcopy(
                    self.experiment_df[self._diabolo_name]
                )
                self.calculate_frames_of_interest(
                    self.experiment_df,
                    self._diabolo_name,
                    self.translation_diabolo_intermediate,
                    self.rotation_diabolo_intermediate,
                )
                self.diabolo_data_intermediate = self.experiment_df[self._diabolo_name]
            self.stick_left_data_original = copy.deepcopy(
                self.experiment_df[self._stick_left]
            )
            self.stick_right_data_original = copy.deepcopy(
                self.experiment_df[self._stick_right]
            )
            self.read_raw_files()

        # First the diabolo
        if not self._ignore_diabolo:
            print("Applying offsets to diabolo")
            self.calculate_frames_of_interest(
                self.experiment_df, self._diabolo_name, self.RB_to_diabolo_transform
            )
        else:
            print("Leaving diabolo position/rotation unchanged")
        print("Storing diabolo rotation velocity")
        self.experiment_df["rot_velocity"] = self.get_rotation_speeds(
            self.experiment_df
        )

        # Then the sticks
        print("Applying offsets to " + self._stick_left)
        self.calculate_frames_of_interest(
            self.experiment_df, self._stick_left, self.RB_to_stick_left_transform
        )
        print("Applying offsets to " + self._stick_right)
        self.calculate_frames_of_interest(
            self.experiment_df, self._stick_right, self.RB_to_stick_right_transform
        )
        print("Done applying offsets")
        self.diabolo_data = self.experiment_df[self._diabolo_name]
        self.stick_left_data = self.experiment_df[self._stick_left]
        self.stick_right_data = self.experiment_df[self._stick_right]

        if self.GET_AVG_STRING_LENGTH:
            string_length = 0.0
            elements = 0
            diabolo_axle_radius = 0.0065
            curved_string_angle = math.pi * 160.0 / 180.0

            print("Entries in this df = " + str(len(self.diabolo_data)))
            for i in range(len(self.diabolo_data)):
                d_p = np.array(self.diabolo_data.loc[i, "Position"].values)
                ls_p = np.array(self.stick_left_data.loc[i, "Position"].values)
                rs_p = np.array(self.stick_right_data.loc[i, "Position"].values)
                a = np.hstack((d_p, ls_p, rs_p))
                if not np.isnan(a).any():
                    elements += 1
                    string_length = (
                        string_length
                        + math.sqrt(
                            np.linalg.norm(ls_p - d_p) ** 2 - (diabolo_axle_radius ** 2)
                        )
                        + math.sqrt(
                            np.linalg.norm(rs_p - d_p) ** 2 - (diabolo_axle_radius ** 2)
                        )
                        + diabolo_axle_radius * curved_string_angle
                    )
            if elements != 0:
                string_length /= elements

            print(
                "Average string length in this sim (method2) is " + str(string_length)
            )

        if self.DEBUG_DISPLAY:
            print("diabolo_data.head() = ")
            print(self.diabolo_data.head())
            print("stick_left.head() = ")
            print(self.stick_left_data.head())

    def get_RB_to_frame_transformations(self):
        """
        Get the offsets for each rigid body. The offset transforms from the center of the rigid body
        to the point of interest (stick tip or diabolo center).
        """
        print("Calculating offsets")
        experiment_directory = os.path.join(
            self._package_directory, "experiments/recordings/2020-07-16_calibration/"
        )
        diabolo_calib_directory = os.path.join(
            self._package_directory, "experiments/recordings/2020-08-20_calibration/"
        )
        f = pd.read_csv(
            os.path.join(
                diabolo_calib_directory, "diabolo_red_with_two_markers_2020_8_20.csv"
            ),
            header=[2, 4, 5],
        )
        translation_diabolo_red = f["diabolo_red"]["Position"].iloc[1].values  #
        rotation_diabolo_red = f["diabolo_red"]["Rotation"].iloc[1].values
        translation_marker_0 = (
            f["diabolo_red_axis_center_markers:center_marker_back"]["Position"]
            .iloc[1]
            .values
        )  #
        translation_marker_1 = (
            f["diabolo_red_axis_center_markers:center_marker_front"]["Position"]
            .iloc[1]
            .values
        )  #

        ### ===
        # 1) Use average of the two marker positions to get the center position of the diabolo
        trans_d_center_marker_w_0 = translation_marker_0[0:3]
        trans_d_center_marker_w_1 = translation_marker_1[0:3]

        translation_diabolo_marker_mid_world = np.ones(4)
        translation_diabolo_marker_mid_world[0:3] = (
            trans_d_center_marker_w_0 + trans_d_center_marker_w_1
        ) / 2.0

        # Translation and rotation from RB frame to world frame
        diabolo_rb_to_world_trans = translation_diabolo_red
        diabolo_rb_to_world_rot = rotation_diabolo_red
        T1 = tftf.translation_matrix(diabolo_rb_to_world_trans)
        R1 = tftf.quaternion_matrix(diabolo_rb_to_world_rot)
        # Matrix to convert from rigid body frame to world frame
        W_T_RB = tftf.concatenate_matrices(T1, R1)
        # Get position of the mid point of the markers in the rigid body frame
        diabolo_marker_mid_RB = np.dot(
            tftf.inverse_matrix(W_T_RB), translation_diabolo_marker_mid_world
        )[0:3]
        # Now find the actual center of the diabolo
        # Actual diabolo center is the point on the line parallel to the diabolo axis and passing through the diabolo center of rotation
        # This point is the vector sum of the diabolo center of rotation and
        # the projection of the vector from the center of rotation to the marker mid point on the axis vector
        rot_center_to_marker_mid = (
            diabolo_marker_mid_RB - self.diabolo_rb_center_of_rotation
        )
        self.diabolo_center_in_rb_frame = (
            self.diabolo_rb_center_of_rotation
            + self.diabolo_axis_vector_rb
            * (
                np.dot(rot_center_to_marker_mid, self.diabolo_axis_vector_rb)
                / (
                    np.linalg.norm(self.diabolo_axis_vector_rb)
                    * np.linalg.norm(self.diabolo_axis_vector_rb)
                )
            )
        )
        print("Diabolo center in rigid body frame is: ")
        print(self.diabolo_center_in_rb_frame)

        # Now calculate the rotation matrix between the rigid body frame and the diabolo frame
        # Using the angle-axis method. Find the matrix to rotate the x axis of the rigids body frame to the diabolo axis
        # This will give us an arbitrary orientation of the y and z axis of the diabolo frame, but it does not matter

        rotation_axis = np.cross(np.array([1.0, 0.0, 0.0]), self.diabolo_axis_vector_rb)
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

        rotation_angle = math.acos(
            np.dot(np.array([1.0, 0.0, 0.0]), self.diabolo_axis_vector_rb)
        )

        R1 = tftf.rotation_matrix(rotation_angle, rotation_axis)
        T1 = tftf.translation_matrix(self.diabolo_center_in_rb_frame)
        self.RB_to_diabolo_transform = tftf.concatenate_matrices(T1, R1)

        ###
        # apply_offsets stick positions
        f = pd.read_csv(
            os.path.join(
                experiment_directory, "Take 2020-07-16 stick_left_recalib.csv"
            ),
            header=[2, 4, 5],
        )
        translation_left_stick_to_tip_marker = (
            f["tracker:tracking_marker_0"]["Position"].iloc[1].values
        )
        translation_left_stick_to_world = f["stick_left"]["Position"].iloc[1].values
        rotation_left_stick_to_world = f["stick_left"]["Rotation"].iloc[1].values
        f = pd.read_csv(
            os.path.join(
                experiment_directory, "Take 2020-07-16 stick_right_recalib.csv"
            ),
            header=[2, 4, 5],
        )
        translation_right_stick_to_tip_marker = (
            f["tracker:tracking_marker_0"]["Position"].iloc[1].values
        )
        translation_right_stick_to_world = f["stick_right"]["Position"].iloc[1].values
        rotation_right_stick_to_world = f["stick_right"]["Rotation"].iloc[1].values

        ### ===
        # TODO: Save stick tip to stick rigid body transform as well. Leaving as is for now
        # Obtain the stick positions. The tip of the stick was put at the origin during calibration,
        # so the transformation to the origin is the translation to the tip.

        # We have the postion of the marker as the actual stick tip.
        # We must offset by the marker thickness in the direction of the RB coordinate frame
        marker_thickness = 0.0115  # 11.5 mm
        translation_left_stick_to_tip = (
            translation_left_stick_to_tip_marker
            - marker_thickness
            * translation_left_stick_to_tip_marker
            / np.linalg.norm(translation_left_stick_to_tip_marker)
        )
        translation_right_stick_to_tip = (
            translation_right_stick_to_tip_marker
            - marker_thickness
            * translation_right_stick_to_tip_marker
            / np.linalg.norm(translation_right_stick_to_tip_marker)
        )

        # Here we get the position of the stick tip in the world frame
        trans_ls_to_tip_world = translation_left_stick_to_tip
        translation_left_stick_to_tip_world = np.ones(4)
        translation_left_stick_to_tip_world[0:3] = trans_ls_to_tip_world
        # Get the homogenous transformation matrix to convert from rigid body frame to world frame
        left_stick_to_world_trans = translation_left_stick_to_world
        left_stick_to_world_rot = rotation_left_stick_to_world
        T1 = tftf.translation_matrix(left_stick_to_world_trans)
        R1 = tftf.quaternion_matrix(left_stick_to_world_rot)
        # Rigid body frame to world frame
        M = tftf.concatenate_matrices(T1, R1)
        # Get position of the stick tip in the rigid body frame
        translation_left_stick_to_tip_rb = np.dot(
            tftf.inverse_matrix(M), translation_left_stick_to_tip_world
        )[0:3]
        self.RB_to_stick_left_transform = tftf.translation_matrix(
            translation_left_stick_to_tip_rb
        )

        # self.translation_left_stick_to_tip[2] = self.translation_left_stick_to_tip[2] + 0.004  # The small offset is due to the thickness of the stick

        # Here we get the position of the stick tip in the world frame
        trans_rs_to_tip_world = translation_right_stick_to_tip
        translation_right_stick_to_tip_world = np.ones(4)
        translation_right_stick_to_tip_world[0:3] = trans_rs_to_tip_world
        # Get the homogenous transformation matrix to convert rigid body frame to world frame
        right_stick_to_world_trans = translation_right_stick_to_world
        right_stick_to_world_rot = rotation_right_stick_to_world
        T1 = tftf.translation_matrix(right_stick_to_world_trans)
        R1 = tftf.quaternion_matrix(right_stick_to_world_rot)
        # Rigid body frame to world frame
        M = tftf.concatenate_matrices(T1, R1)
        # Get position of the stick tip in the rigid body frame
        translation_right_stick_to_tip_rb = np.dot(
            tftf.inverse_matrix(M), translation_right_stick_to_tip_world
        )
        self.RB_to_stick_right_transform = tftf.translation_matrix(
            translation_right_stick_to_tip_rb
        )

    def write_offset_files(
        self, filename="experiments/output/2020-04-05/Take_diabolo_red.csv"
    ):
        try:
            head, tail = os.path.split(os.path.join(self._package_directory, filename))
            os.makedirs(head)
        except:
            pass
        self.experiment_df.to_csv(os.path.join(self._package_directory, filename))

    def calculate_frames_of_interest(
        self, df, rigid_body_name, rigid_body_to_object_transform
    ):
        """
        Finds the entries for a rigid body in the DataFrame df, transforms it and replaces it in the DataFrame.
        """
        try:
            for i in range(len(df)):
                # Calculate the transform to convert from rigid body frame to world frame
                p1 = np.array(
                    df.loc[i, (rigid_body_name, "Position")].values
                )  # This is the position of the rigid body in the world frame
                quat1 = np.array(df.loc[i, (rigid_body_name, "Rotation")].values)
                T1 = tftf.translation_matrix(p1)
                R1 = tftf.quaternion_matrix(quat1)
                world_to_rb_transform = tftf.concatenate_matrices(T1, R1)
                # Get position of origin of object of interest frame in world frame
                M1 = tftf.concatenate_matrices(
                    world_to_rb_transform, rigid_body_to_object_transform
                )

                # The system needs to be rotated
                q0 = tftf.quaternion_from_euler(0.0, 0, -90.0 / 180.0 * math.pi)
                R0 = tftf.quaternion_matrix(q0)
                M1 = np.dot(R0, M1)
                p_new = M1[:, 3]
                q_new = tftf.quaternion_from_matrix(M1)

                p_new[0] = p_new[0] + self.GLOBAL_X_OFFSET
                p_new[1] = p_new[1] + self.GLOBAL_Y_OFFSET
                p_new[2] = p_new[2] + self.GLOBAL_Z_OFFSET

                df.at[i, (rigid_body_name, "Position", "X")] = p_new[0]
                df.at[i, (rigid_body_name, "Position", "Y")] = p_new[1]
                df.at[i, (rigid_body_name, "Position", "Z")] = p_new[2]

                df.at[i, (rigid_body_name, "Rotation", "X")] = q_new[0]
                df.at[i, (rigid_body_name, "Rotation", "Y")] = q_new[1]
                df.at[i, (rigid_body_name, "Rotation", "Z")] = q_new[2]
                df.at[i, (rigid_body_name, "Rotation", "W")] = q_new[3]
        except KeyboardInterrupt:
            print("Keyboard interrupt; stopping loop.")
            pass

    def get_rotation_speeds(self, df):
        """
        Returns a vector with the diabolo's rotation speed.
        """
        rot_velocities = []
        current_rot_speed = 0.0
        frames_per_sec = 120.0
        try:
            for i in range(len(df)):
                # Calculate the rotation speed of the diabolo using the rotation matrix of the next diabolo frame
                if i < (len(df) - 1):  # This is not the last entry in the data
                    # Get next transform
                    quat1 = np.array(df.loc[i, (self._diabolo_name, "Rotation")].values)
                    quat2 = np.array(
                        df.loc[i + 1, (self._diabolo_name, "Rotation")].values
                    )

                    if not np.isnan(quat1).any() and not np.isnan(quat2).any():
                        R1 = tftf.quaternion_matrix(quat1)
                        R2 = tftf.quaternion_matrix(quat2)
                        # Get rotation from one time step to the next
                        R = tftf.concatenate_matrices(R2, tftf.inverse_matrix(R1))
                        angle_change, direc, point = tftf.rotation_from_matrix(R)
                        current_rot_speed = angle_change * frames_per_sec
                        current_rot_speed = abs(current_rot_speed)
                # The last two entries will be the same, as the speed is not calculated for the last step
                rot_velocities.append(current_rot_speed)
            return rot_velocities
        except KeyboardInterrupt:
            print("Keyboard interrupt; stopping loop.")
            pass

    def smooth_and_offset_data(
        self,
        experiment_filename="../experiments/recordings/2020-04-05/Take_diabolo_red.csv",
        output_filename="../experiments/output/2020-04-05/Take_diabolo_red_smoothed.csv",
        calibration_file_directory="../experiments/recordings/2020-04-05/",
        smooth_frame_window=15,
    ):
        """Read in, smooth position data and offset frames using calibration files,
        so that the diabolo is centered and the stick data by the center.
        """
        f = pd.read_csv(experiment_filename, header=[2, 4, 5])
        f2 = f.copy()
        f_smoothed = f.rolling(smooth_frame_window).mean()
        f2.loc[:, (self._diabolo_name, "Position")] = f_smoothed
        f2.loc[:, (self._stick_left, "Position")] = f_smoothed
        f2.loc[:, (self._stick_right, "Position")] = f_smoothed
        c.experiment_df = f2

        c.diabolo_data = c.experiment_df[self._diabolo_name]
        c.stick_left_data = c.experiment_df[self._stick_left]
        c.stick_right_data = c.experiment_df[self._stick_right]

        c.apply_offsets(experiment_directory)
        c.write_offset_files(output_filename)


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("Usage: rosrun diabolo_play data_preparation.py <folder in experiments/recording/>")
        print("e.g.: rosrun diabolo_play data_preparation.py 2020-09-14_motion_extraction/linear_accel_stick_motion.csv")
        print("(use ls and tab-complete in the terminal to get copy-pastable file locations)")

    else:
        short_filepath = sys.argv[1]
        try:
            rospack = rospkg.RosPack()
            package_directory = rospack.get_path("diabolo_play")
        except:
            package_directory = "/Users/felix/dev/diabolo-play/diabolo_play"
            print(
                "No ROS installation found. Using default location: "
                + package_directory
            )
        diabolo_name = "diabolo_red"
        print("Default diabolo name (the column header) is " + diabolo_name)
        print("Press enter to use this diabolo name, or enter an alternate name")
        i = raw_input()
        if not i == "":
            diabolo_name = i

        full_filepath = os.path.join(
            package_directory, "experiments", "recordings", short_filepath
        )
        if not os.path.exists(full_filepath):
            print("Attempted to open: " + full_filepath)
            print("This file does not exist")

        else:
            try:
                c = DataPreparationClass(package_directory)
                c._ignore_diabolo = True
                c._diabolo_name = diabolo_name
                c._stick_right = "stick_right"  # This can be e.g. "stick_right_2020_10" as well
                i = 1
                while i:
                    print("Enter 1 to load raw experiment data.")
                    print("Enter 2 to apply offsets to the experiment data.")
                    print("Enter w to write data to 'corrected' data file.")
                    print("Enter x to exit.")
                    i = raw_input()
                    if i == "1":
                        c.read_raw_files(filename=full_filepath)
                    elif i == "2":
                        c.apply_offsets()
                    elif i == "w":
                        c.write_offset_files(
                            filename=os.path.join(
                                package_directory,
                                "experiments",
                                "output",
                                short_filepath,
                            )
                        )
                    elif i == "x":
                        break
                    elif i == "":
                        continue

            except:
                raise
