#!/usr/bin/env python
import rospy
import sys
import os
import rospkg
from diabolo_play.srv import SimulateDiabolo, SimulateDiaboloRequest
from geometry_msgs.msg import Point, Pose, PoseArray
from diabolo_gazebo.msg import DiaboloState
import math
import pandas as pd
import numpy as np
import copy
import matplotlib.pyplot as plt
import visualization_msgs.msg
import random


class DataCollector:
    def __init__(self):
        rospy.init_node("diabolo_data_collector", anonymous=True)
        self._rospack = rospkg.RosPack()
        self._package_directory = self._rospack.get_path("diabolo_play")
        self.get_analytical_sim_diabolo_state_service = rospy.ServiceProxy(
            "/simulate_diabolo", SimulateDiabolo
        )
        self.get_nn_sim_diabolo_state_service = rospy.ServiceProxy(
            "/learning_predictor", SimulateDiabolo
        )
        self.marker_array_pub = rospy.Publisher(
            "visualization_marker_array",
            visualization_msgs.msg.MarkerArray,
            queue_size=100,
        )
        # This is a list of trials performed and outcomes generated
        # It might be better to store these here and plot them rather than outputting to a csv
        self.ideal_throw_parameters = {}
        self.error_dicts = (
            []
        )  # A list of dicts each contining the step by step error for each trial conducted with a set of parameters
        self.create_markers()
        self.frame_rate = 120.0
        self.sim_time_step = 0.001
        self.experiment_filenames = {
            "throw_1.csv": [0, -1],
            "deceleration_diabolo_red_2.csv": [0, -1],
            "y_swing_2.csv": [0, -1],
            "z_hop_1_cut.csv": [0, -1],
            "acceleration_1_cut.csv": [0, -1],
            "circular_accel_stick_motion.csv": [0, -1],
        }
        self.store_param_list()

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

    def dataframe_line_to_pose(self, dataframe, line_number=1):
        """
        Takes a dataframe containing one rigid body as formatted by Motive (with rotation in Euler angles in degrees)
        (e.g. self.diabolo_data) and returns the pose at the index line_number.
        """
        quat = dataframe["Rotation"].iloc[line_number].tolist()
        translation = dataframe["Position"].iloc[line_number].values
        pose = Pose()
        pose.position.x = translation[0]
        pose.position.y = translation[1]
        pose.position.z = translation[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def index_has_nan(self, index):
        """
        Check if the stick positions and the diabolo position for this index have nan values
        Also check if the next index diabolo position has a nan value
        """
        try:
            diabolo_pos = self.diabolo_data["Position"].iloc[index].values
            diabolo_next_pos = self.diabolo_data["Position"].iloc[index + 1].values
            left_pos = self.stick_left_data["Position"].iloc[index].values
            right_pos = self.stick_right_data["Position"].iloc[index].values

            values = []
            values.extend(diabolo_pos)
            values.extend(diabolo_next_pos)
            values.extend(left_pos)
            values.extend(right_pos)

            if np.isnan(values).any():
                return True

            return False
        except Exception as e:
            print(e)
            return True

    def simulate_diabolo(
        self,
        start_index=0,
        stop_index=-1,
        get_collective_error=True,
        model="analytical",
    ):
        """
        Conduct the simulation and return the min error calculated for the parameters in self.param_list
        The sim is conducted between start index and stop index, which are the pose numbers in the file
        Parameters:
        If get_collective_error is True, get the total error from start point to end point
        If false, get the error of the last point
        """
        self.error_dicts = []
        req = SimulateDiaboloRequest()
        start_index_ = start_index
        stop_index_ = stop_index

        if stop_index_ < 0:
            stop_index_ = len(self.stick_left_data) + stop_index_

        # Ensure that the poses at the start and stop positions are not nan
        # print("Start index was: " + str(start_index_))

        if self.index_has_nan(start_index_):
            min_errors = {
                "position": {"error": 10000000.0, "params": []},
                "vel_mag": {"error": 1000000.0, "params": []},
                "vel_angle": {"error": 1000000.0, "params": []},
                "rot_vel": {"error": 1000000.0, "params": []},
            }
            print("Cannot perform this simulation. Start index >= Stop index")
            return min_errors

        pose0 = self.dataframe_line_to_pose(self.diabolo_data, start_index_)
        pose1 = self.dataframe_line_to_pose(self.diabolo_data, start_index_ + 1)

        self.initial_diabolo_pose = copy.deepcopy(pose0)
        # print("Initial diabolo position passed to predictor = " + str(pose0.position))
        initial_velocity = Point()
        initial_velocity.x = (pose1.position.x - pose0.position.x) * self.frame_rate
        initial_velocity.y = (pose1.position.y - pose0.position.y) * self.frame_rate
        initial_velocity.z = (pose1.position.z - pose0.position.z) * self.frame_rate

        # Create initial sim state config
        # Initial poses
        req.sim_config.initial_poses.poses.append(copy.deepcopy(pose0))
        req.sim_config.initial_poses.poses.append(
            copy.deepcopy(
                self.dataframe_line_to_pose(self.stick_left_data, start_index_)
            )
        )
        req.sim_config.initial_poses.poses.append(
            copy.deepcopy(
                self.dataframe_line_to_pose(self.stick_right_data, start_index_)
            )
        )
        # Initial rot velocity
        req.sim_config.rot_velocity = copy.deepcopy(
            self.rot_velocity_data.iloc[start_index_][0]
        )
        req.sim_config.trans_velocity = initial_velocity

        # Time step
        req.sim_config.time_step = self.sim_time_step

        req.sim_config.mass = 0.2
        req.sim_config.axle_radius = 0.0065
        req.sim_config.string_length = 1.46

        # Add the stick poses to the stick message
        ## Ignore the first pose as it is used to calculate the diabolo velocity
        # print("Time of recording = " , self.experiment_df.loc[i,("Unnamed: 1_level_0", "Unnamed: 1_level_1", "Time (Seconds)")])
        min_errors = []
        if stop_index_ - start_index_ > 1:
            for i in range(
                start_index_ + 1, stop_index_ + 1
            ):  # Include poses at stop index
                if not np.isnan(
                    np.array(self.stick_left_data.loc[i, "Position"].values)
                ).any():
                    pose = self.dataframe_line_to_pose(self.stick_left_data, i)
                    req.left_stick_poses.poses.append(pose)
                else:
                    # If the position is nan, keep the pose unchanged
                    req.left_stick_poses.poses.append(
                        copy.deepcopy(req.left_stick_poses.poses[-1])
                    )

            for i in range(
                start_index_ + 1, stop_index_ + 1
            ):  # Include poses at stop index
                if not np.isnan(
                    np.array(self.stick_right_data.loc[i, "Position"].values)
                ).any():
                    pose = self.dataframe_line_to_pose(self.stick_right_data, i)
                    req.right_stick_poses.poses.append(pose)
                else:
                    # If the position is nan, keep the pose unchanged
                    req.right_stick_poses.poses.append(
                        copy.deepcopy(req.right_stick_poses.poses[-1])
                    )

        else:
            if not np.isnan(
                np.array(self.stick_left_data.loc[stop_index_, "Position"].values)
            ).any():
                pose = self.dataframe_line_to_pose(self.stick_left_data, stop_index_)
                req.left_stick_poses.poses.append(pose)
            else:
                # If the position is nan, cannot evaluate the error
                return None
            if not np.isnan(
                np.array(self.stick_right_data.loc[stop_index_, "Position"].values)
            ).any():
                pose = self.dataframe_line_to_pose(self.stick_right_data, stop_index_)
                req.right_stick_poses.poses.append(pose)
            else:
                # If the position is nan, cannot evaluate the error
                return None

        # Set stick update time step
        req.stick_update_time_step = 1.0 / self.frame_rate
        req.animate = False
        req.constrain_to_YZ = False

        sim_poses_to_plot = []
        real_poses_to_plot = []
        errors_to_plot = {}
        if model == "nn":
            print("Calling nn model service")
            resp = self.get_nn_sim_diabolo_state_service(req)
            print(len(resp.diabolo_states))
            (
                errors_to_plot,
                sim_poses_to_plot,
                self.real_poses_to_plot,
                self.left_stick_poses,
                self.right_stick_poses,
            ) = self.evaluate_simulation_results(
                resp.diabolo_states,
                1.0 / self.frame_rate,
                1.0 / self.frame_rate,
                start_index_,
                stop_index_,
            )
            self.error_dicts.append(errors_to_plot)

        elif model == "analytical":
            print("Calling analytical model service")
            for params in self.param_list:
                # Parameters
                req.sim_config.pv_pre_cap_scale = params[0]
                req.sim_config.pv_cap_scale = params[1]
                req.sim_config.pv_post_cap_scale = params[2]
                req.sim_config.velocity_diffusion_factor = params[3]
                resp = self.get_analytical_sim_diabolo_state_service(copy.deepcopy(req))

                # Now to store the data and display the results
                # The total time period of the motion
                (
                    errors_to_plot,
                    sim_poses_to_plot,
                    self.real_poses_to_plot,
                    self.left_stick_poses,
                    self.right_stick_poses,
                ) = self.evaluate_simulation_results(
                    resp.diabolo_states,
                    req.sim_config.time_step,
                    1.0 / self.frame_rate,
                    start_index_,
                    stop_index_,
                )
                self.error_dicts.append(errors_to_plot)

        if get_collective_error:
            min_errors = self.get_optimal_params(self.param_list)

        stick_time_step = 1.0 / self.frame_rate
        stick_steps = stop_index_ - start_index_ - 1
        time_period = stick_steps * stick_time_step
        self.real_diabolo_state_times = np.arange(
            0.0, time_period + stick_time_step, stick_time_step
        )

        # sim_diabolo_poses = []
        # sim_diabolo_poses.append(sim_poses_to_plot)
        # self.plot_errors(errors_to_plot, real_diabolo_state_times, self.param_list[-1])
        # self.compare_trajectories(sim_diabolo_poses, real_poses_to_plot, left_stick_poses, right_stick_poses, stick_time_step)
        return min_errors, errors_to_plot, sim_poses_to_plot

    def store_param_list(self):
        # Get list of all parameters
        p_list = []
        pv_pre_cap = np.array((0.13))
        pv_cap = np.array((0.13))
        pv_post_cap = np.array((0.07))
        vel_diffusion_factor = np.array((0.9999))
        p_list.extend((pv_pre_cap, pv_cap, pv_post_cap, vel_diffusion_factor))
        self.param_list = np.stack(
            np.meshgrid(pv_pre_cap, pv_cap, pv_post_cap, vel_diffusion_factor), -1
        ).reshape(-1, len(p_list))

    def get_optimal_params(self, param_list):
        """
        Calculate the optimal parameters among those in parameter list for each type of error
        Return a dict of the error types, containing the smallest error and corresponding parameters for each type
        """
        min_pos_error = 100000000
        min_vel_angle_error = 100000000
        min_rot_vel_error = 100000000
        min_vel_mag_error = 100000000
        best_index = -1
        # pass this dict back to the calling function
        min_errors = {
            "position": {"error": 10000000.0, "params": []},
            "vel_mag": {"error": 1000000.0, "params": []},
            "vel_angle": {"error": 1000000.0, "params": []},
            "rot_vel": {"error": 1000000.0, "params": []},
        }
        # error_dicts contains the total errors for the current experiment
        for i in range(
            len(self.error_dicts)
        ):  # For each trial for a set of params (the ith set in param_list)
            for key in min_errors:  # For each type of error in the min_errors dict
                points = 0
                error = 0.0
                error_list = self.error_dicts[i][key]
                if type(error_list) == type(
                    {}
                ):  # Make sure this is a list and not a dict
                    error_list = self.error_dicts[i][key]["abs"]

                for j in range(
                    len(error_list)
                ):  # Get the total error by iterating over all the points in the error dict
                    if error_list[j]:
                        error += error_list[j]
                        points += 1

                error = error / points  # Average over the numbner of non-None points

                if (
                    error < min_errors[key]["error"]
                ):  # Store the min error and the params for that type of error
                    min_errors[key]["error"] = error
                    min_errors[key]["params"] = param_list[i]

        return min_errors

    def plot_errors(self, errors):
        # x = np.linspace(0, 2*np.pi, 400)
        # y = np.sin(x**2)
        # f, axs = plt.subplots(2, 2, sharex=True)
        # axs[0,0].plot(x, y)
        # axs[1,0].set_title('Sharing X axis')
        # axs[1,0].scatter(x, y)
        params = self.param_list[0]

        fig, axs = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle(
            "params = "
            + str(params[0])
            + "_"
            + str(params[1])
            + "_"
            + str(params[2])
            + "_"
            + str(params[3]),
            fontsize=16,
        )
        axs[0, 0].plot(
            self.real_diabolo_state_times[1:],
            errors[0]["position"],
            marker="o",
            color=(0.8, 0.8, 0.0),
        )
        axs[0, 1].plot(
            self.real_diabolo_state_times[1:],
            errors[0]["rot_vel"]["abs"],
            marker=".",
            color="r",
        )
        axs[1, 0].plot(self.real_diabolo_state_times[1:], errors[0]["vel_mag"]["abs"])
        axs[1, 1].plot(self.real_diabolo_state_times[1:], errors[0]["vel_angle"])

        # ax_rel_rot_vel = axs[0,1].twinx() #To show relative quantities on the same graph
        # ax_rel_vel_mag = axs[1,0].twinx() #To show relative quantities on the same graph

        axs[0, 1].plot(
            self.real_diabolo_state_times[1:],
            errors[0]["rot_vel"]["rel"],
            marker=".",
            color=(0.8, 0.8, 0.0),
        )

        if len(errors) == 2:
            # This means the nn model's errors are in this as well
            axs[0, 0].plot(
                self.real_diabolo_state_times[1:],
                errors[1]["position"],
                marker=".",
                color=(0, 0.8, 0.8),
            )
            axs[0, 1].plot(
                self.real_diabolo_state_times[1:],
                errors[1]["rot_vel"]["rel"],
                marker=".",
                color=(0, 0.8, 0.8),
            )

        axs[0, 0].set_title("Position error", fontsize="x-large")
        axs[0, 1].set_title("Rotational velocity", fontsize="x-large")
        axs[1, 0].set_title("Velocity magnitude error", fontsize="x-large")
        axs[1, 1].set_title("Velocity angle error", fontsize="x-large")

        for a in axs.flatten():
            a.set_xlabel("Time (s)", fontsize="large")
            a.tick_params(axis="y")

        axs[0, 0].set_ylim(top=0.25, bottom=0)
        axs[0, 1].set_ylim(top=160, bottom=0)

        axs[0, 0].tick_params(axis="y")
        axs[0, 0].set_ylabel("Position error [m]", fontsize="large")
        axs[0, 1].set_ylabel("Rotational velocity [rad/s]", fontsize="large")
        axs[1, 0].set_ylabel("Absolute error [m/s]", fontsize="large")
        axs[1, 1].set_ylabel("Absolute error [rad]", fontsize="large")

        plt.show()

    def compare_trajectories(self, sim_diabolo_poses):
        """
        Show the trajectories of the real and sim diabolos in rviz for debugging
        """
        # Make markers for real and sim diabolos
        time_step = 1 / self.frame_rate
        rate = rospy.Rate(1 / time_step)
        a = "a"
        i = 0
        # print("First sim diabolo pose  = \n" + str(sim_diabolo_poses[0].position))
        for i in range(len(self.left_stick_poses)):
            # while a:
            marker_list = []
            for m in self.real_diabolo_markers:
                m.pose = self.real_poses_to_plot[i]
                marker_list.append(m)

            for m in self.analytical_sim_diabolo_markers:
                m.pose = sim_diabolo_poses[0][i]
                marker_list.append(m)

            if len(sim_diabolo_poses) == 2:
                for m in self.nn_sim_diabolo_markers:
                    m.pose = sim_diabolo_poses[1][i]
                    marker_list.append(m)

            self.left_stick_tip.pose = self.left_stick_poses[i]
            marker_list.append(self.left_stick_tip)
            self.right_stick_tip.pose = self.right_stick_poses[i]
            marker_list.append(self.right_stick_tip)

            self.marker_array_pub.publish(marker_list)
            rospy.sleep(rospy.Duration(time_step))
            # a = raw_input()
            # if a == 'd' and i < len(self.left_stick_poses):
            #     i +=1
            # elif a == 'a' and i > 0:
            #     i -= 1
        marker_list = []
        for i in range(len(self.left_stick_poses)):
            real_diabolo_point = self._make_marker_from_mesh(
                "", color=(1.0, 0.0, 0), scale=[0.01, 0.01, 0.01], namespace=""
            )
            real_diabolo_point.type = visualization_msgs.msg.Marker.SPHERE
            real_diabolo_point.pose = self.real_poses_to_plot[i]

            sim_diabolo_point = self._make_marker_from_mesh(
                "", color=(1.0, 1.0, 0.0), scale=[0.01, 0.01, 0.01], namespace=""
            )
            sim_diabolo_point.type = visualization_msgs.msg.Marker.SPHERE
            sim_diabolo_point.pose = sim_diabolo_poses[0][i]

            marker_list.append(copy.deepcopy(real_diabolo_point))
            marker_list.append(copy.deepcopy(sim_diabolo_point))

            if len(sim_diabolo_poses) == 2:
                nn_diabolo_point = self._make_marker_from_mesh(
                    "", color=(0.0, 1.0, 1.0), scale=[0.01, 0.01, 0.01], namespace=""
                )
                nn_diabolo_point.type = visualization_msgs.msg.Marker.SPHERE
                nn_diabolo_point.pose = sim_diabolo_poses[1][i]
                marker_list.append(copy.deepcopy(nn_diabolo_point))

            # diabolo_to_goal_marker = self._make_marker_from_mesh("", color=(1.0, 1.0, 1.0), scale=[0.005, 0.01, 0.02], namespace="ns")
            # diabolo_to_goal_marker.type = visualization_msgs.msg.Marker.ARROW
            # diabolo_to_goal_base = self.real_poses_to_plot[i].position
            # diabolo_to_goal_tip =  sim_diabolo_poses[i].position
            # diabolo_to_goal_marker.points.append(diabolo_to_goal_base)
            # diabolo_to_goal_marker.points.append(diabolo_to_goal_tip)
            # marker_list.append(copy.deepcopy(diabolo_to_goal_marker))

        real_diabolo_traj = self._make_marker_from_mesh(
            mesh_filename="",
            namespace="real_diabolo_traj",
            scale=(0.01, 1, 1),
            color=(1, 0, 0),
        )
        real_diabolo_traj.type = visualization_msgs.msg.Marker.LINE_STRIP
        for i in self.real_poses_to_plot:
            real_diabolo_traj.points.append(i.position)

        sim_diabolo_traj = self._make_marker_from_mesh(
            mesh_filename="",
            namespace="analytical_sim_diabolo_traj",
            scale=(0.01, 1, 1),
            color=(1, 1, 0),
        )
        sim_diabolo_traj.type = visualization_msgs.msg.Marker.LINE_STRIP
        for i in sim_diabolo_poses[0]:
            sim_diabolo_traj.points.append(i.position)

        marker_list.append(real_diabolo_traj)
        marker_list.append(sim_diabolo_traj)

        if len(sim_diabolo_poses) == 2:
            nn_diabolo_traj = self._make_marker_from_mesh(
                mesh_filename="",
                namespace="nn_sim_diabolo_traj",
                scale=(0.01, 1, 1),
                color=(0, 1, 1),
            )
            nn_diabolo_traj.type = visualization_msgs.msg.Marker.LINE_STRIP
            for i in sim_diabolo_poses[1]:
                nn_diabolo_traj.points.append(i.position)
            marker_list.append(nn_diabolo_traj)

        self.marker_array_pub.publish(marker_list)
        self.marker_count = 0
        self.create_markers()

    def evaluate_simulation_results(
        self,
        sim_diabolo_states,
        sim_time_step,
        stick_time_step,
        start_index_,
        stop_index_,
    ):

        # print(str(len(sim_diabolo_states)) + " states returned")
        stick_steps = stop_index_ - start_index_
        time_period = stick_steps * stick_time_step
        # print("time period = " + str(time_period))
        sim_diabolo_state_times = np.arange(
            start_index_ * stick_time_step,
            start_index_ * stick_time_step + time_period + sim_time_step,
            sim_time_step,
        )
        real_diabolo_state_times = np.arange(
            start_index_ * stick_time_step,
            start_index_ * stick_time_step + time_period + stick_time_step,
            stick_time_step,
        )

        sim_state_poses_to_plot = [sim_diabolo_states[0].pose]
        real_state_poses_to_plot = [
            self.dataframe_line_to_pose(self.diabolo_data, start_index_)
        ]
        left_stick_poses = [
            self.dataframe_line_to_pose(self.stick_left_data, start_index_)
        ]
        right_stick_poses = [
            self.dataframe_line_to_pose(self.stick_right_data, start_index_)
        ]

        sim_diabolo_state_count = 0
        # Velocity magnitude and rotational velocity have absolute and relative errors
        error_dict = {
            "position": [],
            "vel_mag": {"abs": [], "rel": []},
            "vel_angle": [],
            "rot_vel": {"abs": [], "rel": []},
        }

        for i in range(start_index_ + 1, stop_index_):
            errors = {"position": 0.0, "vel_mag": {}, "vel_angle": 0.0, "rot_vel": {}}
            # Find the diabolo sim time closest to this real diabolo time
            while (
                sim_diabolo_state_times[sim_diabolo_state_count]
                < real_diabolo_state_times[i - (start_index_)]
            ):
                # Will exit this loop at the point that the diabolo time and the sim time are the same/close
                sim_diabolo_state_count += 1

            # Evaluate the error between the diabolos at this point in time
            pose0 = []
            if not i == 0:
                pose0 = np.array(self.diabolo_data.loc[i - 1, "Position"].values)
            else:
                pos0 = self.initial_diabolo_pose.position
                pose0 = np.array((pos0.x, pos0.y, pos0.z))

            if (
                not np.isnan(
                    np.array(self.diabolo_data.loc[i, "Position"].values)
                ).any()
                and not np.isnan(pose0).any()
            ):
                # print("Real time = " + str(real_diabolo_state_times[i]))
                # print("Sim time = " + str(sim_diabolo_state_times[sim_diabolo_state_count]))
                try:
                    errors = self.evaluate_step_error(
                        sim_diabolo_states[sim_diabolo_state_count], i, stick_time_step
                    )
                except:
                    print("diabolo state count = " + str(sim_diabolo_state_count))
                    raise
                real_state_poses_to_plot.append(
                    self.dataframe_line_to_pose(self.diabolo_data, i)
                )
                sim_state_poses_to_plot.append(
                    sim_diabolo_states[sim_diabolo_state_count].pose
                )
                left_stick_poses.append(
                    self.dataframe_line_to_pose(self.stick_left_data, i)
                )
                right_stick_poses.append(
                    self.dataframe_line_to_pose(self.stick_right_data, i)
                )

            else:
                errors["position"] = None
                errors["vel_mag"]["abs"] = None
                errors["vel_mag"]["rel"] = None
                errors["vel_angle"] = None
                errors["rot_vel"]["abs"] = None
                errors["rot_vel"]["rel"] = None
                real_state_poses_to_plot.append(real_state_poses_to_plot[-1])
                sim_state_poses_to_plot.append(sim_state_poses_to_plot[-1])
                left_stick_poses.append(left_stick_poses[-1])
                right_stick_poses.append(right_stick_poses[-1])

            error_dict["position"].append(errors["position"])
            error_dict["vel_mag"]["abs"].append(errors["vel_mag"]["abs"])
            error_dict["vel_mag"]["rel"].append(errors["vel_mag"]["rel"])
            error_dict["vel_angle"].append(errors["vel_angle"])
            error_dict["rot_vel"]["abs"].append(errors["rot_vel"]["abs"])
            error_dict["rot_vel"]["rel"].append(errors["rot_vel"]["rel"])

        # Error dicts contains all the errors for all the sets of parameters a sim is conducted for
        return (
            error_dict,
            sim_state_poses_to_plot,
            real_state_poses_to_plot,
            left_stick_poses,
            right_stick_poses,
        )

    def evaluate_step_error(
        self, sim_diabolo_state, real_diabolo_state_number, stick_time_step
    ):

        error = {}
        real_pos = self.dataframe_line_to_pose(
            self.diabolo_data, real_diabolo_state_number
        ).position
        sim_pos = sim_diabolo_state.pose.position
        # print("Sim pos = \n" + str(sim_pos))
        # print("Real pos = \n" + str(real_pos))
        # Position error
        error["position"] = math.sqrt(
            (real_pos.x - sim_pos.x) ** 2
            + (real_pos.y - sim_pos.y) ** 2
            + (real_pos.z - sim_pos.z) ** 2
        )
        # error["position"] = math.sqrt((real_pos.y - sim_pos.y)**2 + (real_pos.z - sim_pos.z)**2)

        # Velocity error
        pose0 = Pose()
        if real_diabolo_state_number == 0:
            pose0 = self.initial_diabolo_pose

        else:
            pose0 = self.dataframe_line_to_pose(
                self.diabolo_data, real_diabolo_state_number - 1
            )

        current_real_pose = self.dataframe_line_to_pose(
            self.diabolo_data, real_diabolo_state_number
        )

        real_velocity = np.array(
            (
                current_real_pose.position.x - pose0.position.x,
                current_real_pose.position.y - pose0.position.y,
                current_real_pose.position.z - pose0.position.z,
            )
        )
        real_velocity = real_velocity / stick_time_step
        sim_velocity = np.array(
            (
                sim_diabolo_state.trans_velocity.x,
                sim_diabolo_state.trans_velocity.y,
                sim_diabolo_state.trans_velocity.z,
            )
        )

        # Velocity magnitude error
        error["vel_mag"] = {}
        error["vel_mag"]["abs"] = np.linalg.norm(sim_velocity - real_velocity)
        error["vel_mag"]["rel"] = error["vel_mag"]["abs"] / np.linalg.norm(
            real_velocity
        )

        # Velocity angle error
        # if np.isnan(sim_velocity).any() or np.isnan(real_velocity).any():
        #   return None

        norm_sim_vel = sim_velocity / np.linalg.norm(sim_velocity)
        norm_real_vel = real_velocity / np.linalg.norm(real_velocity)
        error["vel_angle"] = np.arccos(np.dot(norm_real_vel, norm_sim_vel))
        # Rotational velocity error
        error["rot_vel"] = {}
        error["rot_vel"]["abs"] = abs(
            self.rot_velocity_data.iloc[real_diabolo_state_number][0]
        )
        error["rot_vel"]["rel"] = abs(sim_diabolo_state.rot_velocity)
        # error["rot_vel"]["abs"] = abs(self.rot_velocity_data.iloc[real_diabolo_state_number][0] - sim_diabolo_state.rot_velocity)
        # error["rot_vel"]["rel"] = error["rot_vel"]["abs"]/abs(self.rot_velocity_data.iloc[real_diabolo_state_number][0])

        return error

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
        # marker.header.stamp = rospy.Time.now()
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

    def create_markers(self):
        # Create marker objects from the meshes
        self.marker_count = 0
        self.real_diabolo_shell_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_shell.stl",
            color=(1, 0, 0),
            scale=[0.001, 0.001, 0.001],
            namespace="real_diabolo",
        )
        self.real_diabolo_fixator_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_fixators.stl",
            color=(0.1, 0.1, 0.1),
            scale=[0.001, 0.001, 0.001],
            namespace="real_diabolo",
        )
        self.real_diabolo_axis_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_axis.stl",
            color=(0.7, 0.7, 0.7),
            scale=[0.001, 0.001, 0.001],
            namespace="real_diabolo",
        )
        self.analytical_sim_diabolo_shell_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_shell.stl",
            color=(1.0, 1.0, 0.0),
            scale=[0.001, 0.001, 0.001],
            namespace="sim_diabolo",
        )
        self.analytical_sim_diabolo_fixator_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_fixators.stl",
            color=(0.1, 0.1, 0.1),
            scale=[0.001, 0.001, 0.001],
            namespace="sim_diabolo",
        )
        self.analytical_sim_diabolo_axis_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_axis.stl",
            color=(0.7, 0.7, 0.7),
            scale=[0.001, 0.001, 0.001],
            namespace="sim_diabolo",
        )
        self.nn_sim_diabolo_shell_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_shell.stl",
            color=(0.0, 1.0, 1.0),
            scale=[0.001, 0.001, 0.001],
            namespace="sim_diabolo",
        )
        self.nn_sim_diabolo_fixator_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_fixators.stl",
            color=(0.1, 0.1, 0.1),
            scale=[0.001, 0.001, 0.001],
            namespace="sim_diabolo",
        )
        self.nn_sim_diabolo_axis_marker = self._make_marker_from_mesh(
            "package://diabolo_scene_description/meshes/diabolo_axis.stl",
            color=(0.7, 0.7, 0.7),
            scale=[0.001, 0.001, 0.001],
            namespace="sim_diabolo",
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
        self.real_diabolo_markers = [
            self.real_diabolo_shell_marker,
            self.real_diabolo_fixator_marker,
            self.real_diabolo_axis_marker,
        ]
        self.analytical_sim_diabolo_markers = [
            self.analytical_sim_diabolo_shell_marker,
            self.analytical_sim_diabolo_fixator_marker,
            self.analytical_sim_diabolo_axis_marker,
        ]
        self.nn_sim_diabolo_markers = [
            self.nn_sim_diabolo_shell_marker,
            self.nn_sim_diabolo_fixator_marker,
            self.nn_sim_diabolo_axis_marker,
        ]

        for m in self.nn_sim_diabolo_markers:
            m.color.a = 0.5
        for m in self.analytical_sim_diabolo_markers:
            m.color.a = 0.5
        self.left_stick_tip = self._make_marker_from_mesh(
            "", color=(0.0, 0.0, 1.0), scale=[0.05, 0.05, 0.05], namespace=""
        )
        self.left_stick_tip.type = visualization_msgs.msg.Marker.SPHERE
        self.right_stick_tip = self._make_marker_from_mesh(
            "", color=(0.0, 0.0, 1.0), scale=[0.05, 0.05, 0.05], namespace=""
        )
        self.right_stick_tip.type = visualization_msgs.msg.Marker.SPHERE

    def conduct_parameter_optimization(self):
        # Set a random initial paramete vector. Check the resulting error, and random walk to get ideal parameters
        min_cap = 0.05
        max_cap = 0.8
        min_diffusion_factor = 0.9999999
        max_diffusion_factor = 1.0
        self.param_list = [[0.5, 0.5, 0.5, 0.9999]]
        current_param_list = copy.deepcopy(self.param_list)
        min_error = 10000000.0
        error = 0
        # Perform the sim 500 times
        random.seed()
        for i in range(100):
            error = 0
            candidate_params = copy.deepcopy(current_param_list)
            for p in range(len(candidate_params[0][:-1])):
                candidate_params[0][p] = (
                    candidate_params[0][p] + (0.1 - (-0.1)) * random.random() + (-0.1)
                )

            candidate_params[0][3] = (
                candidate_params[0][3]
                + (0.00009 - (-0.00009)) * random.random()
                + (-0.00009)
            )
            for p in range(len(candidate_params[0][:-1])):
                if candidate_params[0][p] < min_cap:
                    candidate_params[0][p] = min_cap
                elif candidate_params[0][p] > max_cap:
                    candidate_params[0][p] = max_cap

            if candidate_params[0][3] < min_diffusion_factor:
                candidate_params[0][3] = min_diffusion_factor

            elif candidate_params[0][3] > max_diffusion_factor:
                candidate_params[0][3] = max_diffusion_factor

            self.param_list = copy.deepcopy(candidate_params)
            print("Running sims for params: " + str(self.param_list))
            for key in self.experiment_filenames:
                # print("Running sims for: " + key)
                min_errors = {
                    "position": {"error": 10000000.0, "params": [0, 0, 0, 0]},
                    "vel_mag": {"error": 1000000.0, "params": [0, 0, 0, 0]},
                    "vel_angle": {"error": 1000000.0, "params": [0, 0, 0, 0]},
                    "rot_vel": {"error": 1000000.0, "params": [0, 0, 0, 0]},
                }

                # Read the appropriate data file, reset the error_dicts list and run the simulation, for each motion type in self.experiment_filenames
                try:
                    filename = os.path.join(
                        rospack.get_path("diabolo_play"),
                        "experiments",
                        "output",
                        "2020-07-11_basic_physics",
                        key,
                    )
                    self.read_transformed_experiment_data(filename)
                    min_errors, a, b = self.simulate_diabolo(
                        0, -1, True, model="analytical"
                    )

                except Exception as e:
                    raise
                    print("Could not simulate for " + key)

                error += min_errors["position"]["error"]

            if error < min_error:
                print("Error reduced to: " + str(error))
                print("Current params are = " + str(candidate_params))
                current_param_list = copy.deepcopy(candidate_params)
                min_error = error

        print("The best params found are : " + str(current_param_list[0]))
        print("The collective position error was : " + str(min_error))

    def conduct_automated_simulations(self):
        self.store_param_list()
        dirname = os.path.join(self._package_directory, "experiments")
        models = ["analytical", "nn"]
        for model in models:
            file = open(os.path.join(dirname, model + "_plugin_params.csv"), "w")
            file.write(
                ",error,pre_cap_scale,cap_scale,pose_cap_scale,diffusion_factor\n"
            )
            for key in self.experiment_filenames:
                print("Running sims for: " + key)
                file.write(key + "\n")
                min_errors = {
                    "position": {"error": 10000000.0, "params": [0, 0, 0, 0]},
                    "vel_mag": {"error": 1000000.0, "params": [0, 0, 0, 0]},
                    "vel_angle": {"error": 1000000.0, "params": [0, 0, 0, 0]},
                    "rot_vel": {"error": 1000000.0, "params": [0, 0, 0, 0]},
                }

                # Read the appropriate data file, reset the error_dicts list and run the simulation, for each motion type in self.experiment_filenames
                try:
                    filename = os.path.join(
                        rospack.get_path("diabolo_play"),
                        "experiments",
                        "output",
                        "2020-07-11_basic_physics",
                        key,
                    )
                    self.read_transformed_experiment_data(filename)
                    min_errors, a, b = self.simulate_diabolo(0, -1, True, model=model)

                except Exception as e:
                    raise
                    print("Could not simulate for " + key)

                for k in min_errors:
                    file.write(
                        k
                        + ","
                        + str(min_errors[k]["error"])
                        + ","
                        + str(min_errors[k]["params"][0])
                        + ","
                        + str(min_errors[k]["params"][1])
                        + ","
                        + str(min_errors[k]["params"][2])
                        + ","
                        + str(min_errors[k]["params"][3])
                    )
                    file.write("\n")
            file.close()

    def calculate_error_evolution(
        self,
        start_index,
        sim_time,
        compare_with_nn=False,
        plot=False,
        num_sub_points=10,
    ):
        """
        To check the evolution in error over small 'intervals' of a second.
        Must pass the INDEX and NOT the time to the simulation function
        num_sub_points is the number of points at which the error is calculated until sim_time.
        More points result in a higher plot resolution.
        """
        # These are the times at which to evaluate the error
        # This is to evaluate how the error changes over time (Position error)
        evaluation_times = np.arange(
            sim_time / num_sub_points,
            sim_time + sim_time / num_sub_points,
            sim_time / num_sub_points,
        )
        print(evaluation_times)
        evaluation_indices = []
        for i in range(len(evaluation_times)):
            evaluation_indices.append(
                start_index + int(evaluation_times[i] * self.frame_rate)
            )

        if evaluation_indices[-1] > self.diabolo_data.shape[0]:
            print("ERROR: Last index is past range of the data")
            return [], []

        ana_errors = []
        min_ana_errors, ana_errors_to_plot, ana_poses_to_plot = self.simulate_diabolo(
            start_index, evaluation_indices[-1], True, model="analytical"
        )
        for stop_index in evaluation_indices:  # Extract position errors to return
            ana_errors.append(
                copy.deepcopy(
                    self.error_dicts[-1]["position"][stop_index - 1 - (start_index + 1)]
                )
            )

        nn_errors = []
        min_nn_errors, nn_errors_to_plot, nn_poses_to_plot = self.simulate_diabolo(
            start_index, evaluation_indices[-1], True, model="nn"
        )
        for stop_index in evaluation_indices:  # Extract position errors to return
            nn_errors.append(
                copy.deepcopy(
                    self.error_dicts[-1]["position"][stop_index - 1 - (start_index + 1)]
                )
            )

        if plot:
            fig, ax = plt.subplots(figsize=(10, 10))
            ax.plot(evaluation_times, ana_errors, "-bo")
            ax.set_ylim(top=1.0, bottom=0)
            plt.show()
        return ana_errors, nn_errors

    def compare_models(self):
        min_ana_errors, ana_erros_to_plot, ana_poses_to_plot = self.simulate_diabolo(
            0, -1, True, model="analytical"
        )
        min_nn_errors, nn_errors_to_plot, nn_poses_to_plot = self.simulate_diabolo(
            0, -1, True, model="nn"
        )

        errors = [ana_erros_to_plot, nn_errors_to_plot]
        sim_diabolo_poses = [ana_poses_to_plot, nn_poses_to_plot]
        self.plot_errors(errors)

        self.compare_trajectories(sim_diabolo_poses)

    def write_aggregated_error_evolution_to_file(
        self, data, name="analytical", duration=2.0, num_sub_points=10
    ):
        # Data needs to be a list of lists, with each sublist being a list of errors from the start time.
        dirname = os.path.join(self._package_directory, "experiments", "plotting")
        file = open(os.path.join(dirname, name + "_error_evol.csv"), "w")
        file.write("t_from_prediction_start,error\n")
        times = np.arange(
            duration / num_sub_points,
            duration + duration / num_sub_points,
            duration / num_sub_points,
        )
        for error_series in data:
            for time, error in zip(times, error_series):
                file.write(str(time) + "," + str(error) + "\n")
        file.close()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: rosrun diabolo_play data_preparation.py <data file to correct>")

    else:
        file = sys.argv[1]
        rospack = rospkg.RosPack()

        try:
            dirname = sys.argv[2]
        except:
            dirname = "2020-07-11_basic_physics"
            print("Default directory name is " + dirname)
            print(
                "Press enter to use this directory, or enter an alternate directory name"
            )
            i = raw_input()
            if not i == "":
                dirname = i

        filename = os.path.join(
            rospack.get_path("diabolo_play"), "experiments", "output", dirname, file
        )
        if not os.path.exists(filename):
            print("Attempted to open: " + filename)
            print("This file does not exist")

        else:
            try:
                c = DataCollector()
                c.read_transformed_experiment_data(filename=filename)
                c.dirname = dirname
                i = 1
                while i:
                    print("Enter 1 to load raw experiment data.")
                    print("Enter s to get simulation results with this data file.")
                    print("Enter a to get error tables for current parameters")
                    print("Enter p to conduct random walk parameter optimizations.")
                    print("Enter nn to compare analytical and nn models for this file")
                    print("Enter e to plot error evolution.")
                    print(
                        "Enter ee to output csv files to plot aggregated error evolution."
                    )
                    print("Enter f to open a different file")
                    print("Enter x to exit.")
                    i = raw_input()
                    if i == "1":
                        c.filename = filename
                        c.read_transformed_experiment_data(filename=filename)
                    elif i == "s":
                        c.simulate_diabolo(start_index=0, stop_index=-1)
                    elif i == "a":
                        c.conduct_automated_simulations()
                    elif i == "p":
                        c.conduct_parameter_optimization()
                    elif i == "e":
                        check = True
                        print("Enter the desired start time.")
                        while check:
                            e = raw_input()
                            if e == "x":
                                break
                            start_time = float(e)
                            print("The start time is: " + str(start_time))
                            start_index = int(start_time * c.frame_rate)
                            print("The start index is: " + str(start_index))
                            if c.index_has_nan(start_index):
                                print(
                                    "This time has nan values in the data. Please try again, press x to cancel"
                                )
                                check = True
                            else:
                                check = False
                        if not check:
                            time = 1.0
                            print(
                                "Enter the length of time to conduct the simulation starting from the start time"
                            )
                            print("Press enter to use the default time of 1 second")
                            e = raw_input()
                            if not e == "":
                                time = float(e)
                            compare_with_nn = False
                            print("Compare with nn model? y/n")
                            e = raw_input()
                            if e == "y":
                                compare_with_nn = True
                            print("Calculating")
                            c.calculate_error_evolution(
                                start_index, time, compare_with_nn, plot=True
                            )
                    elif i == "ee":
                        errors_ana = []
                        errors_nn = []
                        # start_times = range(1,280,10)  # For diabolo_red_random_felix_1.csv
                        # start_times = range(1,2300,3)    # For diabolo_red_random_felix_3.csv (large) ("highres" with 50 subpoints)
                        start_times = range(
                            1, 2300, 100
                        )  # For diabolo_red_random_felix_3.csv (large) ("highres")
                        for start_time in start_times:
                            if rospy.is_shutdown():
                                break
                            start_index = int(start_time * c.frame_rate)
                            try:
                                err_ana, err_nn = c.calculate_error_evolution(
                                    start_index, 2.0, num_sub_points=50
                                )
                            except:
                                print(
                                    "Encountered error at start_time ",
                                    start_time,
                                    ". Skipping.",
                                )
                                continue
                            errors_ana.append(err_ana)
                            errors_nn.append(err_nn)
                        c.write_aggregated_error_evolution_to_file(
                            errors_ana, name="ana", duration=2.0, num_sub_points=50
                        )
                        c.write_aggregated_error_evolution_to_file(
                            errors_nn, name="nn", duration=2.0, num_sub_points=50
                        )

                    elif i == "nn":
                        c.compare_models()
                    elif i == "f":
                        print("Enter a new filename")
                        new_file = raw_input()
                        filename = os.path.join(
                            rospack.get_path("diabolo_play"),
                            "experiments",
                            "output",
                            dirname,
                            new_file,
                        )
                        if not os.path.exists(filename):
                            print("Attempted to open: " + filename)
                            print("This file does not exist")
                            print("Still using " + c.filename)
                        else:
                            c.read_transformed_experiment_data(filename=filename)
                            print("Changed file to " + new_file)
                            c.filename = new_file

                    elif i == "x":
                        break
                    elif i == "":
                        continue

            except:
                raise
