#!/usr/bin/env python
import sys
import copy
import rospy
import tf_conversions
import tf.transformations as transform
import tf
from math import pi
import math
import thread
import os
import random
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_msgs.msg
import shape_msgs.msg
import visualization_msgs.msg
import diabolo_gazebo.msg
from diabolo_play.srv import SetInitialStickPositionsRequest, SetInitialStickPositions
from diabolo_play.srv import CreateSticksTrajectoryRequest, CreateSticksTrajectory
from diabolo_play.srv import CreateRobotTrajectory, CreateRobotTrajectoryRequest
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
import pandas as pd
import numpy as np
from gazebo_msgs.srv import (
    DeleteModel,
    DeleteModelRequest,
    SpawnModel,
    SpawnModelRequest,
)
from diabolo_play.msg import DiaboloMotionSplineSeeds
from diabolo_play.srv import GetDiaboloState, GetDiaboloStateRequest
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest
import rospkg
from diabolo_gazebo.msg import DiaboloState
from scipy import interpolate
import matplotlib.pyplot as plt
from diabolo_play.motion_knot_points import KnotPointsServer
import yaml
import pickle


class PlayerClass:
    def __init__(self):
        rospy.init_node("diabolo_player", anonymous=True)
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
            latch=True,
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
        self.a_bot_command_pub = rospy.Publisher(
            "/a_bot/scaled_pos_joint_traj_controller/command",
            JointTrajectory,
            queue_size=1,
        )
        self.b_bot_command_pub = rospy.Publisher(
            "/b_bot/scaled_pos_joint_traj_controller/command",
            JointTrajectory,
            queue_size=1,
        )
        self.set_robots_initial_position_service = rospy.ServiceProxy(
            "/initialize_robots_from_stick_positions", SetInitialStickPositions
        )
        self.command_robot_trajectory_service = rospy.ServiceProxy(
            "/command_robot_traj_from_stick_traj", CreateRobotTrajectory
        )
        self.a_bot_display_traj_pub = rospy.Publisher(
            "/display_a_bot_bioik_trajectory",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )
        self.b_bot_display_traj_pub = rospy.Publisher(
            "/display_b_bot_bioik_trajectory",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )
        self.pause_gazebo_service = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause_gazebo_service = rospy.ServiceProxy(
            "/gazebo/unpause_physics", Empty
        )
        self.get_diabolo_state_service = rospy.ServiceProxy(
            "/get_observed_diabolo_state", GetDiaboloState
        )
        self.generate_trajectory_service = rospy.ServiceProxy(
            "/generate_stick_trajectory", CreateSticksTrajectory
        )
        self.get_planning_scene_service = rospy.ServiceProxy(
            "/get_planning_scene", GetPlanningScene
        )
        self.latest_diabolo_state = None
        self.create_markers()
        self.sim_recorder = None  # This will be filled with a DiaboloSimRecorder type if running automated trials
        self.current_rot_velocity = 0.0  # TODO: Calculate rotational velocity from experiment data and store here
        self.left_traj_plan_marker = None
        self.right_traj_plan_marker = None
        # self.timer = rospy.Timer(rospy.Duration(0.01), self.get_observed_diabolo_state , oneshot=False)
        # If this is true, the intermediate frames are displayed. Useful if the calibration seems off, or the rotations are not correct.
        self.pub_rate = 50.0
        rospy.set_param("/stick_pose_publish_rate", self.pub_rate)
        # This parameter is set by the gazebo launch file if robots are being spawned in gazebo
        # If the parameter is true, the program will wait for the service to initialize robot positions

        self.constrain_to_plane = True  # If true, ignore motion x coordinates
        self.DEFAULT_X_COORD = (
            0.55  # Set robots to this coordinate if motion constrained to plane
        )
        self.filename = ""
        # This is a dictionary of the functions gotten by spline interpolation from the data
        self.motion_functions = {}
        # This is the name of the current motion being executed.
        # The function(s) to be used can be extracted using this by appending
        # "_sl" (for left stick)
        # "_sr" (for right stick)
        # and using it as the key for the self.motion_functions dictionary
        self.current_motion = ""
        self.frame_rate = 120.0
        self.last_stick_positions = {}

        self.read_transformed_motion_data(
            folder=("experiments/output/2020-09-14_motion_extraction/")
        )
        self.initialize_motion_functions()
        self.get_stick_tips_from_tf()
        self.create_knot_server()
        self.stop_motion_flag = True
        self.tilt_offset = 0.0
        self.changed_tilt_offset_flag = False

        print("Started experiment playback class")

    def get_stick_tips_from_tf(self):
        # Get initial stick positions from tf.
        # If not available, assume robots are at 'diabolo_ready' position
        a_t = geometry_msgs.msg.Point()
        b_t = geometry_msgs.msg.Point()

        try:
            self.tf_listener.waitForTransform(
                "/world", "/a_bot_diabolo_stick_tip", rospy.Time(), rospy.Duration(1.0)
            )
            print("Got stick tip positions from tf")
            (a_trans, a_rot) = self.tf_listener.lookupTransform(
                "/world", "/a_bot_diabolo_stick_tip", rospy.Time(0)
            )
            (b_trans, b_rot) = self.tf_listener.lookupTransform(
                "/world", "/b_bot_diabolo_stick_tip", rospy.Time(0)
            )

            a_t.x = a_trans[0]
            a_t.y = a_trans[1]
            a_t.z = a_trans[2]

            b_t.x = b_trans[0]
            b_t.y = b_trans[1]
            b_t.z = b_trans[2]

        except:
            print("Transforms not available")
            print("Initializing with initial position for this robot")
            pose_left = self.motion_functions[self.current_motion + "_sl"][
                "initial_pose"
            ]
            pose_right = self.motion_functions[self.current_motion + "_sr"][
                "initial_pose"
            ]
            a_t = pose_right.position
            b_t = pose_left.position
        self.last_stick_positions = {"pos_left": b_t, "pos_right": a_t}

    def create_knot_server(self):
        """
        Create a knot server with the number of points given by the number knots in the current motion
        """
        interactive_knot_points = len(
            self.motion_functions[self.current_motion + "_sl"]["time_seed"]
        )
        if (
            self.motion_functions[self.current_motion + "_sl"]["motion_type"]
            == "periodic"
        ):
            interactive_knot_points -= 1
        pos_left = self.last_stick_positions["pos_left"]
        pos_right = self.last_stick_positions["pos_right"]
        left_seed_positions = []
        right_seed_positions = []
        left_seed_positions.append(pos_left)
        right_seed_positions.append(pos_right)

        left_dict = self.motion_functions[self.current_motion + "_sl"]
        right_dict = self.motion_functions[self.current_motion + "_sr"]
        for i in range(interactive_knot_points):
            # Fill the seed position arrays with the initial seeds if available
            left_seed = geometry_msgs.msg.Point(
                left_dict["x_knot_seed"][i],
                left_dict["y_knot_seed"][i],
                left_dict["z_knot_seed"][i],
            )
            right_seed = geometry_msgs.msg.Point(
                right_dict["x_knot_seed"][i],
                right_dict["y_knot_seed"][i],
                right_dict["z_knot_seed"][i],
            )

            left_seed_positions.append(left_seed)
            right_seed_positions.append(right_seed)

        self.knot_point_server = KnotPointsServer(
            interactive_knot_points, [left_seed_positions, right_seed_positions]
        )

    def get_observed_diabolo_state(self, timer):
        # Store the real pose/simulated pose of the diabolo to use for prediction
        try:
            req = GetDiaboloStateRequest()
            req.header.stamp = rospy.Time.now()
            resp = self.get_diabolo_state_service(req)
            if resp.success:
                self.latest_diabolo_state = resp.state
            # else:
            #   self.latest_diabolo_pose = None
            #   self.latest_diabolo_trans_vel = None
        except:
            self.latest_diabolo_state = None

    def _make_marker_from_mesh(
        self,
        mesh_filename="package://diabolo_play/meshes/diabolo_shell.stl",
        namespace="diabolo",
        scale=(1, 1, 1),
        color=(1, 1, 1),
        alpha=1.0,
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
        marker.color.a = alpha
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.mesh_resource = mesh_filename
        return marker

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
            "", color=(0.0, 0.0, 1.0), scale=[0.08, 0.08, 0.08], namespace=""
        )
        self.sphere_marker_1.type = visualization_msgs.msg.Marker.SPHERE
        self.sphere_marker_2 = self._make_marker_from_mesh(
            "", color=(0.0, 0.0, 1.0), scale=[0.08, 0.08, 0.08], namespace=""
        )
        self.sphere_marker_2.type = visualization_msgs.msg.Marker.SPHERE

    def update_and_publish_markers(self, poses):
        """
        poses needs to be a dict containing "diabolo", "stick_left", "stick_right" poses as geometry_msgs.msg.Pose
        """
        self.sphere_marker_1.pose = poses["stick_left"]
        self.sphere_marker_2.pose = poses["stick_right"]
        # Flip orientations for correct display of the sticks

        marker_array = [self.sphere_marker_1, self.sphere_marker_2]
        self.marker_array_pub.publish(marker_array)

    def read_transformed_motion_data(
        self, folder="experiments/output/2020-09-14_motion_extraction/"
    ):
        # This is a different function because the header is formatted differently in the transformed CSV file
        linear_accel_file = "linear_accel_stick_motion.csv"
        circular_accel_right_file = "circular_accel_right_stick_motion.csv"
        circular_accel_left_file = "circular_accel_left_stick_motion.csv"
        self.motion_data_dict = {}
        # Get linear acceleration stick positions
        motion_df = pd.read_csv(
            os.path.join(self._package_directory, folder, linear_accel_file),
            header=[0, 1, 2],
        )
        self.motion_data_dict["lin_accel_sl"] = motion_df["stick_left"]
        self.motion_data_dict["lin_accel_sr"] = motion_df["stick_right"]
        # Get circular acceleration stick positions
        motion_df = pd.read_csv(
            os.path.join(self._package_directory, folder, circular_accel_right_file),
            header=[0, 1, 2],
        )
        self.motion_data_dict["circ_accel_sr"] = motion_df["stick_right"]
        self.motion_data_dict["circ_accel_sl"] = motion_df["stick_left"]

    def force_add_motion_function_(self):
        """
        This is a helper function to add and overwrite motion functions in the database.
        """
        self.current_motion = "lin_accel"
        self.motion_functions["circ_accel_sr"]["time_seed"] = (
            0.35,
            0.65,
            0.95,
            1.25,
            1.55,
            1.85,
        )
        self.motion_functions["circ_accel_sr"]["motion_type"] = "periodic"
        self.motion_functions["circ_accel_sl"]["time_seed"] = (
            0.35,
            0.65,
            0.95,
            1.25,
            1.55,
            1.85,
        )
        self.motion_functions["circ_accel_sl"]["motion_type"] = "periodic"

        ### For horizontal impulse
        left_pose = Pose()
        right_pose = Pose()

        left_pose.position.x = self.DEFAULT_X_COORD
        right_pose.position.x = self.DEFAULT_X_COORD
        left_pose.position.y = 0.05
        right_pose.position.y = -0.05
        left_pose.position.z = 1.25
        right_pose.position.z = 1.25
        self.motion_functions["horizontal_impulse_short_left_sl"] = {
            "x_knot_seed": (0.0, 0.0),
            "y_knot_seed": (-0.23, -0.1),
            "z_knot_seed": (0.0, 0.0),
            "time_seed": (0.6, 1.0, 1.7),
            "initial_pose": copy.deepcopy(left_pose),
            "motion_type": "periodic",
        }
        self.motion_functions["horizontal_impulse_short_left_sr"] = {
            "x_knot_seed": (0.0, 0.0),
            "y_knot_seed": (-0.23, -0.1),
            "z_knot_seed": (0.0, 0.0),
            "time_seed": (0.8, 1.0, 1.7),
            "initial_pose": copy.deepcopy(right_pose),
            "motion_type": "periodic",
        }

        ### For lin_accel
        self.motion_functions["lin_accel_sr"]["time_seed"] = (0.25, 0.5, 0.9, 1.2)
        self.motion_functions["lin_accel_sr"]["motion_type"] = "periodic"
        self.motion_functions["lin_accel_sl"]["time_seed"] = (0.25, 0.5, 0.9, 1.2)
        self.motion_functions["lin_accel_sl"]["motion_type"] = "periodic"

        self.motion_functions["lin_accel_sr"]["x_knot_seed"] = (0.0, 0.0, 0.0)
        self.motion_functions["lin_accel_sr"]["y_knot_seed"] = (0.05, 0.0, 0.05)
        self.motion_functions["lin_accel_sr"]["z_knot_seed"] = (0.2, 0.4, 0.2)
        self.motion_functions["lin_accel_sl"]["x_knot_seed"] = (0.0, 0.0, 0.0)
        self.motion_functions["lin_accel_sl"]["y_knot_seed"] = (-0.05, 0.0, -0.05)
        self.motion_functions["lin_accel_sl"]["z_knot_seed"] = (-0.15, -0.3, -0.15)

        self.motion_functions["lin_accel_sl"][
            "initial_pose"
        ].position.x = self.DEFAULT_X_COORD
        self.motion_functions["lin_accel_sl"]["initial_pose"].position.y = 0.1
        self.motion_functions["lin_accel_sl"]["initial_pose"].position.z = 1.47
        self.motion_functions["lin_accel_sr"][
            "initial_pose"
        ].position.x = self.DEFAULT_X_COORD
        self.motion_functions["lin_accel_sr"]["initial_pose"].position.y = -0.1
        self.motion_functions["lin_accel_sr"]["initial_pose"].position.z = 1.07

        ### For vertical throw
        left_pose = Pose()
        right_pose = Pose()
        left_pose.position.x = self.DEFAULT_X_COORD
        right_pose.position.x = self.DEFAULT_X_COORD
        left_pose.position.y = 0.05
        right_pose.position.y = -0.05
        left_pose.position.z = 1.25
        right_pose.position.z = 1.25

        ### throw_1.bag and throw_1b.bag settings
        # self.motion_functions["vertical_throw_sl"] = {"x_knot_seed":(0.0, 0.0, 0.0), \
        #                                               "y_knot_seed":(0.2, 0.65, 0.728), \
        #                                               "time_seed": (0.4, 0.8, 0.96), \
        # self.motion_functions["vertical_throw_sr"] = {"x_knot_seed":(0.0, 0.0, 0.0), \
        #                                               "y_knot_seed":(-0.2, -0.65, -0.728), \
        #                                               "time_seed": (0.4, 0.8, 0.96), \

        ### throw_2.bag settings
        # self.motion_functions["vertical_throw_sl"] = {"x_knot_seed":(0.0, 0.0, 0.0), \
        #                                               "y_knot_seed":(0.2, 0.65, 0.729), \
        #                                               "z_knot_seed":(0.0, 0.0, 0.0), \
        #                                               "time_seed": (0.4, 0.8, 0.94), \
        # self.motion_functions["vertical_throw_sr"] = {"x_knot_seed":(0.0, 0.0, 0.0), \
        #                                               "y_knot_seed":(-0.2, -0.65, -0.729), \
        #                                               "z_knot_seed":(0.0, 0.0, 0.0), \
        #                                               "time_seed": (0.4, 0.8, 0.94), \

        self.motion_functions["vertical_throw_sl"] = {
            "x_knot_seed": (0.0, 0.0, 0.0),
            "y_knot_seed": (0.2, 0.65, 0.731),
            "z_knot_seed": (0.0, 0.0, 0.0),
            "time_seed": (0.4, 0.8, 0.92),
            "flight_time": 0.5,
            "initial_pose": copy.deepcopy(left_pose),
            "motion_type": "oneshot",
        }
        self.motion_functions["vertical_throw_sr"] = {
            "x_knot_seed": (0.0, 0.0, 0.0),
            "y_knot_seed": (-0.2, -0.65, -0.731),
            "z_knot_seed": (0.0, 0.0, 0.0),
            "time_seed": (0.4, 0.8, 0.92),
            "flight_time": 0.5,
            "initial_pose": copy.deepcopy(right_pose),
            "motion_type": "oneshot",
        }

    def add_motion_function(self, name, num_knot_points=5):
        # TODO: Create a new motion and add it to self.motion_list and self.motion_functions under that key
        # TODO: Use self.motion_functions.keys instead of maintaining self.motion_list
        left_pose = Pose()
        right_pose = Pose()

        left_pose.position.x = self.DEFAULT_X_COORD
        right_pose.position.x = self.DEFAULT_X_COORD
        left_pose.position.y = 0.05
        right_pose.position.y = -0.05
        left_pose.position.z = 1.25
        right_pose.position.z = 1.25
        self.motion_functions[name + "_sl"] = {
            "x_knot_seed": [0.0] * num_knot_points,
            "y_knot_seed": range(0.1, (num_knot_points + 1) * 0.1, 0.1),
            "z_knot_seed": range(0.05, (num_knot_points + 1) * 0.05, 0.05),
            "time_seed": range(0.5, (num_knot_points + 1) * 0.5, 0.5),
            "initial_pose": copy.deepcopy(left_pose),
            "motion_type": "periodic",
        }
        self.motion_functions[name + "_sr"] = {
            "x_knot_seed": [0.0] * num_knot_points,
            "y_knot_seed": range(0.1, (num_knot_points + 1) * 0.1, 0.1),
            "z_knot_seed": range(0.05, (num_knot_points + 1) * 0.05, 0.05),
            "time_seed": range(0.5, (num_knot_points + 1) * 0.5, 0.5),
            "initial_pose": copy.deepcopy(right_pose),
            "motion_type": "periodic",
        }

    def initialize_motion_functions(
        self, use_saved_values=True, filename="default.pkl"
    ):

        # First add the period motions, for which there is motion capture data
        self.motion_functions = {}
        path = os.path.join(self._package_directory, "config", filename)
        if os.path.exists(path) and use_saved_values:
            print("Using stored motion function values")
            with open(path, "r") as f:
                self.motion_functions = pickle.load(f)
        else:
            print("Using hardcoded values")
            self.motion_list = []
            # Make the last position in the data the same as the first postion, to make the motion cyclic
            for key in self.motion_data_dict:
                pos_data = np.array(self.motion_data_dict[key]["Position"])
                delta_x = pos_data[-1] - pos_data[0]
                total_steps = pos_data.shape[0]
                for i in range(total_steps):
                    pos_data[i] = pos_data[i] - delta_x * (
                        float(i) / float(total_steps - 1)
                    )
                # Using two "cycles" of the data for interpolation to ensure I get the correct slope at the end points
                # That is why pos_data is made by appending two of the data arrays
                pos_data = np.append(pos_data, pos_data).reshape(
                    pos_data.shape[0] * 2, -1
                )

                time_steps = np.arange(pos_data.shape[0]) / self.frame_rate
                # Create spline functions by interpolating between the data positions, ignoring the nan values
                good_indices = np.where(np.isfinite(pos_data))[0].reshape(-1, 3)[
                    :, 0
                ]  # Indices where array is finite

                # Store the functions returning spline functions, time period of this motion and the initial position of the motion
                self.motion_functions[key] = {
                    "X": interpolate.InterpolatedUnivariateSpline(
                        time_steps[good_indices], pos_data[good_indices, 0]
                    ),
                    "Y": interpolate.InterpolatedUnivariateSpline(
                        time_steps[good_indices], pos_data[good_indices, 1]
                    ),
                    "Z": interpolate.InterpolatedUnivariateSpline(
                        time_steps[good_indices], pos_data[good_indices, 2]
                    ),
                    "period": pos_data.shape[0] / (2.0 * self.frame_rate),
                }
                self.motion_functions[key]["initial_pose"] = self.stick_pose_at_time(
                    function=self.motion_functions[key], time=0
                )
                self.motion_list.append(key)

            # There are 6 knot points for circular accel and linear accel, but the last point is the same as the initial position
            # Therefore, the number of interactive markers should be len(time_seed)-1 for circular motions
            self.motion_functions["circ_accel_sr"]["time_seed"] = (
                0.3,
                0.6,
                0.9,
                1.2,
                1.5,
                1.8,
            )
            self.motion_functions["circ_accel_sr"]["motion_type"] = "periodic"
            self.motion_functions["circ_accel_sl"]["time_seed"] = (
                0.3,
                0.6,
                0.9,
                1.2,
                1.5,
                1.8,
            )
            self.motion_functions["circ_accel_sl"]["motion_type"] = "periodic"

            self.motion_functions["circ_accel_sr"]["x_knot_seed"] = (
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self.motion_functions["circ_accel_sr"]["y_knot_seed"] = (
                -0.042,
                -0.24,
                -0.41,
                -0.371,
                -0.163,
            )
            self.motion_functions["circ_accel_sr"]["z_knot_seed"] = (
                0.20,
                0.30,
                0.2,
                0.0,
                -0.076,
            )
            self.motion_functions["circ_accel_sl"]["x_knot_seed"] = (
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self.motion_functions["circ_accel_sl"]["y_knot_seed"] = (
                -0.061,
                0.0592,
                0.2619,
                0.3100,
                0.1410,
            )
            self.motion_functions["circ_accel_sl"]["z_knot_seed"] = (
                0.1801,
                0.3820,
                0.344,
                0.15914,
                0.03543,
            )

            self.motion_functions["lin_accel_sr"]["time_seed"] = (0.3, 0.6, 0.9, 1.2)
            self.motion_functions["lin_accel_sr"]["motion_type"] = "periodic"
            self.motion_functions["lin_accel_sl"]["time_seed"] = (0.3, 0.6, 0.9, 1.2)
            self.motion_functions["lin_accel_sl"]["motion_type"] = "periodic"

            self.motion_functions["lin_accel_sr"]["x_knot_seed"] = (0.0, 0.0, 0.0)
            self.motion_functions["lin_accel_sr"]["y_knot_seed"] = (-0.05, 0.0, -0.05)
            self.motion_functions["lin_accel_sr"]["z_knot_seed"] = (0.1, 0.2, 0.1)
            self.motion_functions["lin_accel_sl"]["x_knot_seed"] = (0.0, 0.0, 0.0)
            self.motion_functions["lin_accel_sl"]["y_knot_seed"] = (0.05, 0.0, 0.05)
            self.motion_functions["lin_accel_sl"]["z_knot_seed"] = (-0.1, -0.2, -0.1)

            self.motion_functions["lin_accel_sl"][
                "initial_pose"
            ].position.x = self.DEFAULT_X_COORD
            self.motion_functions["lin_accel_sl"]["initial_pose"].position.y = 0.1
            self.motion_functions["lin_accel_sl"]["initial_pose"].position.z = 1.42
            self.motion_functions["lin_accel_sr"][
                "initial_pose"
            ].position.x = self.DEFAULT_X_COORD
            self.motion_functions["lin_accel_sr"]["initial_pose"].position.y = -0.1
            self.motion_functions["lin_accel_sr"]["initial_pose"].position.z = 1.12

            left_pose = Pose()
            right_pose = Pose()
            left_pose.position.x = self.DEFAULT_X_COORD
            left_pose.position.y = 0.21
            left_pose.position.z = 1.27
            right_pose.position.x = self.DEFAULT_X_COORD
            right_pose.position.y = -0.28
            right_pose.position.z = 1.27
            # Now store the initial position for throwing motions
            # self.motion_functions["circ_accel_sr"]["initial_pose"] = circ_accel_initial_pose_right
            left_pose = Pose()
            right_pose = Pose()

            left_pose.position.x = self.DEFAULT_X_COORD
            right_pose.position.x = self.DEFAULT_X_COORD
            left_pose.position.y = 0.05
            right_pose.position.y = -0.05
            left_pose.position.z = 1.25
            right_pose.position.z = 1.25

            self.motion_functions["vertical_throw_sl"] = {
                "x_knot_seed": (0.0, 0.0, 0.0),
                "y_knot_seed": (0.1, 0.74, 0.748),
                "z_knot_seed": (0.0, 0.0, 0.0),
                "time_seed": (0.2, 0.55, 0.6),
                "flight_time": 0.5,
                "initial_pose": copy.deepcopy(left_pose),
                "motion_type": "oneshot",
            }
            self.motion_functions["vertical_throw_sr"] = {
                "x_knot_seed": (0.0, 0.0, 0.0),
                "y_knot_seed": (-0.1, -0.74, -0.748),
                "z_knot_seed": (0.0, 0.0, 0.0),
                "time_seed": (0.2, 0.55, 0.6),
                "flight_time": 0.5,
                "initial_pose": copy.deepcopy(right_pose),
                "motion_type": "oneshot",
            }
            left_pose.position.y = 0.15
            right_pose.position.y = -0.15
            left_pose.position.z = 1.20
            right_pose.position.z = 1.30
            self.motion_functions["right_throw_sl"] = {
                "x_knot_seed": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                "y_knot_seed": (-0.1, 0.33, 0.46),
                "z_knot_seed": (0.2, 0.42, 0.439),
                "time_seed": (0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
                "flight_time": 0.5,
                "initial_pose": copy.deepcopy(left_pose),
                "motion_type": "oneshot",
            }
            self.motion_functions["right_throw_sr"] = {
                "x_knot_seed": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                "y_knot_seed": (0.1, -0.33, -0.46),
                "z_knot_seed": (-0.2, -0.42, -0.439),
                "time_seed": (0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
                "flight_time": 0.5,
                "initial_pose": copy.deepcopy(right_pose),
                "motion_type": "oneshot",
            }

            self.motion_functions["left_throw_sl"] = {
                "x_knot_seed": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                "y_knot_seed": (-0.1, 0.2, 0.304652, 0.4, 0.5, 0.6, 0.7, 0.8),
                "z_knot_seed": (0.1, -0.22814, -0.38441, -0.4, -0.5, -0.6, -0.7, -0.8),
                "time_seed": (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8),
                "flight_time": 0.5,
                "initial_pose": copy.deepcopy(left_pose),
                "motion_type": "oneshot",
            }
            self.motion_functions["left_throw_sr"] = {
                "x_knot_seed": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                "y_knot_seed": (0.1, -0.22814, -0.38441, -0.4, -0.5, -0.6, -0.7, -0.8),
                "z_knot_seed": (0.1, 0.2, 0.304652, 0.4, 0.5, 0.6, 0.7, 0.8),
                "time_seed": (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8),
                "flight_time": 0.5,
                "initial_pose": copy.deepcopy(right_pose),
                "motion_type": "oneshot",
            }
            self.motion_list.append("vertical_throw")
            self.motion_list.append("right_throw")
            self.motion_list.append("left_throw")
            self.motion_list = [
                m.replace("_sr", "").replace("_sl", "") for m in self.motion_list
            ]
            self.motion_list = list(set(self.motion_list))
            #
            print("Available motions are: " + str(self.motion_list))
        self.motion_list = []
        for key in self.motion_data_dict:
            self.motion_list.append(key)

        self.motion_list.append("vertical_throw")
        self.motion_list.append("right_throw")
        self.motion_list.append("left_throw")
        self.motion_list = [
            m.replace("_sr", "").replace("_sl", "") for m in self.motion_list
        ]
        self.motion_list = list(set(self.motion_list))
        left_pose = Pose()
        right_pose = Pose()

        left_pose.position.x = self.DEFAULT_X_COORD
        right_pose.position.x = self.DEFAULT_X_COORD
        left_pose.position.y = 0.05
        right_pose.position.y = -0.05
        left_pose.position.z = 1.25
        right_pose.position.z = 1.25

        self.motion_functions["horizontal_impulse_sl"] = {
            "x_knot_seed": (0.0, 0.0, 0.0, 0.0),
            "y_knot_seed": (0.23, 0.12, -0.12, 0.23),
            "z_knot_seed": (0.0, 0.0, 0.0, 0.0),
            "time_seed": (0.9, 1.4, 2.0, 2.8, 3.5),
            "initial_pose": copy.deepcopy(left_pose),
            "motion_type": "periodic",
        }
        self.motion_functions["horizontal_impulse_sr"] = {
            "x_knot_seed": (0.0, 0.0, 0.0, 0.0),
            "y_knot_seed": (0.23, 0.12, -0.12, 0.23),
            "z_knot_seed": (0.0, 0.0, 0.0, 0.0),
            "time_seed": (0.9, 1.4, 2.0, 2.8, 3.5),
            "initial_pose": copy.deepcopy(right_pose),
            "motion_type": "periodic",
        }
        self.motion_functions["vertical_throw_sr"]["y_knot_seed"] = (
            -0.1,
            -0.725,
            -0.735,
        )
        self.motion_functions["vertical_throw_sl"]["y_knot_seed"] = (0.1, 0.725, 0.735)
        left_pose.position.z = 1.35
        right_pose.position.z = 1.35
        self.motion_functions["left_throw_sl"]["initial_pose"] = left_pose
        self.motion_functions["left_throw_sr"]["initial_pose"] = right_pose
        self.current_motion = "circ_accel"

    def get_traj_for_transition_to_motion(self, desired_motion):
        """
        Return stick poses between the current position and start point of the desired motion
        Parameters:
        desired_motion: A string naming the motion desired. This must one of the accepted names contained in the self.motion_list list
        If current motion is the same as desired motion, returns without doing anything
        """
        # if(self.current_motion == desired_motion):
        #   print("Already executing this. Returning")0.9, 1.2, 1.6
        # Get start position of desired motion for each arm
        sr_target_pose = self.motion_functions[desired_motion + "_sl"]["initial_pose"]
        sl_target_pose = self.motion_functions[desired_motion + "_sr"]["initial_pose"]
        # Get direction of travel for both sticks as as vector
        sr_target_pose_vec = np.array(
            (
                sr_target_pose.position.x,
                sr_target_pose.position.y,
                sr_target_pose.position.z,
            )
        )
        sl_target_pose_vec = np.array(
            (
                sl_target_pose.position.x,
                sl_target_pose.position.y,
                sl_target_pose.position.z,
            )
        )

        sr_current_pose_vec = np.array(
            (
                self.last_stick_positions["pos_right"].x,
                self.last_stick_positions["pos_right"].y,
                self.last_stick_positions["pos_right"].z,
            )
        )
        sl_current_pose_vec = np.array(
            (
                self.last_stick_positions["pos_left"].x,
                self.last_stick_positions["pos_left"].y,
                self.last_stick_positions["pos_left"].z,
            )
        )

        # Get directions of travel for both arms
        left_dir = sl_target_pose_vec - sl_current_pose_vec
        right_dir = sr_target_pose_vec - sr_current_pose_vec
        if np.linalg.norm(left_dir) < 0.01 or np.linalg.norm(right_dir) < 0.01:
            self.initialize_robot_positions()
            return False, False
        time_to_target = 0.8  # Set a constant time to get to the target
        steps = int(time_to_target * self.pub_rate)  # the number of steps to target

        left_step_length = np.linalg.norm(left_dir) / steps
        right_step_length = np.linalg.norm(right_dir) / steps
        print(
            "Step lengths are : \n Left: "
            + str(left_step_length)
            + "\n Right: "
            + str(right_step_length)
        )

        # Get normal of direction vectors
        if left_step_length != 0.0:
            left_dir = left_dir / np.linalg.norm(left_dir)
        if right_step_length != 0.0:
            right_dir = right_dir / np.linalg.norm(right_dir)
        left_stick_pose_array = PoseArray()
        right_stick_pose_array = PoseArray()
        for i in range(0, steps + 1):
            pose_l = Pose()
            pose_r = Pose()
            # Get next waypoint by adding normal direction vector * step length to current position
            sr_current_pose_vec = sr_current_pose_vec + right_dir * right_step_length
            sl_current_pose_vec = sl_current_pose_vec + left_dir * left_step_length

            pose_r.position.x = sr_current_pose_vec[0]
            pose_r.position.y = sr_current_pose_vec[1]
            pose_r.position.z = sr_current_pose_vec[2]

            pose_l.position.x = sl_current_pose_vec[0]
            pose_l.position.y = sl_current_pose_vec[1]
            pose_l.position.z = sl_current_pose_vec[2]
            left_stick_pose_array.poses.append(copy.deepcopy(pose_l))
            right_stick_pose_array.poses.append(copy.deepcopy(pose_r))
        # The last pose published should be the target pose

        self.current_motion = desired_motion
        return left_stick_pose_array, right_stick_pose_array
        # self.start_publish()

    def stick_pose_at_time(self, function, time, rate=1.0):
        """
        Parameters:
        function: The motion function(s) to use. Expects a dictionary containing a function object for each X, Y and Z coordinates at time t
        as well as the time period of the function
        rate: Rate of rotation. Greater than one for faster motion
        time: The time at which to calculate the coordinates
        """

        t = (rate * time) % function["period"]  # Time
        pose = Pose()
        if self.constrain_to_plane:
            pose.position.x = self.DEFAULT_X_COORD
        else:
            pose.position.x = function["X"](t) + 0.3

        pose.position.y = function["Y"](t) + 0.2
        pose.position.z = function["Z"](t) - 0.1

        return pose

    def initialize_robot_positions(self):

        print("Waiting for robot arm initialization service")
        try:
            rospy.wait_for_service(
                "/initialize_robots_from_stick_positions", timeout=1.0
            )
            pose_left = self.motion_functions[self.current_motion + "_sl"][
                "initial_pose"
            ]
            pose_right = self.motion_functions[self.current_motion + "_sr"][
                "initial_pose"
            ]

            req = SetInitialStickPositionsRequest()
            req.left_stick_position = pose_left.position
            req.right_stick_position = pose_right.position

            self.set_robots_initial_position_service(req)
            self.last_stick_positions = {
                "pos_left": pose_left.position,
                "pos_right": pose_right.position,
            }
        except rospy.ROSException:
            print(
                "Service not found. Did you start the stick position to joint converter node?"
            )

        self.create_knot_server()

    def start_publish(self, loop=False, speed_factor=1.0):
        print("Starting publish")
        # self.check_amplitude()
        thread.start_new_thread(self._run_publish, (loop, speed_factor))

    def stop_publish(self):
        self.exit_publish_flag = True

    def _run_publish(self, loop=False, speed_factor=1.0):
        "This function is meant to be called in a separate thread by play_experiment"
        print("Starting publish 2")
        self.exit_publish_flag = False
        self.pause_publish_flag = False

        r = rospy.Rate(self.pub_rate)
        initial_time = rospy.get_time()
        motion = self.current_motion
        while True:
            time = rospy.get_time() - initial_time

            # Adding an empty pose as the robot controller requires a pose array msg
            pose_array = PoseArray()
            pose_l = self.stick_pose_at_time(
                function=self.motion_functions[motion + "_sl"],
                time=time,
                rate=speed_factor,
            )
            pose_r = self.stick_pose_at_time(
                function=self.motion_functions[motion + "_sr"],
                time=time,
                rate=speed_factor,
            )
            pose_array.poses.append(pose_l)
            pose_array.poses.append(pose_r)
            self.pub_stick_poses.publish(pose_array)
            self.last_stick_positions = {
                "pos_left": pose_l.position,
                "pos_right": pose_r.position,
            }
            self.update_and_publish_markers(
                {"stick_left": pose_l, "stick_right": pose_r}
            )
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
        pose_left = self.motion_functions[self.current_motion + "_sl"]["initial_pose"]
        pose_right = self.motion_functions[self.current_motion + "_sr"]["initial_pose"]

        left_pos = pose_left.position
        right_pos = pose_right.position
        rospy.set_param(
            "/right_stick_initial_position",
            [
                float(self.last_stick_positions["pos_right"].x),
                float(self.last_stick_positions["pos_right"].y),
                float(self.last_stick_positions["pos_right"].z),
            ],
        )
        rospy.set_param(
            "/left_stick_initial_position",
            [
                float(self.last_stick_positions["pos_left"].x),
                float(self.last_stick_positions["pos_left"].y),
                float(self.last_stick_positions["pos_left"].z),
            ],
        )
        # The initial rotational velocity of the diabolo
        rospy.set_param("/diabolo_initial_rot_velocity", 25.0)

        print("Done setting params")
        self._spawn_diabolo_in_gazebo()
        return True

    def _spawn_diabolo_in_gazebo(self):
        # Create service proxy
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        rospy.wait_for_service("/gazebo/spawn_urdf_model")

        # Load URDF
        with open(self.diabolo_urdf_file_path, "r") as f:
            poses = dict()
            model_xml = f.read()
            # Spawn model
            req = SpawnModelRequest()
            req.model_name = "diabolo"
            # req.initial_pose = diabolo_pose
            pose = Pose()
            pose.position.x = 0.7
            pose.position.y = 0.0
            pose.position.z = 0.7
            req.initial_pose.position = pose.position
            req.model_xml = model_xml
            req.robot_namespace = "/"
            req.reference_frame = "world"
            if spawn_model(req).success:
                print("Spawning diabolo in gazebo")

            rospy.sleep(0.2)

    def execute_periodic_trajectory_(
        self,
        a_bot_trajectory,
        b_bot_trajectory,
        speed_factor=0.5,
        confirm_execution=True,
        start_time=None,
    ):

        req = GetPlanningSceneRequest()
        req.components.components = req.components.ROBOT_STATE
        planning_scene = self.get_planning_scene_service(req)

        try:
            display_a_bot_traj = moveit_msgs.msg.DisplayTrajectory()
            display_b_bot_traj = moveit_msgs.msg.DisplayTrajectory()
            a_bot_robot_traj = moveit_msgs.msg.RobotTrajectory()
            b_bot_robot_traj = moveit_msgs.msg.RobotTrajectory()
            a_bot_robot_traj.joint_trajectory = a_bot_trajectory
            b_bot_robot_traj.joint_trajectory = b_bot_trajectory
            display_a_bot_traj.trajectory.append(a_bot_robot_traj)
            display_b_bot_traj.trajectory.append(b_bot_robot_traj)
            display_a_bot_traj.trajectory_start = planning_scene.scene.robot_state
            display_b_bot_traj.trajectory_start = planning_scene.scene.robot_state
            self.a_bot_display_traj_pub.publish(display_a_bot_traj)
            self.b_bot_display_traj_pub.publish(display_b_bot_traj)

            time_to_start = start_time
            if not time_to_start:
                time_to_start = rospy.Time.now()
            if confirm_execution:
                print("Execute this trajectory? y/n")
                e = raw_input()
                if e == "y":
                    time_to_start = rospy.Time.now() + rospy.Duration(0.1)
                    a_bot_trajectory.header.stamp = time_to_start
                    b_bot_trajectory.header.stamp = time_to_start
                    self.a_bot_command_pub.publish(a_bot_trajectory)
                    self.b_bot_command_pub.publish(b_bot_trajectory)

            else:
                # print("Auto execution selected. Executing")
                a_bot_trajectory.header.stamp = time_to_start
                b_bot_trajectory.header.stamp = time_to_start
                # if(time_to_start.to_sec() > rospy.Time.now().to_sec()):
                #   print("Time in header = " + str(time_to_start.to_sec()))
                #   print("Published time = " + str(rospy.Time.now().to_sec()))
                # else:
                #   rospy.logerr("Time in header = " + str(time_to_start.to_sec()))
                #   rospy.logerr("Published time = " + str(rospy.Time.now().to_sec()))
                self.a_bot_command_pub.publish(a_bot_trajectory)
                self.b_bot_command_pub.publish(b_bot_trajectory)

            return time_to_start + a_bot_trajectory.points[-1].time_from_start
        except:
            raise

    def execute_throw_trajectory_(
        self,
        a_bot_trajectory,
        b_bot_trajectory,
        time_of_flight=0.5,
        speed_factor=1.0,
        reverse=False,
        confirm_execution=True,
        start_time=None,
    ):

        a_bot_whole_trajectory = copy.deepcopy(a_bot_trajectory)
        b_bot_whole_trajectory = copy.deepcopy(b_bot_trajectory)
        last_time = rospy.Duration(0)
        old_last_time = rospy.Duration(0)
        new_a_bot_trajectory = copy.deepcopy(a_bot_whole_trajectory)
        for i in range(len(a_bot_whole_trajectory.points)):

            if not i == 0:
                step_length = (
                    a_bot_whole_trajectory.points[i].time_from_start
                    - a_bot_whole_trajectory.points[i - 1].time_from_start
                )
            else:
                step_length = a_bot_whole_trajectory.points[i].time_from_start

            new_step_length = step_length / speed_factor
            new_a_bot_trajectory.points[i].time_from_start = new_step_length + last_time
            last_time = new_step_length + last_time

        last_time = rospy.Duration(0)
        old_last_time = rospy.Duration(0)
        new_b_bot_trajectory = copy.deepcopy(b_bot_whole_trajectory)
        for i in range(len(b_bot_whole_trajectory.points)):

            if not i == 0:
                step_length = (
                    b_bot_whole_trajectory.points[i].time_from_start
                    - b_bot_whole_trajectory.points[i - 1].time_from_start
                )
            else:
                step_length = b_bot_whole_trajectory.points[i].time_from_start

            new_step_length = step_length / speed_factor
            new_b_bot_trajectory.points[i].time_from_start = new_step_length + last_time
            last_time = new_step_length + last_time
        time_to_start = start_time

        req = GetPlanningSceneRequest()
        req.components.components = req.components.ROBOT_STATE
        planning_scene = self.get_planning_scene_service(req)
        a_bot_display_traj = moveit_msgs.msg.DisplayTrajectory()
        b_bot_display_traj = moveit_msgs.msg.DisplayTrajectory()
        a_bot_robot_traj = moveit_msgs.msg.RobotTrajectory()
        a_bot_robot_traj.joint_trajectory = copy.deepcopy(new_a_bot_trajectory)
        b_bot_robot_traj = moveit_msgs.msg.RobotTrajectory()
        b_bot_robot_traj.joint_trajectory = copy.deepcopy(new_b_bot_trajectory)
        a_bot_display_traj.trajectory.append(a_bot_robot_traj)
        b_bot_display_traj.trajectory.append(b_bot_robot_traj)
        a_bot_display_traj.trajectory_start = planning_scene.scene.robot_state
        b_bot_display_traj.trajectory_start = planning_scene.scene.robot_state
        self.a_bot_display_traj_pub.publish(a_bot_display_traj)
        self.b_bot_display_traj_pub.publish(b_bot_display_traj)

        if confirm_execution:
            print("Execute this trajectory? y/n")
            e = raw_input()
            if e == "y":
                now = rospy.Time.now() + rospy.Duration(1.0)
                new_a_bot_trajectory.header.stamp = now
                new_b_bot_trajectory.header.stamp = now
                self.a_bot_command_pub.publish(new_a_bot_trajectory)
                self.b_bot_command_pub.publish(new_b_bot_trajectory)

        else:
            print("Auto execution selected. Executing")
            new_a_bot_trajectory.header.stamp = time_to_start
            new_b_bot_trajectory.header.stamp = time_to_start
            self.a_bot_command_pub.publish(new_a_bot_trajectory)
            self.b_bot_command_pub.publish(new_b_bot_trajectory)

            return time_to_start + new_a_bot_trajectory.points[-1].time_from_start

        # if(reverse):
        #   self.last_stick_positions["pos_left"]  = left_stick_traj.poses[0].position
        #   self.last_stick_positions["pos_right"] = right_stick_traj.poses[0].position
        # Increase the time from start for all the

        #   rospy.sleep(time_of_flight)
        #   TEMP: Reverse motion nack to starting point of the trajectory
        #   req = CreateRobotTrajectoryRequest()
        #   number_of_poses = len(left_stick_traj.poses)

        # resp = self.command_robot_trajectory_service(req)
        # if(resp.success):
        #   print("Reverse trajectory executed!")

    def make_prediction_request_msg_(
        self, planned_left_poses=None, planned_right_poses=None
    ):
        # Goal positions and velocities are arrays of the appropriate type
        req = CreateSticksTrajectoryRequest()
        ################### Set current Sim Config
        req.current_sim_config.pv_pre_cap_scale = 0.13
        req.current_sim_config.pv_post_cap_scale = 0.13
        req.current_sim_config.pv_cap_scale = 0.07
        req.current_sim_config.velocity_diffusion_factor = 0.9999
        if (
            self.motion_functions[self.current_motion + "_sl"]["motion_type"]
            == "oneshot"
        ):
            req.motion_flag = CreateSticksTrajectoryRequest.THROW

        elif (
            self.motion_functions[self.current_motion + "_sl"]["motion_type"]
            == "periodic"
        ):
            req.motion_flag = CreateSticksTrajectoryRequest.LOOP
        # Diabolo constant parameters
        req.current_sim_config.mass = 0.2
        req.current_sim_config.axle_radius = 0.0065
        req.current_sim_config.string_length = 1.58

        if planned_left_poses and planned_right_poses:
            req.planned_left_stick_poses = copy.deepcopy(planned_left_poses)
            req.planned_right_stick_poses = copy.deepcopy(planned_right_poses)

        return req

    def run_oneshot_motion(
        self,
        interactive=True,
        confirm_execution=True,
        preparatory_motion="horizontal_impulse",
    ):

        planned_left_poses = None
        planned_right_poses = None
        # The trajectory begins one second from now
        trajectory_start_time = rospy.Time.now() + rospy.Duration(1.0)
        prediction_time = 0
        diab_state_req = GetDiaboloStateRequest()
        diab_state_req.header.stamp = rospy.Time.now()
        diabolo_state_resp = copy.deepcopy(
            self.get_diabolo_state_service(diab_state_req)
        )
        self.latest_diabolo_state = copy.deepcopy(diabolo_state_resp.state)
        # Get diabolo orientation here, to handle pitch and yaw
        dp = self.latest_diabolo_state.pose.orientation

        self.get_stick_tips_from_tf()
        left_stick_start_pos = self.last_stick_positions["pos_left"]
        right_stick_start_pos = self.last_stick_positions["pos_right"]

        # Execute a pre-defined motion (e.g. to give a horizontal impulse for sideways throws)
        if self.current_motion == "left_throw" or self.current_motion == "right_throw":
            motion = copy.deepcopy(self.current_motion)
            self.current_motion = do_preparatory_motion
            prediction_start_time = rospy.Time.now()
            (
                a_bot_trajectory,
                b_bot_trajectory,
                left_stick_poses,
                right_stick_poses,
            ) = self.call_prediction_service(
                interactive=False,
                planned_left_poses=planned_left_poses,
                planned_right_poses=planned_right_poses,
                left_stick_start_pos=left_stick_start_pos,
                right_stick_start_pos=right_stick_start_pos,
                plan=False,
            )
            trajectory_end_time = self.execute_periodic_trajectory_(
                a_bot_trajectory,
                b_bot_trajectory,
                1.0,
                True,
                start_time=trajectory_start_time,
            )
            safe_prediction_time = rospy.Duration(0.5)

            sleep_time = (trajectory_end_time - rospy.Time.now()) - safe_prediction_time
            rospy.sleep(sleep_time)
            diab_state_req = GetDiaboloStateRequest()
            diab_state_req.header.stamp = rospy.Time.now()
            diabolo_state_resp = copy.deepcopy(
                self.get_diabolo_state_service(diab_state_req)
            )
            self.latest_diabolo_state = copy.deepcopy(diabolo_state_resp.state)
            trajectory_start_time = trajectory_end_time
            self.current_motion = motion

        (
            a_bot_trajectory,
            b_bot_trajectory,
            left_stick_poses,
            right_stick_poses,
        ) = self.call_prediction_service(
            interactive=True,
            planned_left_poses=planned_left_poses,
            planned_right_poses=planned_right_poses,
            left_stick_start_pos=left_stick_start_pos,
            right_stick_start_pos=right_stick_start_pos,
            plan=False,
        )

        if a_bot_trajectory:
            trajectory_end_time = self.execute_throw_trajectory_(
                a_bot_trajectory,
                b_bot_trajectory,
                1.0,
                1.0,
                True,
                False,
                start_time=trajectory_start_time,
            )

            # Create reverse trajectory
            # TODO: Add the initial point to the reversed trajectory. The calculated trajectory does not have the first point.
            # There is probably also not much point keeping the last point of the old trajectory (That is the current position)
            reverse_a_bot_trajectory = JointTrajectory()
            reverse_b_bot_trajectory = JointTrajectory()
            # print(len(a_bot_trajectory.points))

            reverse_a_bot_trajectory.joint_names = a_bot_trajectory.joint_names
            reverse_b_bot_trajectory.joint_names = b_bot_trajectory.joint_names

            for i in range(len(a_bot_trajectory.points)):
                # print("Now adding " + str(a_bot_trajectory.points[i].time_from_start.to_sec()) + " time from start reverse traj")
                reverse_a_bot_trajectory.points.append(
                    copy.deepcopy(
                        a_bot_trajectory.points[len(a_bot_trajectory.points) - 1 - i]
                    )
                )
                reverse_a_bot_trajectory.points[
                    i
                ].time_from_start = a_bot_trajectory.points[i].time_from_start

            for i in range(len(b_bot_trajectory.points)):
                reverse_b_bot_trajectory.points.append(
                    copy.deepcopy(
                        b_bot_trajectory.points[len(b_bot_trajectory.points) - 1 - i]
                    )
                )
                reverse_b_bot_trajectory.points[
                    i
                ].time_from_start = b_bot_trajectory.points[i].time_from_start
            # rospy.logwarn("Forward trajectory")
            # print(a_bot_trajectory)

            # rospy.logwarn("Reverse trajectory")
            # print(reverse_a_bot_trajectory)

            print("Reverse trajectory? y/n?")
            e = raw_input()
            if e == "y":
                trajectory_start_time = rospy.Time.now() + rospy.Duration(0.01)
                trajectory_end_time = self.execute_throw_trajectory_(
                    reverse_a_bot_trajectory,
                    reverse_b_bot_trajectory,
                    1.0,
                    0.8,
                    False,
                    False,
                    start_time=trajectory_start_time,
                )

        else:
            rospy.logerr("Could not find a trajectory")

    def start_periodic_motion(
        self,
        interactive=True,
        confirm_execution=True,
        preparatory_motion="horizontal_impulse",
    ):
        self.stop_motion_flag = False
        thread.start_new_thread(
            self.run_periodic_motion,
            (interactive, confirm_execution, preparatory_motion),
        )

    def stop_periodic_motion(self):
        self.stop_motion_flag = True

    def run_periodic_motion(
        self,
        interactive=False,
        confirm_execution=True,
        preparatory_motion="horizontal_impulse",
    ):
        #### IMPORTANT: This assumes that the points in the trajectory are evenly spaced
        #### That must be assured by the node providing the stick trajectory/robot trajectory generating service
        planned_left_poses = None
        planned_right_poses = None
        # The trajectory begins one second from now
        trajectory_start_time = rospy.Time.now() + rospy.Duration(1.0)
        prediction_time = 0
        diab_state_req = GetDiaboloStateRequest()
        diab_state_req.header.stamp = rospy.Time.now()
        diabolo_state_resp = copy.deepcopy(
            self.get_diabolo_state_service(diab_state_req)
        )
        self.latest_diabolo_state = copy.deepcopy(diabolo_state_resp.state)
        # Get diabolo orientation here, to handle pitch and yaw
        dp = self.latest_diabolo_state.pose.orientation

        self.get_stick_tips_from_tf()
        left_stick_start_pos = self.last_stick_positions["pos_left"]
        right_stick_start_pos = self.last_stick_positions["pos_right"]

        # First, execute the pre-defined horizontal motion
        motion = copy.deepcopy(self.current_motion)
        if preparatory_motion:
            self.current_motion = preparatory_motion

            (
                a_bot_trajectory,
                b_bot_trajectory,
                left_stick_poses,
                right_stick_poses,
            ) = self.call_prediction_service(
                interactive=False,
                planned_left_poses=None,
                planned_right_poses=None,
                left_stick_start_pos=left_stick_start_pos,
                right_stick_start_pos=right_stick_start_pos,
                plan=False,
            )
            trajectory_end_time = self.execute_periodic_trajectory_(
                a_bot_trajectory,
                b_bot_trajectory,
                1.0,
                confirm_execution,
                start_time=trajectory_start_time,
            )

            # In this part of the code, "prediction" means motion generation
            safe_prediction_time = rospy.Duration(0.5)
            prediction_start_time = (
                trajectory_end_time - trajectory_start_time
            ) - safe_prediction_time
            print("Prediction start time = " + str(prediction_start_time.to_sec()))
            # Break out if there is not enough time to plan
            if prediction_start_time.to_sec() < 0.0:
                print(
                    "Prediction time is too long. Is = "
                    + str(safe_prediction_time.to_sec())
                )
                prediction_start_time = trajectory_end_time
                planned_left_poses = None
                planned_right_poses = None
                last_traj_end_time = rospy.Time.now() + rospy.Duration(1.0)
            else:
                # rospy.logwarn("prediction_start_time is " + str(prediction_start_time.to_sec()))
                # rospy.logwarn("Trajectory length is " + str((trajectory_end_time - trajectory_start_time).to_sec()))
                # Find the point from which to get planned poses
                planned_left_poses = geometry_msgs.msg.PoseArray()
                planned_right_poses = geometry_msgs.msg.PoseArray()
                # This is the id of the last pose to execute in the sent trajectory
                last_pose_to_execute = 0
                for j in range(len(a_bot_trajectory.points) - 1):
                    # print("Time from start for j = " + str(j) + " is" + str(a_bot_trajectory.points[j].time_from_start.to_sec()))
                    if (
                        a_bot_trajectory.points[j].time_from_start
                        <= prediction_start_time
                        and a_bot_trajectory.points[j + 1].time_from_start
                        > prediction_start_time
                    ):
                        # pass the left and right poses from the ith position onwards as planned trajectories to the prediction node
                        planned_left_poses.poses = left_stick_poses.poses[j:]
                        planned_right_poses.poses = right_stick_poses.poses[j:]
                        last_pose_to_execute = j
                        break

                # Store end position of start trajectory as start position of old trajectory
                print(
                    "last_pose to execute is "
                    + str(last_pose_to_execute)
                    + " at time "
                    + str(
                        a_bot_trajectory.points[
                            last_pose_to_execute
                        ].time_from_start.to_sec()
                    )
                )
                left_stick_start_pos = planned_left_poses.poses[0].position
                right_stick_start_pos = planned_right_poses.poses[0].position
                planned_left_poses.poses = planned_left_poses.poses[1:]
                planned_right_poses.poses = planned_right_poses.poses[1:]

            # Sleep until the next trajectory is at left_stick_start_pos
            now = rospy.Time.now()
            sleep_time1 = (
                trajectory_start_time - now
            )  # Until current trajectory is over
            sleep_time2 = (
                trajectory_end_time - trajectory_start_time
            ) - safe_prediction_time
            # Until next trajectory is at left_stick_start_pos
            sleep_time = sleep_time1 + sleep_time2
            rospy.sleep(sleep_time)

            # Change the current motion back to what it was before applying the impulse
            self.current_motion = motion
            trajectory_start_time = trajectory_end_time
        else:
            trajectory_start_time = rospy.Time.now() + rospy.Duration(1.0)
            prediction_time = 0

        diab_state_req = GetDiaboloStateRequest()
        diab_state_req.header.stamp = rospy.Time.now()
        diabolo_state_resp = copy.deepcopy(
            self.get_diabolo_state_service(diab_state_req)
        )
        self.latest_diabolo_state = copy.deepcopy(diabolo_state_resp.state)

        while True:
            prediction_start_time = rospy.Time.now()
            (
                a_bot_trajectory,
                b_bot_trajectory,
                left_stick_poses,
                right_stick_poses,
            ) = self.call_prediction_service(
                interactive=True,
                planned_left_poses=planned_left_poses,
                planned_right_poses=planned_right_poses,
                left_stick_start_pos=left_stick_start_pos,
                right_stick_start_pos=right_stick_start_pos,
                plan=False,
            )

            # If user-set flag is true, stop moving the arms
            if self.stop_motion_flag or rospy.is_shutdown():
                break

            # Ensure that the prediction service found something
            if a_bot_trajectory:
                reverse = False
                if (
                    self.motion_functions[self.current_motion + "_sl"]["motion_type"]
                    == "periodic"
                ):
                    # Queue up the trajectory in the driver, so it will be executed next
                    trajectory_end_time = self.execute_periodic_trajectory_(
                        a_bot_trajectory,
                        b_bot_trajectory,
                        1.0,
                        confirm_execution,
                        start_time=trajectory_start_time,
                    )

                # Time that this prediction/motion generation took
                prediction_time = rospy.Time.now() - prediction_start_time
                # Add safety buffer
                safe_prediction_time = prediction_time + prediction_time * 0.3

                # This should be the time from start for the first pose for the list of "planned poses"
                # when planning the next trajectory
                prediction_start_time = (
                    trajectory_end_time - trajectory_start_time
                ) - safe_prediction_time

                # Don't plan another trajectory if there is not enough time to plan it
                if prediction_start_time.to_sec() < 0.0:
                    # print("Prediction time is too long. Is = "  + str(safe_prediction_time.to_sec()))
                    prediction_start_time = trajectory_end_time
                    planned_left_poses = None
                    planned_right_poses = None
                    last_traj_end_time = rospy.Time.now() + rospy.Duration(1.0)
                    break

                else:  # Prepare next loop
                    # rospy.logwarn("prediction_start_time is " + str(prediction_start_time.to_sec()))
                    # rospy.logwarn("Trajectory length is " + str((trajectory_end_time - trajectory_start_time).to_sec()))
                    # Find the point from which to get planned poses
                    planned_left_poses = geometry_msgs.msg.PoseArray()
                    planned_right_poses = geometry_msgs.msg.PoseArray()

                    # Find the last point in the stick trajectory to be executed before beginning the prediction
                    # for the diabolo state at the end of the current trajectory
                    # Trim planned poses to contain only remainder of next trajectory. This will be used during the
                    # next iteration.
                    last_pose_to_execute = 0
                    for j in range(len(a_bot_trajectory.points) - 1):
                        if (
                            a_bot_trajectory.points[j].time_from_start
                            <= prediction_start_time
                            and a_bot_trajectory.points[j + 1].time_from_start
                            > prediction_start_time
                        ):
                            # pass the left and right poses from the ith positon onwards as planned trajectories to the prediction node
                            # print("len(a_bot_trajectory.points)", len(a_bot_trajectory.points))
                            # print("len(left_stick_poses.poses)", len(left_stick_poses.poses))
                            planned_left_poses.poses = left_stick_poses.poses[j:]
                            planned_right_poses.poses = right_stick_poses.poses[j:]
                            last_pose_to_execute = j
                            break

            # Store end position of start trajectory as start position of old trajectory

            left_stick_start_pos = planned_left_poses.poses[0].position
            right_stick_start_pos = planned_right_poses.poses[0].position
            planned_left_poses.poses = planned_left_poses.poses[1:]
            planned_right_poses.poses = planned_right_poses.poses[1:]

            # Sleep until the next trajectory is at left_stick_start_pos
            now = rospy.Time.now()
            sleep_time1 = (
                trajectory_start_time - now
            )  # Until current trajectory is over
            sleep_time2 = (
                trajectory_end_time - trajectory_start_time
            ) - safe_prediction_time
            # Until next trajectory is at left_stick_start_pos

            sleep_time = sleep_time1 + sleep_time2
            rospy.sleep(sleep_time)
            # self.pause_gazebo_service()
            diab_state_req = GetDiaboloStateRequest()
            diab_state_req.header.stamp = (
                trajectory_start_time
                + a_bot_trajectory.points[last_pose_to_execute].time_from_start
            )
            self.latest_diabolo_state = copy.deepcopy(
                self.get_diabolo_state_service(diab_state_req).state
            )

            trajectory_start_time = trajectory_end_time
        # self.unpause_gazebo_service()
        return

    def get_diabolo_waypoint_goals(
        self,
        goal_velocity=geometry_msgs.msg.Point(),
        goal_position=geometry_msgs.msg.Point(),
    ):
        # This is the initial position. Do not add to request waypoints
        goal_states = []
        goal_state = DiaboloState()

        diabolo_goal_pos = geometry_msgs.msg.Point()
        diabolo_goal_vel = geometry_msgs.msg.Point()

        # if self.latest_diabolo_pose:
        #   diabolo_goal_pos = copy.deepcopy(self.latest_diabolo_pose.position)
        # else:
        if (
            self.motion_functions[self.current_motion + "_sl"]["motion_type"]
            == "periodic"
        ):
            diabolo_goal_pos = geometry_msgs.msg.Point()
            diabolo_goal_pos.x = self.DEFAULT_X_COORD
            diabolo_goal_pos.y = -0.0382
            diabolo_goal_pos.z = 0.51991

            ## First waypoint
            diabolo_goal_pos.x = self.DEFAULT_X_COORD
            diabolo_goal_pos.y = diabolo_goal_pos.y + 0.3
            diabolo_goal_pos.z = diabolo_goal_pos.z + 0.2

            diabolo_goal_vel.x = 0.0
            diabolo_goal_vel.y = -0.5
            diabolo_goal_vel.z = 1.0

            goal_state.trans_velocity = copy.deepcopy(diabolo_goal_vel)
            goal_state.pose.position = copy.deepcopy(diabolo_goal_pos)
            goal_state.pose.orientation.w = 1.0
            goal_states.append(copy.deepcopy(goal_state))

            ## Second waypoint

            diabolo_goal_pos.x = self.DEFAULT_X_COORD
            diabolo_goal_pos.y = diabolo_goal_pos.y - 0.1
            diabolo_goal_pos.z = diabolo_goal_pos.z + 0.2

            diabolo_goal_vel.x = 0.0
            diabolo_goal_vel.y = -1.0
            diabolo_goal_vel.z = 0.1

            goal_state.trans_velocity = copy.deepcopy(diabolo_goal_vel)
            goal_state.pose.position = copy.deepcopy(diabolo_goal_pos)
            goal_state.pose.orientation.w = 1.0
            goal_states.append(copy.deepcopy(goal_state))

            ## Third waypoint

            diabolo_goal_pos.x = self.DEFAULT_X_COORD
            diabolo_goal_pos.y = diabolo_goal_pos.y - 0.5
            diabolo_goal_pos.z = diabolo_goal_pos.z + 0.0

            diabolo_goal_vel.x = 0.0
            diabolo_goal_vel.y = -0.5
            diabolo_goal_vel.z = -0.5

            goal_state.trans_velocity = copy.deepcopy(diabolo_goal_vel)
            goal_state.pose.position = copy.deepcopy(diabolo_goal_pos)
            goal_state.pose.orientation.w = 1.0
            goal_states.append(copy.deepcopy(goal_state))

            ## Fourth waypoint

            diabolo_goal_pos.x = self.DEFAULT_X_COORD
            diabolo_goal_pos.y = diabolo_goal_pos.y - 0.1
            diabolo_goal_pos.z = diabolo_goal_pos.z - 0.2

            diabolo_goal_vel.x = 0.0
            diabolo_goal_vel.y = 1.0
            diabolo_goal_vel.z = -0.5

            goal_state.trans_velocity = copy.deepcopy(diabolo_goal_vel)
            goal_state.pose.position = copy.deepcopy(diabolo_goal_pos)
            goal_state.pose.orientation.w = 1.0
            goal_states.append(copy.deepcopy(goal_state))
        # End of if current motion is circular acceleration

        else:
            diabolo_goal_pos = geometry_msgs.msg.Point(
                x=self.DEFAULT_X_COORD, y=0.0, z=1.25
            )
            diabolo_goal_vel = geometry_msgs.msg.Point(x=0.0, y=0.0, z=1.4)  # 0.1 m
            # diabolo_goal_vel = geometry_msgs.msg.Point(x=0.0, y=0.0, z=2.0)  # 0.2 m
            # diabolo_goal_vel = geometry_msgs.msg.Point(x=0.0, y=0.0, z=2.8)  # 0.4 m
            # diabolo_goal_vel = geometry_msgs.msg.Point(x=0.0, y=0.0, z=3.4)  # 0.6 m
            # diabolo_goal_vel = geometry_msgs.msg.Point(x=0.0, y=0.0, z=3.97) # 0.8 m
            # diabolo_goal_vel = geometry_msgs.msg.Point(x=0.0, y=0.0, z=4.4)  # 1.0 m
            goal_state.trans_velocity = copy.deepcopy(diabolo_goal_vel)
            goal_state.pose.position = copy.deepcopy(diabolo_goal_pos)
            goal_states.append(goal_state)

        return goal_states

    def save_current_knot_points(self, filename="default.pkl"):
        path = os.path.join(self._package_directory, "config", filename)
        print("Saving to " + path)
        with open(path, "w") as f:
            pickle.dump(self.motion_functions, f)

    def call_prediction_service(
        self,
        planned_left_poses=None,
        planned_right_poses=None,
        interactive=False,
        left_stick_start_pos=None,
        right_stick_start_pos=None,
        plan=True,
    ):
        """
        Call the service that returns a robot trajectory for a given set of diabolo goal states.
        The service starts planning a new motion starting from a point in the future.
        planned_left_poses, planned_right_poses are the stick trajectories that will be executed
        until that point in the future.

        The current diabolo state is added inside this method, and the diabolo state in the future
        estimated using the planned_poses.
        left_stick_start_pos, right_stick_start_pos are the stick positions *before* that prediction,
        or the start position to plan from if the planned_poses are empty.
        """
        ## Set diabolo goal states
        # rospy.logwarn("Entered prediction service function")
        ## TODO: Change this to allow multiple waypoint goals
        req = self.make_prediction_request_msg_(planned_left_poses, planned_right_poses)
        req.goal_states = self.get_diabolo_waypoint_goals()

        ## Set current sim config

        # Diabolo velocity and pose
        diabolo_pose = Pose()
        diabolo_vel = geometry_msgs.msg.Point()

        if self.latest_diabolo_state:
            # print("Using actual diabolo starting position")
            diabolo_pose = self.latest_diabolo_state.pose
            diabolo_vel = self.latest_diabolo_state.trans_velocity

        else:
            rospy.logwarn("Using default diabolo coordinates for prediction service")
            diabolo_pose.position.x = self.DEFAULT_X_COORD
            diabolo_pose.position.y = 0.053
            diabolo_pose.position.z = 0.554

        req.current_sim_config.trans_velocity = geometry_msgs.msg.Point()
        sl_pose = Pose()
        sr_pose = Pose()

        # self.get_stick_tips_from_tf()
        ## TODO: Get the actual current stick poses. This is temporary
        sl_pose.position = left_stick_start_pos
        sr_pose.position = right_stick_start_pos

        req.current_sim_config.initial_poses.poses.append(diabolo_pose)

        req.current_sim_config.trans_velocity = diabolo_vel
        req.current_sim_config.initial_poses.poses.append(sl_pose)
        req.current_sim_config.initial_poses.poses.append(sr_pose)

        # IMPORTANT: Must give stick update rate and sim time step
        req.stick_update_time_step = 1.0 / self.pub_rate
        req.current_sim_config.time_step = 0.002
        req.optimize = plan
        req.constrain_to_YZ = True
        # Set spline knot point seeds
        # Set seeds for left stick

        time_seed = self.motion_functions[self.current_motion + "_sl"]["time_seed"]
        # Plot the stick trajectories/splines
        # These are the seeds for the chosen motion
        if not interactive:
            """
            If not interactive, use the motion seeds precalculated for the current motion
            """
            left_motion = self.motion_functions[self.current_motion + "_sl"]
            right_motion = self.motion_functions[self.current_motion + "_sr"]

            # The number of knot points should correspond to the number of time seeds

            for i in range(len(time_seed)):
                left_knot_point = geometry_msgs.msg.Point()
                right_knot_point = geometry_msgs.msg.Point()
                if (
                    i == len(time_seed) - 1
                    and self.motion_functions[self.current_motion + "_sl"][
                        "motion_type"
                    ]
                    == "periodic"
                ):
                    # If this is a periodic motion, the last knot point must be at the initial robot position
                    left_knot_point = geometry_msgs.msg.Point()
                    right_knot_point = geometry_msgs.msg.Point()

                else:
                    left_knot_point.x = copy.deepcopy(left_motion["x_knot_seed"][i])
                    left_knot_point.y = copy.deepcopy(left_motion["y_knot_seed"][i])
                    left_knot_point.z = copy.deepcopy(left_motion["z_knot_seed"][i])
                    right_knot_point.x = copy.deepcopy(right_motion["x_knot_seed"][i])
                    right_knot_point.y = copy.deepcopy(right_motion["y_knot_seed"][i])
                    right_knot_point.z = copy.deepcopy(right_motion["z_knot_seed"][i])

                if self.changed_tilt_offset_flag:
                    # This means the tilt offset has changed since the last trajectory.
                    # Must add tilt over the course of this new trajectory
                    print(
                        "Setting left x knot seed at "
                        + str(self.tilt_offset * ((float(i + 1)) / len(time_seed)))
                    )
                    left_knot_point.x -= self.tilt_offset * (
                        float((i + 1.0)) / len(time_seed)
                    )
                    right_knot_point.x += self.tilt_offset * (
                        float((i + 1.0)) / len(time_seed)
                    )

                req.knot_seeds.left_seed.append(copy.deepcopy(left_knot_point))
                req.knot_seeds.right_seed.append(copy.deepcopy(right_knot_point))
                req.knot_seeds.time_seed.append(copy.deepcopy(time_seed[i]))

        else:
            """
            If interactive, set the motion seeds to the points gotten from the interactive markers
            """

            marker_positions = copy.deepcopy(
                self.knot_point_server.get_marker_positions(relative=True)
            )
            # Replace current knot seeds with newly set knot seeds
            self.motion_functions[self.current_motion + "_sl"]["x_knot_seed"] = []
            self.motion_functions[self.current_motion + "_sl"]["y_knot_seed"] = []
            self.motion_functions[self.current_motion + "_sl"]["z_knot_seed"] = []
            self.motion_functions[self.current_motion + "_sr"]["x_knot_seed"] = []
            self.motion_functions[self.current_motion + "_sr"]["z_knot_seed"] = []
            self.motion_functions[self.current_motion + "_sr"]["y_knot_seed"] = []
            left_positions = marker_positions[0]
            right_positions = marker_positions[1]
            # print(left_positions)

            for i in range(len(time_seed)):
                left_knot_point = geometry_msgs.msg.Point()
                right_knot_point = geometry_msgs.msg.Point()
                if (
                    i == len(time_seed) - 1
                    and self.motion_functions[self.current_motion + "_sl"][
                        "motion_type"
                    ]
                    == "periodic"
                ):
                    # If this is a periodic motion, the last knot point must be at the initial robot position
                    left_knot_point = geometry_msgs.msg.Point()
                    right_knot_point = geometry_msgs.msg.Point()

                else:
                    left_knot_point = copy.deepcopy(left_positions[i])
                    right_knot_point = copy.deepcopy(right_positions[i])
                    self.motion_functions[self.current_motion + "_sl"][
                        "x_knot_seed"
                    ].append(left_knot_point.x)
                    self.motion_functions[self.current_motion + "_sl"][
                        "y_knot_seed"
                    ].append(left_knot_point.y)
                    self.motion_functions[self.current_motion + "_sl"][
                        "z_knot_seed"
                    ].append(left_knot_point.z)
                    self.motion_functions[self.current_motion + "_sr"][
                        "x_knot_seed"
                    ].append(right_knot_point.x)
                    self.motion_functions[self.current_motion + "_sr"][
                        "y_knot_seed"
                    ].append(right_knot_point.y)
                    self.motion_functions[self.current_motion + "_sr"][
                        "z_knot_seed"
                    ].append(right_knot_point.z)

                if self.changed_tilt_offset_flag:
                    # This means the tilt offset has changed since the last trajectory.
                    # Must add tilt over the course of this new trajectory
                    print(
                        "Setting left x knot seed at "
                        + str(self.tilt_offset * ((float(i + 1)) / len(time_seed)))
                    )
                    left_knot_point.x -= self.tilt_offset * (
                        float((i + 1.0)) / len(time_seed)
                    )
                    right_knot_point.x += self.tilt_offset * (
                        float((i + 1.0)) / len(time_seed)
                    )

                req.knot_seeds.left_seed.append(copy.deepcopy(left_knot_point))
                req.knot_seeds.right_seed.append(copy.deepcopy(right_knot_point))
                req.knot_seeds.time_seed.append(copy.deepcopy(time_seed[i]))

            self.changed_tilt_offset_flag = False

        # rospy.logwarn("Calling prediction service")
        resp = self.generate_trajectory_service(req)
        # rospy.logwarn("Prediction service returned")
        if resp.success:
            # print("Got trajectories!")
            self.marker_count = 0
            marker_array = []

            self.left_traj_plan_marker = self._make_marker_from_mesh(
                mesh_filename="",
                namespace="left_stick_plan",
                scale=(0.01, 1, 1),
                color=(1, 0, 0),
            )
            self.left_traj_plan_marker.type = visualization_msgs.msg.Marker.LINE_STRIP
            for i in resp.left_stick_poses.poses:
                self.left_traj_plan_marker.points.append(i.position)

            self.right_traj_plan_marker = self._make_marker_from_mesh(
                mesh_filename="",
                namespace="right_stick_plan",
                scale=(0.01, 1, 1),
                color=(0, 0, 1),
            )
            self.right_traj_plan_marker.type = visualization_msgs.msg.Marker.LINE_STRIP
            for i in resp.right_stick_poses.poses:
                self.right_traj_plan_marker.points.append(i.position)

            marker_array.append(copy.deepcopy(self.left_traj_plan_marker))
            marker_array.append(copy.deepcopy(self.right_traj_plan_marker))

            self.sphere_marker_1.pose = sr_pose
            self.sphere_marker_1.pose.orientation.w = 1.0
            self.sphere_marker_1.ns = "right_stick_plan"
            self.sphere_marker_1.color.r = 0
            self.sphere_marker_1.color.g = 0
            self.sphere_marker_1.color.b = 1

            self.sphere_marker_2.pose = sl_pose
            self.sphere_marker_2.ns = "left_stick_plan"
            self.sphere_marker_2.pose.orientation.w = 1.0
            self.sphere_marker_2.color.r = 1
            self.sphere_marker_2.color.g = 0
            self.sphere_marker_2.color.b = 0

            marker_array.append(copy.deepcopy(self.sphere_marker_1))
            marker_array.append(copy.deepcopy(self.sphere_marker_2))

            # Display diabolo start state with a white marker
            initial_pos_shell_marker = self._make_marker_from_mesh(
                "package://diabolo_scene_description/meshes/diabolo_shell.stl",
                color=(1.0, 1.0, 1.0),
                scale=[0.001, 0.001, 0.001],
                namespace="initial_pos",
            )
            initial_pos_shell_marker.pose = diabolo_pose
            initial_pos_fixator_marker = self._make_marker_from_mesh(
                "package://diabolo_scene_description/meshes/diabolo_fixators.stl",
                color=(1.0, 1.0, 1.0),
                scale=[0.001, 0.001, 0.001],
                namespace="initial_pos",
            )
            initial_pos_fixator_marker.pose = diabolo_pose
            initial_pos_axis_marker = self._make_marker_from_mesh(
                "package://diabolo_scene_description/meshes/diabolo_axis.stl",
                color=(1.0, 1.0, 1.0),
                scale=[0.001, 0.001, 0.001],
                namespace="initial_pos",
            )
            initial_pos_axis_marker.pose = diabolo_pose

            marker_array.append(copy.deepcopy(initial_pos_shell_marker))
            marker_array.append(copy.deepcopy(initial_pos_fixator_marker))
            marker_array.append(copy.deepcopy(initial_pos_axis_marker))

            initial_diabolo_vel_marker = self._make_marker_from_mesh(
                "",
                color=(1.0, 1.0, 1.0),
                scale=[0.03, 0.02, 0.02],
                namespace="initial_pos",
            )
            initial_vel_base = geometry_msgs.msg.Point()
            initial_vel_tip = geometry_msgs.msg.Point()
            initial_diabolo_vel_marker.type = visualization_msgs.msg.Marker.ARROW
            initial_vel_base = diabolo_pose.position
            initial_vel_tip.x = initial_vel_base.x + (diabolo_vel.x) / 2.0
            initial_vel_tip.y = initial_vel_base.y + (diabolo_vel.y) / 2.0
            initial_vel_tip.z = initial_vel_base.z + (diabolo_vel.z) / 2.0
            initial_diabolo_vel_marker.points.append(initial_vel_base)
            initial_diabolo_vel_marker.points.append(initial_vel_tip)

            marker_array.append(copy.deepcopy(initial_diabolo_vel_marker))
            marker_count = self.marker_count
            diabolo_shell_marker = self._make_marker_from_mesh(
                "package://diabolo_scene_description/meshes/diabolo_shell.stl",
                color=(0.0, 1.0, 0.0),
                scale=[0.001, 0.001, 0.001],
                namespace="",
            )
            diabolo_fixator_marker = self._make_marker_from_mesh(
                "package://diabolo_scene_description/meshes/diabolo_fixators.stl",
                color=(0.1, 0.1, 0.1),
                scale=[0.001, 0.001, 0.001],
                namespace="",
            )
            diabolo_axis_marker = self._make_marker_from_mesh(
                "package://diabolo_scene_description/meshes/diabolo_axis.stl",
                color=(0.7, 0.7, 0.7),
                scale=[0.001, 0.001, 0.001],
                namespace="",
            )
            goal_diabolo_shell_marker = self._make_marker_from_mesh(
                "package://diabolo_scene_description/meshes/diabolo_shell.stl",
                color=(100.0 / 255.0, 255.0 / 255.0, 50.0 / 255.0),
                scale=[0.001, 0.001, 0.001],
                namespace="",
            )
            goal_diabolo_fixator_marker = self._make_marker_from_mesh(
                "package://diabolo_scene_description/meshes/diabolo_fixators.stl",
                color=(0.1, 0.1, 0.1),
                scale=[0.001, 0.001, 0.001],
                namespace="",
            )
            goal_diabolo_axis_marker = self._make_marker_from_mesh(
                "package://diabolo_scene_description/meshes/diabolo_axis.stl",
                color=(0.7, 0.7, 0.7),
                scale=[0.001, 0.001, 0.001],
                namespace="",
            )
            goal_diabolo_vel_marker = self._make_marker_from_mesh(
                "",
                color=(100.0 / 255.0, 255.0 / 255.0, 50.0 / 255.0),
                scale=[0.02, 0.02, 0.02],
                namespace="",
            )
            diabolo_to_goal_marker = self._make_marker_from_mesh(
                "",
                color=(1.0, 1.0, 1.0),
                scale=[0.01, 0.02, 0.02],
                namespace="",
                alpha=0.5,
            )
            self.marker_count = marker_count + 1
            for i in range(len(req.goal_states)):
                ns = "waypoint_" + str(i)
                # diabolo_shell_marker.pose = resp.diabolo_states[i].pose
                # diabolo_fixator_marker.pose = resp.diabolo_states[i].pose
                # diabolo_axis_marker.pose = resp.diabolo_states[i].pose
                self.sphere_marker_g = self._make_marker_from_mesh(
                    "",
                    color=(240.0 / 255.0, 230.0 / 255.0, 50.0 / 255.0),
                    scale=[0.05, 0.05, 0.05],
                    namespace="closest_point_to_goal",
                )
                self.sphere_marker_g.type = visualization_msgs.msg.Marker.SPHERE
                self.sphere_marker_g.pose = resp.diabolo_states[i].pose
                self.sphere_marker_g.id = self.marker_count
                self.marker_count += 1
                marker_array.append(copy.deepcopy(self.sphere_marker_g))

                goal_diabolo_shell_marker.pose = req.goal_states[i].pose
                goal_diabolo_shell_marker.id = self.marker_count
                goal_diabolo_shell_marker.ns = "goal_states"
                self.marker_count += 1

                goal_diabolo_fixator_marker.pose = req.goal_states[i].pose
                goal_diabolo_fixator_marker.id = self.marker_count
                goal_diabolo_fixator_marker.ns = "goal_states"
                self.marker_count += 1

                goal_diabolo_axis_marker.pose = req.goal_states[i].pose
                goal_diabolo_axis_marker.id = self.marker_count
                goal_diabolo_axis_marker.ns = "goal_states"
                self.marker_count += 1

                marker_array.append(copy.deepcopy(goal_diabolo_shell_marker))
                marker_array.append(copy.deepcopy(goal_diabolo_fixator_marker))
                marker_array.append(copy.deepcopy(goal_diabolo_axis_marker))

                ## The goal state velocity
                # goal_vel_base = geometry_msgs.msg.Point()
                # goal_vel_tip = geometry_msgs.msg.Point()
                # goal_diabolo_vel_marker.type = visualization_msgs.msg.Marker.ARROW
                # goal_vel_base = req.goal_states[i].pose.position
                # goal_vel_tip.x = goal_vel_base.x + (req.goal_states[i].trans_velocity.x)/2.0
                # goal_vel_tip.y = goal_vel_base.y + (req.goal_states[i].trans_velocity.y)/2.0
                # goal_vel_tip.z = goal_vel_base.z + (req.goal_states[i].trans_velocity.z)/2.0
                # goal_diabolo_vel_marker.points.append(goal_vel_base)
                # goal_diabolo_vel_marker.points.append(goal_vel_tip)

                ## The distance between the goal state and the closest point
                # FIXME: This doesn't seem to point to the closest point.
                diabolo_to_goal_base = geometry_msgs.msg.Point()
                diabolo_to_goal_tip = geometry_msgs.msg.Point()
                diabolo_to_goal_marker.type = visualization_msgs.msg.Marker.ARROW
                diabolo_to_goal_base = req.goal_states[i].pose.position
                diabolo_to_goal_tip = resp.diabolo_states[i].pose.position
                diabolo_to_goal_marker.points.append(diabolo_to_goal_base)
                diabolo_to_goal_marker.points.append(diabolo_to_goal_tip)
                diabolo_to_goal_marker.id = self.marker_count
                diabolo_to_goal_marker.ns = "from_goal_to_closest_point"
                self.marker_count += 1
                marker_array.append(copy.deepcopy(diabolo_to_goal_marker))

                # predicted_diabolo_vel_marker = self._make_marker_from_mesh("", color=(0.,1.,0.), scale=[0.02, 0.02, 0.02], namespace=ns)
                # predicted_vel_base = geometry_msgs.msg.Point()
                # predicted_vel_tip = geometry_msgs.msg.Point()
                # predicted_diabolo_vel_marker.type = visualization_msgs.msg.Marker.ARROW
                # predicted_vel_base = resp.diabolo_states[i].pose.position
                # predicted_vel_tip.x = predicted_vel_base.x + (resp.diabolo_states[i].trans_velocity.x)/2.0
                # predicted_vel_tip.y = predicted_vel_base.y + (resp.diabolo_states[i].trans_velocity.y)/2.0
                # predicted_vel_tip.z = predicted_vel_base.z + (resp.diabolo_states[i].trans_velocity.z)/2.0
                # predicted_diabolo_vel_marker.points.append(predicted_vel_base)
                # predicted_diabolo_vel_marker.points.append(predicted_vel_tip)

                # marker_array.append(copy.deepcopy(predicted_diabolo_vel_marker))

            self.marker_array_pub.publish(marker_array)
            # time_of_flight = 2.0*(resp.diabolo_trans_vel.z)/9.81
            # rospy.logwarn("Returning trajectories")

            return (
                resp.a_bot_trajectory,
                resp.b_bot_trajectory,
                resp.left_stick_poses,
                resp.right_stick_poses,
            )

        else:
            # print("Trajectory not found. Aborting")
            return None, None, None, None


if __name__ == "__main__":

    try:
        c = PlayerClass()
        i = 1

        # print(c.motion_functions)
        c.force_add_motion_function_()
        prep_motions = ["None", "horizontal_impulse", "horizontal_impulse_short_left"]
        # prep_motion = prep_motions[2]
        prep_motion = ""

        while not rospy.is_shutdown():
            rospy.loginfo("Enter 1 to load motion data")
            rospy.loginfo(
                "Enter 2 to initialize the motion functions with hardcoded values."
            )
            rospy.loginfo("Enter 3 to initialize the robot positions.")
            rospy.loginfo("Enter d to spawn diabolo in simulation")
            rospy.loginfo("Enter sx to start playback at custom rate.")
            rospy.loginfo("Enter m to change the motion being executed")
            rospy.loginfo("Enter n to change the preparatory motion")
            rospy.loginfo("Enter ox to start oneshot motion")
            rospy.loginfo("Enter px to start continuous periodic motion")
            rospy.loginfo("Enter t to stop motion.")
            rospy.loginfo("Enter f to tilt the diabolo forward.")
            rospy.loginfo("Enter b to tilt the diabolo backward.")
            rospy.loginfo("Enter k to save the current knot points")
            rospy.loginfo("Enter x to exit.")

            i = raw_input()

            if i == "1":
                c.read_transformed_motion_data(
                    folder=("experiments/output/2020-09-14_motion_extraction/")
                )
            elif i == "2":
                c.initialize_motion_functions(use_saved_values=False)
            elif i == "3":
                c.initialize_robot_positions()
            elif i == "d" or i == "D":
                print("Default parameters are (0.13, 0.13, 0.07, .9999). Change? y/n")
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
                        parameters=(0.13, 0.13, 0.07, 0.9999)
                    )  # Set the diabolo plugin parameters and spawn the diabolo

            # One-shot / continuous motion execution call
            elif i == "ox" or i == "OX":
                print(
                    "This will execute the motion without asking for confirmation. \n Meant for execution in simulation \n Are you sure? y/n?"
                )
                e = raw_input()
                if e == "y":
                    # TODO: pass preparatory_motion?
                    c.run_oneshot_motion(interactive=True, confirm_execution=False)
                else:
                    print("Aborting")
            elif i == "px" or i == "PX":
                print(
                    "This will execute the motion without asking for confirmation. \n Meant for execution in simulation \n Are you sure? y/n?"
                )
                e = raw_input()
                if e == "y":
                    c.start_periodic_motion(
                        interactive=True,
                        confirm_execution=False,
                        preparatory_motion=prep_motion,
                    )
                else:
                    print("Aborting")
            elif i == "T" or i == "t":
                c.stop_periodic_motion()

            elif i == "f":
                # To tilt the diabolo forward, the right hand goes forward
                c.tilt_offset = 0.03
                c.changed_tilt_offset_flag = True
            elif i == "b":
                c.tilt_offset = -0.03
                c.changed_tilt_offset_flag = True

            ## Changing motion / prep. motion
            elif i == "m" or i == "M":
                print("The current motion is " + c.current_motion)
                print("Change? y/n")
                i = raw_input()
                if i == "y":
                    print("List of available functions is as follows: ")
                    print(
                        "Enter the appropriate index number to choose the motion to change to"
                    )
                    for i in range(len(c.motion_list)):
                        print(str(i) + ": " + str(c.motion_list[i]))
                    i = raw_input()
                    try:
                        c.current_motion = c.motion_list[int(i)]
                    except:
                        print("Incorrect index. Aborting")
                        raise
            elif i == "n" or i == "N":
                print("The current preparatory motion is " + prep_motion)
                print("Change? y/n")
                i = raw_input()
                if i == "y":
                    print("List of available motions: ")
                    print(
                        "Enter the appropriate index number to choose the motion to change to"
                    )
                    for i in range(len(prep_motions)):
                        print(str(i) + ": " + str(prep_motions[i]))
                    i = raw_input()
                    try:
                        prep_motion = prep_motions[int(i)]
                        if prep_motion == "None":
                            prep_motion = ""
                    except:
                        print("Incorrect index. Aborting")
                        raise

            elif i == "r":
                c.tilt_offset = 0.0
            elif i == "k" or i == "K":
                c.save_current_knot_points()

            elif i == "x":
                # c.stop_publish()
                break
            elif i == "":
                continue
    except rospy.ROSInterruptException:
        pass
