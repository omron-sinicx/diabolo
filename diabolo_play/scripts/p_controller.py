#!/usr/bin/env python

import time
import math
import numpy as np
import signal, sys

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray

from collections import deque

try:
    import tf
    import tf_conversions
    from tf_conversions import transformations as tftf
except:
    # For the notebooks
    import transformations as tftf
from diabolo_play.srv import SetInitialStickPositionsRequest, SetInitialStickPositions


class PController(object):
    def __init__(self):
        self.diabolo_state_history = deque(
            [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        )  # [[t, pitch_{t}, yaw_{t}], [t+1, pitch_{t+1}, yaw_{t+1}], [t+2, pitch_{t+2}, yaw_{t+2}]]

        self.init_stick_marker()
        self.right_stick_pos = Point()
        self.left_stick_pos = Point()

        self.init_diabolo_axis_marker()

        # hyper parameters for control
        self.pitch_p = 0.1

        self.initial_position_set = False

        # hyper parameters for geometry
        self.stick_center_pos = [0.45, 0, 1.1]  # [m]
        self.stick_width = 0.3
        self.stick_x_max_width = 0.2
        self.stick_z_max_width = 0.3
        self.stick_z_cycle_time = 30  # [s]

        # subscriber for diabolo orientation
        self.diabolo_orientation_sub = rospy.Subscriber(
            "/visualization_marker_array",
            MarkerArray,
            self.diabolo_orientation_cb,
            queue_size=1,
        )
        # TODO: Change this to a dedicated topic for the diabolo pose/state

        # Publisher for stick position
        self.left_stick_pub = rospy.Publisher(
            "/diabolo_stick_point_left", geometry_msgs.msg.Point, queue_size=1
        )
        self.right_stick_pub = rospy.Publisher(
            "/diabolo_stick_point_right", geometry_msgs.msg.Point, queue_size=1
        )
        self.stick_pose_pub = rospy.Publisher(
            "/diabolo_stick_poses", PoseArray, queue_size=1
        )
        # publisher for stick marker
        self.right_stick_marker_pub = rospy.Publisher(
            "/right_stick_marker", Marker, queue_size=1
        )
        self.left_stick_marker_pub = rospy.Publisher(
            "/left_stick_marker", Marker, queue_size=1
        )
        self.diabolo_axis_marker_pub = rospy.Publisher(
            "/diabolo_axis_marker", Marker, queue_size=1
        )
        self.set_initial_robot_position_proxy = rospy.ServiceProxy(
            "/initialize_robots_from_stick_positions", SetInitialStickPositions
        )

    def set_initial_robot_position(self):
        rospy.wait_for_service("/initialize_robots_from_stick_positions")
        try:
            req = SetInitialStickPositionsRequest()
            (
                self.left_stick_pos,
                self.right_stick_pos,
            ) = self.calculate_stick_pos_from_diabolo_pose(0)
            req.left_stick_position = self.left_stick_pos
            req.right_stick_position = self.right_stick_pos
            resp = self.set_initial_robot_position_proxy(req)
            return resp

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def diabolo_orientation_cb(self, msg):
        for marker in msg.markers:
            if marker.id == 0:
                ori_x = marker.pose.orientation.x
                ori_y = marker.pose.orientation.y
                ori_z = marker.pose.orientation.z
                ori_w = marker.pose.orientation.w

                theta = math.acos(ori_w) * 2

                axis_x = ori_x / math.sin(theta / 2.0)
                axis_y = ori_y / math.sin(theta / 2.0)
                axis_z = ori_z / math.sin(theta / 2.0)

                np_n = np.array([axis_x, axis_y, axis_z])
                np_r = np.array([1, 0, 0])
                np_r2_ = (
                    np.dot(np_r, np_n) * np_n
                    + (np_r - np.dot(np_r, np_n) * np_n) * math.cos(theta)
                    + np.cross(np_n, np_r) * math.sin(theta)
                )

                axis_r2_x = np_r2_[0]
                axis_r2_y = np_r2_[1]
                axis_r2_z = np_r2_[2]

                self.diabolo_axis_marker.points = []
                for i in range(-1, 2, 2):
                    p = Point()
                    p.x = axis_r2_x * 0.5 * i
                    p.y = axis_r2_y * 0.5 * i
                    p.z = axis_r2_z * 0.5 * i
                    self.diabolo_axis_marker.points.append(p)
                self.diabolo_axis_marker_pub.publish(self.diabolo_axis_marker)

                pitch = math.atan2(
                    axis_r2_z, math.sqrt(axis_r2_x ** 2 + axis_r2_y ** 2)
                )
                yaw = math.atan2(axis_r2_y, axis_r2_x)
                # print("pitch {}, yaw {}".format(pitch, yaw))

                self.diabolo_state_history.appendleft([time.time(), pitch, yaw])
                self.diabolo_state_history.pop()

    def calculate_stick_pos_from_diabolo_pose(self, time_since_start):

        # calculate stick x
        stick_x = self.p_control(self.diabolo_state_history[0])
        stick_x = max(min(self.stick_x_max_width, stick_x), -self.stick_x_max_width)

        # calculate stick z
        tmp_current_time = time_since_start % self.stick_z_cycle_time
        if tmp_current_time < self.stick_z_cycle_time / 2.0:
            stick_z = (
                -self.stick_z_max_width
                + tmp_current_time
                / (self.stick_z_cycle_time / 2.0)
                * 2
                * self.stick_z_max_width
            )
        if tmp_current_time < (self.stick_z_cycle_time / 2.0):
            stick_z = (
                -self.stick_z_max_width
                + (tmp_current_time / (self.stick_z_cycle_time / 2.0))
                * 2
                * self.stick_z_max_width
            )
        else:
            stick_z = (
                -self.stick_z_max_width
                + (2 - (tmp_current_time / (self.stick_z_cycle_time / 2.0)))
                * 2
                * self.stick_z_max_width
            )

        # get pitch and yaw
        pitch = self.diabolo_state_history[0][1]
        yaw = self.diabolo_state_history[0][2]

        # set stick pos
        theta = yaw

        theta_rot = np.mat(
            [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
        )
        tmp_right_stick_pos_2d = (
            theta_rot * np.mat([[-stick_x], [-self.stick_width / 2.0]])
        ).T.tolist()[0]
        tmp_left_stick_pos_2d = (
            theta_rot * np.mat([[stick_x], [self.stick_width / 2.0]])
        ).T.tolist()[0]

        tmp_right_stick_pos = (
            np.array(tmp_right_stick_pos_2d + [stick_z])
            + np.array(self.stick_center_pos)
        ).tolist()
        tmp_left_stick_pos = (
            np.array(tmp_left_stick_pos_2d + [-stick_z])
            + np.array(self.stick_center_pos)
        ).tolist()

        right_stick_pos = Point(
            x=tmp_right_stick_pos[0], y=tmp_right_stick_pos[1], z=tmp_right_stick_pos[2]
        )
        left_stick_pos = Point(
            x=tmp_left_stick_pos[0], y=tmp_left_stick_pos[1], z=tmp_left_stick_pos[2]
        )
        return left_stick_pos, right_stick_pos

    def p_control(self, s):
        pitch = s[1]
        return pitch * self.pitch_p

    def control(self):
        r = rospy.Rate(10)
        start_time = time.time()
        while True:
            time_since_start = time.time() - start_time
            (
                self.left_stick_pos,
                self.right_stick_pos,
            ) = self.calculate_stick_pos_from_diabolo_pose(time_since_start)

            # if not self.initial_position_set: #True if the robot is at the initial stick position
            #     self.set_initial_robot_position()
            #     self.initial_position_set = True

            # publish stick marker
            self.publish_stick_marker()
            self.right_stick_marker.pose.position = self.right_stick_pos
            self.left_stick_marker.pose.position = self.left_stick_pos

            # Publish stick position commands
            self.publish_stick_positions()

            r.sleep()

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

    def init_diabolo_axis_marker(self):
        self.diabolo_axis_marker = Marker()
        self.diabolo_axis_marker.header.frame_id = "world"
        self.diabolo_axis_marker.header.stamp = rospy.Time.now()

        self.diabolo_axis_marker.ns = "diabolo_axis"
        self.diabolo_axis_marker.id = 0

        self.diabolo_axis_marker.action = Marker.ADD

        self.diabolo_axis_marker.pose.orientation.x = 0.0
        self.diabolo_axis_marker.pose.orientation.y = 0.0
        self.diabolo_axis_marker.pose.orientation.z = 0.0
        self.diabolo_axis_marker.pose.orientation.w = 1.0

        self.diabolo_axis_marker.color.r = 1
        self.diabolo_axis_marker.color.g = 0
        self.diabolo_axis_marker.color.b = 0
        self.diabolo_axis_marker.color.a = 1.0

        self.diabolo_axis_marker.scale.x = 0.01
        self.diabolo_axis_marker.scale.y = 0.1
        self.diabolo_axis_marker.scale.z = 0.1

        self.diabolo_axis_marker.lifetime = rospy.Duration()

        self.diabolo_axis_marker.type = 5

    def publish_stick_marker(self):
        # right stick
        self.right_stick_marker.pose.position = self.right_stick_pos
        self.right_stick_marker_pub.publish(self.right_stick_marker)

        # left stick
        self.left_stick_marker.pose.position = self.left_stick_pos
        self.left_stick_marker_pub.publish(self.left_stick_marker)

    def publish_stick_positions(self):
        self.left_stick_pub.publish(self.left_stick_pos)
        self.right_stick_pub.publish(self.right_stick_pos)

        left_pose = Pose()
        right_pose = Pose()
        left_pose.position = self.left_stick_pos
        right_pose.position = self.right_stick_pos
        pose_array = PoseArray()
        pose_array.poses.append(left_pose)
        pose_array.poses.append(right_pose)
        self.stick_pose_pub.publish(pose_array)


if __name__ == "__main__":
    # sigint handler
    signal.signal(signal.SIGINT, lambda x, y: sys.exit(0))

    rospy.init_node("p_control")

    p_controller = PController()
    while not rospy.is_shutdown():
        rospy.loginfo("1: Go to initial stick pose")
        rospy.loginfo("2: Start controller")
        rospy.loginfo("x: Exit ")
        rospy.loginfo(" ")
        r = raw_input()
        if r == "1":
            p_controller.set_initial_robot_position()
        if r == "2":
            p_controller.control()
        if r == "x":
            break
