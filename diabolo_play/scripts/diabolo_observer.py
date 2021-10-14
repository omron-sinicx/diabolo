#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from diabolo_gazebo.msg import DiaboloState
from geometry_msgs.msg import PoseStamped, Point
from diabolo_play.srv import GetDiaboloState, GetDiaboloStateResponse
import copy
import tf
import tf.transformations as transform
import numpy as np


class DiaboloObserver:
    """
    This node offers services for the player node to query the diabolo state at a specific time.
    It will return the state that has the closest time stamp to the one in the request from the states available.
    The node keeps 2 lists, one of simulated and one of real diabolo states.

    This node keeps track of the state of the diabolo spawned in gazebo and also the state of the real diabolo with MoCap input.
    The simulated diabolo's state (both pose and velocity) is obtained from the topic published to by the diabolo gazebo plugin.
    The real diabolo's pose is obtained from the topic where the diabolo pose is published (obtained from MoCap).
    The real diabolo's velocity must be calculated from previous poses.
    """

    def __init__(self):
        # Services to advertise
        self.plugin_diabolo_state_service = rospy.Service(
            "/get_plugin_diabolo_state", GetDiaboloState, self.send_plugin_diabolo_state
        )
        self.real_diabolo_state_service = rospy.Service(
            "/get_real_diabolo_state", GetDiaboloState, self.send_real_diabolo_state
        )
        self.diabolo_state_service = rospy.Service(
            "/get_observed_diabolo_state", GetDiaboloState, self.send_diabolo_state
        )
        # Check if gazebo is running, and set up a timer to query it for diabolo poses if it is
        self.plugin_diabolo_state_sub = rospy.Subscriber(
            "/diabolo_gazebo_node/diabolo_current_state",
            DiaboloState,
            self.plugin_diabolo_state_callback,
        )
        self.real_diabolo_state_sub = rospy.Subscriber(
            "/mocap/rigid_bodies/diabolo_blue_2020_10/pose",
            PoseStamped,
            self.real_diabolo_state_callback,
        )
        self.diabolo_euler_pub = rospy.Publisher("/diabolo_rpy", Point, queue_size=100)
        # self.real_diabolo_state_sub = rospy.Subscriber('/mocap/diabolo_current_state', PoseStamped, self.real_diabolo_state_callback)

        # 100 states will be stored for each the simulated diabolo and the real diabolo
        self.listener = tf.TransformListener()
        self.states_to_save = 100
        self.plugin_diabolo_states = []
        self.real_diabolo_states = []
        # A vector containing the euler angles determining the orientation of the diabolo for all available states
        self.real_diabolo_euler_angles = []

    def plugin_diabolo_state_callback(self, msg):
        while len(self.plugin_diabolo_states) >= self.states_to_save:
            # Remove the first state
            del self.plugin_diabolo_states[0]

        self.plugin_diabolo_states.append(copy.deepcopy(msg))

    def real_diabolo_state_callback(self, msg):
        while len(self.real_diabolo_states) >= self.states_to_save:
            # Remove the first state
            del self.real_diabolo_states[0]
            del self.real_diabolo_euler_angles[0]
        try:
            pose = copy.deepcopy(msg)
            self.listener.waitForTransform(
                "world", pose.header.frame_id, rospy.Time(0), rospy.Duration(0.1)
            )
            pose = self.listener.transformPose("world", pose)

            ds = DiaboloState()
            ds.header = copy.deepcopy(pose.header)
            ds.pose = copy.deepcopy(pose.pose)

            dp = ds.pose.orientation
            diabolo_quat = np.array((dp.x, dp.y, dp.z, dp.w))
            angles = transform.euler_from_quaternion(diabolo_quat)
            de = Point()
            de.x = angles[0]
            de.y = angles[1]
            de.z = angles[2]
            self.real_diabolo_euler_angles.append(de)
            self.diabolo_euler_pub.publish(de)
            self.real_diabolo_states.append(copy.deepcopy(ds))

        except:
            print("Transform not available")

        if not len(self.real_diabolo_states) == 0:
            time_step = ds.header.stamp - self.real_diabolo_states[-1].header.stamp
            ts = time_step.to_sec()
            if ts < 0.000001:
                ts = (
                    1.0 / 120.0
                )  # Fill with nominal frame rate of the mocap setup (120 fps) to avoid division by 0
            velocity = Point()

            velocity.x = (
                ds.pose.position.x - self.real_diabolo_states[-1].pose.position.x
            ) / ts
            velocity.y = (
                ds.pose.position.y - self.real_diabolo_states[-1].pose.position.y
            ) / ts
            velocity.z = (
                ds.pose.position.z - self.real_diabolo_states[-1].pose.position.z
            ) / ts
            ds.trans_velocity = copy.deepcopy(velocity)

    def send_real_diabolo_state(self, req):
        # Do not know if this is necessary.
        resp = GetDiaboloStateResponse()
        available_diabolo_states = copy.deepcopy(self.real_diabolo_states)
        if len(available_diabolo_states) == 0:
            resp.success = False
            return resp
        for i in range(len(available_diabolo_states)):
            if i == len(available_diabolo_states) - 1:
                # If no closer state found so far, return the last pose in the array
                resp.state = copy.deepcopy(available_diabolo_states[-1])
                resp.success = True
                return resp

            elif abs(req.header.stamp - available_diabolo_states[i].header.stamp) < abs(
                req.header.stamp - available_diabolo_states[i + 1].header.stamp
            ):
                # If the difference in time between this header and the requested header is less than that
                # of the next header and the requested header, this is the best time available
                resp.state = copy.deepcopy(available_diabolo_states[i])
                resp.success = True
                return resp

        resp.state = copy.deepcopy(available_diabolo_states[-1])
        return resp

    def send_plugin_diabolo_state(self, req):
        # Do not know if this is necessary.
        resp = GetDiaboloStateResponse()
        available_diabolo_states = copy.deepcopy(self.plugin_diabolo_states)
        if len(available_diabolo_states) == 0:
            resp.success = False
            return resp
        for i in range(len(available_diabolo_states)):
            if i == len(available_diabolo_states) - 1:
                # If no closer state found so far, return the last pose in the array
                resp.state = copy.deepcopy(available_diabolo_states[-1])
                resp.success = True
                return resp

            if abs(req.header.stamp - available_diabolo_states[i].header.stamp) < abs(
                req.header.stamp - available_diabolo_states[i + 1].header.stamp
            ):
                # If the difference in time between this header and the requested header is less than that
                # of the next header and the requested header, this is the best time available
                resp.state = copy.deepcopy(available_diabolo_states[i])
                resp.success = True
                return resp

        resp.state = copy.deepcopy(available_diabolo_states[-1])
        return resp

    def send_diabolo_state(self, req):
        # This service will send back the available diabolo state, either observed or mocap
        # This assumes that only one of either mocap data OR gazebo data is available
        # Will send mocap data if both are available
        resp = GetDiaboloStateResponse()
        available_diabolo_states = []
        if not len(self.real_diabolo_states) == 0:
            available_diabolo_states = copy.deepcopy(self.real_diabolo_states)
            angles = copy.deepcopy(self.real_diabolo_euler_angles)
            resp.type = GetDiaboloStateResponse.REAL
            avg_angles = Point()
            for a in angles:
                avg_angles.x += a.x
                avg_angles.y += a.y
                avg_angles.z += a.z

            avg_angles.x /= float(len(angles))
            avg_angles.y /= float(len(angles))
            avg_angles.z /= float(len(angles))
            resp.filtered_angles = avg_angles
        elif not len(self.plugin_diabolo_states) == 0:
            available_diabolo_states = copy.deepcopy(self.plugin_diabolo_states)
            resp.type = GetDiaboloStateResponse.SIM

        else:
            resp.success = False
            return resp

        # Do not know if this is necessary.
        for i in range(len(available_diabolo_states)):
            if i == len(available_diabolo_states) - 1:
                # If no closer state found so far, return the last pose in the array
                resp.state = copy.deepcopy(available_diabolo_states[-1])
                resp.success = True
                return resp

            if abs(req.header.stamp - available_diabolo_states[i].header.stamp) < abs(
                req.header.stamp - available_diabolo_states[i + 1].header.stamp
            ):
                # If the difference in time between this header and the requested header is less than that
                # of the next header and the requested header, this is the best time available
                resp.state = copy.deepcopy(available_diabolo_states[i])
                resp.success = True
                return resp

        resp.state = copy.deepcopy(available_diabolo_states[-1])
        return resp


if __name__ == "__main__":

    rospy.init_node("diabolo_observer", anonymous=True)

    obs = DiaboloObserver()

    rospy.spin()
