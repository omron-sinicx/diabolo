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

import rospy
from visualization_msgs.msg import Marker, MarkerArray

# This just publishes a VisualizationMarker for the bounding boxes of the IK seed interpolation

class RobotBB:  # Robot bounding box marker
    def __init__(self):
        rospy.init_node("show_robotBB", anonymous=True)  # Robot bounding boxes
        self.a_bot_BB_center = [0.6, -0.5, 1.3]
        self.b_bot_BB_center = [0.6, 0.5, 1.3]
        self.a_bot_bounds_marker_pub = rospy.Publisher(
            "/a_bot_bounds_marker", Marker, queue_size=1
        )
        self.b_bot_bounds_marker_pub = rospy.Publisher(
            "/b_bot_bounds_marker", Marker, queue_size=1
        )

        self.marker_x_scale = 0.35
        self.marker_y_scale = 0.9
        self.marker_z_scale = 1.0
        self.init_bounds_marker()
        # Also publish these bounds as rosparams

        rospy.set_param(
            "/a_bot_ik_bounds",
            [
                round((self.a_bot_BB_center[0] - self.marker_x_scale / 2.0), 3),
                round((self.a_bot_BB_center[0] + self.marker_x_scale / 2.0), 3),
                round((self.a_bot_BB_center[1] - self.marker_y_scale / 2.0), 3),
                round((self.a_bot_BB_center[1] + self.marker_y_scale / 2.0), 3),
                round((self.a_bot_BB_center[2] - self.marker_z_scale / 2.0), 3),
                round((self.a_bot_BB_center[2] + self.marker_z_scale / 2.0), 3),
            ],
        )
        rospy.set_param(
            "/b_bot_ik_bounds",
            [
                round((self.b_bot_BB_center[0] - self.marker_x_scale / 2.0), 3),
                round((self.b_bot_BB_center[0] + self.marker_x_scale / 2.0), 3),
                round((self.b_bot_BB_center[1] - self.marker_y_scale / 2.0), 3),
                round((self.b_bot_BB_center[1] + self.marker_y_scale / 2.0), 3),
                round((self.b_bot_BB_center[2] - self.marker_z_scale / 2.0), 3),
                round((self.b_bot_BB_center[2] + self.marker_z_scale / 2.0), 3),
            ],
        )

    def init_bounds_marker(self):
        # a_bot
        self.right_bounds_marker = Marker()
        self.right_bounds_marker.header.frame_id = "world"
        self.right_bounds_marker.header.stamp = rospy.Time.now()

        self.right_bounds_marker.ns = "a_bot_BB"  # Bounding box
        self.right_bounds_marker.id = 0

        self.right_bounds_marker.action = Marker.ADD

        self.right_bounds_marker.pose.orientation.x = 0.0
        self.right_bounds_marker.pose.orientation.y = 0.0
        self.right_bounds_marker.pose.orientation.z = 1.0
        self.right_bounds_marker.pose.orientation.w = 0.0

        self.right_bounds_marker.color.r = 1
        self.right_bounds_marker.color.g = 0
        self.right_bounds_marker.color.b = 0
        self.right_bounds_marker.color.a = 0.3

        self.right_bounds_marker.scale.x = self.marker_x_scale
        self.right_bounds_marker.scale.y = self.marker_y_scale
        self.right_bounds_marker.scale.z = self.marker_z_scale

        self.right_bounds_marker.lifetime = rospy.Duration()

        self.right_bounds_marker.type = Marker.CUBE

        # b_bot
        self.left_bounds_marker = Marker()
        self.left_bounds_marker.header.frame_id = "world"
        self.left_bounds_marker.header.stamp = rospy.Time.now()

        self.left_bounds_marker.ns = "b_bot_BB"  # Bounding box
        self.left_bounds_marker.id = 0

        self.left_bounds_marker.action = Marker.ADD

        self.left_bounds_marker.pose.orientation.x = 0.0
        self.left_bounds_marker.pose.orientation.y = 0.0
        self.left_bounds_marker.pose.orientation.z = 1.0
        self.left_bounds_marker.pose.orientation.w = 0.0

        self.left_bounds_marker.color.r = 0
        self.left_bounds_marker.color.g = 1
        self.left_bounds_marker.color.b = 0
        self.left_bounds_marker.color.a = 0.3

        self.left_bounds_marker.scale.x = self.marker_x_scale
        self.left_bounds_marker.scale.y = self.marker_y_scale
        self.left_bounds_marker.scale.z = self.marker_z_scale

        self.left_bounds_marker.lifetime = rospy.Duration()

        self.left_bounds_marker.type = Marker.CUBE

    def publish_bounds_marker(self):
        self.right_bounds_marker.pose.position.x = self.a_bot_BB_center[0]
        self.right_bounds_marker.pose.position.y = self.a_bot_BB_center[1]
        self.right_bounds_marker.pose.position.z = self.a_bot_BB_center[2]
        self.a_bot_bounds_marker_pub.publish(self.right_bounds_marker)

        self.left_bounds_marker.pose.position.x = self.b_bot_BB_center[0]
        self.left_bounds_marker.pose.position.y = self.b_bot_BB_center[1]
        self.left_bounds_marker.pose.position.z = self.b_bot_BB_center[2]
        self.b_bot_bounds_marker_pub.publish(self.left_bounds_marker)


if __name__ == "__main__":
    r = RobotBB()
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        r.publish_bounds_marker()
        rate.sleep()
