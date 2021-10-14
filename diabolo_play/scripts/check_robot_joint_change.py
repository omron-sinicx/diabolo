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
# Author: Devwrat Joshi

# This program subscribes to the /a_bot_controller/command and /b_bot_controller/command topics
# and outputs the change in joint angles per time step, to compare which joints change more per time step.

import rospy
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import TwistStamped


class CheckJointCommands:
    def __init__(self):
        rospy.init_node("check_joint_commands", anonymous=True)

        self.a_bot_initial_joints = [
            3.14159,
            -1.91986,
            -3.14159 / 2.0,
            -1.22173,
            3.14159,
            0.0,
        ]
        self.b_bot_initial_joints = [
            3.14159,
            -1.22173,
            3.14159 / 2.0,
            -1.91986,
            3.14159,
            0.0,
        ]
        self.pub_a_bot_angles = rospy.Publisher(
            "/a_bot_joint_angles", TwistStamped, queue_size=1
        )
        self.pub_a_bot_change = rospy.Publisher(
            "/a_bot_joint_angles_change", TwistStamped, queue_size=1
        )
        self.pub_b_bot_angles = rospy.Publisher(
            "/b_bot_joint_angles", TwistStamped, queue_size=1
        )
        self.pub_b_bot_change = rospy.Publisher(
            "/b_bot_joint_angles_change", TwistStamped, queue_size=1
        )
        self.sub_a_bot_command = rospy.Subscriber(
            "/a_bot_controller/command", JointTrajectory, self.a_bot_callback
        )
        self.sub_b_bot_command = rospy.Subscriber(
            "/b_bot_controller/command", JointTrajectory, self.b_bot_callback
        )

    def a_bot_callback(self, msg):
        position = TwistStamped()
        change = TwistStamped()

        position.twist.angular.x = msg.points[0].positions[0]  # Shoulder pan
        position.twist.angular.y = msg.points[0].positions[1]  # Shoulder lift
        position.twist.angular.z = msg.points[0].positions[2]  # Elbow
        position.twist.linear.x = msg.points[0].positions[3]  # wrist 1
        position.twist.linear.y = msg.points[0].positions[4]  # wrist 2
        position.twist.linear.z = msg.points[0].positions[5]  # wrist 3

        change.twist.angular.x = abs(
            msg.points[0].positions[0] - self.a_bot_initial_joints[0]
        )  # Shoulder pan
        change.twist.angular.y = abs(
            msg.points[0].positions[1] - self.a_bot_initial_joints[1]
        )  # Shoulder lift
        change.twist.angular.z = abs(
            msg.points[0].positions[2] - self.a_bot_initial_joints[2]
        )  # Elbow
        change.twist.linear.x = abs(
            msg.points[0].positions[3] - self.a_bot_initial_joints[3]
        )  # wrist 1
        change.twist.linear.y = abs(
            msg.points[0].positions[4] - self.a_bot_initial_joints[4]
        )  # wrist 2
        change.twist.linear.z = abs(
            msg.points[0].positions[5] - self.a_bot_initial_joints[5]
        )  # wrist 3

        self.a_bot_initial_joints = [
            msg.points[0].positions[i] for i in range(len(msg.points[0].positions))
        ]

        position.header.stamp = rospy.Time.now()
        change.header.stamp = rospy.Time.now()
        self.pub_a_bot_angles.publish(position)
        self.pub_a_bot_change.publish(change)

    def b_bot_callback(self, msg):
        position = TwistStamped()
        change = TwistStamped()

        position.twist.angular.x = msg.points[0].positions[0]  # Shoulder pan
        position.twist.angular.y = msg.points[0].positions[1]  # Shoulder lift
        position.twist.angular.z = msg.points[0].positions[2]  # Elbow
        position.twist.linear.x = msg.points[0].positions[3]  # wrist 1
        position.twist.linear.y = msg.points[0].positions[4]  # wrist 2
        position.twist.linear.z = msg.points[0].positions[5]  # wrist 3

        change.twist.angular.x = abs(
            msg.points[0].positions[0] - self.b_bot_initial_joints[0]
        )  # Shoulder pan
        change.twist.angular.y = abs(
            msg.points[0].positions[1] - self.b_bot_initial_joints[1]
        )  # Shoulder lift
        change.twist.angular.z = abs(
            msg.points[0].positions[2] - self.b_bot_initial_joints[2]
        )  # Elbow
        change.twist.linear.x = abs(
            msg.points[0].positions[3] - self.b_bot_initial_joints[3]
        )  # wrist 1
        change.twist.linear.y = abs(
            msg.points[0].positions[4] - self.b_bot_initial_joints[4]
        )  # wrist 2
        change.twist.linear.z = abs(
            msg.points[0].positions[5] - self.b_bot_initial_joints[5]
        )  # wrist 3

        self.b_bot_initial_joints = [
            msg.points[0].positions[i] for i in range(len(msg.points[0].positions))
        ]

        position.header.stamp = rospy.Time.now()
        change.header.stamp = rospy.Time.now()

        self.pub_b_bot_angles.publish(position)
        self.pub_b_bot_change.publish(change)


if __name__ == "__main__":
    c = CheckJointCommands()

    while not rospy.is_shutdown():
        rospy.spin()
