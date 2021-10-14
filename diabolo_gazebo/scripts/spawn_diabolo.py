#!/usr/bin/python

import rospy, tf
import random
from gazebo_msgs.srv import DeleteModel, SpawnModel, SpawnModelRequest
from geometry_msgs.msg import *
import rospkg
import os
import sys

rp = rospkg.RosPack()
pack_dir = rp.get_path("diabolo_gazebo")
urdf_file = os.path.join(pack_dir, "urdf", "diabolo.urdf")

# Part orientations to put them in the right position for insertion


def spawn_diabolo(position):
    # Create service proxy
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
    rospy.wait_for_service("/gazebo/spawn_urdf_model")

    # Load URDF
    with open(urdf_file, "r") as f:
        model_xml = f.read()

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # Spawn model
        req = SpawnModelRequest()
        req.model_name = "diabolo"
        req.initial_pose = pose
        req.model_xml = model_xml
        req.robot_namespace = "/"
        req.reference_frame = "world"
        spawn_model(req)
        rospy.sleep(0.2)


if __name__ == "__main__":
    position = (0.2, 0.3, 1.5)
    if not len(sys.argv) == 4:
        print("Position parameters not provided. Using defaults")

    else:
        position = (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    print("Spawning diabolo at " + str(position))
    spawn_diabolo(position)
