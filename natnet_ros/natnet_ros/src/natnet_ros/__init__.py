# coding: utf-8
"""NatNet client ROS node.

Copyright (c) 2017, Matthew Edwards.  This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/mje-nz/natnet_ros/blob/master/LICENSE.
No part of natnet_ros, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file."""

from __future__ import print_function

import itertools
import os.path

import rospy
import rospkg
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, TransformStamped, Vector3
from std_msgs.msg import ColorRGBA, Header
import tf
from visualization_msgs.msg import Marker

import natnet
from natnet_msgs.msg import MarkerList


class RosLogger(natnet.Logger):

    # Until Lunar rospy.log* are instance methods and from Lunar on they're
    # functions, so in order to work in both cases we have to wrap them

    def debug(self, *args, **kwargs):
        rospy.logdebug(*args, **kwargs)

    def info(self, *args, **kwargs):
        rospy.loginfo(*args, **kwargs)

    def warning(self, *args, **kwargs):
        rospy.logwarn(*args, **kwargs)

    def error(self, *args, **kwargs):
        rospy.logerr(*args, **kwargs)

    def fatal(self, *args, **kwargs):
        rospy.logfatal(*args, **kwargs)


def validate_name(name):
    """Remove illegal characters from a name to make it a valid ROS name.

    Args:
        name (str):
    """
    # TODO: Be more thorough
    return name.replace(' ', '_').replace(':', '_').replace('-', '_').replace('/', '_')


class NatnetClientNode(object):

    def __init__(self):
        self.log = RosLogger()

        # {rigid body id: rigid body pose publisher}
        self.rigid_body_pubs = {}  # type: dict[int, rospy.Publisher]

        # {rigid body id: rigid body name}
        self.rigid_body_names = {}  # type: dict[int, str]

        # {skeleton id: skeleton name}
        self.skeleton_names = {}  # type: dict[int, str]

        # {(skeleton id, bone id): bone name}
        self.bone_names = {}  # type: dict[tuple[int, int], str]

        # {(skeleton id, bone id): bone pose publisher}
        self.bone_pubs = {}  # type: dict[tuple[int, int], rospy.Publisher]

        # {(rigid body id, marker id): rigid body marker position publisher}
        self.marker_pubs = {}  # type: dict[tuple[int, int], rospy.Publisher]

        # {rigid body id: rigid body marker position list publisher}
        self.markerlist_pubs = {}  # type: dict[int, rospy.Publisher]

        self.leftover_markers_pub = rospy.Publisher('~markers/leftovers', MarkerList, queue_size=10)
        self.marker_vis_pub = rospy.Publisher('~markers/vis', Marker, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.client = None  # type: natnet.comms.Client
        self.mocap_frame = rospy.get_param('~mocap_frame', 'mocap')
        self.tf_remap = rospy.get_param('~tf_remap', {})  # type: dict

    def model_definitions_callback(self, rigid_bodies, skeletons, markersets):
        # TODO: Make sure it's ok to just throw publishers away like this
        self.rigid_body_pubs = {}
        self.rigid_body_names = {}
        self.skeleton_names = {}
        self.bone_names = {}
        self.marker_pubs = {}
        self.markerlist_pubs = {}

        for b in rigid_bodies:
            name = validate_name(b.name)
            self.rigid_body_names[b.id_] = name

            if b.id_ in self.rigid_body_pubs:
                # TODO: Handle this more gracefully
                raise RuntimeError('Two rigid bodies with the same streaming ID')

            pub = rospy.Publisher('~rigid_bodies/{}/pose'.format(name), PoseStamped, queue_size=10)
            self.rigid_body_pubs[b.id_] = pub

            pub = rospy.Publisher('~rigid_bodies/{}/markers'.format(name), MarkerList, queue_size=10)
            self.markerlist_pubs[b.id_] = pub

            for marker_id in range(1, len(b.marker_positions) + 1):
                topic = '~rigid_bodies/{}/marker{}'.format(name, marker_id)
                pub = rospy.Publisher(topic, PointStamped, queue_size=10)
                self.marker_pubs[(b.id_, marker_id)] = pub

        for s in skeletons:
            name = validate_name(s.name)
            if s.id_ in self.skeleton_names:
                raise RuntimeError('Two skeletons with the same streaming ID')
            self.skeleton_names[s.id_] = name

            print('Skeleton names:', self.skeleton_names)

            # TODO: How to publish skeleton?

            for bone in s.rigid_bodies:
                if (s.id_, bone.id_) in self.bone_pubs:
                    raise RuntimeError('Two bones with the same streaming ID')
                bone_name = validate_name(bone.name)
                self.bone_names[(s.id_, bone.id_)] = bone_name
                topic = '~skeletons/{}/{}/pose'.format(name, bone_name)
                print('Will publish skeleton {} bone {} ({}) on {} '.format(s.id_, bone.id_, bone_name, topic))
                pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
                self.bone_pubs[(s.id_, bone.id_)] = pub

    def _publish_rigid_body_poses(self, header, rigid_bodies):
        """Publish ~rigid_bodies/name/pose topics."""
        for b in rigid_bodies:
            message = PoseStamped()
            message.header = header
            message.pose.position = Point(*b.position)
            message.pose.orientation = Quaternion(*b.orientation)
            try:
                self.rigid_body_pubs[b.id_].publish(message)
            except KeyError:
                print('Unknown rigid body {}'.format(b.id_))

    def _publish_rigid_body_tfs(self, header, rigid_bodies):
        """Publish mocap->name transform for each rigid body."""
        for b in rigid_bodies:
            message = TransformStamped()
            message.header = header
            try:
                child_frame = self.rigid_body_names[b.id_]
                if child_frame in self.tf_remap:
                    child_frame = self.tf_remap[child_frame]
                message.child_frame_id = child_frame
                message.transform.translation = Vector3(*b.position)
                message.transform.rotation = Quaternion(*b.orientation)
                # rospy.loginfo("published tf")
                self.tf_broadcaster.sendTransformMessage(message)
            except KeyError:
                print('Unknown rigid body {}'.format(b.id_))

    def _publish_bone_poses(self, header, bones):
        """Publish ~skeletons/name/bone/pose topics."""
        for b in bones:
            message = PoseStamped()
            message.header = header
            message.pose.position = Point(*b.position)
            message.pose.orientation = Quaternion(*b.orientation)
            try:
                bone_id = b.id_ & 0xFFFF
                skeleton_id = b.id_ >> 16
                self.bone_pubs[(skeleton_id, bone_id)].publish(message)
            except KeyError:
                print('Unknown rigid body ({}, {})'.format(skeleton_id, bone_id))

    def _publish_marker_vis(self, header, markers):
        """Publish ~markers/vis topic."""
        message = Marker()
        message.header = header
        message.ns = 'natnet_ros'
        message.id = 0
        message.type = Marker.SPHERE_LIST
        positions = [Point(*m.position) for m in markers]
        message.points = positions
        sizes = [m.size for m in markers]
        mean_size = sum(sizes) / len(sizes)
        message.scale = Vector3(mean_size, mean_size, mean_size)
        message.color = ColorRGBA(1, 1, 1, 1)
        self.marker_vis_pub.publish(message)

    def _publish_marker_points(self, header, markers):
        """Publish a PointStamped topic for each marker.

        Markers which are part of a rigid body are published as ~rigid_bodies/name/markeri. The rest are published in
        a natnet_msgs/MarkerList as ~markers/leftovers.
        """
        leftovers_msg = MarkerList()
        leftovers_msg.header = header

        for rigid_body_id, rigid_body_markers in itertools.groupby(markers, lambda m: m.model_id):
            if rigid_body_id != 0 and rigid_body_id in self.markerlist_pubs.keys():
                markers_msg = MarkerList()
                markers_msg.header = header
                for m in rigid_body_markers:
                    try:
                        message = PointStamped()
                        message.header = header
                        message.point = Point(*m.position)
                        marker_pub = self.marker_pubs[(rigid_body_id, m.marker_id)]
                        marker_pub.publish(message)

                        markers_msg.ids.append(m.marker_id)
                        markers_msg.positions.append(Point(*m.position))
                    except KeyError:
                        print('Unknown marker {} from rigid body {}'.format(m.marker_id, rigid_body_id))
                markerlist_pub = self.markerlist_pubs[rigid_body_id]
                markerlist_pub.publish(markers_msg)
            else:
                # TODO: Test
                for m in rigid_body_markers:
                    leftovers_msg.ids.append(m.marker_id)
                    leftovers_msg.positions.append(Point(*m.position))
        self.leftover_markers_pub.publish(leftovers_msg)

    def mocap_frame_callback(self, rigid_bodies, skeletons, markers, timing):
        """Handle a frame of mocap data."""
        self.log.debug('{:.1f}s: Received mocap frame'.format(timing.timestamp))

        header = Header()
        header.frame_id = self.mocap_frame
        # header.stamp = rospy.Time(timing.timestamp)
        header.stamp = rospy.Time.now()

        if rigid_bodies:
            self._publish_rigid_body_poses(header, rigid_bodies)
            self._publish_rigid_body_tfs(header, rigid_bodies)
        if skeletons:
            for skeleton in skeletons:
                # This doesn't seem to ever be called in V3
                # TODO: Was this called in V2?
                self._publish_bone_poses(header, skeleton.rigid_bodies)
        if markers:
            self._publish_marker_points(header, markers)
            self._publish_marker_vis(header, markers)

    def fake_connect(self, use_v2=False):
        rospack = rospkg.RosPack()
        path = rospack.get_path('python_natnet')
        test_data_path = os.path.join(path, 'test_data')
        if not os.path.exists(test_data_path):
            # Awkwardly work around the folder layout being different in the devel space
            test_data_path = os.path.join(path, 'python_natnet', 'test_data')
            assert os.path.exists(test_data_path)
        rate = rospy.get_param('~rate', 100)
        try:
            rate = float(rate)
        except ValueError:
            self.log.warning('Invalid rate %s, using no rate limit', rate)
            rate = None
        self.log.info('Using fake client' + (' (with NatNet v2)' if use_v2 else ''))
        fake_connect = natnet.fakes.SingleFrameFakeClient.fake_connect
        if use_v2:
            fake_connect = natnet.fakes.SingleFrameFakeClient.fake_connect_v2
        return fake_connect(rate, self.log, test_data_path)

    def run(self):
        if rospy.get_param('~fake', False):
            client = self.fake_connect()
        elif rospy.get_param('~fake_v2', False):
            client = self.fake_connect(use_v2=True)
        else:
            server = rospy.get_param('~server', None)
            client = natnet.Client.connect(server, logger=self.log)
        client.set_model_callback(self.model_definitions_callback)
        client.set_callback(self.mocap_frame_callback)
        client.spin()
