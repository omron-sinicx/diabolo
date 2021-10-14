# NatNet 3 ROS driver
[![Build Status](https://travis-ci.org/mje-nz/natnet_ros.svg?branch=master)](https://travis-ci.org/mje-nz/natnet_ros)
[![codecov](https://codecov.io/gh/mje-nz/natnet_ros/branch/master/graph/badge.svg)](https://codecov.io/gh/mje-nz/natnet_ros)

This package contains a ROS driver for the NatNet protocol used by the NaturalPoint OptiTrack motion capture system.
It supports NatNet version 3.0 (the version used by Motive 2.0), which is a bit more efficient and adds more accurate timing.
The actual NatNet implementation is in [mje-nz/python_natnet](https://github.com/mje-nz/python_natnet), which is included as a submodule and repackaged.

Only regularly tested on Motive 2.0, although 2.1 is reported to work.
Support for older versions (i.e., older NatNet protocol versions) and skeletons is in progress.
Force plates, and other peripherals probably mostly work in the underlying library but are not tested and are not published as ROS topics.

Only supported on ROS Kinetic, but it works on Indigo and probably on newer distributions too.
The underlying library supports Python 2.7 and 3.4-3.6 on Linux, Windows and macOS.
Both this package and the library have CI for all supported platforms.

Features:

* Doesn't crash all the time, unlike mocap_optitrack did when I last tried it many years ago
* Synchronizes clocks to get timestamps right, unlike vrpn_client_ros
* All topics are timestamped with the camera mid-exposure time (give or take a few tenths of a millisecond)
* Publishes rigid bodies (including skeleton bones) as `geometry_msgs/PoseStamped`
* Publishes markers as `geometry_msgs/PointStamped`
* Publishes markers that aren't in a rigid body as `natnet_msgs/MarkerList`, because they change ID a lot
* Publishes all markers together as `visualization_msgs/Marker` (`SPHERE_LIST`) for Rviz


## TODO

* **Merge to_test branch**
* Add duplicate marker ID check and report bug
* Document "solver replaces occlusion" behaviour and implement workaround
* Rename MocapFrame to FrameOfData
* Make clock sync optional
* Release python_natnet on PyPI
* Add CONTRIBUTING etc
* Make sure unknown message IDs, failure to decode messages etc doesn't result in a crash
* Make sure to use the same variable signedness as the SDK
* Use rospy.spin
* Add warning when unnecessary data is being streamed
* Add guidance for appropriate Motive settings


## Installation

From Kinetic (any metapackage) or Indigo (any metapackage plus pip):

```
mkdir -p catkin_ws/src/
cd catkin_ws/src
git clone --recursive https://github.com/mje-nz/natnet_ros.git
cd ..
rosdep install -y --from-paths src --ignore-src
catkin_make
source devel/setup.bash
```

Then, if you have a ROS Master running, you can test it with a fake server:

```
rosrun natnet_ros client _fake:=true
```

or run against your real Motive instance:

```
rosrun natnet_ros client _server:=(Motive IP)
```


## Motive settings

**TODO**

Only multicast mode is supported.
For old versions of Motive (< 2.0), only the default address (239.255.42.99) and port (1511) are supported.


## ROS API

The `client` node connects to a NatNet server and publishes the data as ROS topics.
The data and descriptions are used directly, so make to give your rigid bodies unique streaming IDs and sensible names, and set the axis convention to z-up.
If you need Motive to use y-up for some reason, you can fix it with:

```
rosrun tf static_transform_publisher 0 0 0 0 0 1.57079632679 mocap_z_up mocap 100
```


### Published topics

* `~rigid_bodies/<name>/pose` (geometry_msgs/PoseStamped)

  Rigid body pose.
  The name is taken from the NatNet stream (i.e., from Motive), with a bit of an attempt to make sure it's a valid ROS name.

* `~rigid_bodies/<name>/marker<id>` (geometry_msgs/PointStamped)

  Position of each marker of each rigid body.
  The ID is taken from the NatNet stream (i.e., from Motive).

* `~rigid_bodies/<name>/markers` (natnet_msgs/MarkerList)

  Position and ID of each marker of each marker of each rigid body as a list.

* `~skeletons/<skeleton_name>/<bone_name>/pose` (geometry_msgs/PoseStamped)

  Skeleton bone pose.
  The names are taken from the NatNet stream (i.e., from Motive), with a bit of an attempt to make sure they're valid ROS names.

* `~markers/leftovers` (natnet_msgs/MarkerList)

  Position and ID of any markers that aren't in a rigid body.

* `~markers/vis` (visualization_msgs/Marker)

  Position of all markers, for visualization with Rviz.
  The size is set to the average estimated marker size.


### Parameters

* `~server` (`string`, optional)

  NatNet server to connect to; will autodetect if not provided.

* `~fake` (`bool`, default: false)

  Use fake data instead of connecting to a real server.

* `~fake_v2` (`bool`, default: false)

  Use fake NatNet 2.10 data instead of connecting to a real server.

* `~debug` (`bool`, default: false)

  Enable debug logging.

* `~rate` (`int`, default: 100)

  If `fake` or `fake_v2` is true, the rate at which to publish the fake data (in Hz).

* `~mocap_frame` (`string`, default: 'mocap')

  The name of the `tf` frame for the mocap data.

* `~tf_remap` (`dict`, optional)

  A dictionary mapping rigid body names to the name of the `tf` frame they should be published as.
  For example, `{'UAV': 'base_link'}` would cause the pose for the rigid body named `UAV` to be published as a transform from `mocap` to `base_link`.

  Dict params are a bit tricky to use in Kinetic or earlier: a literal works with `rosrun`, but using a dict literal as a parameter value in a launch file doesn't work until Lunar.
  Using a `rosparam` tag works though.

