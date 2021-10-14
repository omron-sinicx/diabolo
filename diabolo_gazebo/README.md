## Package structure

This package primarily contains the following:

* Source code for a diabolo gazebo model plugin

This plugin controls the movement of the diabolo, like gyroscopic forces and angular velocity.  
Source code : _gazebo_diaboloplugin.cpp_

* URDF of the diabolo model

This uses stl data from the package _diabolo_scene_description_

### Steps to take: 

1. `roslaunch diabolo_gazebo diabolo_gazebo.launch`  
  This just starts a paused empty world for now
2. `rosrun diabolo_gazebo spawn_diabolo.py`  
  This spawns the diabolo in gazebo. The plugin should start up and subscribe to the required rostopics. 
  **Issue** The topics subscribed to are being prepended with the name of the rosnode. 

#### Notes  
  Have to rebuild package after making changes to plugin code _**and** have to restart Gazebo_


#### Helpful links

[Basic model plugin](http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin)  
[Setting joint and link velocities](http://gazebosim.org/tutorials?tut=set_velocity&cat=)  
[Gazebo apply force](https://answers.gazebosim.org//question/23389/ways-of-applying-force-to-a-body-in-gazebo/)
[Example gazebo force plugin source](http://docs.ros.org/jade/api/gazebo_plugins/html/gazebo__ros__force_8cpp_source.html)
[Example gazebo force plugin include](https://github.com/weiweikong/husky_simulation_jade/blob/master/gazebo_ros_pkgs/gazebo_plugins/include/gazebo_plugins/gazebo_ros_force.h)
[Ignition math intro](https://ignitionrobotics.org/api/math/6.4/cppgetstarted.html)
[Publish to ros eco from gazebo plugin](https://answers.gazebosim.org//question/12980/how-to-publish-data-to-ros-from-gazebo-plugin/)
[https://wet-robots.ghost.io/simple-method-for-distance-to-ellipse/]

##### Known Issues
* Removing gravity from a single link by setting the _gravity_ tag in diabolo.urdf to 0 does not work. Following message raised by gazebo:  
> multiple inconsistent <gravity> exists due to fixed joint reduction overwriting previous value [0] with [true]
  _Resolved_: Set all link gravities to 0 using the <turnGravityOff>true</turnGravityOff> tag in the urdf
