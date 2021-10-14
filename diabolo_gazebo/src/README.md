### Plugin description

The gazebo_diaboloplugin subscribes to a left_stick_postion and a right_stick_position topic
The positions of the sticks are stored internally

The position of the diabolo model is stored internally by using an internal model pointer call 

#### Determining the state of the diabolo (On string, Off string etc)

The _state_ of the diabolo is determined to be on the string if the absolute value of sum of its distance to each stick minus the toal length of the string is determined to be less than a Relaxation Error parameter (Currently defined as 5mm).
#### Publishing the ellipse frame to tf

Calculations are easier to do in the ellipsoid frame of reference. Esp calculating tangents and normals
So, the frame of the ellipsoid will be published to tf and calculations of diabolo velocity done in the ellipsoid frame

##### Conventions for the frame
* Origin lies on the center of the ellipsoid
* X axis points outward
* Y axis on the line between focii (on the major axis), pointing toward the left stick
* Z axis points upward 

Steps to publish ellipse frame to tf
1. Calculate the coordinates of the center of the ellipsoid using the midpoint formula on the latest positions of the sticks
2. Assume no rotation about the y axis. This is because only the focii (2 points) are known, and the y axis is assumed to be along the line between them.
3. Calculate axis of rotation taking cross product of (0, 1, 0) (i.e. ey_old) and normalzed vector from right stick position towards left stick position (ey_new). (This is the vector before rotation x vector after rotation)
4. Calculate the quaternion of the rotation using the axis and angle. ignition::math::Quaternion class has a public member function to do this
5. Publish the ellipse frame to tf 

#### Visualising the ellipse, diabolo, sticks and strings
This is done by creating messages of type ["visualization_msgs/Marker"](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/Marker.html) for each object to be visualized. The marker messages are added to an array of type ["visualization_msgs/MarkerArray"](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html) and published to a topic. Rviz subscribes to this topic through the "Marker Array" add-on and displays the required objects. Common usage and conventions are described [here](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes). 

**Note**: When specifying the source file for a mesh in a marker message, Rviz expects it to be in the form:

  package://(package name)/path/to/mesh/mesh.stl
**CURRENT ISSUE**
#### Constraining the position of the diabolo
The diabolo will drift away from the string if its position is not constrained to the string. If the diabolo is  determined to have left the string by phasing through it (i.e. If the velocity of the diabolo is outward from the current ellipse and the diabolo is not on the string though it was on it in the precvious time step), then its position is shifted to be on the ellipse. 

Two algorithms are being considered for this. Assuming that the diabolo needs position correction:
* Find the point lying on the ellipse along the line between the center of the ellipse and the diabolo frame (Using polar coordinates)
  1. Calculate the angles between the position vector of the diabolo and the axes of the ellipse frame
  2. Calculate the distance from the center of the ellipse (r) of a point lying on the ellipse, the position vector of which inscribes the same angles with the axes
  3. Calculate the cartesian coordinates of this point using the parametric form of the ellipse
  4. Set the diabolo to these coordinates

**Issues** :
1. The diabolo is constrained to the y=0 plane after crossing the lowest point in its trajectory if using this method (BUG)
  * If the angle of the position vector of the diabolo is calculated with respect to the x axis using their dot product (cos_theta), the diabolo moves constrained to the y=0 plane
  * If the angle of the position vector of the diabolo is calculated with respect to the y axis using their dot product (sin_theta), the diabolo moves constrained to the x=0 plane  
  _Resolved_: cos and sin were being calculated using cos^2 + sin^2 = 1. Taking the squsre root must have led to a loss of information, causing problems. Sin and cos are now calculated using definitions.

* Find the point closest to the diabolo that lies on the ellipse (Using iterative Newton-raphson approximation)

#### Constraining the direction of velocity of the diabolo

If the diabolo is on the string, its velocity must be in a direction along the tangent to the ellipse formed by the two stick ends at its resp. focii. 
The equation of this ellipse is found, and the normal at the diabolo's location is determined. 
* If the diabolo has a **positive** component in the normal direction (moving outward from the ellipse), the new velocity the calculated as the current velocity's projection along the tangent at the current location.   
* If the diabolo has a **negative** component in the normal direction (moving inward into the ellipse), the new velocity is kept the same as the current velocity

The normal of an ellipse at a given point is easily calculated if the ellipse axes are aligned to the reference axes. Calculations can be done as follows: 

* The following is done for each time step:  
1. Publish the center of the ellipse with the two sticks at its focii to tf as an independant coordinate frame 
2. Calculate the position of the diabolo in the ellipse center frame using tf (_from **world frame** to **ellipse frame**_)
3. Calculate the normal to the ellipse at the diabolo's location in the ellipse frame  
  _Start of calculations in ellipse center frame_  
4. Calculate the new location of the diabolo enforcing that it is _**constrained to the string**_
5. Calculate the new velocity of the diabolo in the allowed direction (_**Do not allow it to move outside the string**_)
6. Convert the location of the diabolo and the new velocity to world frame  
  _End of calculations in ellipse frame_
7. Apply the velocity

#### Constraining the magnitude of velocity of the diabolo

The diabolo cannot move faster then the sticks are pulling it. However, in simulation, due to the way the diabolo pull velocity is calculated, the instananeous velocity applied to the diabolo may be greater.  
The velocity is therefore _capped_ at a reasonable level
The magnitude of the cap is calculated based on the following:  
* **Linear velocity** added by the sticks to the diabolo (here, called _pull velocity_) is due to:  
  1. Change in location of the origin  
    This is simply the velocity of the origin. Calculate from successive positions of the origin.  
    If the origin has a certain velocity at time t, the diabolo velocity due to change in location of the ellipse cannot be greater than this
    * Steps:  
      1. Get the current position of the origin at the beginning of the time step. 
      2. Use the last position and the current position of the ellipse to calculate the velocity of the origin
      3. Find the projection of the origin velocity in the direction of the normal to the ellipse at time t at the corrected location of the diabolo
      4. Store this velocity

  2. Reduction in length of ellipse minor axis (Change in distance between sticks)  
    This is the velocity due to the minor axis being shortened. Calculate using the rate of change of the length of the minor axis.  
    * Steps:  
      1. Get the current length of the minor axis at the beginning of the time step. Use the relation between major axis length, minor axis length and distance between foci
      2. Use the last and current lengths of the minor axis to find its rate of change. This is the velocity of the poles of the ellipse
      3. Find the projection of this velocity on the ellipse normal at the location of the diabolo. (**Need confirmation**)
      4. Store this velocity

  The sum of these velocities is the cap of the pull velocity on the diabolo, provided they point inward. **Take either cap = 0 if their got product with the ellipse normal is negative**


#### Notes
* The **create** static public member function used by AdvertiseOptions and SubscribeOptions does **not** stand for the creation of a new topic. It only creates a new Advertise / Subscribe options object with the given options. See [this](http://docs.ros.org/kinetic/api/roscpp/html/structros_1_1AdvertiseOptions.html) for Advertise options reference and [this](http://docs.ros.org/kinetic/api/roscpp/html/structros_1_1SubscribeOptions.html) for subscribe options reference. 

* There are (at least) 2 ways to publish to a topic.
  1. Use **publish** as described [here](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers). _Standard way_
  2. Can use pubqueue to _push_ messages to a _pubqueue_ instead. The reference is [here](http://docs.ros.org/jade/api/gazebo_plugins/html/classPubQueue.html). An example of the usage is [here](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/347575b109e5f92b7cde406d3675909b10cb9741/gazebo_plugins/src/gazebo_ros_laser.cpp#L202). 

* Declaring a variable as _static_ in the include file prevents gazebo from loading the plugin. It throws the following error.  
[Err] [Plugin.hh:180] Failed to load plugin libdiabolo_plugin.so: /root/catkin_ws/devel/lib/libdiabolo_plugin.so: undefined symbol: _ZN6gazebo13DiaboloPlugin2brE

