### This folder contains data to be used for ik interpolation with the o2ac/diabolo setup
#### The files were obtained with the following procedure:

### OLD METHOD (Only 2 end coordinates)  
#### This method was difficult to extend to multiple knot points. See below for current method
1. Run roslaunch diabolo_gazebo diabolo-gazebo.launch 
2. Run roslaunch diabolo_moveit_config diabolo_moveit_planning_execution.launch sim:=true 
  This also spawns the node to show the interpolation grid. **The center point and cube size must be set explicitly in the script**
3. Run rosrun diabolo_play stick_target_to_joint_target_converter 
  This provides the service to initialize robot positions by setting bio_ik goals   
    * Goals used : Position goal and parallel wrist_2 goal
4. The file IK_trilinear_interpolation_edge_calculator.xlsx was created to calculate the required edge positions based on grid center positions and grid side length. The joint angles obtained for each vertex are stored in the same file
  **NOTE:** The interpolation function expects the values to be in the following order:
  C000, C001, C010, C011, C100, C101, C110, C111
  Here, CXYZ is the value of a particular joint at (X,Y,Z) coordinate. 
  X=0 means the nearer coordinate, X=1 means the further coordinate
  Y=0 means the coordinate to the right, and Y=1 means the coordinate to the left
  Z=0 means the lower coordinate, and Z=1 means the higher coordinate

    Please see the interpolation function _getInterpolatedJointAngles_ helper_functions.h file  for more details.

  5. Finally, the joint angles in the correct order were stored in the file joint_angle_edge_values_in_order.xlsx in the diabolo_moveit_config/config_data folder, with the top 8 rows for a_bot and the remaining for b_bot  
    The rows of data correspond to joint angles in the order C000, C001, C010, C011, C100, C101, C110, C111 from top to bottom, and the columns correspond to joints, in the order **shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3**
  6. **VERY IMPORTANT**: The file used currently is called *joint_angle_edge_values_in_order_with_coords.csv* 
    The format for this file is similar to joint_angle_edge_values_in_order.xlsx, except 2 extra rows are added.   
    Row 0 from the top and row 9 from the top are the **coordinates of the corner vertices for interpolation** for a_bot (row 0) and b_bot (row 9) resp. 
    The order of the coordinates is: x_near, x_far, y_right, y_left, z_low, z_high  
    **The csv reading function in helper_functions.h has been changed to reflect this**

### NEW METHOD   
#### Allows setting multiple knot points per coordinate 

Currently, the y coordinate has 3 knot points, the z coordinate has 3, and the x coordinate has 2

In the current approach, the values of joint angles and the cartesian coordinates at which those joint values acan be expected are kept in seperate files. The file used for the joint angle values is called:
  __joints\_angle\_edge\_values.csv__  

  Format: Each robot has 2 `$\times$`3`$\times$`3 = 18 sets of joint angles. 
  The joint angles are arranged in the order of C000, C001, C002, C010, C011, C012 .. and so on from top to bottom for a_bot and then b_bot with no line in between.
  From left to right, the angles are for the joints from shoulder_pan joint to wrist_3 joint in order  

The file used for the cartesian coordinates is called:    
  __edge\_coordinates.csv__  

  From left to right, the file lists coordinates for a_bot and b_bot as:  
  a_bot_x, a_bot_y, a_bot_z, b_bot_x ..   
  The columns are the smallest edge coordinate to the largest edge coordinate value for each coordinate

