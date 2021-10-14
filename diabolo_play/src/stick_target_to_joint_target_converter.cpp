#include "diabolo_play/stick_target_to_joint_target_converter.h"

/* Program to convert desired end effector  positions to robot joint velocities */
// Determine target stick pose (here: get it from topic)
// Publish joint trajectory to:
// /b_bot/pos_traj_controller/command
#define DEFAULT_POSITION_GOAL_WEIGHT 5.0     // The weight to assign the postion goal when setting from ik
#define DEFAULT_ORIENTATION_GOAL_WEIGHT 1.0  // The weight to assign the orientation goal when setting from ik
#define DEFAULT_MIN_JOINT_DISP_GOAL_WEIGHT                                                                             \
  1.0  // The weight to assign the minimum joint displacement goal when setting from ik
#define DEFAULT_POSITION_GOAL_TOLERANCE 0.0  // The default tolerance for the position goal when setting from ik
StickTargetToJointTargetConverter::StickTargetToJointTargetConverter()
  : a_bot_group_("a_bot")
  , b_bot_group_("b_bot")
  ,robot_state_(robot_model_loader::RobotModelLoader("robot_description").getModel())
  , planning_scene_(robot_model_loader::RobotModelLoader("robot_description").getModel(),
                    collision_detection::WorldPtr(new collision_detection::World()))
  , interpolation_flag(true)
{
  // Set up MoveGroups
  a_bot_group_.setEndEffectorLink("a_bot_diabolo_stick_tip");
  b_bot_group_.setEndEffectorLink("b_bot_diabolo_stick_tip");

  store_joint_limits();

  get_planning_scene_client = n_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  ros::Duration(1.0).sleep();

  robot_state_ = *a_bot_group_.getCurrentState();

  updatePlanningScene();
  timer = n_.createTimer(ros::Duration(2.0), &StickTargetToJointTargetConverter::checkStickPosePubRate, this);
  // Topics to publish
  pubMarkerArray_ = n_.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
  pubRobotState_ = n_.advertise<moveit_msgs::DisplayRobotState>("diabolo_robot_state", 10);
  pubPositionEval_ = n_.advertise<std_msgs::Float32>("/bio_ik_debug/position_eval", 1);
  pubOrientEval_ = n_.advertise<std_msgs::Float32>("/bio_ik_debug/orientation_eval", 1);
  pubJointFuncEval_ = n_.advertise<std_msgs::Float32>("/bio_ik_debug/joint_func_eval", 1);

  pubCommandA_ = n_.advertise<trajectory_msgs::JointTrajectory>("/a_bot/scaled_pos_joint_traj_controller/command", 1);
  pubCommandB_ = n_.advertise<trajectory_msgs::JointTrajectory>("/b_bot/scaled_pos_joint_traj_controller/command", 1);
  a_bot_display_seed_traj_pub = n_.advertise<moveit_msgs::DisplayTrajectory>("/a_bot_display_seed_trajectory", 1);
  store_corner_orientations();

  if (interpolation_flag)
  {
    /// Not ideal, but adding one publisher for every interpolate data state
    interpolate_state_markers_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/ik_interpolate_states", 10);
    seed_state_markers_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/interpolated_seed_state", 10);
    store_interpolation_data();
    publish_interpolation_data_markers();
  }

  publishMarkers();

  // Get ik goal tuning factors from parameter server
  if (!n_.getParam("/diabolo_stick_position_goal_weight", this->position_goal_weight))
  {
    ROS_WARN_STREAM("Stick position goal weight not set. Using default value");
    this->position_goal_weight = DEFAULT_POSITION_GOAL_WEIGHT;
  }
  ROS_INFO_STREAM("Stick position goal weight set to  " << this->position_goal_weight);

  if (!n_.getParam("/diabolo_stick_orientation_goal_weight", this->orientation_goal_weight))
  {
    ROS_WARN_STREAM("Stick orientation goal weight not set. Using default value");
    this->orientation_goal_weight = DEFAULT_ORIENTATION_GOAL_WEIGHT;
  }
  ROS_INFO_STREAM("Stick orientation goal weight set to  " << this->orientation_goal_weight);

  if (!n_.getParam("/min_joint_disp_goal_weight", this->min_joint_disp_goal_weight))
  {
    ROS_WARN_STREAM("Min joint displacement goal weight not set. Using default value");
    this->min_joint_disp_goal_weight = DEFAULT_MIN_JOINT_DISP_GOAL_WEIGHT;
  }

  if (!n_.getParam("/position_goal_tolerance", this->position_goal_tolerance))
  {
    ROS_WARN_STREAM("Position goal tolerance not set. Using default value");
    this->position_goal_tolerance = DEFAULT_POSITION_GOAL_TOLERANCE;
  }
  ROS_INFO_STREAM("Position goal tolerance  " << this->position_goal_tolerance);

  // Topics to subscribe to
  subStickPos_ = n_.subscribe("/diabolo_stick_poses", 1, &StickTargetToJointTargetConverter::setRobotPoseCallback, this);
  jointStateSub_ = n_.subscribe("/joint_states", 1, &StickTargetToJointTargetConverter::storeJointStates, this);
  set_to_initial_service = n_.advertiseService("/initialize_robots_from_stick_positions",
                                               &StickTargetToJointTargetConverter::setInitialPosition, this);
  command_robot_traj_from_stick_traj_service =
      n_.advertiseService("/command_robot_traj_from_stick_traj", &StickTargetToJointTargetConverter::sendRobotTrajectory, this);
  ROS_INFO("Diabolo playback node has started up!");
}

void StickTargetToJointTargetConverter::store_interpolation_data()
{
  std::vector<std::vector<double>> joint_angles;
  std::vector<std::vector<double>> knot_coordinates;
  std::string pack = ros::package::getPath("diabolo_moveit_config");
  std::string joint_angles_path = pack + "/config_data/joint_angle_edge_values.csv";
  std::string knot_points_path = pack + "/config_data/edge_coordinates.csv";
  readCsvto2DVector(36, 6, joint_angles_path, joint_angles);
  readCsvto2DVector(3, 6, knot_points_path, knot_coordinates);
  // Store interpolation_data at corner_coords
  ROS_WARN("Clearing interpolation data vectors");
  this->a_bot_interpolation_data.clear();
  this->b_bot_interpolation_data.clear();
  this->a_bot_interpolation_data.resize(18);
  this->b_bot_interpolation_data.resize(18);

  for (int i = 0; i < 18; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      this->a_bot_interpolation_data[i].push_back(joint_angles[i][j]);
      this->b_bot_interpolation_data[i].push_back(joint_angles[i + 18][j]);
    }
  }
  // Store corner coords
  this->a_bot_interpolation_corner_coords.clear();
  this->b_bot_interpolation_corner_coords.clear();
  std::vector<double> a_bot_x_knots = { knot_coordinates[0][0], knot_coordinates[1][0] };
  std::vector<double> b_bot_x_knots = { knot_coordinates[0][3], knot_coordinates[1][3] };
  this->a_bot_interpolation_corner_coords.push_back(a_bot_x_knots);
  this->b_bot_interpolation_corner_coords.push_back(b_bot_x_knots);
  std::vector<double> a_bot_y_knots = { knot_coordinates[0][1], knot_coordinates[1][1], knot_coordinates[2][1] };
  std::vector<double> b_bot_y_knots = { knot_coordinates[0][4], knot_coordinates[1][4], knot_coordinates[2][4] };
  this->a_bot_interpolation_corner_coords.push_back(a_bot_y_knots);
  this->b_bot_interpolation_corner_coords.push_back(b_bot_y_knots);
  std::vector<double> a_bot_z_knots = { knot_coordinates[0][2], knot_coordinates[1][2], knot_coordinates[2][2] };
  std::vector<double> b_bot_z_knots = { knot_coordinates[0][5], knot_coordinates[1][5], knot_coordinates[2][5] };
  this->a_bot_interpolation_corner_coords.push_back(a_bot_z_knots);
  this->b_bot_interpolation_corner_coords.push_back(b_bot_z_knots);

  for (std::vector<double> v : this->a_bot_interpolation_data)
  {
    std::cout << v.size() << " ";

  }
}

void StickTargetToJointTargetConverter::create_new_interpolation_data(
    std::vector<double> a_bot_x_coords, std::vector<double> a_bot_y_coords, std::vector<double> a_bot_z_coords,
    std::vector<double> b_bot_x_coords, std::vector<double> b_bot_y_coords, std::vector<double> b_bot_z_coords)
{
  // First create a vector of vectors  with all the corner positons to go to
  std::vector<std::vector<double>> new_a_bot_interpolation_data;
  std::vector<std::vector<double>> new_b_bot_interpolation_data;

  moveit::core::RobotState robot_state = robot_state_;

  const robot_state::JointModelGroup* a_joint_model_group = robot_state.getJointModelGroup("a_bot");
  const robot_state::JointModelGroup* b_joint_model_group = robot_state.getJointModelGroup("b_bot");

  for (int i = 0; i < a_bot_x_coords.size(); i++)
  {
    for (int j = 0; j < a_bot_y_coords.size(); j++)
    {
      for (int k = 0; k < a_bot_z_coords.size(); k++)
      {
        // For each coordinate, call set from ik to get the joint angles
        std::vector<double> a_bot_interp_values, b_bot_interp_values;
        geometry_msgs::Point a, b;
        a.x = a_bot_x_coords[i];
        a.y = a_bot_y_coords[j];
        a.z = a_bot_z_coords[k];

        b.x = b_bot_x_coords[i];
        b.y = b_bot_y_coords[j];
        b.z = b_bot_z_coords[k];

        geometry_msgs::Pose a_pose = getPoseFromPoint(a, "right_arm");
        geometry_msgs::Pose b_pose = getPoseFromPoint(b, "left_arm");
        bool ok = false;
        int counter = 0;
        while (!ok && counter < 10)
        {
          ok = setRobotStateIK(robot_state, a_joint_model_group, a_pose, "a_bot");
          counter += 1;
        }

        if (!ok)
        {
          ROS_ERROR_STREAM("Could not get interpolation position at (" << a.x << ", " << a.y << ", " << a.z
                                                                       << ") for a_bot"
                                                                       << "\n");
          new_a_bot_interpolation_data.push_back({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        }
        else
        {
          robot_state.copyJointGroupPositions("a_bot", a_bot_interp_values);
          new_a_bot_interpolation_data.push_back({ a_bot_interp_values[0], a_bot_interp_values[1],
                                                   a_bot_interp_values[2], a_bot_interp_values[3],
                                                   a_bot_interp_values[4], a_bot_interp_values[5] });
        }

        ok = false;
        counter = 0;
        while (!ok && counter < 10)
        {
          ok = setRobotStateIK(robot_state, b_joint_model_group, b_pose, "b_bot");
          counter += 1;
        }

        if (!ok)
        {
          ROS_ERROR_STREAM("Could not get interpolation position at (" << b.x << ", " << b.y << ", " << b.z
                                                                       << ") for b_bot"
                                                                       << "\n");
          new_b_bot_interpolation_data.push_back({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        }
        else
        {
          robot_state.copyJointGroupPositions("b_bot", b_bot_interp_values);

          new_b_bot_interpolation_data.push_back({ b_bot_interp_values[0], b_bot_interp_values[1],
                                                   b_bot_interp_values[2], b_bot_interp_values[3],
                                                   b_bot_interp_values[4], b_bot_interp_values[5] });
        }
      }
    }
  }

  this->a_bot_interpolation_data.clear();
  this->b_bot_interpolation_data.clear();
  this->a_bot_interpolation_corner_coords.clear();
  this->b_bot_interpolation_corner_coords.clear();

  this->a_bot_interpolation_corner_coords.push_back(a_bot_x_coords);
  this->a_bot_interpolation_corner_coords.push_back(a_bot_y_coords);
  this->a_bot_interpolation_corner_coords.push_back(a_bot_z_coords);

  this->b_bot_interpolation_corner_coords.push_back(b_bot_x_coords);
  this->b_bot_interpolation_corner_coords.push_back(b_bot_y_coords);
  this->b_bot_interpolation_corner_coords.push_back(b_bot_z_coords);

  this->a_bot_interpolation_data = new_a_bot_interpolation_data;
  this->b_bot_interpolation_data = new_b_bot_interpolation_data;
}

void StickTargetToJointTargetConverter::publish_interpolated_seed_state(moveit::core::RobotState robot_state,
                                                               std::string robot_name)
{
  std::vector<std::string> all_link_names;
  std::vector<std::string> robot_link_names;
  visualization_msgs::MarkerArray seed_state_markers;
  all_link_names = robot_state.getRobotModel()->getLinkModelNames();
  for (int i = 0; i < all_link_names.size(); i++)
  {
    if ((all_link_names[i].substr(0, 5) == "a_bot" && robot_name == "a_bot") ||
        (all_link_names[i].substr(0, 5) == "b_bot" && robot_name == "b_bot"))
    {
      if (i > 2)  // Unneeded links before this position
      {
        robot_link_names.push_back(all_link_names[i]);
      }
    }
  }

  robot_state.getRobotMarkers(seed_state_markers, robot_link_names,
                              true);  // This function pushes back markers to the end of the passed marker array
  int id = 0;
  for (int k = 0; k < seed_state_markers.markers.size(); k++, id++)
  {
    if (robot_name == "a_bot")
      seed_state_markers.markers[k].ns = "a_bot_seed_state";  // Change namespace to indicate the robot
    else if (robot_name == "b_bot")
      seed_state_markers.markers[k].ns = "b_bot_seed_state";  // Change namespace to indicate the robot
    seed_state_markers.markers[k].id = id;
    seed_state_markers.markers[k].color.r = 0.8;
    seed_state_markers.markers[k].color.g = 0.8;
    seed_state_markers.markers[k].color.b = 0.8;
    seed_state_markers.markers[k].color.a = 0.5;
  }
  seed_state_markers_pub_.publish(seed_state_markers);

}

void StickTargetToJointTargetConverter::publish_interpolation_data_markers()
{
  moveit::core::RobotState robot_state = robot_state_;
  // This is to access the cached(?) robot link colors
  std::vector<std::string> C{ "C000", "C001", "C002", "C010", "C011", "C012", "C020", "C021", "C022",
                              "C100", "C101", "C102", "C110", "C111", "C112", "C120", "C121", "C122" };
  std::string state_topic_base = "/diabolo_interpolation_state_";
  visualization_msgs::MarkerArray comp_state_markers;  // Compiled joint states for all corner positions
  std::vector<std::string> all_link_names;
  std::vector<std::string> robot_link_names;
  all_link_names = robot_state.getRobotModel()->getLinkModelNames();
  // Crude, but only include robot links in marker array if first letter of link name is 'a' or 'b'
  for (int i = 0; i < all_link_names.size(); i++)
  {
    if (all_link_names[i][0] == 'a' || all_link_names[i][0] == 'b')
    {
      robot_link_names.push_back(all_link_names[i]);
    }
  }
  for (int i = 0; i < a_bot_interpolation_data.size(); i++)  // Cycle through positions used for interpolation
  {
    std::vector<double> a_bot_joint_angles, b_bot_joint_angles;
    for (int j = 0; j < 6; j++)  // For each variable in the joint group
    {
      a_bot_joint_angles.push_back(this->a_bot_interpolation_data[i][j]);
      b_bot_joint_angles.push_back(this->b_bot_interpolation_data[i][j]);
    }

    robot_state.setJointGroupPositions("a_bot",
                                       a_bot_joint_angles);  // Set the joint group positions for a_bot in robot_state
    robot_state.setJointGroupPositions("b_bot",
                                       b_bot_joint_angles);  // Set the joint group positions for b_bot in robot_state
    unsigned int id = comp_state_markers.markers.size();
    unsigned int prev_size = comp_state_markers.markers.size();
    robot_state.getRobotMarkers(comp_state_markers, robot_link_names,
                                true);  // This function pushes back markers to the end of the passed marker array

    for (int k = prev_size; k < comp_state_markers.markers.size(); k++, ++id)
    {
      comp_state_markers.markers[k].ns = std::to_string(i);  // Change namespace to indicate corner position
      comp_state_markers.markers[k].id = id;
      comp_state_markers.markers[k].color.b = 0.8;
      comp_state_markers.markers[k].color.a = 0.5;
    }
  }

  interpolate_state_markers_pub_.publish(comp_state_markers);
}
void StickTargetToJointTargetConverter::store_joint_limits()  // Storing the values of joint limits
{
  this->a_bot_joint_limits_[0][0] = M_PI / 2.0;
  this->a_bot_joint_limits_[0][1] = 3 * M_PI / 2.0;
  this->a_bot_joint_limits_[1][0] = -M_PI;
  this->a_bot_joint_limits_[1][1] = -M_PI / 4.0;
  this->a_bot_joint_limits_[2][0] = -M_PI;
  this->a_bot_joint_limits_[2][1] = 0.0;
  this->a_bot_joint_limits_[3][0] = -M_PI;
  this->a_bot_joint_limits_[3][1] = 0.0;
  this->a_bot_joint_limits_[4][0] = M_PI / 2.0;
  this->a_bot_joint_limits_[4][1] = 3 * M_PI / 2;
  this->a_bot_joint_limits_[5][0] = -0.1;
  this->a_bot_joint_limits_[5][1] = 0.1;

  this->b_bot_joint_limits_[0][0] = M_PI / 2.0;
  this->b_bot_joint_limits_[0][1] = 3 * M_PI / 2.0;
  this->b_bot_joint_limits_[1][0] = -M_PI;
  this->b_bot_joint_limits_[1][1] = 0.;
  this->b_bot_joint_limits_[2][0] = 0.0;
  this->b_bot_joint_limits_[2][1] = M_PI;
  this->b_bot_joint_limits_[3][0] = -M_PI;
  this->b_bot_joint_limits_[3][1] = 0.;
  this->b_bot_joint_limits_[4][0] = M_PI / 2.0;
  this->b_bot_joint_limits_[4][1] = 3.0 * M_PI / 2.0;
  this->b_bot_joint_limits_[5][0] = -0.1;
  this->b_bot_joint_limits_[5][1] = 0.1;
}

void StickTargetToJointTargetConverter::store_corner_orientations()
{
  // Read these from parameter server
  // Hardcoding for now
  tf2::Quaternion tf_rotate_90_plus, tf_rotate_90_minus;
  tf_rotate_90_plus.setRPY(3.14 / 2.0, 0, 0);
  tf_rotate_90_minus.setRPY(-3.14 / 2.0, 0, 0);
  corner_orientations[0] = tf2::Quaternion(0.233, 0.228, -0.672, 0.665) * tf_rotate_90_minus;
  corner_orientations[1] = tf2::Quaternion(0.233, 0.228, -0.672, 0.665) * tf_rotate_90_minus;
  corner_orientations[2] = tf2::Quaternion(-0.107, -0.109, -0.703, 0.695) * tf_rotate_90_minus;
  corner_orientations[3] = tf2::Quaternion(-0.107, -0.109, -0.703, 0.695) * tf_rotate_90_minus;
  corner_orientations[4] = tf2::Quaternion(-0.202, 0.195, 0.682, 0.676) * tf_rotate_90_plus;
  corner_orientations[5] = tf2::Quaternion(-0.202, 0.195, 0.682, 0.676) * tf_rotate_90_plus;
  corner_orientations[6] = tf2::Quaternion(0.231, -0.228, 0.666, 0.671) * tf_rotate_90_plus;
  corner_orientations[7] = tf2::Quaternion(0.231, -0.228, 0.666, 0.671) * tf_rotate_90_plus;

  double x_close = 0.4;
  double x_far = 0.8;
  double y_left = 0.6;
  double y_right = -0.6;
  double z_up = 1.41;
  double z_down = 0.8;

  corner_coords[0] = x_close;
  corner_coords[1] = x_far;
  corner_coords[2] = y_left;
  corner_coords[3] = y_right;
  corner_coords[4] = z_down;
  corner_coords[5] = z_up;

  marker_count = 0;
}
bool StickTargetToJointTargetConverter::sendRobotTrajectory(diabolo_play::CreateRobotTrajectory::Request& request,
                                                   diabolo_play::CreateRobotTrajectory::Response& response)
{
  // request has a series of left and right stick poses
  // Convert each of those poses to a single joint trajectory msg for either robot arm
  // Time between each joint state in the trajectory should be the reciprocal of the player node publish rate
  // This rate is stored as this->stick_pose_pub_rate
  // IMPORTANT: This function assumes both pose arrays are of equal length
  int steps = request.left_stick_poses.poses.size();
  ROS_INFO_STREAM(steps << " trajectory steps to execute");
  // NOTE: It is possible that the IK will not be resolved for certain stick positions
  // In such an event, the function will not add a point to the robot trajectory
  // In effect, if n number of points starting with the k th point do not resolve to a joint state and the k+n+1 th
  // point does, then the robot controller will get a trajectory in which the time to get from the kth point to the
  // k+n+1th point is time_between_steps*n

  const robot_state::JointModelGroup* a_bot_joint_model_group;
  const robot_state::JointModelGroup* b_bot_joint_model_group;
  a_bot_joint_model_group = robot_state_.getJointModelGroup("a_bot");
  b_bot_joint_model_group = robot_state_.getJointModelGroup("b_bot");
  std::vector<double> a_bot_joint_state, b_bot_joint_state;
  std::vector<std::string> a_bot_joint_names = a_bot_joint_model_group->getJointModelNames();
  std::vector<std::string> b_bot_joint_names = b_bot_joint_model_group->getJointModelNames();

  trajectory_msgs::JointTrajectory a_bot_traj, b_bot_traj;
  this->a_bot_seed_traj.points.clear();
  this->a_bot_seed_traj.joint_names.clear();
  a_bot_traj.header.frame_id = "world";
  b_bot_traj.header.frame_id = "world";
  a_bot_traj.joint_names.resize(6);
  b_bot_traj.joint_names.resize(6);
  this->a_bot_seed_traj.joint_names.resize(6);
  for (int i = 0; i < 6; i++)
  {
    a_bot_traj.joint_names[i] = a_bot_joint_names[i];
    this->a_bot_seed_traj.joint_names[i] = a_bot_joint_names[i];
    b_bot_traj.joint_names[i] = b_bot_joint_names[i];
  }
  moveit_msgs::DisplayTrajectory seed_display_traj;
  moveit_msgs::RobotTrajectory robot_traj;

  double time_between_steps = 1.0 / this->stick_pose_publish_rate;
  for (int i = 0; i < steps; i++)
  {
    trajectory_msgs::JointTrajectoryPoint a_bot_traj_point;
    trajectory_msgs::JointTrajectoryPoint b_bot_traj_point;
    // Get joint state for this stick position for either arm
    // Store initial joint state
    robot_state_.copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_state);
    robot_state_.copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_state);

    // TODO(felixvd): Add "attempts" parameter to this function to avoid IK failures, or switch to analytical IK
    bool a_bot_IK_valid = setRobotStateIK(robot_state_, a_bot_joint_model_group, request.right_stick_poses.poses[i],
                                          "a_bot");  // Set the new joint positions from IK
    bool b_bot_IK_valid = setRobotStateIK(robot_state_, b_bot_joint_model_group, request.left_stick_poses.poses[i],
                                          "b_bot");  // Set the new joint positions from IK
    if (a_bot_IK_valid)
    {
      robot_state_.copyJointGroupPositions(a_bot_joint_model_group, a_bot_joint_state);

      a_bot_traj_point.positions.resize(6);
      for (int j = 0; j < 6; j++)
      {
        a_bot_traj_point.positions[j] = a_bot_joint_state[j];
      }
      a_bot_traj_point.time_from_start = ros::Duration(time_between_steps * (i + 1));  // Add delay if required

      a_bot_traj.points.push_back(a_bot_traj_point);
      this->a_bot_seed_traj.points[this->a_bot_seed_traj.points.size() - 1].time_from_start =
          ros::Duration(time_between_steps * (i + 1));
    }
    else
    {
      this->a_bot_seed_traj.points.pop_back();
    }

    if (b_bot_IK_valid)
    {
      robot_state_.copyJointGroupPositions(b_bot_joint_model_group, b_bot_joint_state);
      b_bot_traj_point.positions.resize(6);
      for (int j = 0; j < 6; j++)
      {
        b_bot_traj_point.positions[j] = b_bot_joint_state[j];
      }
      b_bot_traj_point.time_from_start = ros::Duration(time_between_steps * (i + 1));  // Add delay if required
      b_bot_traj.points.push_back(b_bot_traj_point);
    }
    publishRobotState(robot_state_);
  }
  ROS_WARN_STREAM(a_bot_traj.points.size() << " points in a_bot trajectory resolved before smoothening");
  ROS_WARN_STREAM(a_bot_traj.points[a_bot_traj.points.size() - 1].time_from_start.toSec() << " is the time period of "
                                                                                             "a_bot's trajectory");

  if (request.smooth_robot_trajectory)
  {
    ROS_WARN("Getting smooth trajectory");
    // If the IK fails, gaps remain in the trajectory. To fix them, we
    // interpolate linearly. A cubic spline fitting method is in the git history.

    // This is the number of steps in the trajectory
    // floor-ed, so add one last point in trajectory if required
    trajectory_msgs::JointTrajectoryPoint a_bot_last_point, b_bot_last_point;
    a_bot_last_point = a_bot_traj.points[a_bot_traj.points.size() - 1];
    b_bot_last_point = b_bot_traj.points[b_bot_traj.points.size() - 1];
    double time_period = a_bot_last_point.time_from_start.toSec();
    int trajectory_steps = time_period / request.stick_update_time_step;
    if ((trajectory_steps > a_bot_traj.points.size()) || (trajectory_steps > b_bot_traj.points.size()))
    {
      ROS_WARN_STREAM("trajectories are not filled up! Each trajectory should have = " << trajectory_steps << " steps");
      ROS_WARN_STREAM("a_bot trajectory has        = " << a_bot_traj.points.size() << " steps");
      ROS_WARN_STREAM("b_bot trajectory has        = " << b_bot_traj.points.size() << " steps");

      ROS_WARN_STREAM(a_bot_traj.points[a_bot_traj.points.size() - 1].time_from_start.toSec() << " is the time period "
                                                                                                 "of a_bot's "
                                                                                                 "trajectory");
      // If any points are missing, fill them in with linear interpolation (Calculating with splines takes too long)
      // Loop for a_bot
      for (int i = 0; i < a_bot_traj.points.size(); i++)
      {
        if (a_bot_traj.points[i].positions.size() < 6)
          ROS_ERROR_STREAM("Point " << i << " has " << a_bot_traj.points[i].positions.size()
                                    << " points before fixing traj");
      }

      for (int i = 0; i < b_bot_traj.points.size(); i++)
      {
        if (b_bot_traj.points[i].positions.size() < 6)
          ROS_ERROR_STREAM("Point" << i << " has " << b_bot_traj.points[i].positions.size()
                                   << " points before fixing traj");
      }

      fill_trajectory(a_bot_traj, ros::Duration(request.stick_update_time_step), trajectory_steps);

      for (int i = 0; i < a_bot_traj.points.size(); i++)
      {
        if (a_bot_traj.points[i].positions.size() < 6)
          ROS_ERROR_STREAM("a_bot Point " << i << " has " << a_bot_traj.points[i].positions.size() << " points");
      }
      ROS_INFO_STREAM("a_bot trajectory has        = " << a_bot_traj.points.size() << " steps after filling points.");

      fill_trajectory(b_bot_traj, ros::Duration(request.stick_update_time_step), trajectory_steps);

      for (int i = 0; i < b_bot_traj.points.size(); i++)
      {
        if (b_bot_traj.points[i].positions.size() < 6)
          ROS_ERROR_STREAM("b_bot Point" << i << " has " << b_bot_traj.points[i].positions.size() << " points");
      }

      ROS_INFO_STREAM("b_bot trajectory has        = " << b_bot_traj.points.size() << " steps after filling points.");

      const robot_model::RobotModel& rmodel = a_bot_joint_model_group->getParentModel();

      // Solve the fk for the new trajectory points to get the stick positions for each trajectory point
      moveit::core::RobotState robot_state = robot_state_;
      ROS_WARN_STREAM("a_bot trajectory has        = " << a_bot_traj.points.size() << " right before the loop.");
      ROS_WARN_STREAM("b_bot trajectory has        = " << b_bot_traj.points.size() << " right before the loop.");
      geometry_msgs::PoseArray left_poses, right_poses;

      right_poses.poses.resize(a_bot_traj.points.size());
      for (int i = 0; i < a_bot_traj.points.size(); i++)
      {
        geometry_msgs::Pose right_pose;
        robot_state.setJointGroupPositions("a_bot", a_bot_traj.points[i].positions);
        const Eigen::Affine3d& right_stick_state = robot_state.getGlobalLinkTransform("a_bot_diabolo_stick_tip");

        right_pose.position.x = right_stick_state.translation().x();
        right_pose.position.y = right_stick_state.translation().y();
        right_pose.position.z = right_stick_state.translation().z();

        right_poses.poses[i] = right_pose;
      }
      response.new_right_stick_poses = right_poses;
      left_poses.poses.resize(b_bot_traj.points.size());
      for (int i = 0; i < b_bot_traj.points.size(); i++)
      {
        geometry_msgs::Pose left_pose;
        robot_state.setJointGroupPositions("b_bot", b_bot_traj.points[i].positions);
        const Eigen::Affine3d& left_stick_state = robot_state.getGlobalLinkTransform("b_bot_diabolo_stick_tip");

        left_pose.position.x = left_stick_state.translation().x();
        left_pose.position.y = left_stick_state.translation().y();
        left_pose.position.z = left_stick_state.translation().z();

        left_poses.poses[i] = left_pose;
      }
      response.new_left_stick_poses = left_poses;
    }
    else
    {
      response.new_right_stick_poses = request.right_stick_poses;
      response.new_left_stick_poses = request.left_stick_poses;
    }
  }
  else
  {
    response.new_right_stick_poses = request.right_stick_poses;
    response.new_left_stick_poses = request.left_stick_poses;
  }
  // Store the accelerations and velocities for the points in the trajectory

  robot_traj.joint_trajectory = this->a_bot_seed_traj;
  seed_display_traj.trajectory.push_back(robot_traj);
  a_bot_display_seed_traj_pub.publish(seed_display_traj);
  ROS_INFO_STREAM("Number of stick trajectory points = " << response.new_left_stick_poses.poses.size());

  response.a_bot_trajectory = a_bot_traj;
  response.b_bot_trajectory = b_bot_traj;
  response.success = true;
}

void StickTargetToJointTargetConverter::fill_trajectory(trajectory_msgs::JointTrajectory& a_bot_traj, ros::Duration time_step,
                                               int trajectory_steps)
{
  int new_a_bot_traj_count = 0;
  int i = 0;
  ros::Duration next_time_from_start(0);

  trajectory_msgs::JointTrajectory new_a_bot_traj;
  new_a_bot_traj.joint_names = a_bot_traj.joint_names;
  new_a_bot_traj.points.resize(trajectory_steps + 100);
  while (i < a_bot_traj.points.size())
  {
    if (i == 0)
    {
      new_a_bot_traj.points[new_a_bot_traj_count] = a_bot_traj.points[i];
      next_time_from_start = a_bot_traj.points[i].time_from_start;

      i++;
    }
    else
    {

      if (a_bot_traj.points[i].time_from_start > next_time_from_start)  // = a point was skipped
      {
        trajectory_msgs::JointTrajectoryPoint left_knot_point, right_knot_point;
        left_knot_point = a_bot_traj.points[i - 1];
        right_knot_point = a_bot_traj.points[i];

        trajectory_msgs::JointTrajectoryPoint new_point;
        new_point.time_from_start = next_time_from_start;
        // Add the required number of points by 1D interpolation for each joint
        new_point.positions.resize(6);
        for (int k = 0; k < 6; k++)
        {
          new_point.positions[k] = interpolate1D(next_time_from_start.toSec(), left_knot_point.time_from_start.toSec(),
                                                 right_knot_point.time_from_start.toSec(), left_knot_point.positions[k],
                                                 right_knot_point.positions[k]);
        }


        new_a_bot_traj.points[new_a_bot_traj_count] = new_point;
      }
      else
      {
        new_a_bot_traj.points[new_a_bot_traj_count] = a_bot_traj.points[i];

        i++;
      }

    }  // If i != 0

    new_a_bot_traj_count++;
    next_time_from_start += time_step;

  }  // Finished filling in missing a_bot points

  a_bot_traj.points.clear();
  a_bot_traj = new_a_bot_traj;
  if (a_bot_traj.points.size() > (new_a_bot_traj_count))  // = the size of the vector is bigger than
  {
    ROS_WARN("Erasing points");
    // Remove the trailing empty elements
    a_bot_traj.points.erase(a_bot_traj.points.begin() + new_a_bot_traj_count, a_bot_traj.points.end());
    ROS_WARN("Erased points");
  }
  ROS_WARN_STREAM("Trajectory has " << a_bot_traj.points.size() << " after filling points");
}
bool StickTargetToJointTargetConverter::get_splines_from_trajectory(std::vector<PiecewiseSpline>& splines,
                                                           double initial_joint_angles[],
                                                           const trajectory_msgs::JointTrajectory& traj)
{
  splines.clear();
  for (int j = 0; j < 6; j++)
  {
    // Make vector of knot points for each joint in the vector
    std::vector<std::vector<double>> knot_points;

    // The first knot point for the spline is the initial joint position
    knot_points.push_back({ 0.0, initial_joint_angles[j] });

    for (int i = 0; i < traj.points.size(); i++)  // For all the joints in the trajectory
    {
      knot_points.push_back({ traj.points[i].time_from_start.toSec(), traj.points[i].positions[j] });
    }
    // Get a spline object for each of the joint angles
    PiecewiseSpline p(knot_points, 0.0, 0.0);
    splines.push_back(p);
  }
  return true;
}
bool StickTargetToJointTargetConverter::apply_bounds(const robot_model::RobotModel& rmodel,
                                            trajectory_msgs::JointTrajectory& traj,
                                            std::vector<std::string>& joint_names)
{
  // Ignore the first and last points

  // Apply velocity bounds
  for (int i = 1; i < traj.points.size() - 1; i++)
  {
    // The index of the joint exceeding its velocity / acceleration bounds the most
    int max_bound_exceeding_joint_index = -1;
    float max_bound_error = 0;
    float bound_to_use = 0;
    for (int j = 0; j < 6; j++)
    {
      // Find the maximum amount a joint exceeds its velocity bounds
      // Get bounds for this joint
      const robot_model::VariableBounds& b = rmodel.getVariableBounds(joint_names[j]);
      // First ensure that the trajectory respects velocity bounds
      // Find the joint that deviates from its velocity bound by the greatest amount
      // Use that bound to calculate the amount of time to add to time_from_start
      if (b.velocity_bounded_)
      {
        double bound = b.max_velocity_;
        if (abs(traj.points[i].velocities[j]) > bound)
        {
          if (max_bound_error < abs(traj.points[i].velocities[j]) - bound)
          {
            max_bound_error = abs(traj.points[i].velocities[j]) - bound;
            bound_to_use = bound;
            max_bound_exceeding_joint_index = j;
          }
        }
      }
    }
    if (max_bound_exceeding_joint_index != -1)
    {
      double q1, q2, q3, dt1, dt2;
      if (i == 0)
      {
        // First point
        q1 = traj.points[1].positions[max_bound_exceeding_joint_index];
        q2 = traj.points[0].positions[max_bound_exceeding_joint_index];
        q3 = q1;

        dt1 = (traj.points[1].time_from_start.toSec() - traj.points[0].time_from_start.toSec());
        dt2 = dt1;
      }
      else if (i < traj.points.size() - 1)
      {
        // middle points
        q1 = traj.points[i - 1].positions[max_bound_exceeding_joint_index];
        q2 = traj.points[i].positions[max_bound_exceeding_joint_index];
        q3 = traj.points[i + 1].positions[max_bound_exceeding_joint_index];

        dt1 = (traj.points[i].time_from_start.toSec() - traj.points[i - 1].time_from_start.toSec());
        dt2 = (traj.points[i + 1].time_from_start.toSec() - traj.points[i].time_from_start.toSec());
      }
      else
      {
        // last point
        q1 = traj.points[i - 1].positions[max_bound_exceeding_joint_index];
        q2 = traj.points[i].positions[max_bound_exceeding_joint_index];
        q3 = q1;

        dt2 = (traj.points[traj.points.size() - 1].time_from_start.toSec() -
               traj.points[traj.points.size() - 2].time_from_start.toSec());
        dt1 = dt2;
      }
      double extra_time = 0.;
      // Add this extra time to the time from start of each point from here to the last point
      double old_time = -traj.points[i - 1].time_from_start.toSec() + traj.points[i].time_from_start.toSec();
      extra_time = (abs(q2 - q1) / bound_to_use) - old_time;
      for (int k = i; k < traj.points.size(); k++)
      {
        traj.points[k].time_from_start += ros::Duration(extra_time);
      }
    }
  }
  // Calculate the new accelerations and velocities after changing times to suit velocity bounds
  this->calculate_trajectory_acceleration_and_velocity(traj, 0);

  // TODO: Apply acceleration bounds

}

bool StickTargetToJointTargetConverter::calculate_trajectory_acceleration_and_velocity(
    trajectory_msgs::JointTrajectory& traj, std::vector<PiecewiseSpline>& traj_splines, int start_point)
{
  for (int i = start_point; i < traj.points.size(); i++)
  {
    traj.points[i].velocities.clear();
    traj.points[i].accelerations.clear();
    ROS_WARN_STREAM("Spline dynamics evaluated at " << traj.points[i].time_from_start.toSec());
    for (int j = 0; j < 6; j++)  // For all the joints in the trajectory
    {
      double vel = traj_splines[j].evaluate_first_differential((double)traj.points[i].time_from_start.toSec());
      double acc = traj_splines[j].evaluate_second_differential(traj.points[i].time_from_start.toSec());

      traj.points[i].velocities.push_back(vel);
      traj.points[i].accelerations.push_back(acc);
    }
  }

  return true;
}
bool StickTargetToJointTargetConverter::calculate_trajectory_acceleration_and_velocity(trajectory_msgs::JointTrajectory& traj,
                                                                              int start_point)
{
  // Parameters:
  // traj: The trajectory to calculate for
  // start_point: calculate vel and acceleration from here to the end of the trajectory
  // Calculate the positions and velocities of each point from position and time from start
  for (int i = start_point; i < traj.points.size(); i++)
  {
    traj.points[i].velocities.clear();
    traj.points[i].accelerations.clear();
    for (int j = 0; j < 6; j++)  // For all the joints in the trajectory
    {
      double q1, q2, q3, dt1, dt2;
      if (i == 0)
      {
        // First point
        q1 = traj.points[1].positions[j];
        q2 = traj.points[0].positions[j];
        q3 = q1;

        dt1 = (traj.points[1].time_from_start.toSec() - traj.points[0].time_from_start.toSec());
        dt2 = dt1;
      }
      else if (i < traj.points.size() - 1)
      {
        // middle points
        q1 = traj.points[i - 1].positions[j];
        q2 = traj.points[i].positions[j];
        q3 = traj.points[i + 1].positions[j];

        dt1 = (traj.points[i].time_from_start.toSec() - traj.points[i - 1].time_from_start.toSec());
        dt2 = (traj.points[i + 1].time_from_start.toSec() - traj.points[i].time_from_start.toSec());
      }
      else
      {
        // last point
        q1 = traj.points[i - 1].positions[j];
        q2 = traj.points[i].positions[j];
        q3 = q1;

        dt2 = (traj.points[traj.points.size() - 1].time_from_start.toSec() -
               traj.points[traj.points.size() - 2].time_from_start.toSec());
        dt1 = dt2;
      }

      double v1 = (q2 - q1) / dt1;
      double v2 = (q3 - q2) / dt2;

      traj.points[i].velocities.push_back((v1 + v2) / 2.0);
      traj.points[i].accelerations.push_back((v2 - v1) / (dt1 + dt2));
    }
  }

  return true;
}

bool StickTargetToJointTargetConverter::setInitialPosition(diabolo_play::SetInitialStickPositions::Request& request,
                                                  diabolo_play::SetInitialStickPositions::Response& response)
{
  double target_joint_state[6];
  bool ok = false;
  int count = 0;
  geometry_msgs::Point a, b;
  a = request.right_stick_position;
  b = request.left_stick_position;
  geometry_msgs::Pose a_pose = getPoseFromPoint(a, "right_arm");
  ROS_INFO_STREAM("point a ");
  ROS_INFO_STREAM(a);
  const robot_state::JointModelGroup* a_joint_model_group = robot_state_.getJointModelGroup("a_bot");

  ok = setRobotStateIK(robot_state_, a_joint_model_group, a_pose, "a_bot");

  if (ok)
  {
    robot_state_.copyJointGroupPositions(a_joint_model_group, target_joint_state);

    std::vector<double> a_bot_joint_target;
    a_bot_joint_target.resize(6);
    for (int i = 0; i < 6; i++)
    {
      a_bot_joint_target[i] = target_joint_state[i];
    }

    a_bot_group_.setJointValueTarget(a_bot_joint_target);

    // Store the initial joint position as the first joint state deque
    a_bot_group_.move();
  }
  else
  {
    ROS_WARN("IK not found for a_bot. Not moving.");
  }

  const robot_state::JointModelGroup* b_joint_model_group = robot_state_.getJointModelGroup("b_bot");
  geometry_msgs::Pose b_pose = getPoseFromPoint(b, "left_arm");
  ok = setRobotStateIK(robot_state_, b_joint_model_group, b_pose, "b_bot");
  // Empty the joint state deque
  if (ok)
  {
    robot_state_.copyJointGroupPositions(b_joint_model_group, target_joint_state);

    std::vector<double> b_bot_joint_target;
    b_bot_joint_target.resize(6);
    for (int i = 0; i < 6; i++)
    {
      b_bot_joint_target[i] = target_joint_state[i];
    }

    b_bot_group_.setJointValueTarget(b_bot_joint_target);

    // Store the initial joint position as the first joint state deque
    b_bot_group_.move();
  }
  else
  {
    ROS_WARN("IK not found for b_bot. Not moving.");
  }

  response.success = true;
  return true;
}

bool StickTargetToJointTargetConverter::setRobotState(std::vector<double> joint_positions, std::string robot_name)
{
  moveit::planning_interface::MoveGroupInterface* group_pointer = robotNameToMoveGroup(robot_name);
  ;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success, motion_done;
  group_pointer->setMaxVelocityScalingFactor(1.0);
  group_pointer->setJointValueTarget(joint_positions);

  success = (group_pointer->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    motion_done = group_pointer->execute(my_plan);
    return (motion_done == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  else
  {
    ROS_ERROR("Could not plan to before_tool_pickup joint state. Abort!");
    return false;
  }
}

geometry_msgs::Pose StickTargetToJointTargetConverter::getPoseFromPoint(const geometry_msgs::Point point,
                                                               std::string left_or_right_arm)
{
  // TODO: Set a reasonable orientation for a point
  // use linear interpolation between two quaternions based on world z-coordinate

  // Counter is used to index the array containing the desired orientations at the extreme edges of the area the arms
  // are allowed to move in
  int counter = 0;
  if (left_or_right_arm == "right_arm")
    counter = 4;
  double z = point.z;

  if (z < corner_coords[4])
  {
    z = corner_coords[4];
  }

  else if (z > corner_coords[5])
  {
    z = corner_coords[5];
  }
  tf2::Quaternion q = corner_orientations[counter].slerp(
      corner_orientations[counter + 3], ((z - corner_coords[4]) / (corner_coords[5] - corner_coords[4])));
  geometry_msgs::Pose p;

  p.position = point;
  p.orientation.x = q.getX();
  p.orientation.y = q.getY();
  p.orientation.z = q.getZ();
  p.orientation.w = q.getW();
  return p;
}

geometry_msgs::Pose StickTargetToJointTargetConverter::getPoseFromPoint(const geometry_msgs::PointConstPtr point_msg,
                                                               std::string left_or_right_arm)
{
  // TODO: Set a reasonable orientation for a point
  // use linear interpolation between two quaternions based on world z-coordinate

  return getPoseFromPoint(*point_msg, left_or_right_arm);
}
void StickTargetToJointTargetConverter::setRobotPoseCallback(const geometry_msgs::PoseArrayConstPtr msg)
{
  // First point is left stick, second point is right stick
  std::cout << msg->poses[0].position << std::endl;
  setRobotPose(msg->poses[0], "b_bot");
  setRobotPose(msg->poses[1], "a_bot");
  planning_scene_.setCurrentState(robot_state_);
}

void StickTargetToJointTargetConverter::storeJointStates(const sensor_msgs::JointStateConstPtr msg)
{
  // If using real robots, each individual message might have all the a_bot joints and all b_bot joints,
  
  std::vector<std::string> joints = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                      "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };
  this->current_joint_state_ = *msg;
  for (int i = 0; i < this->current_joint_state_.name.size(); i++)
  {
    // Look for the appropriate index for the required joint angle in the latest joint state message
    for (int j = 0; j < joints.size(); j++)
    {
      if ("a_bot_" + joints[j] == this->current_joint_state_.name[i])
      {
        this->a_bot_current_joint_angles_[j] = this->current_joint_state_.position[i];
        continue;
      }
      else if ("b_bot_" + joints[j] == this->current_joint_state_.name[i])
      {
        this->b_bot_current_joint_angles_[j] = this->current_joint_state_.position[i];
        continue;
      }
    }
  }
}

std::string StickTargetToJointTargetConverter::getRobotNamefromJMG(const robot_state::JointModelGroup* jmg)
{
  std::vector<std::string> joint_names = jmg->getVariableNames();
  char rob = joint_names[0][0];
  if (rob == 'a')
    return "a_bot";

  else if (rob == 'b')
    return "b_bot";

  else
    return "unknown";
}

bool StickTargetToJointTargetConverter::checkStateValidity(moveit::core::RobotState* robot_state,
                                                  const robot_state::JointModelGroup* joint_group,
                                                  const double* joint_group_variable_values)
{
  std::vector<std::string> joint_names = joint_group->getVariableNames();  // Get variable (joint) names
  // Store each joint state and check the difference between old and hypothetical joint angles
  double old_joint_state[6], new_joint_state[6];
  robot_state_.copyJointGroupPositions(joint_group, old_joint_state);

  for (int i = 0; i < 6; i++)  // Storing ik values received from solver in new_joint_state
  {
    new_joint_state[i] = joint_group_variable_values[i];
  }
  std::string robot_name = getRobotNamefromJMG(joint_group);

  double(*limits_ptr)[6][2];
  if (robot_name == "a_bot")
  {
    limits_ptr = &a_bot_joint_limits_;
  }

  else if (robot_name == "b_bot")
  {
    limits_ptr = &b_bot_joint_limits_;
  }
  double error_width = 90.0 * M_PI / 180.0;  // 5 degrees allowance around the set joint limits
  bool joint_outside_limits = false;
  for (int i = 0; i < 6; i++)
  {
    if (joint_group_variable_values[i] > ((*limits_ptr)[i][1] + error_width) ||
        joint_group_variable_values[i] < ((*limits_ptr)[i][0] - error_width))
    {
      joint_outside_limits = true;

      ROS_DEBUG_STREAM("Joint " << joint_names[i] << " is outside error limits");
      ROS_DEBUG_STREAM("Desired joint value = " << joint_group_variable_values[i]);
      // joint_group_variable_values++;
    }
  }
  if (joint_outside_limits)
    return false;

  bool joint_state_too_far = false;
  for (int i = 0; i < 6; i++)
  {
    if (abs(old_joint_state[i] - new_joint_state[i]) > 0.5)
    {
      joint_state_too_far = true;
    }
  }

  if (joint_state_too_far)
  {
    return false;
  }  // If any of the joints would move more than a set number of degrees, this ik is invalid


  return true;
}

void StickTargetToJointTargetConverter::a_bot_wrist2_parallel_to_ground(std::vector<double>& joint_variables)
{
  joint_variables[2] = -joint_variables[0] - joint_variables[1] + (-3 * M_PI / 2.0);
}

void StickTargetToJointTargetConverter::b_bot_wrist2_parallel_to_ground(std::vector<double>& joint_variables)
{
  joint_variables[2] = -joint_variables[0] - joint_variables[1] + (-M_PI / 2.0);
}
void StickTargetToJointTargetConverter::a_bot_wrist3_zero(std::vector<double>& joint_variables)
{
  joint_variables[0] = 0.0;  // Keep wrist 3 = 0
}
void StickTargetToJointTargetConverter::b_bot_wrist3_zero(std::vector<double>& joint_variables)
{
  joint_variables[0] = 0.0;  // Keep wrist 3 = 0
}

bool StickTargetToJointTargetConverter::setRobotStateIK(moveit::core::RobotState& robot_state,
                                               const robot_state::JointModelGroup* jmg,
                                               const geometry_msgs::Pose pose_in_world, std::string robot_name)
{
  // FOR DEBUGGING
  // if(robot_name == "a_bot") // Stop a_bot temporarily
  //   return true;
  int attempts = 5;  // How many times setFromIK is called
  bio_ik::BioIKKinematicsQueryOptions ik_options;
  ik_options.replace = true;
  ik_options.return_approximate_solution = true;

  // Goal position and small joint displacements will be prioritised over goal orientation.
  // This means setting a lower weight for goal orientation and tuning the parameters

  auto* position_goal = new bio_ik::PositionGoal();
  auto* a_bot_wrist_2_parallel_goal = new bio_ik::JointFunctionGoal();
  auto* b_bot_wrist_2_parallel_goal = new bio_ik::JointFunctionGoal();
  auto* a_bot_wrist_3_zero_goal = new bio_ik::JointFunctionGoal();
  auto* b_bot_wrist_3_zero_goal = new bio_ik::JointFunctionGoal();
  auto* a_bot_wrist_2_angle_goal = new bio_ik::JointVariableGoal();
  auto* b_bot_wrist_2_angle_goal = new bio_ik::JointVariableGoal();
  auto* uniform_regularization_primary_goal = new bio_ik::RegularizationGoal();
  // set goals as KinematicsQueryOptions
  ik_options.goals.emplace_back(position_goal);
  // ik_options.goals.emplace_back(uniform_regularization_primary_goal);

  // Declare JointFunctionGoal Functions
  std::function<void(std::vector<double>&)> a_bot_wrist_2_parallel_goal_function, b_bot_wrist_2_parallel_goal_function;
  std::function<void(std::vector<double>&)> a_bot_wrist_3_zero_goal_function, b_bot_wrist_3_zero_goal_function;
  // Set JointFunctionGoal variables
  std::vector<std::string> a_bot_wrist_2_parallel_goal_variables, b_bot_wrist_2_parallel_goal_variables;
  std::vector<std::string> a_bot_wrist_3_zero_goal_variables, b_bot_wrist_3_zero_goal_variables;

  std::vector<std::string> joint_names = jmg->getVariableNames();
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = pose_in_world.position.x;
  goal_pose.position.y = pose_in_world.position.y;
  goal_pose.position.z = pose_in_world.position.z;
  // set link names
  position_goal->setLinkName(robot_name + "_diabolo_stick_tip");

  // set goals
  position_goal->setPosition(
      tf2::Vector3(pose_in_world.position.x, pose_in_world.position.y, pose_in_world.position.z));

  // set goal weights
  position_goal->setWeight(this->position_goal_weight);

  // Create Group State Validity Callback Function object
  boost::function<bool(moveit::core::RobotState * robot_state, const robot_state::JointModelGroup* joint_group,
                       const double* joint_group_variable_values)>
      gsvcf = boost::bind(&StickTargetToJointTargetConverter::checkStateValidity, this, boost::placeholders::_1,
                          boost::placeholders::_2, boost::placeholders::_3);

  // If using interpolation, (interpolation_flag = true), then interpolate here using the values in position_goal
  // Fill interpolated joint states in jmg, and then call setFromIK
  if (interpolation_flag)
  {
    if (robot_name == "a_bot")
    {
      std::vector<double> a_bot_seed;  // Seed joint angles received from interpolation function
      a_bot_seed =
          getInterpolatedJointAngles(pose_in_world.position.x, pose_in_world.position.y, pose_in_world.position.z,
                                     this->a_bot_interpolation_corner_coords, this->a_bot_interpolation_data);

      robot_state.setJointGroupPositions(jmg, a_bot_seed);
      std::vector<double> seed_joint_values;
      robot_state.copyJointGroupPositions("a_bot", seed_joint_values);
      trajectory_msgs::JointTrajectoryPoint current_a_bot_seed;
      current_a_bot_seed.positions.resize(6);
      // Add a trajectory point to the a_bot_seed_trajectory for display
      for (int i = 0; i < 6; i++)
      {
        current_a_bot_seed.positions[i] = seed_joint_values[i];
      }
      this->a_bot_seed_traj.points.push_back(current_a_bot_seed);
      const Eigen::Affine3d& right_stick_state = robot_state.getGlobalLinkTransform("a_bot_diabolo_stick_tip");
      Eigen::Quaterniond q = (Eigen::Quaterniond)right_stick_state.linear();

      goal_pose.orientation.x = q.x();
      goal_pose.orientation.y = q.y();
      goal_pose.orientation.z = q.z();
      goal_pose.orientation.w = q.w();


      // Set wrist_2_angle_goal variable name and position
      ik_options.goals.emplace_back(a_bot_wrist_2_angle_goal);
      a_bot_wrist_2_angle_goal->setVariableName(joint_names.at(4));
      a_bot_wrist_2_angle_goal->setVariablePosition(a_bot_seed[4]);
    }
    else if (robot_name == "b_bot")
    {
      std::vector<double> b_bot_seed;  // Seed joint angles received from interpolation function
      b_bot_seed =
          getInterpolatedJointAngles(pose_in_world.position.x, pose_in_world.position.y, pose_in_world.position.z,
                                     this->b_bot_interpolation_corner_coords, this->b_bot_interpolation_data);
      robot_state.setJointGroupPositions(jmg, b_bot_seed);
      const Eigen::Affine3d& left_stick_state = robot_state.getGlobalLinkTransform("b_bot_diabolo_stick_tip");
      Eigen::Quaterniond q = (Eigen::Quaterniond)left_stick_state.linear();

      goal_pose.orientation.x = q.x();
      goal_pose.orientation.y = q.y();
      goal_pose.orientation.z = q.z();
      goal_pose.orientation.w = q.w();
      // Set wrist_2_angle_goal variable name and position
      ik_options.goals.emplace_back(b_bot_wrist_2_angle_goal);
      b_bot_wrist_2_angle_goal->setVariableName(joint_names.at(4));
      b_bot_wrist_2_angle_goal->setVariablePosition(b_bot_seed[4]);
    }

    // Publish marker showing interpolated seed position

    publish_interpolated_seed_state(robot_state, robot_name);

  }  // End if setting interpolated seed state

  if (robot_name == "a_bot")
  {
    ik_options.goals.emplace_back(a_bot_wrist_2_parallel_goal);
    ik_options.goals.emplace_back(a_bot_wrist_3_zero_goal);

    // Set wrist_2_parallel_goal function and variables
    a_bot_wrist_2_parallel_goal_function =
        boost::bind(&StickTargetToJointTargetConverter::a_bot_wrist2_parallel_to_ground, this, boost::placeholders::_1);
    for (int i = 1; i < 4; i++)  // Add shoulder, elbow and wrist1 joints to function
    {
      // std::cout <<  joint_names.at(i) << " ";
      a_bot_wrist_2_parallel_goal_variables.push_back(joint_names.at(i));
    }

    a_bot_wrist_2_parallel_goal->setJointVariableNames(a_bot_wrist_2_parallel_goal_variables);
    a_bot_wrist_2_parallel_goal->setJointVariableFunction(a_bot_wrist_2_parallel_goal_function);
    // Set wrist 3 goal function and variables
    a_bot_wrist_3_zero_goal_function =
        boost::bind(&StickTargetToJointTargetConverter::a_bot_wrist3_zero, this, boost::placeholders::_1);
    a_bot_wrist_3_zero_goal_variables.push_back(joint_names.at(5));  // Regulating wrist 3
    a_bot_wrist_3_zero_goal->setJointVariableNames(a_bot_wrist_3_zero_goal_variables);
    a_bot_wrist_3_zero_goal->setJointVariableFunction(a_bot_wrist_3_zero_goal_function);

  }  // end of setting a_bot goals

  else if (robot_name == "b_bot")
  {
    ik_options.goals.emplace_back(b_bot_wrist_2_parallel_goal);
    ik_options.goals.emplace_back(b_bot_wrist_3_zero_goal);

    // Set wrist_2_parallel_goal function and variables
    b_bot_wrist_2_parallel_goal_function =
        boost::bind(&StickTargetToJointTargetConverter::b_bot_wrist2_parallel_to_ground, this, boost::placeholders::_1);
    for (int i = 1; i < 4; i++)  // Add shoulder, elbow and wrist1 joints to function
    {
      b_bot_wrist_2_parallel_goal_variables.push_back(joint_names.at(i));
    }

    b_bot_wrist_2_parallel_goal->setJointVariableNames(b_bot_wrist_2_parallel_goal_variables);
    b_bot_wrist_2_parallel_goal->setJointVariableFunction(b_bot_wrist_2_parallel_goal_function);
    // Set wrist 3 goal function and variables
    b_bot_wrist_3_zero_goal_function =
        boost::bind(&StickTargetToJointTargetConverter::b_bot_wrist3_zero, this, boost::placeholders::_1);
    b_bot_wrist_3_zero_goal_variables.push_back(joint_names.at(5));  // Regulating wrist 3
    b_bot_wrist_3_zero_goal->setJointVariableNames(b_bot_wrist_3_zero_goal_variables);
    b_bot_wrist_3_zero_goal->setJointVariableFunction(b_bot_wrist_3_zero_goal_function);

    // Set wrist_2_angle_goal function and variables

  }  // end of setting b_bot goals

  bool ok = false;
  int counter = 0;
  while (!ok && counter < attempts)
  {
    // // // For bio ik
    ok = robot_state.setFromIK(jmg,                            // active PR2 joints
                               EigenSTL::vector_Isometry3d(),  // no explicit poses here
                               std::vector<std::string>(),     // no end effector links here
                               0.0,                            // take values from YAML file

                               gsvcf, ik_options);

    // For kdl (and similar)
    // ok = robot_state.setFromIK(
    //           jmg,           // active PR2 joints
    //           goal_pose, // no explicit poses here
    //           0.01,                      // take values from YAML file
    //           gsvcf
    //           );
    // ROS_WARN_STREAM(" ================ Finished IK for: " << robot_name << " ");
    // if (!ok)
    // {
    //   ROS_ERROR_STREAM(" ================ IK for: " << robot_name << "failed!");
    // }

    // ok = robot_state.setFromIK(jmg, pose_in_world, robot_name+"_diabolo_stick_tip"); // Setting ik without kinematic
    // query options publishRobotState(robot_state);

    counter += 1;
  }
  if (counter > 1)
    ROS_WARN_STREAM("IK solver took " << counter << " attempts for " << robot_name << ".");
  if (!ok)
    ROS_ERROR_STREAM("Was not able to set IK for " << robot_name << "!");
  return ok;
}

bool StickTargetToJointTargetConverter::setRobotPose(const geometry_msgs::Pose pose_in_world, std::string robot_name)
{
  const robot_state::JointModelGroup* joint_model_group = robot_state_.getJointModelGroup(robot_name);
  // TODO: Set joint (variable) limits for the joint model group here

  std::vector<double> old_joint_state, new_joint_state;
  robot_state_.copyJointGroupPositions(joint_model_group, old_joint_state);
  std::vector<double> stick_pos{ pose_in_world.position.x, pose_in_world.position.y, pose_in_world.position.z };

  bool IK_valid = setRobotStateIK(robot_state_, joint_model_group, pose_in_world,
                                  robot_name);  // Set the new joint positions from IK
  if (IK_valid)
  {
    robot_state_.copyJointGroupPositions(joint_model_group, new_joint_state);

    publishRobotState(robot_state_);
    sendRobotCommand(robot_state_, robot_name);
  }

  else if (!IK_valid)
  {
    ROS_INFO_STREAM(robot_name << " cannot move due to invalid IK");
  }
}

moveit::planning_interface::MoveGroupInterface* StickTargetToJointTargetConverter::robotNameToMoveGroup(std::string robot_name)
{
  // This function converts the name of the robot to a pointer to the member variable containing the move group
  if (robot_name == "a_bot")
    return &a_bot_group_;
  if (robot_name == "b_bot")
    return &b_bot_group_;
}

// This forces a refresh of the planning scene.
bool StickTargetToJointTargetConverter::updatePlanningScene()
{
  moveit_msgs::GetPlanningScene srv;
  get_planning_scene_client.call(srv);
  if (get_planning_scene_client.call(srv))
  {
    ROS_INFO("Got planning scene from move group.");
    planning_scene_.setPlanningSceneMsg(srv.response.scene);
    return true;
  }
  else
  {
    ROS_ERROR("Failed to get planning scene from move group.");
    return false;
  }
}

bool StickTargetToJointTargetConverter::sendRobotCommand(moveit::core::RobotState robot_state, std::string robot_name)
{
  // TODO: This can be optimized if it is slow

  // Make and publish joint trajectory for a_bot
  const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup(robot_name);
  trajectory_msgs::JointTrajectory traj;
  std::vector<double> joint_state;
  robot_state.copyJointGroupPositions(robot_name, joint_state);
  // Make sure the deque has at least 2 elements

  traj.header.frame_id = "world";

  // (workaround) There can be more joints than the 6 relevant ones in the list for some reason
  std::vector<std::string> joint_names = joint_model_group->getJointModelNames();
  traj.joint_names.resize(6);
  traj.points.resize(1);
  traj.points[0].positions.resize(6);
  for (int i = 0; i < 6; i++)
  {
    traj.points[0].positions[i] = joint_state[i];
    traj.joint_names[i] = joint_names[i];
  }
  double time = 1.0 / this->stick_pose_publish_rate;
  ROS_WARN_STREAM("Time to goal is " << time);
  traj.points[0].time_from_start = ros::Duration(time + 0.05);
  if (robot_name == "a_bot")
    pubCommandA_.publish(traj);
  else if (robot_name == "b_bot")
    pubCommandB_.publish(traj);
  return true;
}
visualization_msgs::Marker StickTargetToJointTargetConverter::make_marker_(std::string const& mesh_filename,
                                                                  std::string const& frame_id, tf2::Vector3 position,
                                                                  tf2::Quaternion orientation, std::string const& ns,
                                                                  int type, float scale[], float col[])
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = position.getX();
  marker.pose.position.y = position.getY();
  marker.pose.position.z = position.getZ();
  marker.pose.orientation.x = orientation.getX();
  marker.pose.orientation.y = orientation.getY();
  marker.pose.orientation.z = orientation.getZ();
  marker.pose.orientation.w = orientation.getW();
  marker.mesh_resource = mesh_filename;
  marker.scale.x = scale[0];
  marker.scale.y = scale[1];
  marker.scale.z = scale[2];
  marker.color.r = col[0];
  marker.color.g = col[1];
  marker.color.b = col[2];
  marker.color.a = col[3];
  marker.lifetime = ros::Duration();
  marker.id = marker_count++;
  return marker;
}
void StickTargetToJointTargetConverter::publishMarkers()
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;
  ros::Time now = ros::Time::now();
  // Get stick orientations at the border points and publish visualization markers
  float c[4], s[3];
  // Brown stick color
  c[0] = (165. / 255.);
  c[1] = (107. / 255.);
  c[2] = (70. / 255.);
  c[3] = 1.0f;
  ;

  s[0] = 0.3f;
  s[1] = 0.02f;
  s[2] = 0.02f;

  // bottom close left marker
  marker = make_marker_("", "/world", tf2::Vector3(corner_coords[0], corner_coords[2], corner_coords[4]),
                        corner_orientations[0], "corner_orient", visualization_msgs::Marker::SPHERE, s, c);
  marker.header.stamp = ros::Time();
  marker_array.markers.push_back(marker);

  // bottom close right marker
  marker = make_marker_("", "/world", tf2::Vector3(corner_coords[0], corner_coords[3], corner_coords[4]),
                        corner_orientations[4], "corner_orient", visualization_msgs::Marker::SPHERE, s, c);
  marker.header.stamp = ros::Time();
  marker_array.markers.push_back(marker);
  pubMarkerArray_.publish(marker_array);

  // top close left marker
  marker = make_marker_("", "/world", tf2::Vector3(corner_coords[0], corner_coords[2], corner_coords[5]),
                        corner_orientations[2], "corner_orient", visualization_msgs::Marker::SPHERE, s, c);
  marker.header.stamp = ros::Time();
  marker_array.markers.push_back(marker);
  pubMarkerArray_.publish(marker_array);

  // top close right marker
  marker = make_marker_("", "/world", tf2::Vector3(corner_coords[0], corner_coords[3], corner_coords[5]),
                        corner_orientations[6], "corner_orient", visualization_msgs::Marker::SPHERE, s, c);
  marker.header.stamp = ros::Time();
  marker_array.markers.push_back(marker);
  pubMarkerArray_.publish(marker_array);

  marker_count = 0;
}

bool StickTargetToJointTargetConverter::publishRobotState(moveit::core::RobotState robot_state)
{
  moveit_msgs::DisplayRobotState state_msg;
  moveit::core::robotStateToRobotStateMsg(robot_state, state_msg.state);
  pubRobotState_.publish(state_msg);
  return true;
}

void StickTargetToJointTargetConverter::checkStickPosePubRate(const ros::TimerEvent& event)
{
  if (!n_.getParam("/stick_pose_publish_rate", this->stick_pose_publish_rate))
  {
    this->stick_pose_publish_rate = 10.0;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diabolo_playback_robot");
  ros::AsyncSpinner spinner(1);  // Needed for MoveIt to work.
  spinner.start();

  StickTargetToJointTargetConverter ss;
  std::vector<double> a_bot_x_coords = { 0.425, 0.775 };
  std::vector<double> a_bot_y_coords = { -0.95, -0.05 };
  std::vector<double> a_bot_z_coords = { 0.8, 1.8 };
  std::vector<double> b_bot_x_coords = { 0.425, 0.775 };
  std::vector<double> b_bot_y_coords = { 0.05, 0.95 };
  std::vector<double> b_bot_z_coords = { 0.8, 1.8 };

  while (ros::ok())
  {
    ros::WallDuration(.01).sleep();
    if (ss.interpolation_flag)
    {
      ss.publish_interpolation_data_markers();
    }
    ros::spinOnce();
  }

  return 0;
}
