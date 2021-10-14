#include "diabolo_play/diabolo_motion_generator.h"
#define ON_STRING 1
// Off the string and below at least one stick
#define OFF_STRING_LOOSE 2
// Off the string and above the sticks
#define FLYING 3
// Outside the border of the ellipse. Requires position correction
#define OUTSIDE_STRING 4

DiaboloMotionGenerator::DiaboloMotionGenerator()
{
  // Advertise services
  calc_trajectories_service =
      this->n_.advertiseService("/generate_stick_trajectory", &DiaboloMotionGenerator::calculateStickTrajectory, this);
  simulate_diabolo_service =
      this->n_.advertiseService("/simulate_diabolo", &DiaboloMotionGenerator::simulateDiabolo, this);
  get_robot_trajectories_client =
      this->n_.serviceClient<diabolo_play::CreateRobotTrajectory>("/command_robot_traj_from_stick_traj");
  this->marker_array_pub_ =
      this->n_.advertise<visualization_msgs::MarkerArray>("/predicted_state/visualization_marker_array", 10);
  this->marker_count_ = 0;
  ROS_INFO("Diabolo trajectory generation service now online!");
}

DiaboloMotionGenerator::~DiaboloMotionGenerator()
{
}
std::vector<double> DiaboloMotionGenerator::get_new_trajectory_coefficients(std::vector<double> coeffs,
                                                                            double min_change, double max_change)
{
  // Return a vector of random numbers
  // min_change and max_change are the range of random additions to the current coefficient values
  std::vector<double> new_coeffs;
  new_coeffs.resize(coeffs.size());
  for (int i = 0; i < coeffs.size() - 1; i++)
  {
    // RANDOM WALK:
    new_coeffs[i] = coeffs[i] + ((double)rand() * (max_change - min_change) / (RAND_MAX)) +
                    min_change;  // Get random coeffs between min and max
  }
  new_coeffs[coeffs.size() - 1] = 0.;  // Constant coeff is always 0
  new_coeffs[coeffs.size() - 2] = 0.;  // Constant coeff is always 0
  return new_coeffs;
}
std::vector<PiecewiseSpline> DiaboloMotionGenerator::get_randomized_spline_objects(
    std::vector<std::vector<double>>& coord_values, std::vector<double>& time_coords, double& max_time,
    double stick_time_step, double coord_min_step, double coord_max_step, int motion_flag)
{
  std::vector<PiecewiseSpline> new_splines;
  // Randomize the values in coord values using coord_min and coord_max step
  // min_change and max_change are the range of random additions to the current coefficient values

  // IMPORTANT: This changes the values of the coord_values directly
  // Take care to send a copy of the coordianates being used as a parameter, and overwite them only if error reduces
  // when using resulting splines
  double max_y = coord_values[coord_values.size() - 1][1];
  double min_y = coord_values[coord_values.size() - 1][4];
  for (int j = 0; j < coord_values.size(); j++)
  {
    if (motion_flag != diabolo_play::CreateSticksTrajectoryRequest::THROW || j != coord_values.size() - 1)
    {
      for (int i = 0; i < coord_values[j].size(); i++)
      {
        coord_values[j][i] =
            coord_values[j][i] + ((double)rand() * (coord_max_step - coord_min_step) / (RAND_MAX)) + coord_min_step;
        if (motion_flag == diabolo_play::CreateSticksTrajectoryRequest::THROW)
        {
          if (coord_values[j][i] >= max_y)
            coord_values[j][i] = max_y - 0.01;

          if (coord_values[j][i] <= min_y)
            coord_values[j][i] = min_y + 0.01;
        }
      }
    }
  }
  double min_allowed_time = 0.3;
  double max_allowed_time = 1.0;

  if (motion_flag == diabolo_play::CreateSticksTrajectoryRequest::LOOP)
  {
    min_allowed_time = 1.5;
    max_allowed_time = 2.5;
  }

  max_time = max_time + ((double)rand() * (0.05 - (-0.05)) / (RAND_MAX)) + (-0.05);
  // Ensure that the max time can be perfectly divided by the stick time step
  // If this is not set, the last point in the trajectory will likely not have 0 velocity
  int stick_steps = max_time / stick_time_step;
  max_time = stick_time_step * stick_steps;

  if (max_time < min_allowed_time)
    max_time = min_allowed_time;
  if (max_time > max_allowed_time)
    max_time = max_allowed_time;

  double last_time = 0.;
  // max -(n-i)*min_displacement - last_time
  // time coord randomized with a minnimum of 100 ms between time steps
  for (int i = 0; i < time_coords.size() - 1; i++)
  {
    double mt = max_time - (time_coords.size() - i - 1) * 0.25 - last_time;
    time_coords[i] = last_time + 0.25 + (double)rand() * (mt - 0.25) / (RAND_MAX);
    last_time = time_coords[i];
  }
  time_coords[time_coords.size() - 1] = max_time;

  // create spline knot points for each coordinate, for each stick position (6 in total)
  // The slopes of all the splines at the extreme points are always 0
  double left_slope = 0.;
  double right_slope = 0.;

  for (int i = 0; i < 6; i++)
  {
    std::vector<std::vector<double>> knots;  // The knot points to calculate the spline from
    knots.push_back({ 0., 0. });  // The first point is the origin, and is not included in the coord_values parameter
    // This loop will iterate for the number of splines to create

    // Create spline objects from candidate vector
    for (int j = 0; j < time_coords.size(); j++)  // Number of steps per coordinate
    {
      int j_side = j;
      // This loop will iterate over all the spline knot points for a single spline

      // coord_values: Vector holding the values of the x,y,z coordinates at the times in time_coords
      //              format: {{left_x1 left_x2... left_xn, right_x1, right_x2..
      //              },{left_y1..right_yn},{left_z1...right_zn}}

      if (i % 2 != 0)  // i = even -> left hand. odd -> right hand (see format of the coord_vector)
      {
        j_side = j + time_coords.size();
      }

      std::vector<double> k = { time_coords[j], coord_values[i / 2][j_side] };  // i/2 is rounded down
      // The last coord must be the same as the first coord if this is supposed to loop
      if (j == time_coords.size() - 1 && motion_flag == diabolo_play::CreateSticksTrajectoryRequest::LOOP)
      {
        k[1] = 0.;
      }
      knots.push_back(k);
    }

    // Create a new spline with the stored knots and add to new_splines
    PiecewiseSpline p{ knots, left_slope, right_slope };
    new_splines.push_back(p);
  }

  return new_splines;
}
std::vector<PiecewiseSpline> DiaboloMotionGenerator::get_spline_objects(std::vector<std::vector<double>>& coord_values,
                                                                        std::vector<double>& time_coords)
{
  std::vector<PiecewiseSpline> new_splines;

  ROS_WARN("Time coords are: ");
  ROS_WARN_STREAM(time_coords[0] << " " << time_coords[1] << " " << time_coords[2]);

  // create spline knot points for each coordinate, for each stick position (6 in total)
  // The slopes of all the splines at the extreme points are always 0
  double left_slope = 0.;
  double right_slope = 0.;

  for (int i = 0; i < 6; i++)
  {
    std::vector<std::vector<double>> knots;  // The knot points to calculate the spline from
    knots.push_back({ 0., 0. });  // The first point is the origin, and is not included in the coord_values parameter
    // This loop will iterate for the number of splines to create

    // time_coords.size() is 3
    // Create spline objects from candidate vector
    for (int j = 0; j < time_coords.size(); j++)  // Number of steps per coordinate
    {
      int j_side = j;
      // This loop will iterate over all the spline knot points for a single spline

      // coord_values: Vector holding the values of the x,y,z coordinates at the times in time_coords
      //              format: {{left_x1m left_x2... left_xn, right_x1, right_x2..
      //              },{left_y1..right_yn},{left_z1...right_zn}}

      if (i % 2 != 0)  // i = even -> left hand. odd -> right hand (see format of the coord_vector)
      {
        j_side = j + time_coords.size();
      }
      std::vector<double> k = { time_coords[j], coord_values[i / 2][j_side] };  // i/2 is rounded down (probably)
      knots.push_back(k);
    }
    // Create a new spline with the stored knots and add to new_splines
    PiecewiseSpline p{ knots, left_slope, right_slope };
    new_splines.push_back(p);
  }

  return new_splines;
}
void DiaboloMotionGenerator::initialize_spline_knots(diabolo_play::DiaboloMotionSplineSeeds knot_seeds,
                                                     std::vector<std::vector<double>>& coord_values,
                                                     std::vector<double>& time_coords, int movement_flag,
                                                     double max_time)
{
  for (int i = 0; i < 3; i++)  // dimensions
  {
    for (int j = 0; j < 2 * time_coords.size(); j++)  // Number of spline points
    {
      // This loop will iterate over all the spline knot points for a single spline
      if (i == 0) // knot point x coordinates
      {
        /// left stick
        if (j < time_coords.size())
        {
          coord_values[i].push_back(knot_seeds.left_seed[j].x);
        }
        /// right stick
        if (j >= time_coords.size())
        {
          coord_values[i].push_back(knot_seeds.right_seed[j - time_coords.size()].x);
        }
      }
      if (i == 1) // knot point y cordinates
      {
        /// left stick
        if (j < time_coords.size())
        {
          // ROS_WARN_STREAM("Y knot seed for j = " << j <<"is " << knot_seeds.left_seed[j].y);
          coord_values[i].push_back(knot_seeds.left_seed[j].y);
        }
        /// right stick
        if (j >= time_coords.size())
        {
          coord_values[i].push_back(knot_seeds.right_seed[j - time_coords.size()].y);
        }
      }

      if (i == 2) // knot point z coordinates
      {
        /// left stick
        if (j < time_coords.size())
        {
          coord_values[i].push_back(knot_seeds.left_seed[j].z);
        }
        /// right stick
        if (j >= time_coords.size())
        {
          coord_values[i].push_back(knot_seeds.right_seed[j - time_coords.size()].z);
        }
      }
    }
  }

  for (int j = 0; j < time_coords.size(); j++)
  {
    time_coords[j] = knot_seeds.time_seed[j];
  }
}

bool DiaboloMotionGenerator::calculateStickTrajectory(diabolo_play::CreateSticksTrajectory::Request& request,
                                                      diabolo_play::CreateSticksTrajectory::Response& response)
{
  this->marker_count_ = 0;
  double x_coord = request.current_sim_config.initial_poses.poses[0].position.x;
  bool to_constrain = request.constrain_to_YZ;
  // Check that non zero stick time step is provided
  double stick_time_step = request.stick_update_time_step;
  if (stick_time_step == 0)
  {
    response.success = false;
    ROS_ERROR("Stick update rate not provided. Aborting trajectory calculation");
    return false;
  }

  srand(ros::Time::now().sec + ros::Time::now().nsec);
  // Initially predict the sim config after the planned stick poses are executed
  // This will be given to the predictor inside the optimization loop
  diabolo_gazebo::DiaboloSimConfig sim_config_after_planned_trajectory;  // After the planned_poses vector
  sim_config_after_planned_trajectory = request.current_sim_config;

  DiaboloPredictor dp_initial(request.current_sim_config);
  if (to_constrain)
  {
    dp_initial.set_2D_constraint(ignition::math::Vector3d(1.0, 0.0, 0.0), ignition::math::Vector3d(x_coord, 0.0, 0.0));
  }
  diabolo_gazebo::DiaboloState state_after_planned_trajectory;
  int steps = request.planned_left_stick_poses.poses.size();
  // If there are planned poses, get the diabolo sim config after those poses are executed
  if (!steps == 0)
  {
    std::vector<double> planned_left_times, planned_right_times;
    planned_left_times.resize(steps);
    planned_right_times.resize(steps);
    for (int i = 1; i < steps + 1; i++)
    {
      planned_left_times[i - 1] = (double)i * stick_time_step;
      planned_right_times[i - 1] = (double)i * stick_time_step;
    }
    std::vector<diabolo_gazebo::DiaboloState> p_states;
    double time = planned_left_times[steps - 1];
    ////////////////////
    int total_states = time / request.current_sim_config.time_step;
    p_states.resize(total_states + 100);  // Larger than it needs to be
    ROS_WARN_STREAM("Should be " << steps << " poses in the planned poses");

    ROS_WARN("Starting initial prediction");
    state_after_planned_trajectory =
        dp_initial.predict(request.planned_left_stick_poses, request.planned_right_stick_poses, planned_left_times,
                           planned_right_times, p_states, true);  // Store the predicted states for visualization
    ROS_WARN("Finished initial prediction");
    float col[4];
    col[0] = 1.0;
    col[1] = 1.0;
    col[2] = 1.0;
    col[3] = 1.0;
    this->publish_predicted_trajectory(p_states, col, "predicted_trajectory_until_plan_start");
    this->publish_stick_traj_markers(request.planned_left_stick_poses, request.planned_right_stick_poses, col,
                                     "predicted_trajectory_until_plan_start");

    sim_config_after_planned_trajectory.initial_poses.poses[0] = state_after_planned_trajectory.pose;
    sim_config_after_planned_trajectory.initial_poses.poses[1] = request.planned_left_stick_poses.poses[steps - 1];
    sim_config_after_planned_trajectory.initial_poses.poses[2] = request.planned_right_stick_poses.poses[steps - 1];
    sim_config_after_planned_trajectory.trans_velocity = state_after_planned_trajectory.trans_velocity;
    sim_config_after_planned_trajectory.rot_velocity = state_after_planned_trajectory.rot_velocity;
  }

  this->goal_states_ = request.goal_states;
  ROS_WARN_STREAM("Throwflag is " << request.motion_flag);
  // Initialize coeficient vector
  // Vector of vectors holding the coefficients of the cubic equation describing stick trajectories for each coordinate
  // Order of coefficients:
  // [0]~[2] left stick
  // [3]~[5] right stick
  // x coordinate, y coordinate, z coordinate
  std::vector<std::vector<double>> coord_vector;
  std::vector<std::vector<double>> coord_vector_candidate;
  std::vector<double> time_vector;
  std::vector<double> time_vector_candidate;
  // The length of the trajectgory is initialized to the last time in the provided seed
  double traj_time = request.knot_seeds.time_seed[request.knot_seeds.time_seed.size() - 1];
  double traj_time_candidate;

  time_vector.resize(request.knot_seeds.time_seed.size());  // The number of knot points determined by the number of
                                                            // seeds
  coord_vector.resize(3);  // 3 dimensions. THIS IS NOT THE SAME AS THE ABOVE 3 FOR TIME_VECTOR

  // initialize coord_vector and time vector
  this->initialize_spline_knots(request.knot_seeds, coord_vector, time_vector, request.motion_flag, 1.0);
  coord_vector_candidate = coord_vector;  // For formatting

  time_vector_candidate = time_vector;

  // Enter the optimization loop
  geometry_msgs::Pose initial_pose_l, initial_pose_r;
  initial_pose_l = sim_config_after_planned_trajectory.initial_poses.poses[1];
  initial_pose_r = sim_config_after_planned_trajectory.initial_poses.poses[2];
  // These are the vectors to store the trajectory giving the best outcome
  geometry_msgs::PoseArray best_left_trajectory;
  geometry_msgs::PoseArray best_right_trajectory;
  diabolo_gazebo::DiaboloState closest_diabolo_state;
  double min_error_score = FLT_MAX;
  std::vector<diabolo_gazebo::DiaboloState> predicted_states;

  // TODO: Calculate reasonable stick poses

  int loop_count = 0;
  int fail_count = 0;
  int error_reduced_count = 0;
  std::deque<double> last_errors = { 1e8, 1e7, 1e6 };

  std::vector<PiecewiseSpline> x_spline_vector;
  x_spline_vector = this->get_spline_objects(coord_vector, time_vector);
  int max_loop_count = 5;
  if (!request.optimize)
  {
    max_loop_count = 1;
  }
  while (loop_count < max_loop_count)
  {
    std::vector<PiecewiseSpline> spline_vector;
    // Breakout criteria
    coord_vector_candidate = coord_vector;
    traj_time_candidate = traj_time;
    time_vector_candidate = time_vector;

    if (((last_errors[1] - last_errors[2]) < 1e-4))
    {
      std::cout << "Error[1] =  " << last_errors[1] << std::endl;
      std::cout << "Error[2] =  " << last_errors[2] << std::endl;
      std::cout << "Relative error =  " << (last_errors[1] - last_errors[2]) << std::endl;
      std::cout << "Breaking out after " << loop_count << " iterations because relative error is too low." << std::endl;
      break;
    }
    if (min_error_score < 1e-3)
    {
      std::cout << "Breaking out after " << loop_count
                << " iterations because absolute error is considered acceptable at " << min_error_score << std::endl;
      break;
    }
    geometry_msgs::PoseArray left_poses, right_poses;
    std::vector<double> left_times, right_times;
    // Adjust the coefficient vectors
    if (request.optimize)
    {
      spline_vector =
          this->get_randomized_spline_objects(coord_vector_candidate, time_vector_candidate, traj_time_candidate,
                                              stick_time_step, -0.01, 0.01, request.motion_flag);
    }
    else
    {
      spline_vector = this->get_spline_objects(coord_vector_candidate, time_vector_candidate);
    }
    // Get the same spline every time
    double time_period =
        time_vector_candidate[time_vector_candidate.size() - 1];  // The last element in the time vector
    int number_of_steps = time_period / stick_time_step;

    // Get all the positions in this trajectory
    left_poses.poses.resize(number_of_steps);
    right_poses.poses.resize(number_of_steps);
    left_times.resize(number_of_steps);
    right_times.resize(number_of_steps);

    for (int i = 1; i < number_of_steps + 1; i++)
    {
      geometry_msgs::Pose pose_l, pose_r;
      pose_l = initial_pose_l;
      pose_r = initial_pose_r;
      // Calculate polynomials defined in helper_functions.h
      // Constraining to the x axis for now
      pose_l.position.x += x_spline_vector[0].evaluate_spline(i * stick_time_step);
      pose_r.position.x += x_spline_vector[1].evaluate_spline(i * stick_time_step);
      pose_l.position.y += spline_vector[2].evaluate_spline(i * stick_time_step);
      pose_r.position.y += spline_vector[3].evaluate_spline(i * stick_time_step);
      pose_l.position.z += spline_vector[4].evaluate_spline(i * stick_time_step);
      pose_r.position.z += spline_vector[5].evaluate_spline(i * stick_time_step);

      left_poses.poses[i - 1] = pose_l;
      right_poses.poses[i - 1] = pose_r;
      left_times[i - 1] = (double)i * stick_time_step;
      right_times[i - 1] = (double)i * stick_time_step;
    }
    if (!this->trajectory_is_safe(left_poses, right_poses, request.current_sim_config.string_length))
    {
      ROS_ERROR("Trajectory rejected!");

      loop_count++;
      fail_count++;
      ROS_ERROR("Continuing");
      continue;
    }

    predicted_states.clear();
    diabolo_gazebo::DiaboloState diabolo_state;
    int total_diabolo_states = time_period / request.current_sim_config.time_step;
    predicted_states.resize(total_diabolo_states + 100);  // Larger than it needs to be
    ROS_WARN("Starting prediction");
    DiaboloPredictor dp(sim_config_after_planned_trajectory);
    if (to_constrain)
    {
      dp.set_2D_constraint(ignition::math::Vector3d(1.0, 0.0, 0.0), ignition::math::Vector3d(x_coord, 0.0, 0.0));
    }
    diabolo_state = dp.predict(left_poses, right_poses, left_times, right_times, predicted_states, true);
    float col[4];
    col[0] = 0.8;
    col[1] = 0.8;
    col[2] = 0.0;
    col[3] = 0.8;
    this->publish_predicted_trajectory(predicted_states, col, "trajectory_candidates");
    ROS_WARN("Finished prediction");

    double error_score = 0.;

    // This will contain the diabolo states determined to be the closest to the waypoint states
    std::vector<diabolo_gazebo::DiaboloState> closest_states;
    closest_states.resize(request.goal_states.size());  // One state for each of the goal waypoints
    ROS_WARN_STREAM("Waypoint goal z position is " << request.goal_states[0].pose.position.z);
    error_score = this->evaluate_trajectory_cost(left_poses, right_poses, request.goal_states, predicted_states,
                                                 closest_states, request.motion_flag);

    std::cout << "Error score is " << error_score << std::endl;
    if (error_score < min_error_score)
    {
      error_reduced_count++;
      min_error_score = error_score;
      coord_vector = coord_vector_candidate;
      time_vector = time_vector_candidate;
      traj_time = traj_time_candidate;
      ROS_WARN_STREAM("New min error is " << min_error_score);
      last_errors.push_back(min_error_score);
      last_errors.pop_front();
      best_left_trajectory.poses.clear();
      best_right_trajectory.poses.clear();
      best_left_trajectory = left_poses;
      best_right_trajectory = right_poses;
      closest_diabolo_state = diabolo_state;

      ROS_WARN_STREAM("New best position is " << closest_diabolo_state.pose.position.x << " "
                                              << closest_diabolo_state.pose.position.y << " "
                                              << closest_diabolo_state.pose.position.z);
      ROS_WARN_STREAM("Points in best traj = " << best_left_trajectory.poses.size());
      response.diabolo_states = closest_states;
    }

    loop_count++;
  }

  ROS_INFO_STREAM("Exited loop after " << loop_count << " attempts");
  ROS_WARN_STREAM("Error reduced " << error_reduced_count << " times");
  ROS_ERROR_STREAM("Unsafe trajectories = " << fail_count);
  for (std::vector<double> cv : coord_vector)
  {
    for (double c : cv)
      std::cout << c << " ";

    std::cout << std::endl;
  }
  for (double t : time_vector)
    std::cout << t << " ";
  std::cout << std::endl;
  if (error_reduced_count == 0)
  {
    ROS_ERROR("Could not find a suitable trajectory");
    response.success = false;
    return true;
  }

  // Get the required robot trajectory from the best stick trajectory found
  float col[4];
  col[0] = 0.8;
  col[1] = 0.8;
  col[2] = 0.0;
  col[3] = 0.8;
  this->publish_predicted_trajectory(predicted_states, col, "best_trajectory");
  this->publish_stick_traj_markers(best_left_trajectory, best_right_trajectory, col, "best_trajectory");
  diabolo_play::CreateRobotTrajectory srv;
  srv.request.left_stick_poses = best_left_trajectory;
  srv.request.right_stick_poses = best_right_trajectory;
  srv.request.smooth_robot_trajectory = true;
  srv.request.stick_update_time_step = stick_time_step;
  get_robot_trajectories_client.call(srv);
  best_left_trajectory.poses.clear();
  best_right_trajectory.poses.clear();
  best_left_trajectory = srv.response.new_left_stick_poses;
  best_right_trajectory = srv.response.new_right_stick_poses;

  response.a_bot_trajectory = srv.response.a_bot_trajectory;
  response.b_bot_trajectory = srv.response.b_bot_trajectory;

  response.left_stick_poses = best_left_trajectory;
  response.right_stick_poses = best_right_trajectory;

  if (this->trajectory_is_safe(best_left_trajectory, best_right_trajectory, request.current_sim_config.string_length))
    response.success = true;
  else
    response.success = false;

  ROS_WARN_STREAM("The new ideal left stick trajectory has " << best_left_trajectory.poses.size() << " poses");
  ROS_WARN_STREAM("The new ideal right stick trajectory has " << best_right_trajectory.poses.size() << " poses");
  return true;
}

bool DiaboloMotionGenerator::simulateDiabolo(diabolo_play::SimulateDiabolo::Request& request,
                                             diabolo_play::SimulateDiabolo::Response& response)
{
  int steps = request.left_stick_poses.poses.size();
  std::vector<double> left_times, right_times;
  left_times.resize(steps);
  right_times.resize(steps);
  for (int i = 1; i < steps + 1; i++)
  {
    left_times[i - 1] = (double)i * request.stick_update_time_step;
    right_times[i - 1] = (double)i * request.stick_update_time_step;
  }
  double time_period = steps * request.stick_update_time_step;
  std::vector<diabolo_gazebo::DiaboloState> predicted_states;
  int total_diabolo_states = time_period / request.sim_config.time_step;
  predicted_states.resize(total_diabolo_states + 100);
  DiaboloPredictor dp(request.sim_config);
  if (request.constrain_to_YZ) // If the diabolo is to be constrained to the vertical plane
  {
    // Constrain to the initial x position of the real diabolo
    double x_coord = request.sim_config.initial_poses.poses[0].position.x;
    dp.set_2D_constraint(ignition::math::Vector3d(1.0, 0.0, 0.0), ignition::math::Vector3d(x_coord, 0.0, 0.0));
  }
  dp.sleep_flag_ = request.animate;
  dp.predict(request.left_stick_poses, request.right_stick_poses, left_times, right_times, predicted_states, true);

  response.diabolo_states = predicted_states;
  return true;
}

bool DiaboloMotionGenerator::trajectory_is_safe(geometry_msgs::PoseArray& left_poses,
                                                geometry_msgs::PoseArray& right_poses, double string_length)
{
  return true;
  // Remove points if they violate string length constraint
  int i = 0;
  // Do not allow trajectory leaving reasonable bounds
  // Get bounds from parameter server. These are the same as the bounds used for interpolation by the robot controller
  // In the form of a vector.
  // Format: [0] x_close, [1]: x_far, [2]: y_right, [3]: y_left, [4]: z_low, [5]: z_high
  std::vector<float> a_bot_bounds, b_bot_bounds;
  if (!this->n_.getParam("/a_bot_ik_bounds", a_bot_bounds))  // This is a double vector
  {
    ROS_WARN_STREAM("a_bot bounds not set. Ignoring diabolo state error for bounds");
  }
  if (!this->n_.getParam("/b_bot_ik_bounds", b_bot_bounds))  // This is a double vector
  {
    ROS_WARN_STREAM("b_bot bounds not set. Ignoring diabolo state error for bounds");
  }
  while (i < left_poses.poses.size())
  {
    double distance_between_tips = sqrt(pow((left_poses.poses[i].position.x - right_poses.poses[i].position.x), 2) +
                                        pow((left_poses.poses[i].position.y - right_poses.poses[i].position.y), 2) +
                                        pow((left_poses.poses[i].position.z - right_poses.poses[i].position.z), 2));

    if (distance_between_tips > string_length)
    {
      // Remove all elements from current index to end of trajectory
      // Need to pass const iterators to "erase".
      ROS_WARN_STREAM("Acceptable max string length is: " << string_length);
      ROS_ERROR_STREAM("This trajectory causes distance between tips =  " << distance_between_tips << " at step number "
                                                                          << i);
      int last_index = left_poses.poses.size() - 1;
      double final_tip_distance =
          sqrt(pow((left_poses.poses[last_index].position.x - right_poses.poses[last_index].position.x), 2) +
               pow((left_poses.poses[last_index].position.y - right_poses.poses[last_index].position.y), 2) +
               pow((left_poses.poses[last_index].position.z - right_poses.poses[last_index].position.z), 2));

      ROS_ERROR_STREAM("Final distance between stick tips would be = " << final_tip_distance);
      return false;
    }
    if (i == left_poses.poses.size() - 1)
    {
      ROS_WARN_STREAM("The distance between stick tips for this trajectory is = " << distance_between_tips);
    }
    // Remove trajectory points in which sticks cross over
    if (left_poses.poses[i].position.y - right_poses.poses[i].position.y < 0)
    {
      ROS_ERROR("Sticks tips cross over in this trajectory");

      return false;
    }

    // Remove trajectory points where stick x coordinate is too close to the table
    if (left_poses.poses[i].position.x < 0.45 || right_poses.poses[i].position.x < 0.45)
    {
      ROS_ERROR("One of the robots is too close to the table");
      return false;
    }

    if (!a_bot_bounds.empty() && !b_bot_bounds.empty())
    {
      if ((left_poses.poses[i].position.x < b_bot_bounds[0]) || (left_poses.poses[i].position.x > b_bot_bounds[1]))
      {
        ROS_INFO_STREAM("b_bot x position out of bounds at " << left_poses.poses[i].position.x);
        return false;
      }
      if ((left_poses.poses[i].position.y < b_bot_bounds[2] - 0.25) ||
          (left_poses.poses[i].position.y > b_bot_bounds[3]))
      {
        ROS_INFO_STREAM("b_bot y position out of bounds at " << left_poses.poses[i].position.y);
        return false;
      }
      if ((left_poses.poses[i].position.z < 0.65) || (left_poses.poses[i].position.z > 2.0))
      {
        ROS_INFO_STREAM("b_bot z position out of bounds at " << left_poses.poses[i].position.z);
        return false;
      }

      if ((right_poses.poses[i].position.x < a_bot_bounds[0]) || (right_poses.poses[i].position.x > a_bot_bounds[1]))
      {
        ROS_INFO_STREAM("a_bot x position out of bounds at " << right_poses.poses[i].position.x);
        return false;
      }
      if ((right_poses.poses[i].position.y < a_bot_bounds[2]) ||
          (right_poses.poses[i].position.y > a_bot_bounds[3] + 0.25))
      {
        ROS_INFO_STREAM("a_bot y position out of bounds at " << right_poses.poses[i].position.y);
        return false;
      }
      if ((right_poses.poses[i].position.z < 0.65) || (right_poses.poses[i].position.z > 2.0))
      {
        ROS_INFO_STREAM("a_bot z position out of bounds at " << right_poses.poses[i].position.z);
        return false;
      }
    }
    i++;
  }
  return true;  // True if the trajectory can be executed
}
double DiaboloMotionGenerator::evaluate_trajectory_cost(
    geometry_msgs::PoseArray left_poses, geometry_msgs::PoseArray right_poses, diabolo_gazebo::DiaboloState goal_state,
    diabolo_gazebo::DiaboloState state, std::vector<diabolo_gazebo::DiaboloState>& closest_states, int motion_flag)
{
  // Normalize the error for each type against max possible error so different types of error can be compared
  // TODO: Compose a reasonable evaluation function. Using euclidian distance for now

  double cost = 0;

  // Get cost due to height difference
  double height_diff = abs(state.pose.position.z - goal_state.pose.position.z);
  cost += height_diff / goal_state.pose.position.z;
  ROS_WARN_STREAM("Goal z height is  = " << goal_state.pose.position.z);
  ROS_WARN_STREAM("State z height is  = " << state.pose.position.z);
  // Get cost due to velocity difference
  ignition::math::Vector3d current_state_vel(state.trans_velocity.x, state.trans_velocity.y, state.trans_velocity.z);
  ignition::math::Vector3d goal_state_vel(goal_state.trans_velocity.x, goal_state.trans_velocity.y,
                                          goal_state.trans_velocity.z);

  // Get magnitude difference between velocities
  double vel_mag_diff = abs(current_state_vel.Length() - goal_state_vel.Length());
  // Normalize mag_diff with magnitude of goal velocity
  if (goal_state_vel.Length() == 0.)
    ROS_ERROR("Goal state vel = 0");
  cost += vel_mag_diff / goal_state_vel.Length();

  // Get angle between velocities
  double vel_angle_diff = abs(acos(current_state_vel.Normalize().Dot(goal_state_vel.Normalize())));
  // Assign a higher cost for angle difference than for magnitude difference

  if (vel_angle_diff > 45.0 * M_PI / 180.0)
  {
    cost += 10000;  
  }

  double max_angle_cost = M_PI;
  cost += vel_angle_diff / max_angle_cost;

  if (motion_flag == diabolo_play::CreateSticksTrajectoryRequest::THROW)
  {
    // Add a cost if the distance between stick tips is not close to string length
    int max_index = left_poses.poses.size() - 1;
    double distance_between_tips =
        sqrt(pow((left_poses.poses[max_index].position.x - right_poses.poses[max_index].position.x), 2) +
             pow((left_poses.poses[max_index].position.y - right_poses.poses[max_index].position.y), 2) +
             pow((left_poses.poses[max_index].position.z - right_poses.poses[max_index].position.z), 2));

    if (state.string_length == 0.)
      ROS_ERROR("String length = 0");

    if (state.string_length - distance_between_tips > 0.02)
    {
      cost += (1.0 - (distance_between_tips / state.string_length)) * 100.0;
    }
    else
    {
      cost += (1.0 - (distance_between_tips / state.string_length)) * 50.0;
    }
  }
  if (motion_flag == diabolo_play::CreateSticksTrajectoryRequest::THROW)
  {
    // Add a cost if the difference between distances from stick tips to the diabolo is large
    int max_index = left_poses.poses.size() - 1;
    double right_tip_to_diabolo = sqrt(pow((state.pose.position.x - right_poses.poses[max_index].position.x), 2) +
                                       pow((state.pose.position.y - right_poses.poses[max_index].position.y), 2) +
                                       pow((state.pose.position.z - right_poses.poses[max_index].position.z), 2));

    double left_tip_to_diabolo = sqrt(pow((state.pose.position.x - left_poses.poses[max_index].position.x), 2) +
                                      pow((state.pose.position.y - left_poses.poses[max_index].position.y), 2) +
                                      pow((state.pose.position.z - left_poses.poses[max_index].position.z), 2));
    if (abs(left_tip_to_diabolo - right_tip_to_diabolo) > 0.05)
    {
      cost += (abs(left_tip_to_diabolo - right_tip_to_diabolo) / state.string_length) * 5.0;
    }
  }
  closest_states[0] = state;
  return cost;
}
double DiaboloMotionGenerator::evaluate_trajectory_cost(geometry_msgs::PoseArray left_poses,
                                                        geometry_msgs::PoseArray right_poses,
                                                        std::vector<diabolo_gazebo::DiaboloState>& waypoint_states,
                                                        std::vector<diabolo_gazebo::DiaboloState>& predicted_states,
                                                        std::vector<diabolo_gazebo::DiaboloState>& closest_states,
                                                        int motion_flag)
{
  // If the requested motion is a looping motion, the diabolo cannot end with being thrown off the string
  // TODO: This will actually be a more complex condition, as the diabolo  may be caught for looping motion
  // But simplifying for now
  if (motion_flag == diabolo_play::CreateSticksTrajectoryRequest::LOOP)
  {
    for (int i = 0; i < predicted_states.size() - 1; i++)
    {
      if (predicted_states[i].string_state == diabolo_gazebo::DiaboloState::FLY)
      {
        ROS_ERROR("This trajectory will result in the diabolo going off the string");
        return FLT_MAX;
      }
    }
  }
  if (waypoint_states.size() == 1)  // If only one waypoint state, then it has to come at the end of the trajectory
  {
    ROS_INFO("Only one goal waypoint. Calling throw appropriate function");
    return evaluate_trajectory_cost(left_poses, right_poses, waypoint_states[0],
                                    predicted_states[predicted_states.size() - 1], closest_states, motion_flag);
  }
  // Normalize the error for each type against max possible error so different types of error can be compared

  double cost = 0;
  // This is the start point from which to search for the diabolo pose closest to the waypoint being considered
  // This is to ensure that the diabolo passes close to the desired waypoints in order of occurance
  int start_step = 0;
  ROS_WARN_STREAM("Number of predicted steps = " << predicted_states.size());
  for (int j = 0; j < waypoint_states.size(); j++)
  {
    double min_cost = FLT_MAX;
    for (int i = start_step; i < predicted_states.size(); i++)
    {
      double current_cost = 0;
      ignition::math::Vector3d current_state_pos(predicted_states[i].pose.position.x,
                                                 predicted_states[i].pose.position.y,
                                                 predicted_states[i].pose.position.z);
      ignition::math::Vector3d goal_state_pos(waypoint_states[j].pose.position.x, waypoint_states[j].pose.position.y,
                                              waypoint_states[j].pose.position.z);
      current_cost += (current_state_pos - goal_state_pos).Length();

      ignition::math::Vector3d current_state_vel(predicted_states[i].trans_velocity.x,
                                                 predicted_states[i].trans_velocity.y,
                                                 predicted_states[i].trans_velocity.z);
      ignition::math::Vector3d goal_state_vel(waypoint_states[j].trans_velocity.x, waypoint_states[j].trans_velocity.y,
                                              waypoint_states[j].trans_velocity.z);

      double vel_mag_diff = abs(current_state_vel.Length() - goal_state_vel.Length());
      // // Normalize mag_diff with magnitude of goal velocity
      if (goal_state_vel.Length() != 0.0)
      {
        current_cost += vel_mag_diff;
      }
      else
      {
        current_cost += vel_mag_diff / goal_state_vel.Length();
      }

      //   // Get angle between velocities
      double vel_angle_diff = abs(acos(current_state_vel.Normalize().Dot(goal_state_vel.Normalize())));
      current_cost += vel_angle_diff / M_PI;

      if (current_cost < min_cost)
      {
        min_cost = current_cost;
        start_step = i;
        closest_states[j] = predicted_states[i];
      }
    }
    cost += min_cost;
  }

  return cost;
}
void DiaboloMotionGenerator::publish_predicted_trajectory(std::vector<diabolo_gazebo::DiaboloState>& states,
                                                          float col[], std::string ns)
{
  ROS_WARN("Publishing diabolo trajectory");
  visualization_msgs::Marker traj_marker;
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.clear();
  ros::Time now = ros::Time::now();
  traj_marker = this->make_marker_("", "/world", ns + "/diabolo_trajectory", visualization_msgs::Marker::LINE_STRIP,
                                   ignition::math::Vector3<float>(0.01, 0.01, 0.01), col);
  traj_marker.id = this->marker_count_++;
  for (diabolo_gazebo::DiaboloState state : states)
    traj_marker.points.push_back(state.pose.position);

  ROS_WARN_STREAM(traj_marker.points.size() << " points in traj_marker");
  traj_marker.header.stamp = now;
  marker_array.markers.push_back(traj_marker);
  this->marker_array_pub_.publish(marker_array);
  ROS_WARN("Published diabolo trajectory");
}
void DiaboloMotionGenerator::publish_stick_traj_markers(geometry_msgs::PoseArray& left_poses,
                                                        geometry_msgs::PoseArray& right_poses, float col[],
                                                        std::string ns)
{
  ROS_WARN("Publishing diabolo trajectory");
  visualization_msgs::Marker left_stick_markers;
  visualization_msgs::Marker right_stick_markers;
  visualization_msgs::Marker left_stick_traj;
  visualization_msgs::Marker right_stick_traj;
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.clear();
  left_stick_traj =
      this->make_marker_("", "/world", ns + "/left_stick_trajectory", visualization_msgs::Marker::LINE_STRIP,
                         ignition::math::Vector3<float>(0.01, 0.01, 0.01), col);
  left_stick_markers =
      this->make_marker_("", "/world", ns + "/left_stick_points", visualization_msgs::Marker::SPHERE_LIST,
                         ignition::math::Vector3<float>(0.02, 0.02, 0.02), col);
  left_stick_traj.id = this->marker_count_++;
  left_stick_markers.id = this->marker_count_++;
  for (geometry_msgs::Pose pose : left_poses.poses)
  {
    left_stick_traj.points.push_back(pose.position);
    left_stick_markers.points.push_back(pose.position);
  }

  right_stick_traj =
      this->make_marker_("", "/world", ns + "right_stick_trajectory", visualization_msgs::Marker::LINE_STRIP,
                         ignition::math::Vector3<float>(0.01, 0.01, 0.01), col);
  right_stick_markers =
      this->make_marker_("", "/world", ns + "right_stick_points", visualization_msgs::Marker::SPHERE_LIST,
                         ignition::math::Vector3<float>(0.02, 0.02, 0.02), col);
  right_stick_traj.id = this->marker_count_++;
  right_stick_markers.id = this->marker_count_++;
  for (geometry_msgs::Pose pose : right_poses.poses)
  {
    right_stick_traj.points.push_back(pose.position);
    right_stick_markers.points.push_back(pose.position);
  }

  marker_array.markers.push_back(left_stick_markers);
  marker_array.markers.push_back(right_stick_markers);
  marker_array.markers.push_back(left_stick_traj);
  marker_array.markers.push_back(right_stick_traj);

  ros::Time now = ros::Time::now();
  for (visualization_msgs::Marker marker : marker_array.markers)
    marker.header.stamp = now;

  this->marker_array_pub_.publish(marker_array);
  ROS_WARN("Published stick pose markers");
}

visualization_msgs::Marker DiaboloMotionGenerator::make_marker_(std::string const& mesh_filename,
                                                                std::string const& frame_id, std::string const& ns,
                                                                int type, ignition::math::Vector3<float> scale,
                                                                float col[])
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.mesh_resource = mesh_filename;
  marker.scale.x = scale.X();
  marker.scale.y = scale.Y();
  marker.scale.z = scale.Z();
  marker.color.r = col[0];
  marker.color.g = col[1];
  marker.color.b = col[2];
  marker.color.a = col[3];

  return marker;
}
//////////////////////////////////////////////// Motion generator functions above
//////////////////////////////////////////////// Predictor functions below

DiaboloPredictor::DiaboloPredictor(diabolo_gazebo::DiaboloSimConfig initial_state)
  : gravity_(ignition::math::Vector3d(0., 0., -9.8))
{
  this->initial_state_ = initial_state;

  this->left_stick_last_position_ = ignition::math::Vector3d(initial_state.initial_poses.poses[1].position.x,
                                                             initial_state.initial_poses.poses[1].position.y,
                                                             initial_state.initial_poses.poses[1].position.z);

  this->right_stick_last_position_ = ignition::math::Vector3d(initial_state.initial_poses.poses[2].position.x,
                                                              initial_state.initial_poses.poses[2].position.y,
                                                              initial_state.initial_poses.poses[2].position.z);

  this->left_stick_current_position_ = this->left_stick_last_position_;
  this->right_stick_current_position_ = this->right_stick_last_position_;
  this->diabolo_last_position_ = ignition::math::Vector3d(initial_state.initial_poses.poses[0].position.x,
                                                          initial_state.initial_poses.poses[0].position.y,
                                                          initial_state.initial_poses.poses[0].position.z);
  this->diabolo_current_position_ = this->diabolo_last_position_;
  ROS_WARN_STREAM("Initial diabolo position =   " << this->diabolo_current_position_.X() << ", "
                                                  << this->diabolo_current_position_.Y() << ", "
                                                  << this->diabolo_current_position_.Z() << "\n");

  this->diabolo_last_state_ = OFF_STRING_LOOSE;  
  this->diabolo_current_velocity_ = ignition::math::Vector3d(
      initial_state.trans_velocity.x, initial_state.trans_velocity.y, initial_state.trans_velocity.z);
  this->diabolo_current_rot_velocity_ = initial_state.rot_velocity;
  this->pv_pre_cap_scaling_factor_ = initial_state.pv_pre_cap_scale;
  this->pv_post_cap_scaling_factor_ = initial_state.pv_post_cap_scale;
  this->pv_cap_scaling_factor_ = initial_state.pv_cap_scale;
  this->velocity_diffusion_factor_ = initial_state.velocity_diffusion_factor;
  this->time_step_ = initial_state.time_step;
  this->string_length_ = initial_state.string_length;
  this->diabolo_axle_radius_ = initial_state.axle_radius;
  this->catching_string_taut_tolerance_ = 0.03;
  this->pv_string_taut_tolerance_ = 0.03;
  this->rot_friction_factor_ = 0.00015;  // MAGIC NUMBER from deceleration sims
  this->get_ros_parameters();

  // TEMP: Remove after debug
  this->tf_broadcaster_ = std::unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);
  this->marker_array_pub_ =
      this->n_.advertise<visualization_msgs::MarkerArray>("/predicted_state/visualization_marker_array", 10);
  this->sleep_flag_ = false;
}
DiaboloPredictor::~DiaboloPredictor()
{
}

diabolo_gazebo::DiaboloState DiaboloPredictor::predict(geometry_msgs::PoseArray left_poses,
                                                       geometry_msgs::PoseArray right_poses, double stick_time_step)
{
  std::vector<diabolo_gazebo::DiaboloState> predicted_states;  // ignored
  std::vector<double> left_times{ stick_time_step };
  std::vector<double> right_times{ stick_time_step };
  return this->predict(left_poses, right_poses, left_times, right_times, predicted_states, false);
}

diabolo_gazebo::DiaboloState DiaboloPredictor::predict(geometry_msgs::PoseArray left_poses,
                                                       geometry_msgs::PoseArray right_poses,
                                                       std::vector<double> left_times, std::vector<double> right_times,
                                                       std::vector<diabolo_gazebo::DiaboloState>& predicted_states,
                                                       bool store_states)
{
  // Store first state in predicted states
  diabolo_gazebo::DiaboloState ds_initial;
  ds_initial.pose.position.x = this->diabolo_current_position_.X();
  ds_initial.pose.position.y = this->diabolo_current_position_.Y();
  ds_initial.pose.position.z = this->diabolo_current_position_.Z();

  ds_initial.trans_velocity.x = this->diabolo_current_velocity_.X();
  ds_initial.trans_velocity.y = this->diabolo_current_velocity_.Y();
  ds_initial.trans_velocity.z = this->diabolo_current_velocity_.Z();

  ds_initial.rot_velocity = this->diabolo_current_rot_velocity_;
  ds_initial.string_length = this->string_length_;
  ds_initial.string_state = this->diabolo_state_;

  ROS_WARN_STREAM("First diabolo state position =   " << ds_initial.pose.position.x << ", "
                                                      << ds_initial.pose.position.y << ", "
                                                      << ds_initial.pose.position.z << "\n");
  if (store_states)
    predicted_states[0] = ds_initial;
  ROS_WARN("Starting prediction trial");
  this->run_prediction_trial(left_poses, right_poses, left_times, right_times, predicted_states, store_states);
  ROS_WARN("Finished prediction trial");

  diabolo_gazebo::DiaboloState ds;
  // Fill the diabolo state variable with results of the trial
  // Position values
  ds.pose.position.x = this->diabolo_current_position_.X();
  ds.pose.position.y = this->diabolo_current_position_.Y();
  ds.pose.position.z = this->diabolo_current_position_.Z();

  // Translational velocity
  ds.trans_velocity.x = this->diabolo_current_velocity_.X();
  ds.trans_velocity.y = this->diabolo_current_velocity_.Y();
  ds.trans_velocity.z = this->diabolo_current_velocity_.Z();
  ds.string_length = this->string_length_;

  // Rotational velocity
  ds.rot_velocity = this->diabolo_current_rot_velocity_;

  return ds;
}

diabolo_gazebo::DiaboloState DiaboloPredictor::get_current_state()
{
  diabolo_gazebo::DiaboloState ds;
  // Position values
  ds.pose.position.x = this->diabolo_current_position_.X();
  ds.pose.position.y = this->diabolo_current_position_.Y();
  ds.pose.position.z = this->diabolo_current_position_.Z();

  // Translational velocity
  ds.trans_velocity.x = this->diabolo_current_velocity_.X();
  ds.trans_velocity.y = this->diabolo_current_velocity_.Y();
  ds.trans_velocity.z = this->diabolo_current_velocity_.Z();
  ds.string_length = this->string_length_;

  // Rotational velocity
  ds.rot_velocity = this->diabolo_current_rot_velocity_;

  return ds;
}

diabolo_gazebo::DiaboloSimConfig DiaboloPredictor::get_current_state_full()
{
  diabolo_gazebo::DiaboloSimConfig cs;
  cs.initial_poses.poses.resize(3);

  cs.initial_poses.poses[0].position.x = this->diabolo_current_position_.X();
  cs.initial_poses.poses[0].position.y = this->diabolo_current_position_.Y();
  cs.initial_poses.poses[0].position.z = this->diabolo_current_position_.Z();

  cs.initial_poses.poses[1].position.x = this->left_stick_current_position_.X();
  cs.initial_poses.poses[1].position.y = this->left_stick_current_position_.Y();
  cs.initial_poses.poses[1].position.z = this->left_stick_current_position_.Z();

  cs.initial_poses.poses[2].position.x = this->right_stick_current_position_.X();
  cs.initial_poses.poses[2].position.y = this->right_stick_current_position_.Y();
  cs.initial_poses.poses[2].position.z = this->right_stick_current_position_.Z();

  // Translational velocity
  cs.trans_velocity.x = this->diabolo_current_velocity_.X();
  cs.trans_velocity.y = this->diabolo_current_velocity_.Y();
  cs.trans_velocity.z = this->diabolo_current_velocity_.Z();

  // Rotational velocity
  cs.rot_velocity = this->diabolo_current_rot_velocity_;

  // Simulation parameters
  cs.string_length = this->string_length_;
  cs.pv_pre_cap_scale = this->pv_pre_cap_scaling_factor_;
  cs.pv_post_cap_scale = this->pv_post_cap_scaling_factor_;
  cs.pv_cap_scale = this->pv_cap_scaling_factor_;
  cs.velocity_diffusion_factor = this->velocity_diffusion_factor_;
  cs.time_step = this->time_step_;
  cs.string_length = this->string_length_;
  cs.axle_radius = this->diabolo_axle_radius_;

  return cs;
}

void DiaboloPredictor::run_prediction_trial(geometry_msgs::PoseArray left_poses, geometry_msgs::PoseArray right_poses,
                                            std::vector<double> left_times, std::vector<double> right_times,
                                            std::vector<diabolo_gazebo::DiaboloState>& predicted_states,
                                            bool store_states)
{
  // TODO: Add interpolation between the available stick tips, as that is what the controller will do as well
  // when using a trajectory
  int left_poses_counter = 0;
  int right_poses_counter = 0;
  double current_sim_time = 0;
  int diabolo_state_counter = 1;  // The initial state is already put in the predicted states array
  // The trajectories do not start with 0 time
  // To get the interpolated stick positions between 0 and the first stick position, the following are used
  double last_left_time = 0.;
  double last_right_time = 0.;
  double next_left_time = left_times[0];
  double next_right_time = right_times[0];
  while (left_poses_counter < left_times.size() ||
         right_poses_counter < right_times.size())  // Run prediction trial for all poses
  {
    if (current_sim_time > left_times[left_poses_counter] && left_poses_counter < left_times.size())
    {
      // Set left stick to the next pose point
      last_left_time = left_times[left_poses_counter];
      this->left_stick_current_position_ = ignition::math::Vector3d(left_poses.poses[left_poses_counter].position.x,
                                                                    left_poses.poses[left_poses_counter].position.y,
                                                                    left_poses.poses[left_poses_counter].position.z);
      left_poses_counter++;
      next_left_time = left_times[left_poses_counter];

      if (left_stick_current_position_.X() < 0.01)  // For debugging (sanity check)
      {
        ROS_WARN_STREAM("x value < 0.01, not normal for the robot. POSE IS LIKELY CORRUPT!");
        ROS_WARN_STREAM("left_stick_current_position_ set to = " << left_stick_current_position_.X() << ", "
                                                                 << left_stick_current_position_.Y() << ", "
                                                                 << left_stick_current_position_.Z());
        ROS_WARN_STREAM("left_poses_counter = " << left_poses_counter);
      }
    }

    if (current_sim_time > right_times[right_poses_counter] && right_poses_counter < right_times.size())
    {
      // Set right stick to the next pose point
      last_right_time = right_times[right_poses_counter];
      this->right_stick_current_position_ = ignition::math::Vector3d(right_poses.poses[right_poses_counter].position.x,
                                                                     right_poses.poses[right_poses_counter].position.y,
                                                                     right_poses.poses[right_poses_counter].position.z);
      right_poses_counter++;
      next_right_time = right_times[right_poses_counter];
      if (right_stick_current_position_.X() < 0.01)  // For debugging (sanity check)
      {
        ROS_WARN_STREAM("x value < 0.01, not normal for the robot. POSE IS LIKELY CORRUPT!");
        ROS_WARN_STREAM("right_stick_current_position_ set to = " << right_stick_current_position_.X() << ", "
                                                                  << right_stick_current_position_.Y() << ", "
                                                                  << right_stick_current_position_.Z());
        ROS_WARN_STREAM("right_poses_counter = " << right_poses_counter);
      }
    }

    this->step();
    current_sim_time += this->time_step_;
    // Store all diabolo states if store_states is true
    if (store_states)
    {
      diabolo_gazebo::DiaboloState ds;
      ds.pose.position.x = this->diabolo_current_position_.X();
      ds.pose.position.y = this->diabolo_current_position_.Y();
      ds.pose.position.z = this->diabolo_current_position_.Z();

      ds.trans_velocity.x = this->diabolo_current_velocity_.X();
      ds.trans_velocity.y = this->diabolo_current_velocity_.Y();
      ds.trans_velocity.z = this->diabolo_current_velocity_.Z();

      ds.rot_velocity = this->diabolo_current_rot_velocity_;
      ds.string_length = this->string_length_;
      ds.string_state = this->diabolo_state_;

      predicted_states[diabolo_state_counter++] = ds;
    }

    // // TEMPORARY: Remove after debug
    this->marker_count_ = 0;
    if (this->sleep_flag_ == true)
    {
      this->publish_visualization_markers();
      ros::Duration(this->time_step_ / 2.0).sleep();
    }
    ////////// End of temporary

    this->right_stick_last_position_ = this->right_stick_current_position_;
    this->left_stick_last_position_ = this->left_stick_current_position_;
  }
  // Erase the empty elements in the predicted states vector
  if (store_states)
    predicted_states.erase(predicted_states.begin() + diabolo_state_counter, predicted_states.end());
  ROS_WARN_STREAM("Predicted states vector size = " << predicted_states.size());
}

void DiaboloPredictor::set_2D_constraint(ignition::math::Vector3d plane_normal, ignition::math::Vector3d plane_point)
{
  this->constrain_to_2D_flag = true;
  this->plane_normal_2D = plane_normal;
  this->plane_normal_2D = this->plane_normal_2D.Normalize();

  this->plane_point_2D = plane_point;
}

void DiaboloPredictor::remove_2D_constraint()
{
  this->constrain_to_2D_flag = false;
}

void DiaboloPredictor::get_ros_parameters()
{
  if (!this->n_.getParam("/constrain_diabolo_2D", this->constrain_to_2D_flag))
  {
    this->constrain_to_2D_flag = false;  // False by default
  }
  else
  {
    if (this->constrain_to_2D_flag)
    {
      std::vector<double> p_normal;
      std::vector<double> p_point;
      ROS_WARN("Constraining diabolo motion to 2D");
      if (!this->n_.getParam("/diabolo_2D_plane_normal", p_normal))  // This is a double vector
      {
        ROS_WARN_STREAM("2D plane normal not set. Defaulting to world x-axis");
        p_normal.clear();
        p_normal.push_back(1.0);
        p_normal.push_back(0.0);
        p_normal.push_back(0.0);
      }
      if (!this->n_.getParam("/diabolo_2D_plane_point", p_point))  // This is a double vector
      {
        ROS_WARN_STREAM("Point on 2D plane not set. Defaulting to (0.7,0,0)");
        p_point.clear();
        p_point.push_back(0.7);
        p_point.push_back(0.0);
        p_point.push_back(0.0);
      }

      this->plane_normal_2D = ignition::math::Vector3d((p_normal.at(0)), (p_normal.at(1)), (p_normal.at(2)));
      this->plane_normal_2D = this->plane_normal_2D.Normalize();
      this->plane_point_2D = ignition::math::Vector3d((p_point.at(0)), (p_point.at(1)), (p_point.at(2)));
    }
  }
}

void DiaboloPredictor::step()
{
  // This step replaces gazebo's internal velocity and acceleration calculation
  // It treats the diabolo like a free body without constraints under gravity_
  this->perform_freebody_calculation();
  this->ellipse_current_velocity_ = this->get_ellipse_velocity();

  if (this->constrain_to_2D_flag)
  {
    this->constrain_to_2D(this->diabolo_current_position_, this->diabolo_current_velocity_);
  }
  // Store information about current ellipse
  this->store_ellipse_axes_lengths();
  // Store current ellipse transform
  this->store_ellipse_transform();
  ignition::math::Pose3d pose;
  pose.Set(this->diabolo_current_position_, ignition::math::Vector3d(0., 0., 0.));
  this->store_diabolo_transform(pose);

  // Get the diabolo state
  this->diabolo_state_ = this->get_diabolo_state();

  if (this->diabolo_state_ == OUTSIDE_STRING)
  {
    ignition::math::Vector3d diabolo_new_position_ = this->constrain_diabolo_position();
    if (!diabolo_new_position_.IsFinite())
    {
      ROS_ERROR_THROTTLE(2.0, "Constrained position is not finite. Not changing");
      diabolo_new_position_ = this->diabolo_current_position_;
    }

    // Get the max pull velocity
    ignition::math::Vector3d pull_velocity_cap = this->get_pull_velocity_cap();
    // Calculate the pull velocity using the new position and the old position
    this->pull_velocity_ = this->get_pull_velocity(ignition::math::Vector3d(this->diabolo_current_position_.X(),
                                                                            this->diabolo_current_position_.Y(),
                                                                            this->diabolo_current_position_.Z()),
                                                   diabolo_new_position_);

    this->diabolo_current_position_ = diabolo_new_position_;
    // Add the pull velocity to the current diabolo velocity

    // Make pull velocity for this time step 0 if it is not directed towards the center of the ellipse
    if (!this->pull_velocity_is_directed_inward())
    {
      this->pull_velocity_ = ignition::math::Vector3d(0, 0, 0);
    }

    if (this->pull_velocity_.Length() > pull_velocity_cap.Length())
    {
      this->pull_velocity_ = pull_velocity_cap;
    }
    if (this->pull_velocity_.Length() > 0.0)
    {
      ignition::math::Vector3d ellipse_y_axis =
          this->left_stick_current_position_ - this->right_stick_current_position_;
      double distance_between_sticks = (ellipse_y_axis).Length();
      if (this->string_length_ - this->pv_string_taut_tolerance_ <= distance_between_sticks)
      {
        this->apply_edge_case_pv_constraint(this->pull_velocity_);
      }
    }
    this->pull_velocity_ = this->pull_velocity_ * this->pv_post_cap_scaling_factor_;

    // Remove the positive component of the diabolo's velocity along the
    // positive direction of the surface normal of the ellipse

    this->diabolo_current_velocity_ = this->constrain_diabolo_velocity(
        this->diabolo_current_velocity_ * this->velocity_diffusion_factor_ + this->pull_velocity_);
    if (this->diabolo_last_state_ == ON_STRING || this->diabolo_last_state_ == OUTSIDE_STRING)
    {
      this->diabolo_current_rot_velocity_ += this->get_rot_velocity_change();
    }
  }

  this->diabolo_last_position_ = this->diabolo_current_position_;
  this->diabolo_last_state_ = this->diabolo_state_;
}

void DiaboloPredictor::store_ellipse_transform()
{
  ignition::math::Vector3d ellipse_y_axis = this->left_stick_current_position_ - this->right_stick_current_position_;
  double distance_between_focii = ellipse_y_axis.Length();
  ellipse_y_axis = ellipse_y_axis.Normalize();
  ignition::math::Vector3d y_axis(0, 1, 0);
  ignition::math::Vector3d rotation_axis = y_axis.Cross(ellipse_y_axis);
  rotation_axis = rotation_axis.Normalize();

  // This is the cosine of the angle between the y axis (world frame) and the y
  // axis of the ellipse
  double rotation_angle = y_axis.Dot(ellipse_y_axis);

  ignition::math::Quaternion<double> q;
  q.Axis(rotation_axis, acos(rotation_angle));

  ignition::math::Vector3d center = this->left_stick_current_position_ + this->right_stick_current_position_;
  center /= 2.0;

  this->ellipse_transform_.setOrigin(tf::Vector3(center.X(), center.Y(), center.Z()));
  this->ellipse_transform_.setRotation(tf::Quaternion(q.X(), q.Y(), q.Z(), q.W()));
}
void DiaboloPredictor::store_ellipse_axes_lengths()
{
  ignition::math::Vector3d between_focii = this->left_stick_current_position_ - this->right_stick_current_position_;
  double distance_between_focii = between_focii.Length();

  this->ellipse_major_axis_length_ = this->string_length_ / 2.0;  // From definition of ellipse
  this->ellipse_minor_axis_length_ =
      sqrt(pow(this->ellipse_major_axis_length_, 2.0) - pow((distance_between_focii / 2.0), 2.0));
}
ignition::math::Vector3d DiaboloPredictor::get_ellipse_velocity()
{
  ignition::math::Vector3d last_origin = (this->left_stick_last_position_ + this->right_stick_last_position_) / 2.0;
  ignition::math::Vector3d current_origin =
      (this->left_stick_current_position_ + this->right_stick_current_position_) / 2.0;

  ignition::math::Vector3d velo = (current_origin - last_origin) / this->time_step_;

  return velo;
}

ignition::math::Vector3d DiaboloPredictor::constrain_to_2D(ignition::math::Vector3d& pos, ignition::math::Vector3d& vel)
{
  // This function constrains the diabolo position and velocity to the desired 2D plane
  this->constrain_position_to_2D(pos);
  this->constrain_velocity_to_2D(vel);
}
ignition::math::Vector3d DiaboloPredictor::constrain_position_to_2D(ignition::math::Vector3d& pos)
{
  // This function projects the diabolo position onto the set 2D plane
  // new_pos = old_pos - projection of (old_pos - plane_point) on the plane normal
  // projection of (old_pos - plane_point) on the plane normal = p_c_normal
  ignition::math::Vector3d p_c_normal = pos - this->plane_point_2D;  // Intermediate step
  p_c_normal = p_c_normal.Dot(this->plane_normal_2D) * this->plane_normal_2D;

  pos = pos - p_c_normal;
}
ignition::math::Vector3d DiaboloPredictor::constrain_velocity_to_2D(ignition::math::Vector3d& vel)
{
  // This function constrains the velocity vel to be parallel to the plane, by removing its component parallel to the
  // plane normal

  vel = vel - vel.Dot(this->plane_normal_2D) * this->plane_normal_2D;
}
int DiaboloPredictor::get_diabolo_state()
{
  // Description:
  // The diabolo has 4 possible states: ON_STRING, OUTSIDE_STRING, OFF_STRING_LOOSE, FLYING
  // ON_STRING: The diabolo is exactly on the string. This state is rare
  // OFF_STRING_LOOSE: The diabolo is off the string, but not so far off that it cannot be caught easily
  //                  i.e. This is when the diabolo jumps, but does not pass the plane containing the y-axis of the
  //                  ellipse and the cross product of the Z-axis in the world frame and the y-axis of the ellipse,
  //                  passing through the stick tip positions
  // FLYING: The diabolo has left the string and crossed the plane described above
  // OUTSIDE_STRING: This is when the diabolo needs to be constrained to the ellipse
  // Transitions
  // Allowed transitions: ON_STRING -> OFF_STRING_LOOSE, OFF_STRING_LOOSE -> ON_STRING, ON_STRING -> FLYING,
  // OFF_STRING_LOOSE -> FLYING FLYING -> ON_STRING is allowed ONLY if the string is (close to) taut
  double distance_between_sticks = this->left_stick_current_position_.Distance(this->right_stick_current_position_);

  bool string_taut_flag = false;
  if (this->string_length_ - this->catching_string_taut_tolerance_ <= distance_between_sticks)
  {
    string_taut_flag = true;
    // ROS_WARN("String is taut enough to catch diabolo");
  }

  double distance_to_stick_1 = this->diabolo_current_position_.Distance(this->left_stick_current_position_);
  double distance_to_stick_2 = this->diabolo_current_position_.Distance(this->right_stick_current_position_);
  double distance_to_sticks = distance_to_stick_1 + distance_to_stick_2;
  ////////////////// Start check if diabolo is flying

  // Get normal to the plane containing the ellipse y axis in the world frame and the world frame x axis
  ignition::math::Vector3d ellipse_y_axis = this->left_stick_current_position_ - this->right_stick_current_position_;
  ignition::math::Vector3d plane_normal;
  if (ellipse_y_axis.Cross(ignition::math::Vector3d(1.0, 0.0, 0.0)).Length() == 0.)
  {
    // If the ellipse is y-axis is parallel to the ground
    // If ellipse y axis parallel to world x axis, plane normal is straight up
    plane_normal = ignition::math::Vector3d(0.0, 0.0, 1.0);
  }
  // Otherwise,
  // The normal to this plane is e_t x (e_z x z_world) where ey = ellipse y axis in world frame, z_world = world frame z
  // axis The diabolo changes to state flying if it is "above" this plane
  else
  {
    plane_normal = ignition::math::Vector3d(1.0, 0.0, 0.0).Cross(ellipse_y_axis);
  }

  plane_normal = plane_normal.Normalize();
  // Get the vector from the ellipse center to the diabolo position in the world frame
  // d_ellipse = d_world - e_world
  ignition::math::Vector3d ellipse_to_diabolo_vec =
      this->diabolo_current_position_ -
      (this->left_stick_current_position_ + this->right_stick_current_position_) / 2.0;
  ellipse_to_diabolo_vec = ellipse_to_diabolo_vec.Normalize();
  // If the dot product of this vector and the plane_normal > 0.0, the diabolo is FLYING
  double check_val = ellipse_to_diabolo_vec.Dot(plane_normal);
  if (check_val > 0.0 && this->diabolo_between_sticks() &&
      (this->diabolo_current_position_.Z() > this->left_stick_current_position_.Z() ||
       this->diabolo_current_position_.Z() > this->right_stick_current_position_.Z()))
  {
    return FLYING;
  }
  // This is the edge case, when the ellipse is close to a line
  if (string_taut_flag)
  {
    if (this->diabolo_last_state_ == FLYING)  // Can transition to OFF_STRING_LOOSE if string is taut
    {
      // Check if the diabolo is between the stick tips
      if (this->diabolo_between_sticks())
      {
        // Check if diabolo is close enough to the line between sticks
        // Get distance from diabolo to line between stick tips
        ignition::math::Vector3d center_to_diabolo =
            this->diabolo_current_position_ -
            (this->left_stick_current_position_ + this->right_stick_current_position_) / 2.0;
        ignition::math::Vector3d axis_to_diabolo =
            center_to_diabolo - center_to_diabolo.Dot(ellipse_y_axis.Normalize()) * ellipse_y_axis.Normalize();
        double diabolo_distance_to_axis = axis_to_diabolo.Length();
        if (diabolo_distance_to_axis < 0.1)  // Catch if distance to the axis is less than 10 cm
        {
          return OFF_STRING_LOOSE;
        }
        else
        {
           ROS_WARN_STREAM("Unable to catch diabolo. Distance to string = " << diabolo_distance_to_axis);
        }
      }
    }
  }
  if ((this->string_length_ - (distance_to_sticks)) > 0.)  // The diabolo is not on the string, but it is not flying
  {
    if (this->diabolo_last_state_ != FLYING)
      return OFF_STRING_LOOSE;  // The diabolo has just "hopped" off the string. It can still get back on the string
                                // without the string being taut

    else
      return FLYING;
  }

  else if ((distance_to_sticks - this->string_length_) > 0.)  // The string is taut and the diabolo needs position
                                                              // correction
  {
    // First the case if the last state was OFF_STRING_LOOSE
    if (this->diabolo_last_state_ != FLYING)
    {
      // Can transition to OUTSIDE_STRING state in this case
      return OUTSIDE_STRING;
    }
    else
      return FLYING;
  }

  return ON_STRING;
}

bool DiaboloPredictor::diabolo_between_sticks()
{
  ignition::math::Vector3d left_stick_to_diabolo, right_stick_to_diabolo;
  left_stick_to_diabolo = this->left_stick_current_position_ - this->diabolo_current_position_;
  right_stick_to_diabolo = this->right_stick_current_position_ - this->diabolo_current_position_;

  double prod = left_stick_to_diabolo.Dot(right_stick_to_diabolo);

  if (prod > 0)
  {
    return false;
  }

  if (prod <= 0)
  {
    return true;
  }
}

ignition::math::Vector3d DiaboloPredictor::constrain_diabolo_position()
{
  // Use parametric equation of ellipse
  // Calculate angles made with axes and solve for distance from center. Then
  // find new coordinates
  // Angle with z axis: phi
  // Angle with x axis: theta

  tf::Vector3 world_frame_pos(this->diabolo_current_position_.X(), this->diabolo_current_position_.Y(),
                              this->diabolo_current_position_.Z());

  tf::Vector3 ellipse_frame_pos = this->ellipse_transform_.inverse() * world_frame_pos;

  ignition::math::Vector3d ellipse_frame_pos_vector(ellipse_frame_pos.getX(), ellipse_frame_pos.getY(),
                                                    ellipse_frame_pos.getZ());

  ignition::math::Vector3d world_position;

  // Angle with xy plane
  if (ellipse_frame_pos_vector.X() == 0 && ellipse_frame_pos_vector.Y() == 0 && ellipse_frame_pos_vector.Z() == 0)
  {
    tf::Vector3 new_ellipse_frame_pos(0, 0, 0);
    world_frame_pos = this->ellipse_transform_ * new_ellipse_frame_pos;

    world_position = ignition::math::Vector3d(world_frame_pos.getX(), world_frame_pos.getY(), world_frame_pos.getZ());
  }
  else
  {
    double cos_phi = ellipse_frame_pos_vector.Z() /
                     sqrt(pow(ellipse_frame_pos_vector.X(), 2.0) + pow(ellipse_frame_pos_vector.Y(), 2.0) +
                          pow(ellipse_frame_pos_vector.Z(), 2.0));
    double sin_phi = sqrt(pow(ellipse_frame_pos_vector.X(), 2.0) + pow(ellipse_frame_pos_vector.Y(), 2.0)) /
                     sqrt(pow(ellipse_frame_pos_vector.X(), 2.0) + pow(ellipse_frame_pos_vector.Y(), 2.0) +
                          pow(ellipse_frame_pos_vector.Z(), 2.0));

    double length = (sqrt(pow(ellipse_frame_pos_vector.X(), 2.0) + pow(ellipse_frame_pos_vector.Y(), 2.0)));
    double new_x_ellipse_frame, new_y_ellipse_frame, new_z_ellipse_frame, r;
    if (!length == 0.0)
    {
      double cos_theta = ellipse_frame_pos_vector.X() /
                         (sqrt(pow(ellipse_frame_pos_vector.X(), 2.0) + pow(ellipse_frame_pos_vector.Y(), 2.0)));
      double sin_theta = ellipse_frame_pos_vector.Y() /
                         (sqrt(pow(ellipse_frame_pos_vector.X(), 2.0) + pow(ellipse_frame_pos_vector.Y(), 2.0)));

      r = sqrt(1.0 / ((pow(cos_theta, 2.0) * pow(sin_phi, 2.0)) / pow(this->ellipse_minor_axis_length_, 2.0) +
                      (pow(sin_theta, 2.0) * pow(sin_phi, 2.0)) / pow(this->ellipse_major_axis_length_, 2.0) +
                      (pow(cos_phi, 2.0)) / pow(this->ellipse_minor_axis_length_, 2.0)));

      // New position of diabolo
      new_x_ellipse_frame = r * cos_theta * sin_phi;
      new_y_ellipse_frame = r * sin_theta * sin_phi;
      new_z_ellipse_frame = r * cos_phi;
    }

    else
    {
      r = this->ellipse_minor_axis_length_;
      new_x_ellipse_frame = 0.;
      new_y_ellipse_frame = 0.;
      new_z_ellipse_frame = r * cos_phi;
    }
    tf::Vector3 new_ellipse_frame_pos(new_x_ellipse_frame, new_y_ellipse_frame, new_z_ellipse_frame);
    world_frame_pos = this->ellipse_transform_ * new_ellipse_frame_pos;

    world_position = ignition::math::Vector3d(world_frame_pos.getX(), world_frame_pos.getY(), world_frame_pos.getZ());
  }

  return world_position;
}
ignition::math::Vector3d DiaboloPredictor::get_pull_velocity_cap()
{
  // This cap is 0 in some cases, so this function determines
  // if the pull velocity is applied at all.
  // The cap is 0 e.g. when the sticks are moving in the same direction as the diabolo

  ignition::math::Vector3d origin_velocity_cap;
  ignition::math::Vector3d minor_axis_cap;
  // Initially get the vector along which the pull velocity will be directed.
  // It is from the current diabolo position to the center of the ellipse
  ignition::math::Vector3d current_center =
      (this->left_stick_current_position_ + this->right_stick_current_position_) / 2.0;
  ignition::math::Vector3d last_center = (this->left_stick_last_position_ + this->right_stick_last_position_) / 2.0;
  ignition::math::Vector3d direction_vector = this->get_ellipse_normal_in_world_frame(this->diabolo_current_position_);
  direction_vector = direction_vector.Normalize();
  direction_vector = -direction_vector;
  ignition::math::Vector3d ellipse_origin_change = current_center - last_center;

  if (direction_vector.Dot(ellipse_origin_change) > 0.0)  // If the change in origin velocity has a positive component
                                                          // along the direction_vector direction
  {
    origin_velocity_cap = direction_vector * (ellipse_origin_change.Length() / this->time_step_);
  }

  else
  {
    origin_velocity_cap = ignition::math::Vector3d(0, 0, 0);
  }

  // Now get the velocity due to the change of length of the minor axis
  double last_minor_axis =
      this->get_ellipse_minor_axis_length(this->left_stick_last_position_, this->right_stick_last_position_);
  double current_minor_axis =
      this->get_ellipse_minor_axis_length(this->left_stick_current_position_, this->right_stick_current_position_);

  if (current_minor_axis <= last_minor_axis)  // If the minor axis length has decreased
  {
    minor_axis_cap = direction_vector * ((last_minor_axis - current_minor_axis) / this->time_step_);
  }
  else
  {
    minor_axis_cap = ignition::math::Vector3d(0, 0, 0);
  }


  return (origin_velocity_cap + minor_axis_cap) * this->pv_cap_scaling_factor_;
}

double DiaboloPredictor::get_ellipse_minor_axis_length(ignition::math::Vector3d foci_1, ignition::math::Vector3d foci_2)
{
  ignition::math::Vector3d between_focii = foci_1 - foci_2;
  double distance_between_focii = between_focii.Length();

  double major_axis_length = this->string_length_ / 2.0;  // From definition of ellipse
  double minor_axis_length_ = sqrt(pow(major_axis_length, 2.0) - pow((distance_between_focii / 2.0), 2.0));

  return minor_axis_length_;
}
ignition::math::Vector3d
DiaboloPredictor::get_ellipse_normal_in_world_frame(ignition::math::Vector3d world_frame_pos_vec)
{
  tf::Vector3 world_frame_pos(world_frame_pos_vec.X(), world_frame_pos_vec.Y(), world_frame_pos_vec.Z());

  tf::Vector3 ellipse_frame_pos = this->ellipse_transform_.inverse() * world_frame_pos;

  // Get normal to ellipse at diabolo's location

  tf::Vector3 ellipse_frame_normal(
      2.0 * (ellipse_frame_pos.getX() / this->ellipse_minor_axis_length_),
      2.0 * (ellipse_frame_pos.getY() / this->ellipse_major_axis_length_),  // major axis is aligned along y direction
      2.0 * (ellipse_frame_pos.getZ() / this->ellipse_minor_axis_length_));

  ellipse_frame_normal = ellipse_frame_normal.normalize();

  // Want only the rotation (not the position) transform of the normal vector
  // Using quaternion rotation
  // v_new_as_quat = Q * v_old_as_quat * Q_inverse
  tf::Quaternion ellipse_to_world_rot =
      this->ellipse_transform_.getRotation();  // Rotation to rotate from ellipse frame to world frame
  tf::Quaternion ellipse_frame_normal_as_quat(ellipse_frame_normal.x(), ellipse_frame_normal.y(),
                                              ellipse_frame_normal.z(), 0.0);

  // Using quaternion rotataion identity v' = qvq*
  tf::Quaternion world_frame_normal_as_quat = ellipse_to_world_rot * ellipse_frame_normal_as_quat;
  world_frame_normal_as_quat = world_frame_normal_as_quat * ellipse_to_world_rot.inverse();

  // Ignoring the w term gives us the new vector
  ignition::math::Vector3d world_frame_normal_vector(
      world_frame_normal_as_quat.getX(), world_frame_normal_as_quat.getY(), world_frame_normal_as_quat.getZ());
  return world_frame_normal_vector.Normalize();
}
ignition::math::Vector3d DiaboloPredictor::get_pull_velocity(ignition::math::Vector3d old_position,
                                                             ignition::math::Vector3d new_position)
{
  ignition::math::Vector3d v_pull = new_position - old_position;
  v_pull = v_pull / this->time_step_;
  ignition::math::Vector3d normal_dir = this->get_ellipse_normal_in_world_frame(new_position);
  normal_dir = -normal_dir;
  v_pull = v_pull.Dot(normal_dir) * normal_dir * this->pv_pre_cap_scaling_factor_;
  return v_pull;
}
bool DiaboloPredictor::pull_velocity_is_directed_inward()
{
  // Get position of ellipse center in world frame
  // Get vector from diabolo to ellipse center
  // Find dot product of calculated pull velocity and the above vector.
  // Return false if dot product is negative, and true if positive

  ignition::math::Vector3d center = (this->left_stick_current_position_ + this->right_stick_current_position_) / 2.0;
  ignition::math::Vector3d direction_vector = center - this->diabolo_current_position_;

  double dot_product = direction_vector.Dot(this->pull_velocity_);
  if (dot_product >= 0)
  {
    return true;
  }

  else
  {
    return false;
  }
}
void DiaboloPredictor::apply_edge_case_pv_constraint(ignition::math::Vector3d& v_pull)
{
  // This is used when the sticks are close to taut, and the ellipse is very thin.
  // Throwing the diabolo in 3D can cause the diabolo to be pulled forward/backward relative to the player,
  // as the curvature changes 

  ignition::math::Vector3d ellipse_y_axis = this->left_stick_current_position_ - this->right_stick_current_position_;
  // Get normal to the plane containing ellipse y axis and world z axis
  ignition::math::Vector3d normal = ellipse_y_axis.Cross(ignition::math::Vector3d(0.0, 0.0, 1.0));
  normal = normal.Normalize();
  double v_pull_mag = v_pull.Length();
  // Get component of the pull velocity in the direction of the normal
  ignition::math::Vector3d v_pull_in_normal_direction = v_pull.Dot(normal) * normal;
  v_pull = (v_pull - v_pull_in_normal_direction).Normalize() * v_pull_mag;  // keep the magnitude of the pull velocity
}
ignition::math::Vector3d DiaboloPredictor::constrain_diabolo_velocity(ignition::math::Vector3d diabolo_velocity)
{
  ignition::math::Vector3d world_frame_normal_vector =
      this->get_ellipse_normal_in_world_frame(this->diabolo_current_position_);

  ignition::math::Vector3d new_velocity, velocity_in_normal_dir;
  double speed_in_normal_dir = diabolo_velocity.Dot(world_frame_normal_vector);
  if (speed_in_normal_dir > 0.0)  // If the diabolo velocity has a positive
                                  // component in the direction of the surface
                                  // normal
  {
    // Project the velocity along the tangent to the ellipse
    velocity_in_normal_dir = world_frame_normal_vector * speed_in_normal_dir;
    new_velocity = diabolo_velocity - velocity_in_normal_dir;
  }

  else
  {
    // If the component of diabolo velocity in the direction of the ellipse
    // normal is negative, no change
    new_velocity = diabolo_velocity;
  }
  return new_velocity;
}
double DiaboloPredictor::get_rot_velocity_change()
{
  // The length of string that moved past the diabolo in the last time step
  double old_dist_to_right_stick = this->right_stick_last_position_.Distance(this->diabolo_last_position_);
  double new_dist_to_right_stick = this->right_stick_current_position_.Distance(this->diabolo_current_position_);

  double string_velo = (new_dist_to_right_stick - old_dist_to_right_stick) / (this->time_step_);
  // This is the speed of the diabolo axle at axle_radius distance from the center
  double axle_velo = this->diabolo_current_rot_velocity_ * this->diabolo_axle_radius_;

  return (((string_velo - axle_velo) * this->rot_friction_factor_) / this->diabolo_axle_radius_);
}
void DiaboloPredictor::perform_freebody_calculation()
{
  this->diabolo_current_position_ = this->diabolo_current_position_ +
                                    this->diabolo_current_velocity_ * this->time_step_ +
                                    this->gravity_ * this->time_step_ * this->time_step_ * 0.5;

  this->diabolo_current_velocity_ = this->diabolo_current_velocity_ + this->gravity_ * this->time_step_;
}

void DiaboloPredictor::store_diabolo_transform(ignition::math::Pose3d pose_)
{
  this->diabolo_transform_.setOrigin(tf::Vector3(pose_.Pos().X(), pose_.Pos().Y(), pose_.Pos().Z()));
  this->diabolo_transform_.setRotation(
      tf::Quaternion(pose_.Rot().X(), pose_.Rot().Y(), pose_.Rot().Z(), pose_.Rot().W()));
}
void DiaboloPredictor::publish_diabolo_frame_to_tf()
{
  tf_broadcaster_->sendTransform(
      tf::StampedTransform(this->diabolo_transform_, ros::Time::now(), "world", "diabolo_frame"));
}

void DiaboloPredictor::publish_ellipse_frame_to_tf()
{
  tf_broadcaster_->sendTransform(
      tf::StampedTransform(this->ellipse_transform_, ros::Time::now(), "world", "ellipse_frame"));
}
visualization_msgs::Marker DiaboloPredictor::make_marker_(std::string const& mesh_filename, std::string const& frame_id,
                                                          std::string const& ns, int type,
                                                          ignition::math::Vector3<float> scale, float col[])
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = this->marker_count_++;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.mesh_resource = mesh_filename;
  marker.scale.x = scale.X();
  marker.scale.y = scale.Y();
  marker.scale.z = scale.Z();
  marker.color.r = col[0];
  marker.color.g = col[1];
  marker.color.b = col[2];
  marker.color.a = col[3];

  return marker;
}
void DiaboloPredictor::publish_visualization_markers()
{
  visualization_msgs::Marker marker;
  this->marker_array.markers.clear();
  ros::Time now = ros::Time::now();
  float c[4];
  // Orange
  c[0] = (255.0 / 255.0);
  c[1] = (165.0 / 255.0);
  c[2] = 0.0f;
  c[3] = 1.0f;

  marker = this->make_marker_("package://diabolo_scene_description/meshes/diabolo_shell.stl", "/world", "diabolo",
                              visualization_msgs::Marker::MESH_RESOURCE,
                              ignition::math::Vector3<float>(0.001, 0.001, 0.001), c);
  marker.pose.position.x = this->diabolo_current_position_.X();
  marker.pose.position.y = this->diabolo_current_position_.Y();
  marker.pose.position.z = this->diabolo_current_position_.Z();
  this->marker_array.markers.push_back(marker);

  c[0] = 0.7f;
  c[1] = 0.7f;
  c[2] = 0.7f;
  c[3] = 1.0f;

  marker = this->make_marker_("package://diabolo_scene_description/meshes/diabolo_axis.stl", "/world", "diabolo",
                              visualization_msgs::Marker::MESH_RESOURCE,
                              ignition::math::Vector3<float>(0.001, 0.001, 0.001), c);
  marker.pose.position.x = this->diabolo_current_position_.X();
  marker.pose.position.y = this->diabolo_current_position_.Y();
  marker.pose.position.z = this->diabolo_current_position_.Z();
  this->marker_array.markers.push_back(marker);

  c[0] = 0.1f;
  c[1] = 0.1f;
  c[2] = 0.1f;
  c[3] = 1.0f;

  marker = this->make_marker_("package://diabolo_scene_description/meshes/diabolo_fixators.stl", "/world", "diabolo",
                              visualization_msgs::Marker::MESH_RESOURCE,
                              ignition::math::Vector3<float>(0.001, 0.001, 0.001), c);
  marker.pose.position.x = this->diabolo_current_position_.X();
  marker.pose.position.y = this->diabolo_current_position_.Y();
  marker.pose.position.z = this->diabolo_current_position_.Z();
  this->marker_array.markers.push_back(marker);

  c[0] = 1.0f;
  c[1] = 0.0f;
  c[2] = 0.0f;
  c[3] = 1.0f;

  marker = this->make_marker_("", "/world", "predicted_left_stick", visualization_msgs::Marker::SPHERE,
                              ignition::math::Vector3<float>(0.1, 0.1, 0.1), c);
  marker.pose.position.x = this->left_stick_current_position_.X();
  marker.pose.position.y = this->left_stick_current_position_.Y();
  marker.pose.position.z = this->left_stick_current_position_.Z();
  this->marker_array.markers.push_back(marker);
  c[0] = 0.0f;
  c[1] = 0.0f;
  c[2] = 1.0f;
  c[3] = 1.0f;
  marker = this->make_marker_("", "/world", "predicted_right_stick", visualization_msgs::Marker::SPHERE,
                              ignition::math::Vector3<float>(0.1, 0.1, 0.1), c);
  marker.pose.position.x = this->right_stick_current_position_.X();
  marker.pose.position.y = this->right_stick_current_position_.Y();
  marker.pose.position.z = this->right_stick_current_position_.Z();
  this->marker_array.markers.push_back(marker);
  this->marker_array_pub_.publish(marker_array);

}
///////////////////////////////////////////////// End of Predictor

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diabolo_motion_generator");
  DiaboloMotionGenerator md;
  ros::AsyncSpinner spinner(2);

  spinner.start();
  ros::waitForShutdown();
  return 0;
}
