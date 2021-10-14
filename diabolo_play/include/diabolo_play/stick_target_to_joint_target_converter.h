#ifndef STICK_TARGET_TO_JOINT_TARGET_CONVERTER_H
#define STICK_TARGET_TO_JOINT_TARGET_CONVERTER_H

#include "ros/ros.h"
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_srvs/SetBool.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sensor_msgs/JointState.h>


#include "helper_functions.h"

#include <tf/transform_listener.h>    // Includes the TF conversions
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <boost/thread/mutex.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_state/conversions.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "visualization_msgs/Marker.h"
#include <diabolo_play/SetInitialStickPositions.h>
#include <diabolo_play/CreateRobotTrajectory.h>
#include "bio_ik/bio_ik.h"

#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
class StickTargetToJointTargetConverter
{
public:
  //Constructor
  StickTargetToJointTargetConverter();
  
  //Helpers (convenience functions)
  bool setRobotState(std::vector<double> joint_positions, std::string robot_name);
  bool setRobotState(geometry_msgs::PoseStamped target_pose, std::string robot_name);
  bool goToNamedPose(std::string pose_name, std::string robot_name, double speed = 1.0, double acceleration = 0.0, bool use_UR_script=false);
  bool updatePlanningScene();
  bool sendRobotCommand(moveit::core::RobotState robot_state, std::string robot_name);
  bool publishRobotState(moveit::core::RobotState robot_state);
  bool publishRobotState(moveit::core::RobotState robot_state, ros::Publisher& publisher);
  moveit::planning_interface::MoveGroupInterface* robotNameToMoveGroup(std::string robot_name);
  geometry_msgs::Pose getPoseFromPoint(const geometry_msgs::Point point, std::string left_or_right_arm);
  geometry_msgs::Pose getPoseFromPoint(const geometry_msgs::PointConstPtr point_msg, std::string left_or_right_arm);
  // Callback declarations
  void setRobotPoseCallback(const geometry_msgs::PoseArrayConstPtr msg);
  void storeJointStates(const sensor_msgs::JointStateConstPtr msg);

  bool setInitialPosition(diabolo_play::SetInitialStickPositions::Request& request, diabolo_play::SetInitialStickPositions::Response& response);
  bool sendRobotTrajectory(diabolo_play::CreateRobotTrajectory::Request& request, diabolo_play::CreateRobotTrajectory::Response& response);
  bool calculate_trajectory_acceleration_and_velocity(trajectory_msgs::JointTrajectory& traj, int start_point);
  bool calculate_trajectory_acceleration_and_velocity(trajectory_msgs::JointTrajectory& traj, 
                                                      std::vector<PiecewiseSpline>& traj_splines,
                                                      int start_point);
  bool apply_bounds(const robot_model::RobotModel& rmodel, 
                    trajectory_msgs::JointTrajectory& traj,
                    std::vector<std::string>& joint_names);
  bool setRobotPose(const geometry_msgs::Pose pose_in_world, std::string robot_name);
  bool setRobotStateIK(moveit::core::RobotState& robot_state, const robot_state::JointModelGroup* jmg, const geometry_msgs::Pose pose_in_world, std::string robot_name);
  bool checkStateValidity(moveit::core::RobotState* robot_state,const robot_state::JointModelGroup* joint_group, const double* joint_group_variable_values);

  visualization_msgs::Marker make_marker_(std::string const &mesh_filename, std::string const &frame_id, tf2::Vector3 position, tf2::Quaternion orientation, 
                                                         std::string const &ns, int type,
                                                         float scale[], float col[]);
  void publishMarkers();
  /// \brief Store the stick tip orientations for the corners of the boundary of robot arm movement
  void store_corner_orientations();

  /// \brief Store joint limits for each robot. Called only once in the constructor
  void store_joint_limits();

  /// \brief Fit the points in a trajectory to a piecewise spline object
  // Parameters: 
  // splines: Empty vector for the spline objects. The function clears the vector beforehand
  // initial_joint_angles: A vector containing the initial joint positions. These are taken as the first knot point of the spline
  // traj: The joint trajectory
  bool get_splines_from_trajectory(std::vector<PiecewiseSpline>& splines,
                                                           double initial_joint_angles[], 
                                                           const trajectory_msgs::JointTrajectory& traj);
  
  /// \brief Read interpolation data points from diabolo_moveit_config/config_data if interpolation_flag is set to true
  void store_interpolation_data(); 
  
  /// \brief Create new interpolation data by using data red in from csv
  void create_new_interpolation_data(std::vector<double> a_bot_x_coords, std::vector<double> a_bot_y_coords, std::vector<double> a_bot_z_coords,
                                     std::vector<double> b_bot_x_coords, std::vector<double> b_bot_y_coords, std::vector<double> b_bot_z_coords); 
  
  /// \brief // Publish the joint state data to interpolate from as robot state markers 
  void publish_interpolation_data_markers(); 
  
  /// \brief Publish the interpolated seed state
  void publish_interpolated_seed_state(moveit::core::RobotState robot_state, std::string robot_name);
  
  /// \brief Store joint limits for a given joint model group. Each one will initially be loaded with the limits from the urdf, so this must be overwritten
  void setJointLimits(robot_state::JointModelGroup* jmg, std::string robot_name);
  
  /// \brief Function to keep the the wrist 2 link parallel to the ground using a custom joint variable function
  
  // These are seperate function to avoid having to define another class variable to store the robot currently being calculated for
  void a_bot_wrist2_parallel_to_ground(std::vector<double>& joint_variables);
  void b_bot_wrist2_parallel_to_ground(std::vector<double>& joint_variables);
  void a_bot_wrist3_zero(std::vector<double>& joint_variables);
  void b_bot_wrist3_zero(std::vector<double>& joint_variables);
  
  std::vector<double> reg_scaling_factors; // Weights for weighted_regularization_function in correct order
                                              // These should ideally be passed to function directly, but solver expects function with one variable
  double current_joint_angles[6];
   ros::Timer timer;   
  void debug_bioik_weights(bio_ik::BioIKKinematicsQueryOptions& options, std::string robot_name);
  std::string getRobotNamefromJMG(const robot_state::JointModelGroup* jmg);
  ros::ServiceServer set_to_initial_service, command_robot_traj_from_stick_traj_service;
  void checkStickPosePubRate(const ros::TimerEvent& event);
  void fill_trajectory(trajectory_msgs::JointTrajectory& a_bot_traj, ros::Duration time_step, int trajectory_steps);
// private:
  ros::NodeHandle n_;
  ros::Subscriber subStickPos_;
  ros::Subscriber jointStateSub_; // A subscriber to keep track of the real robot joint states
  //ros::Subscriber subLeftStickPose_, subRightStickPose_;
  ros::Publisher pubMarkerArray_, pubRobotState_, pubCommandA_, pubCommandB_;
  // Seperate publishers for every robot state
  ros::Publisher interpolate_state_markers_pub_, seed_state_markers_pub_;
  ros::Publisher pubJointFuncEval_, pubPositionEval_, pubOrientEval_; // DEBUG: To check evaluation values for bio_ik solver goals 
  
  // Status variables
  tf::TransformListener tflistener_;
  tf::TransformBroadcaster tfbroadcaster_;
  
  //Goal weights
  double position_goal_weight;
  double orientation_goal_weight;  
  double min_joint_disp_goal_weight;
  
  // The tolerance around the goal position. 
  // bio_ik will consider the postion goal cost = 0 if the solution is inside a sphere with:
  // radius = position_goal_tolerance
  // center at the goal position
  double position_goal_tolerance; 
  
  // Joint limits for either robot. 
  // 6 rows and 2 columns for each. 
  // Columns : [0] = lower bound, [1] = upper bound
  // Rows: [0] = shoulder pan joint, [1] = shoulder_lift_joint, [2] = elbow_joint, [3] = wrist_1_joint, [4] = wrist_2_joint, [5] = wrist_3_joint
  double a_bot_joint_limits_[6][2], b_bot_joint_limits_[6][2];
  std::vector<std::vector<double>> a_bot_interpolation_data, b_bot_interpolation_data;
  std::vector<std::vector<double>> a_bot_interpolation_corner_coords, b_bot_interpolation_corner_coords;
  double stick_pose_publish_rate; // The time between the last stick pose and the current stick pose
  bool interpolation_flag; // To use interpolated joint angle as ik seed if true, current state as initial seed if false
  
  // MoveGroup interfaces
  planning_scene::PlanningScene planning_scene_;
  moveit::core::RobotState robot_state_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::ServiceClient get_planning_scene_client;
  moveit::planning_interface::MoveGroupInterface a_bot_group_, b_bot_group_;
  
  // Stores the current joint state of the robot from the joint states topic
  sensor_msgs::JointState current_joint_state_;
  double a_bot_current_joint_angles_[6];
  double b_bot_current_joint_angles_[6];

  // This vector will contain the quaternions describing orientations at the corners of the region the arms are allowed to move in
  // The values are used for quaternion slerp
  // Order of storing values: 
  // [0 ~ 3] : left arm, [4 ~ 7] : right arm
  // 0 : bottom nearer boundary
  // 1 : bottom further boundary
  // 2 : top nearer boundary
  // 3 : top further boundary

  tf2::Quaternion corner_orientations[8];

  // This array stores the coordinates of the corners of the region the arms are allowed to move in
  // Order of storing values:
  // 0 : x_close, 1 : x_far, 2 : y_left, 3 : y_right, 4 : z_down, 5 : z_up
  double corner_coords[6]; 
  int marker_count;

  trajectory_msgs::JointTrajectory a_bot_seed_traj;
   ros::Publisher a_bot_display_seed_traj_pub;
};//End of class StickTargetToJointTargetConverter

#endif //STICK_TARGET_TO_JOINT_TARGET_CONVERTER_H
