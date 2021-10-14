#ifndef DIABOLO_PLAYBACK_H
#define DIABOLO_PLAYBACK_H

#include "ros/ros.h"
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"

#include "helper_functions.h"

#include <tf/transform_listener.h>    // Includes the TF conversions
#include <tf/transform_broadcaster.h>

#include <chrono>
#include <thread>
#include <cmath>
#include <boost/thread/mutex.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_state/conversions.h>

#include "geometry_msgs/PoseStamped.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "visualization_msgs/Marker.h"


class DiaboloPlayback
{
public:
  //Constructor
  DiaboloPlayback();
  
  //Helpers (convenience functions)
  bool setRobotState(std::vector<double> joint_positions, std::string robot_name);
  bool setRobotState(geometry_msgs::PoseStamped target_pose, std::string robot_name);
  bool goToNamedPose(std::string pose_name, std::string robot_name, double speed = 1.0, double acceleration = 0.0, bool use_UR_script=false);
  bool updatePlanningScene();
  bool publishRobotState(moveit::core::RobotState robot_state);
  moveit::planning_interface::MoveGroupInterface* robotNameToMoveGroup(std::string robot_name);
  
  // Callback declarations
  void setRobotPoseLeft(const geometry_msgs::PoseConstPtr msg);
  void setRobotPoseRight(const geometry_msgs::PoseConstPtr msg);
  
  bool setRobotPose(const geometry_msgs::PoseConstPtr msg, std::string robot_name);

  bool setInitialPosition(geometry_msgs::Point left_stick_position, geometry_msgs::Point right_stick_position);
// private:
  ros::NodeHandle n_;

  ros::Subscriber subLeftStickPose_, subRightStickPose_;
  ros::Publisher pubMarker_, pubRobotState_;
  
  // Status variables
  tf::TransformListener tflistener_;
  tf::TransformBroadcaster tfbroadcaster_;
  
  // MoveGroup connections
  moveit_msgs::PlanningScene planning_scene_;
  moveit::core::RobotState robot_state_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::ServiceClient get_planning_scene_client;
  moveit::planning_interface::MoveGroupInterface a_bot_group_, b_bot_group_;
  
  
};//End of class DiaboloPlayback

#endif //DIABOLO_PLAYBACK_H
