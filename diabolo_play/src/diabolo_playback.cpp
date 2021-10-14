#include "diabolo_play/diabolo_playback.h"

DiaboloPlayback::DiaboloPlayback()
  : a_bot_group_("a_bot")
  , b_bot_group_("b_bot")
  ,
  robot_state_(robot_model_loader::RobotModelLoader("robot_description").getModel())

{
  // Topics to publish
  // pubMarker_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pubRobotState_ = n_.advertise<moveit_msgs::DisplayRobotState>("diabolo_robot_state", 10);

  // Topics to subscribe to
  subLeftStickPose_ = n_.subscribe("diabolo_stick_pose_left", 1, &DiaboloPlayback::setRobotPoseLeft, this);
  subRightStickPose_ = n_.subscribe("diabolo_stick_pose_right", 1, &DiaboloPlayback::setRobotPoseRight, this);

  // Set up MoveGroups
  a_bot_group_.setEndEffectorLink("a_bot_diabolo_stick_tip");
  b_bot_group_.setEndEffectorLink("b_bot_diabolo_stick_tip");
  
  ROS_INFO("Diabolo playback node has started up!");
}

bool DiaboloPlayback::setRobotState(std::vector<double> joint_positions, std::string robot_name)
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

void DiaboloPlayback::setRobotPoseRight(const geometry_msgs::PoseConstPtr msg)
{
  setRobotPose(msg, "a_bot");
}

void DiaboloPlayback::setRobotPoseLeft(const geometry_msgs::PoseConstPtr msg)
{
  setRobotPose(msg, "b_bot");
}

bool DiaboloPlayback::setRobotPose(const geometry_msgs::PoseConstPtr msg, std::string robot_name)
{
  const robot_state::JointModelGroup* joint_model_group = robot_state_.getJointModelGroup(robot_name);
  geometry_msgs::Pose pose_in_world(*msg);

  rotatePoseByRPY(0, M_PI, 0, pose_in_world);

  robot_state_.setFromIK(joint_model_group, pose_in_world, robot_name + "_diabolo_stick_tip");
  if (robot_name == "b_bot")  // The two poses are always published together, but we only need to update the state once
                              // per frame
    publishRobotState(robot_state_);
}

moveit::planning_interface::MoveGroupInterface* DiaboloPlayback::robotNameToMoveGroup(std::string robot_name)
{
  // This function converts the name of the robot to a pointer to the member variable containing the move group
  if (robot_name == "a_bot")
    return &a_bot_group_;
  if (robot_name == "b_bot")
    return &b_bot_group_;
}

// This forces a refresh of the planning scene.
bool DiaboloPlayback::updatePlanningScene()
{
  moveit_msgs::GetPlanningScene srv;
  // Request only the collision matrix
  srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  get_planning_scene_client.call(srv);
  if (get_planning_scene_client.call(srv))
  {
    ROS_INFO("Got planning scene from move group.");
    planning_scene_ = srv.response.scene;
    return true;
  }
  else
  {
    ROS_ERROR("Failed to get planning scene from move group.");
    return false;
  }
}

bool DiaboloPlayback::publishRobotState(moveit::core::RobotState robot_state)
{
  moveit_msgs::DisplayRobotState state_msg;
  moveit::core::robotStateToRobotStateMsg(robot_state, state_msg.state);
  pubRobotState_.publish(state_msg);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diabolo_playback_robot");
  ros::AsyncSpinner spinner(1);  // Needed for MoveIt to work.
  spinner.start();

  DiaboloPlayback ss;
  while (ros::ok())
  {
    ros::Duration(.01).sleep();
    ros::spinOnce();
  }

  return 0;
}
