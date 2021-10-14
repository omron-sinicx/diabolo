#include <gazebo_diaboloplugin.h>

#define ON_STRING 1
// Off the string and below at least one stick
#define OFF_STRING_LOOSE 2
// Off the string and above the sticks
#define FLYING 3
// Outside the border of the ellipse. Requires position correction
#define OUTSIDE_STRING 4
#define DEFAULT_STRING_LENGTH 1.46
#define DEFAULT_DIABOLO_MASS 0.2
#define DEFAULT_DIABOLO_AXIS_RADIUS 0.0065
// Relaxation error is the distance error allowable to determine the diabolo is
// on the string
// TODO Calculate the error from max speed of the diabolo. (Max distance it can
// travel in one time step)
#define DISTANCE_PAST_STRING_LENGTH 0.002
#define PI 3.141592653589793238

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(DiaboloPlugin)

DiaboloPlugin::DiaboloPlugin()
{
}
DiaboloPlugin::~DiaboloPlugin()
{
  this->updateConnection.reset();
  // Removes all callbacks from the queue. Does not wait for calls currently in progress to finish.
  this->rosQueue.clear();
  // Disable the queue, meaning any calls to addCallback() will have no effect.
  this->rosQueue.disable();
  this->rosNode->shutdown();  // This MUST BE CALLED before thread join()!!
  this->rosQueueThread.join();

  // delete this->rosNode; // Not required, since rosNode is a uniquePtr
}

void DiaboloPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;
  this->link_ = this->model->GetLink("axis");
  this->world = this->model->GetWorld();
  this->last_time = this->world->SimTime();
  this->time_step = 0.001;
  this->catching_string_taut_tolerance = 0.03;
  this->pv_string_taut_tolerance = 0.03;
  if (!this->link_)
  {
    printf("Could not get link");
  }
  else
  {
    std::cout << "Got link" << std::endl;
  }

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "diabolo_gazebo_node", ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("diabolo_gazebo_node"));

  // Create a named topic, and subscribe to it.
  if (!_sdf->HasElement("stick_pos_topic"))
  {
    ROS_FATAL("Plugin missing <stick_pos_topic> tag. Cannot proceed");
  }
  else
    this->stick_pos_topic_name_ = "/" + _sdf->GetElement("stick_pos_topic")->Get<std::string>();

  if (!_sdf->HasElement("string_length"))
  {
    ROS_WARN("Plugin missing <string_length> tag. Using default value");
    this->string_length_ = (double)DEFAULT_STRING_LENGTH;
  }
  else
    this->string_length_ = _sdf->GetElement("string_length")->Get<double>();

  if (!_sdf->HasElement("mass"))
  {
    ROS_WARN("Plugin missing <mass> tag. Using default value");
    this->mass_ = (double)DEFAULT_DIABOLO_MASS;
  }
  else
    this->mass_ = _sdf->GetElement("mass")->Get<double>();

  if (!_sdf->HasElement("axis_radius"))
  {
    ROS_WARN("Plugin missing <axis_radius> tag. Using default value");
    this->diabolo_axis_radius_ = (double)DEFAULT_DIABOLO_AXIS_RADIUS;
  }
  else
    this->diabolo_axis_radius_ = _sdf->GetElement("axis_radius")->Get<double>();

  // Get velocity damping factors from parameter server
  if (!this->rosNode->getParam("/pv_cap_scaling_factor", this->pv_cap_scaling_factor))
  {
    ROS_WARN_STREAM("Pull velocity cap scaling factor not set. Using default value = " << 1.0);
    this->pv_cap_scaling_factor = 1.0;
  }
  if (!this->rosNode->getParam("/pv_post_cap_scaling_factor", this->pv_post_cap_scaling_factor))
  {
    ROS_WARN_STREAM("Pull velocity post cap scaling factor not set. Using default value = " << 1.0);
    this->pv_post_cap_scaling_factor = 1.0;
  }
  if (!this->rosNode->getParam("/pv_pre_cap_scaling_factor", this->pv_pre_cap_scaling_factor))
  {
    ROS_WARN_STREAM("Pull velocity pre cap scaling factor not set. Using default value = " << 1.0);
    this->pv_pre_cap_scaling_factor = 1.0;
  }
  if (!this->rosNode->getParam("/constrained_velocity_scaling_factor", this->constrained_velocity_scaling_factor))
  {
    ROS_WARN_STREAM("Constrained velocity scaling factor not set. Using default value = " << 1.0);
    this->constrained_velocity_scaling_factor = 1.0;
  }
  if (!this->rosNode->getParam("/diabolo_initial_rot_velocity", this->diabolo_current_rot_velocity_))
  {
    ROS_WARN_STREAM("Rotational velocity parameter not set. Using default value = " << 0.0);
    this->diabolo_current_rot_velocity_ = 0.0;
  }
  ROS_INFO_STREAM("Initial rot velocity is " << this->diabolo_current_rot_velocity_);
  if (!this->rosNode->getParam("/diabolo_rot_friction_factor", this->rot_friction_factor))
  {
    ROS_WARN_STREAM("Rotational friction factor not set. Using default value = " << 0.01);
    this->rot_friction_factor = 0.01;
  }
  if (!this->rosNode->getParam("/constrain_diabolo_2D", this->constrain_to_2D_flag))
  {
    this->constrain_to_2D_flag = false;  // False by default
    ROS_WARN("/constrain_diabolo_2D flag is not set. Defaulting to 3D");
  }
  else
  {
    if (this->constrain_to_2D_flag)
    {
      std::vector<double> p_normal;
      std::vector<double> p_point;
      ROS_WARN("Constraining diabolo motion to 2D");
      if (!this->rosNode->getParam("/diabolo_2D_plane_normal", p_normal))  // This is a double vector
      {
        ROS_WARN_STREAM("2D plane normal not set. Defaulting to world x-axis");
        p_normal.clear();
        p_normal.push_back(1.0);
        p_normal.push_back(0.0);
        p_normal.push_back(0.0);
      }
      if (!this->rosNode->getParam("/diabolo_2D_plane_point", p_point))  // This is a double vector
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

  if (!this->rosNode->getParam("/left_stick_initial_position", this->left_stick_initial_pos_))  // This is a double
                                                                                                // vector
  {
    ROS_WARN_STREAM("Left stick initial position not set. Using default value");
    this->left_stick_initial_pos_.clear();
    this->left_stick_initial_pos_.push_back(0.0);
    this->left_stick_initial_pos_.push_back(0.5);
    this->left_stick_initial_pos_.push_back(1.5);
  }
  if (!this->rosNode->getParam("/right_stick_initial_position", this->right_stick_initial_pos_))  // This is a double
                                                                                                  // vector
  {
    ROS_WARN_STREAM("Right stick initial position not set. Using default value");
    this->right_stick_initial_pos_.clear();
    this->right_stick_initial_pos_.push_back(0.0);
    this->right_stick_initial_pos_.push_back(-0.5);
    this->right_stick_initial_pos_.push_back(1.5);
  }

  this->left_stick_current_position_ =
      ignition::math::Vector3d((this->left_stick_initial_pos_.at(0)), (this->left_stick_initial_pos_.at(1)),
                               (this->left_stick_initial_pos_.at(2)));

  this->right_stick_current_position_ =
      ignition::math::Vector3d((this->right_stick_initial_pos_.at(0)), (this->right_stick_initial_pos_.at(1)),
                               (this->right_stick_initial_pos_.at(2)));

  this->right_stick_last_position_ = this->right_stick_current_position_;
  this->left_stick_last_position_ = this->left_stick_current_position_;
  this->right_stick_latest_position_ = this->right_stick_current_position_;
  this->left_stick_latest_position_ = this->left_stick_current_position_;
  this->marker_count_ = 0;
  this->diabolo_last_state_ = OFF_STRING_LOOSE;
  // ros::SubscribeOptions stick_pos_so =
  //     ros::SubscribeOptions::create<geometry_msgs::PoseArray>(
  //         this->stick_pos_topic_name_, 1,
  //         boost::bind(&DiaboloPlugin::OnStickPosMsg, this, _1), ros::VoidPtr(),
  //         &this->rosQueue);
  // this->stick_rosSub = this->rosNode->subscribe(stick_pos_so);
  tf_listener_ = std::unique_ptr<tf::TransformListener>(new tf::TransformListener);
  tf_broadcaster_ = std::unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);
  this->marker_array_pub_ = this->rosNode->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  this->pull_velocity_pub_ = this->rosNode->advertise<geometry_msgs::PointStamped>("diabolo_pull_velocity", 1);
  this->diabolo_current_state_pub_ = this->rosNode->advertise<diabolo_gazebo::DiaboloState>("diabolo_current_state", 1);
  ros::SubscribeOptions tf_sub_so = ros::SubscribeOptions::create<tf2_msgs::TFMessage>(
      "/tf", 1, boost::bind(&DiaboloPlugin::OnTfMsg, this, _1), ros::VoidPtr(), &this->rosQueue);
  this->tf_sub_ = this->rosNode->subscribe(tf_sub_so);
  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&DiaboloPlugin::QueueThread, this));

  this->updateConnection = event::Events::ConnectWorldUpdateEnd(std::bind(&DiaboloPlugin::OnUpdate, this));
}

// Called by the world update start event
void DiaboloPlugin::OnUpdate()
{
  common::Time current_time = this->world->SimTime();
  this->time_step = (current_time - this->last_time).Double();

  // Store latest stick positions in the current_position variable
  this->left_stick_current_position_ = this->left_stick_latest_position_;
  this->right_stick_current_position_ = this->right_stick_latest_position_;

  this->ellipse_current_velocity_ = this->get_ellipse_velocity();

  // Get diabolo pose and velocity

  ignition::math::Pose3d pose = this->model->WorldPose();

  // Temporarily set the orientation as unchanging
  pose.Set(pose.Pos(), ignition::math::Vector3d(0., 0., 0.));

  ignition::math::Vector3d diabolo_velocity = this->model->WorldLinearVel();
  this->diabolo_current_velocity_ = diabolo_velocity;
  this->diabolo_current_position_ = pose.Pos();
  this->diabolo_current_orientation_ = pose.Rot();
  if (this->constrain_to_2D_flag)
  {
    this->constrain_to_2D(this->diabolo_current_position_, this->diabolo_current_velocity_);
    pose.Set(this->diabolo_current_position_, ignition::math::Vector3d(0., 0., 0.));
  }

  this->store_diabolo_transform(pose);
  // publish diabolo location to tf
  this->publish_diabolo_frame_to_tf();

  // Store information about current ellipse
  this->store_ellipse_axes_lengths();
  // Store current ellipse transform
  this->store_ellipse_transform();
  // publish ellipse frame to tf
  this->publish_ellipse_frame_to_tf();

  // Get the state of the diabolo
  this->diabolo_state_ = this->get_diabolo_state();
  // get diabolo location in ellipse frame, and check if it is on the string
  // In this case, the diabolo is outside the boundary of the ellipse and needs position/velocity correction
  // if(!first_step)
  // {
  if (this->diabolo_state_ == OUTSIDE_STRING)
  {
    this->diabolo_new_position_ = this->constrain_diabolo_position();
    if (!this->diabolo_new_position_.IsFinite())
    {
      ROS_ERROR("Constrained position is not finite. Not changing");
      diabolo_new_position_ = this->diabolo_current_position_;
    }
    ignition::math::Pose3d new_pose(this->diabolo_new_position_, pose.Rot());
    pose = new_pose;
    // Get the max pull velocity
    ignition::math::Vector3d pull_velocity_cap = this->get_pull_velocity_cap();
    // Calculate the pull velocity using the new position and the old position
    this->pull_velocity = this->get_pull_velocity(ignition::math::Vector3d(this->diabolo_current_position_.X(),
                                                                           this->diabolo_current_position_.Y(),
                                                                           this->diabolo_current_position_.Z()),
                                                  this->diabolo_new_position_);

    this->model->SetLinkWorldPose(new_pose, this->link_);
    this->diabolo_current_position_ = this->diabolo_new_position_;
    // Add the pull velocity to the current diabolo velocity

    // Make pull velocity for this time step 0 if it is not directed towards the center of the ellipse
    if (!this->pull_velocity_is_directed_inward())
    {
      this->pull_velocity = ignition::math::Vector3d(0, 0, 0);
    }

    // ROS_INFO_STREAM("Initial->pull_velocity = " << this->pull_velocity);
    // std::cout << "Pull velocity initially calculated to be " << this->pull_velocity << std::endl;

    if (this->pull_velocity.Length() > pull_velocity_cap.Length())
    {
      this->pull_velocity = pull_velocity_cap;
      // ROS_INFO_STREAM("Pull velocity capped at = " << this->pull_velocity);
    }
    if (this->pull_velocity.Length() > 0.0)
    {
      ignition::math::Vector3d ellipse_y_axis =
          this->left_stick_current_position_ - this->right_stick_current_position_;
      double distance_between_sticks = (ellipse_y_axis).Length();
      if (this->string_length_ - this->pv_string_taut_tolerance <= distance_between_sticks)
      {
        this->get_edge_case_pull_velocity(this->pull_velocity);
        ROS_WARN_STREAM("Sting taut enough for vertical pull velocity");
      }
    }
    this->pull_velocity = this->pull_velocity * this->pv_post_cap_scaling_factor;
    // diabolo_velocity += this->pull_velocity;

    // Remove the positive component of the diabolo's velocity along the
    // direction of the surface normal of the ellipse

    this->diabolo_current_velocity_ = this->constrain_diabolo_velocity(
        diabolo_velocity * this->constrained_velocity_scaling_factor + this->pull_velocity);
    // ROS_INFO_STREAM("Diabolo velocity is set to = " << this->diabolo_current_velocity_);
    this->model->SetLinearVel(this->diabolo_current_velocity_);
    // std::cout << std::endl;
    if (this->diabolo_last_state_ == ON_STRING || this->diabolo_last_state_ == OUTSIDE_STRING)
    {
      this->diabolo_current_rot_velocity_ += this->get_rot_velocity_change();
    }
  }
  // Set the diabolo to the new position
  this->model->SetLinkWorldPose(pose, this->link_);
  this->publish_visualization_markers();
  this->publish_pull_velocity();
  this->publish_diabolo_state();
  // std::cout << diabolo_current_position << std::endl;
  this->time_step = (current_time - this->last_time).Double();
  this->last_time = current_time;
  // Store the current position  of the sticks as the last position
  this->left_stick_last_position_ = this->left_stick_current_position_;
  this->right_stick_last_position_ = this->right_stick_current_position_;
  this->diabolo_last_position_ = this->diabolo_current_position_;
  this->diabolo_last_state_ = this->diabolo_state_;
  // }

  // first_step = false;
}

void DiaboloPlugin::store_ellipse_axes_lengths()
{
  ignition::math::Vector3d between_focii = this->left_stick_current_position_ - this->right_stick_current_position_;
  double distance_between_focii = between_focii.Length();

  this->ellipse_major_axis_length_ = this->string_length_ / 2.0;  // From definition of ellipse
  this->ellipse_minor_axis_length_ =
      sqrt(pow(this->ellipse_major_axis_length_, 2.0) - pow((distance_between_focii / 2.0), 2.0));
}

double DiaboloPlugin::get_ellipse_minor_axis_length(ignition::math::Vector3d foci_1, ignition::math::Vector3d foci_2)
{
  ignition::math::Vector3d between_focii = foci_1 - foci_2;
  double distance_between_focii = between_focii.Length();

  double major_axis_length = this->string_length_ / 2.0;  // From definition of ellipse
  double minor_axis_length_ = sqrt(pow(major_axis_length, 2.0) - pow((distance_between_focii / 2.0), 2.0));

  return minor_axis_length_;
}
ignition::math::Vector3d DiaboloPlugin::get_ellipse_velocity()
{
  ignition::math::Vector3d last_origin = (this->left_stick_last_position_ + this->right_stick_last_position_) / 2.0;
  ignition::math::Vector3d current_origin =
      (this->left_stick_current_position_ + this->right_stick_current_position_) / 2.0;

  ignition::math::Vector3d velo = (current_origin - last_origin) / this->time_step;

  return velo;
}
double DiaboloPlugin::get_rot_velocity_change()
{
  // The length of string that moved past the diabolo in the last time step
  double old_dist_to_right_stick = this->right_stick_last_position_.Distance(this->diabolo_last_position_);
  double new_dist_to_right_stick = this->right_stick_current_position_.Distance(this->diabolo_current_position_);

  double string_velo = (new_dist_to_right_stick - old_dist_to_right_stick) / (this->time_step);
  // This is the speed of the diabolo axle at axle_radius distance from the center
  double axle_velo = this->diabolo_current_rot_velocity_ * this->diabolo_axis_radius_;

  return ((string_velo - axle_velo) * this->rot_friction_factor);
}
void DiaboloPlugin::store_diabolo_transform(ignition::math::Pose3d pose_)
{
  this->diabolo_transform.setOrigin(tf::Vector3(pose_.Pos().X(), pose_.Pos().Y(), pose_.Pos().Z()));
  this->diabolo_transform.setRotation(
      tf::Quaternion(pose_.Rot().X(), pose_.Rot().Y(), pose_.Rot().Z(), pose_.Rot().W()));
}
void DiaboloPlugin::publish_diabolo_frame_to_tf()
{
  tf_broadcaster_->sendTransform(
      tf::StampedTransform(this->diabolo_transform, ros::Time::now(), "world", "diabolo_frame"));
}

void DiaboloPlugin::publish_ellipse_frame_to_tf()
{
  tf_broadcaster_->sendTransform(
      tf::StampedTransform(this->ellipse_transform, ros::Time::now(), "world", "ellipse_frame"));
}

void DiaboloPlugin::store_ellipse_transform()
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

  this->ellipse_transform.setOrigin(tf::Vector3(center.X(), center.Y(), center.Z()));
  this->ellipse_transform.setRotation(tf::Quaternion(q.X(), q.Y(), q.Z(), q.W()));
}

int DiaboloPlugin::get_diabolo_state()
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
  if (this->string_length_ - this->catching_string_taut_tolerance <= distance_between_sticks)
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
    ROS_INFO("Ellipse axis is parallel to the x axis");
    // If ellipse y axis parallel to world x axis, plane normal is straight u
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
      ROS_WARN_STREAM("String taut enough to catch diabolo");
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
          ROS_WARN("Catching diabolo");
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

bool DiaboloPlugin::diabolo_between_sticks()
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
ignition::math::Vector3d DiaboloPlugin::constrain_to_2D(ignition::math::Vector3d& pos, ignition::math::Vector3d& vel)
{
  // This function constrains the diabolo position and velocity to the desired 2D plane
  this->constrain_position_to_2D(pos);
  this->constrain_velocity_to_2D(vel);
}
ignition::math::Vector3d DiaboloPlugin::constrain_position_to_2D(ignition::math::Vector3d& pos)
{
  // This function projects the diabolo position onto the set 2D plane
  // new_pos = old_pos - projection of (old_pos - plane_point) on the plane normal
  // projection of (old_pos - plane_point) on the plane normal = p_c_normal
  ignition::math::Vector3d p_c_normal = pos - this->plane_point_2D;  // Intermediate step
  p_c_normal = p_c_normal.Dot(this->plane_normal_2D) * this->plane_normal_2D;

  pos = pos - p_c_normal;
}
ignition::math::Vector3d DiaboloPlugin::constrain_velocity_to_2D(ignition::math::Vector3d& vel)
{
  // This function constrains the velocity vel to be parallel to the plane, by removing its component parallel to the
  // plane normal

  vel = vel - vel.Dot(this->plane_normal_2D) * this->plane_normal_2D;
}
ignition::math::Vector3d DiaboloPlugin::constrain_diabolo_position()
{
  // Use parametric equation of ellipse
  // Calculate angles made with axes and solve for distance from center. Then
  // find new coordinates
  // Angle with z axis: phi
  // Angle with x axis: theta

  tf::Vector3 world_frame_pos(this->diabolo_current_position_.X(), this->diabolo_current_position_.Y(),
                              this->diabolo_current_position_.Z());

  tf::Vector3 ellipse_frame_pos = this->ellipse_transform.inverse() * world_frame_pos;

  ignition::math::Vector3d ellipse_frame_pos_vector(ellipse_frame_pos.getX(), ellipse_frame_pos.getY(),
                                                    ellipse_frame_pos.getZ());

  ignition::math::Vector3d world_position;

  // Angle with xy plane
  if (ellipse_frame_pos_vector.X() == 0 && ellipse_frame_pos_vector.Y() == 0 && ellipse_frame_pos_vector.Z() == 0)
  {
    tf::Vector3 new_ellipse_frame_pos(0, 0, 0);
    world_frame_pos = this->ellipse_transform * new_ellipse_frame_pos;

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
      // double sin_phi = sqrt(1 - pow(cos_phi, 2.0));

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
    world_frame_pos = this->ellipse_transform * new_ellipse_frame_pos;

    world_position = ignition::math::Vector3d(world_frame_pos.getX(), world_frame_pos.getY(), world_frame_pos.getZ());
  }

  return world_position;
}
ignition::math::Vector3d DiaboloPlugin::get_ellipse_normal_in_world_frame(ignition::math::Vector3d world_frame_pos_vec)
{
  tf::Vector3 world_frame_pos(world_frame_pos_vec.X(), world_frame_pos_vec.Y(), world_frame_pos_vec.Z());

  tf::Vector3 ellipse_frame_pos = this->ellipse_transform.inverse() * world_frame_pos;

  // Get normal to ellipse at diabolo's location

  tf::Vector3 ellipse_frame_normal(
      2.0 * (ellipse_frame_pos.getX() / this->ellipse_minor_axis_length_),
      2.0 * (ellipse_frame_pos.getY() / this->ellipse_major_axis_length_),  // major axis is aligned along y direction
      2.0 * (ellipse_frame_pos.getZ() / this->ellipse_minor_axis_length_));

  ellipse_frame_normal = ellipse_frame_normal.normalize();

  // Want only the rotation (not the position) transform of the noamal vector
  // Did not find an inbuilt function for vector transform using a tf::Transform, so using quaternion rotation
  // v_new_as_quat = Q * v_old_as_quat * Q_inverse
  tf::Quaternion ellipse_to_world_rot =
      this->ellipse_transform.getRotation();  // Rotation to rotate from ellipse frame to world frame
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

ignition::math::Vector3d DiaboloPlugin::constrain_diabolo_velocity(ignition::math::Vector3d diabolo_velocity)
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

ignition::math::Vector3d DiaboloPlugin::get_pull_velocity(ignition::math::Vector3d old_position,
                                                          ignition::math::Vector3d new_position)
{
  ignition::math::Vector3d v_pull = new_position - old_position;
  v_pull = v_pull / this->time_step;
  ignition::math::Vector3d normal_dir = this->get_ellipse_normal_in_world_frame(new_position);
  normal_dir = -normal_dir;
  v_pull = v_pull.Dot(normal_dir) * normal_dir * this->pv_pre_cap_scaling_factor;
  return v_pull;
}

void DiaboloPlugin::get_edge_case_pull_velocity(ignition::math::Vector3d& v_pull)
{
  ignition::math::Vector3d ellipse_y_axis = this->left_stick_current_position_ - this->right_stick_current_position_;
  // Get normal to the plane containing ellipse y axis and world z axis
  ignition::math::Vector3d normal = ellipse_y_axis.Cross(ignition::math::Vector3d(0.0, 0.0, 1.0));
  normal = normal.Normalize();
  double v_pull_mag = v_pull.Length();
  // Get component of the pull velocity in the direction of the normal
  ignition::math::Vector3d v_pull_in_normal_direction = v_pull.Dot(normal) * normal;
  v_pull = (v_pull - v_pull_in_normal_direction).Normalize() * v_pull_mag;  // keep the magnitude of the pull velocity
}

ignition::math::Vector3d DiaboloPlugin::get_pull_velocity_cap()
{
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

  // std::cout << "current center " << current_center << std::endl;
  // std::cout << "last_center " << last_center << std::endl;
  // std::cout << "Origin moved by " << ellipse_origin_change << std::endl;
  // ROS_INFO_STREAM("Origin moved by = " << ellipse_origin_change);
  if (direction_vector.Dot(ellipse_origin_change) > 0.0)  // If the change in origin velocity has a positive component
                                                          // along the directio_vector direction
  {
    origin_velocity_cap = direction_vector * (ellipse_origin_change.Length() / this->time_step);
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
    minor_axis_cap = direction_vector * ((last_minor_axis - current_minor_axis) / this->time_step);
  }
  else
  {
    minor_axis_cap = ignition::math::Vector3d(0, 0, 0);
  }

  // ROS_INFO_STREAM("Origin velocity cap =" << origin_velocity_cap);
  // ROS_INFO_STREAM("Minor axis cap =  " << minor_axis_cap);

  return (origin_velocity_cap + minor_axis_cap) * this->pv_cap_scaling_factor;
}
bool DiaboloPlugin::pull_velocity_is_directed_inward()
{
  // Get position of ellipse center in world frame
  // Get vector from diabolo to ellipse center
  // Find dot product of calculated pull velocity and the above vector.
  // Return false if dot product is negative, and true if positive

  ignition::math::Vector3d center = (this->left_stick_current_position_ + this->right_stick_current_position_) / 2.0;
  ignition::math::Vector3d direction_vector = center - this->diabolo_current_position_;

  double dot_product = direction_vector.Dot(this->pull_velocity);
  if (dot_product >= 0)
  {
    return true;
  }

  else
  {
    return false;
  }
}

void DiaboloPlugin::publish_pull_velocity()
{
  geometry_msgs::PointStamped vel;
  vel.header.stamp = ros::Time::now();
  vel.point.x = this->pull_velocity.X();
  vel.point.y = this->pull_velocity.Y();
  vel.point.z = this->pull_velocity.Z();

  this->pull_velocity_pub_.publish(vel);
}

void DiaboloPlugin::publish_diabolo_state()
{
  diabolo_gazebo::DiaboloState current_state;
  current_state.header.stamp = ros::Time::now();
  current_state.header.frame_id = "world";
  current_state.trans_velocity.x = this->diabolo_current_velocity_.X();
  current_state.trans_velocity.y = this->diabolo_current_velocity_.Y();
  current_state.trans_velocity.z = this->diabolo_current_velocity_.Z();

  current_state.pose.position.x = this->diabolo_current_position_.X();
  current_state.pose.position.y = this->diabolo_current_position_.Y();
  current_state.pose.position.z = this->diabolo_current_position_.Z();

  current_state.pose.orientation.x = this->diabolo_current_orientation_.X();
  current_state.pose.orientation.y = this->diabolo_current_orientation_.Y();
  current_state.pose.orientation.z = this->diabolo_current_orientation_.Z();
  current_state.pose.orientation.w = this->diabolo_current_orientation_.W();

  current_state.rot_velocity = this->diabolo_current_rot_velocity_;
  current_state.mass = this->mass_;
  current_state.string_length = this->string_length_;

  this->diabolo_current_state_pub_.publish(current_state);
}

visualization_msgs::Marker DiaboloPlugin::make_marker_(std::string const& mesh_filename, std::string const& frame_id,
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
  marker.lifetime = ros::Duration();

  return marker;
}

void DiaboloPlugin::publish_visualization_markers()
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;
  ros::Time now = ros::Time::now();

  float c[4];
  if (this->ellipse_minor_axis_length_ != 0.0 && this->ellipse_major_axis_length_ != 0.0)
  {
    c[0] = 1.0f;
    c[1] = 0.0f;
    c[2] = 0.0f;
    c[3] = 0.3f;
    marker = this->make_marker_("", "/ellipse_frame", "ellipse", visualization_msgs::Marker::SPHERE,
                                ignition::math::Vector3<float>(2.0 * this->ellipse_minor_axis_length_,
                                                               2.0 * this->ellipse_major_axis_length_,
                                                               2.0 * this->ellipse_minor_axis_length_),
                                c);
    marker.header.stamp = now;
    marker_array.markers.push_back(marker);
  }

  else
  {
    this->marker_count_++;
  }

  c[0] = 1.0f;
  c[1] = 1.0f;
  c[2] = 1.0f;
  c[3] = 1.0f;
  marker = this->make_marker_("", "/world", "string", visualization_msgs::Marker::ARROW,
                              ignition::math::Vector3<float>(0.01, 0.02, 0.02), c);
  marker.header.stamp = now;
  geometry_msgs::Point point;
  point.x = this->left_stick_current_position_.X();
  point.y = this->left_stick_current_position_.Y();
  point.z = this->left_stick_current_position_.Z();
  marker.points.push_back(point);
  point.x = this->diabolo_current_position_.X();
  point.y = this->diabolo_current_position_.Y();
  point.z = this->diabolo_current_position_.Z();
  marker.points.push_back(point);
  marker_array.markers.push_back(marker);
  marker.points.clear();

  marker = this->make_marker_("", "/world", "string", visualization_msgs::Marker::ARROW,
                              ignition::math::Vector3<float>(0.01, 0.02, 0.02), c);
  point.x = this->right_stick_current_position_.X();
  point.y = this->right_stick_current_position_.Y();
  point.z = this->right_stick_current_position_.Z();
  marker.points.push_back(point);
  point.x = this->diabolo_current_position_.X();
  point.y = this->diabolo_current_position_.Y();
  point.z = this->diabolo_current_position_.Z();
  marker.points.push_back(point);
  marker_array.markers.push_back(marker);

  c[0] = 0.0f;
  c[1] = 0.0f;
  c[2] = 1.0f;
  c[3] = 1.0f;

  marker = this->make_marker_("package://diabolo_scene_description/meshes/diabolo_shell.stl", "/diabolo_frame",
                              "diabolo", visualization_msgs::Marker::MESH_RESOURCE,
                              ignition::math::Vector3<float>(0.001, 0.001, 0.001), c);
  marker_array.markers.push_back(marker);

  c[0] = 0.7f;
  c[1] = 0.7f;
  c[2] = 0.7f;
  c[3] = 1.0f;

  marker = this->make_marker_("package://diabolo_scene_description/meshes/diabolo_axis.stl", "/diabolo_frame",
                              "diabolo", visualization_msgs::Marker::MESH_RESOURCE,
                              ignition::math::Vector3<float>(0.001, 0.001, 0.001), c);
  marker_array.markers.push_back(marker);

  c[0] = 0.1f;
  c[1] = 0.1f;
  c[2] = 0.1f;
  c[3] = 1.0f;

  marker = this->make_marker_("package://diabolo_scene_description/meshes/diabolo_fixators.stl", "/diabolo_frame",
                              "diabolo", visualization_msgs::Marker::MESH_RESOURCE,
                              ignition::math::Vector3<float>(0.001, 0.001, 0.001), c);
  marker_array.markers.push_back(marker);

  c[0] = 0.1f;
  c[1] = 0.1f;
  c[2] = 0.1f;
  c[3] = 1.0f;

  marker = this->make_marker_("package://diabolo_scene_description/meshes/diabolo_stick.stl", "/world", "stick",
                              visualization_msgs::Marker::MESH_RESOURCE,
                              ignition::math::Vector3<float>(0.001, 0.001, 0.001), c);
  marker.pose.position.x = this->left_stick_current_position_.X();
  marker.pose.position.y = this->left_stick_current_position_.Y();
  marker.pose.position.z = this->left_stick_current_position_.Z();
  marker_array.markers.push_back(marker);

  marker = this->make_marker_("package://diabolo_scene_description/meshes/diabolo_stick.stl", "/world", "stick",
                              visualization_msgs::Marker::MESH_RESOURCE,
                              ignition::math::Vector3<float>(0.001, 0.001, 0.001), c);
  marker.pose.position.x = this->right_stick_current_position_.X();
  marker.pose.position.y = this->right_stick_current_position_.Y();
  marker.pose.position.z = this->right_stick_current_position_.Z();
  marker_array.markers.push_back(marker);

  c[0] = 1.0f;
  c[1] = 1.0f;
  c[2] = 1.0f;
  c[3] = 1.0f;

  // if(this->pull_velocity.Length() > 0)
  // {
  marker = this->make_marker_("", "/world", "v_pull", visualization_msgs::Marker::ARROW,
                              ignition::math::Vector3<float>(0.03, 0.02, 0.02), c);
  point.x = this->diabolo_current_position_.X();
  point.y = this->diabolo_current_position_.Y();
  point.z = this->diabolo_current_position_.Z();
  marker.points.push_back(point);
  ignition::math::Vector3d normal_velocity = this->pull_velocity;
  normal_velocity = normal_velocity.Normalize();
  normal_velocity /= 2.0;
  point.x = this->diabolo_current_position_.X() + normal_velocity.X();
  point.y = this->diabolo_current_position_.Y() + normal_velocity.Y();
  point.z = this->diabolo_current_position_.Z() + normal_velocity.Z();
  marker.points.push_back(point);
  marker_array.markers.push_back(marker);
  // }
  // else
  // {
  //   this->marker_count_++;
  // }

  c[0] = 0.0f;
  c[1] = 1.0f;
  c[2] = 0.0f;
  c[3] = 1.0f;
  marker = this->make_marker_("", "/world", "diabolo_new_green", visualization_msgs::Marker::SPHERE,
                              ignition::math::Vector3<float>(0.1, 0.1, 0.1), c);
  marker.pose.position.x = this->diabolo_new_position_.X();
  marker.pose.position.y = this->diabolo_new_position_.Y();
  marker.pose.position.z = this->diabolo_new_position_.Z();

  marker_array.markers.push_back(marker);

  if (this->diabolo_state_ == OUTSIDE_STRING)
  {
    c[0] = 0.0f;
    c[1] = 0.0f;
    c[2] = 1.0f;
    c[3] = 0.5f;
    marker = this->make_marker_("", "/world", "normal_vector", visualization_msgs::Marker::ARROW,
                                ignition::math::Vector3<float>(0.03, 0.02, 0.02), c);
    point.x = this->diabolo_current_position_.X();
    point.y = this->diabolo_current_position_.Y();
    point.z = this->diabolo_current_position_.Z();
    marker.points.push_back(point);
    ignition::math::Vector3d normal_vector = this->get_ellipse_normal_in_world_frame(this->diabolo_current_position_);
    normal_vector = normal_vector.Normalize();
    normal_vector /= 2.0;
    point.x = this->diabolo_current_position_.X() + normal_vector.X();
    point.y = this->diabolo_current_position_.Y() + normal_vector.Y();
    point.z = this->diabolo_current_position_.Z() + normal_vector.Z();
    marker.points.push_back(point);
    marker_array.markers.push_back(marker);
  }
  this->marker_array_pub_.publish(marker_array);
  this->marker_count_ = 0;
}

void DiaboloPlugin::OnStickPosMsg(const geometry_msgs::PoseArrayConstPtr& _msg)
{
  // Check distance between sticks
  // Only set new positions if distance is not greater than string length
  ignition::math::Vector3d temp_left, temp_right;

  temp_left = ignition::math::Vector3d(_msg->poses[0].position.x, _msg->poses[0].position.y, _msg->poses[0].position.z);
  temp_right =
      ignition::math::Vector3d(_msg->poses[1].position.x, _msg->poses[1].position.y, _msg->poses[1].position.z);

  if (temp_left.Distance(temp_right) < this->string_length_)
  {
    this->left_stick_latest_position_ = temp_left;
    this->right_stick_latest_position_ = temp_right;
  }
}
void DiaboloPlugin::OnTfMsg(const tf2_msgs::TFMessageConstPtr& _msg)
{
  ros::Time now = ros::Time::now();
  geometry_msgs::PointStamped right_stick_pos, left_stick_pos;
  geometry_msgs::PointStamped frame_origin;
  frame_origin.header.stamp = ros::Time(0);
  frame_origin.header.frame_id = "/a_bot_diabolo_stick_tip";
  frame_origin.point.x = 0.0;
  frame_origin.point.y = 0.0;
  frame_origin.point.z = 0.0;

  try
  {
    frame_origin.header.frame_id = "/a_bot_diabolo_stick_tip";
    tf_listener_->transformPoint("/world", frame_origin, right_stick_pos);

    frame_origin.header.frame_id = "/b_bot_diabolo_stick_tip";
    tf_listener_->transformPoint("/world", frame_origin, left_stick_pos);
  }
  catch (tf2::TransformException e)
  {
    return;
  }
  this->left_stick_latest_position_ =
      ignition::math::Vector3d(left_stick_pos.point.x, left_stick_pos.point.y, left_stick_pos.point.z);

  this->right_stick_latest_position_ =
      ignition::math::Vector3d(right_stick_pos.point.x, right_stick_pos.point.y, right_stick_pos.point.z);
}

void DiaboloPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
}  // namespace gazebo