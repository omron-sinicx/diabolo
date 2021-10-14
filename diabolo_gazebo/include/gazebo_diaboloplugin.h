#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include <ros/console.h>
#include "ros/subscribe_options.h"
#include <cmath>
#include <functional>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/SetModelStateRequest.h"
#include "std_srvs/Empty.h"
#include "diabolo_gazebo/DiaboloState.h"
#include "tf2_msgs/TFMessage.h"
namespace gazebo
{
  class DiaboloPlugin : public ModelPlugin
  {
  protected:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

  protected:
    void OnUpdate();

  public:
    DiaboloPlugin();
    virtual ~DiaboloPlugin();
    // Pointer to the model
  private:
    physics::ModelPtr model;

  private:
    physics::WorldPtr world;

    // Parameters from the model sdf
  private:
    std::string stick_pos_topic_name_;

  private:
    std::string initial_state_topic_name_;
  private:
    double string_length_;
  private:
    // The sticks must be beyond string_length - string_taut_tolerance for the string to be considered "taut"
    // This one the taut tolerance for "catching" a diabolo
    double catching_string_taut_tolerance;  
    // This one is the taut tolerance while setting the pull velocity direction
    // If the distance between stick positions is more than (string_length - pv_string_taut_tolerance), the pull velocity will be constrained 
    // to a plane containing the ellipse y axis and the world z axis
    double pv_string_taut_tolerance;
  /// \brief Vectors to hold the intial stick positions gotten from the parameter server 
  public:
    std::vector<double> left_stick_initial_pos_, right_stick_initial_pos_, diabolo_initial_velocity_;
  private:
    double mass_;

  private:
    double ellipse_minor_axis_length_;

  private:
    double ellipse_major_axis_length_; // This is along the Y axis of the ellipse
                                       // frame
  private:
    int diabolo_state_; // The current state of the diabolo  
    int diabolo_last_state_;  

  private:
    common::Time last_time;

  private:
    double time_step;

  // The velocity added to the diabolo as a consequence of moving the diabolo strings for this time step 
  // Its direction is given by the direction of motion of the diabolo. 
  // Its magnitude is given by the distance travelled by the diabolo divided by the time step
  private:
    ignition::math::Vector3d pull_velocity;


    // Pointer to the update event connection
  private:
    event::ConnectionPtr updateConnection;
    // physics::ModelPtr model;
  private:
    physics::LinkPtr link_;
    // private: event::ConnectionPtr updateConnection;
    /// \brief A node use for ROS transport
  private:
    std::unique_ptr<ros::NodeHandle> rosNode;

  std::unique_ptr<tf::TransformListener> tf_listener_;
  std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_;

    /// \brief A ROS subscriber
  private:
    ros::Subscriber stick_rosSub;
    ros::Subscriber tf_sub_;



  private:
    ros::Subscriber initial_position_rosSub;
    /// \brief ROS publisher for visualization markers
  private:
    ros::Publisher marker_array_pub_;
  private:
    ros::Publisher pull_velocity_pub_;
    ros::Publisher diabolo_current_state_pub_;

    /// \brief A ROS callbackqueue that helps process messages
  private:
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
  private:
    std::thread rosQueueThread;
    /// \brief Subscriber callbacks
  private:
    void QueueThread();

  private:
    void OnStickPosMsg(const geometry_msgs::PoseArrayConstPtr &_msg);
    void OnTfMsg(const tf2_msgs::TFMessageConstPtr &_msg);
    /// \brief Function to check the current state of the diabolo
    /// Change the name of this function
  private:
    int get_diabolo_state();

    /// \brief Function to correct the position of the diabolo to be on the string
    /// if required
    /// This function is called when the diabolo is still on the string, but its
    /// position has drifted because of simulation effects
  private:
    ignition::math::Vector3d constrain_diabolo_position();
  private:
    ignition::math::Vector3d get_ellipse_normal_in_world_frame(ignition::math::Vector3d world_frame_pos_vec);
    /// \brief Function to keep the diabolo higher than the lowest point allowed
    /// by the string
    /// This means if the diabolo is on the string, the positive component of its
    /// velocity along the direction of the
    /// normal to the ellipse should be removed
  private:
    ignition::math::Vector3d constrain_diabolo_velocity(ignition::math::Vector3d diabolo_velocity);

    /// \brief Used to publish visual markers of the ellipse and the diabolo
    /// This allows them to be visible in rviz by adding the "Marker" tag in the
    /// rviz configuration

  private:
    void publish_visualization_markers();

    /// \brief Function to publish the frame of the diabolo to tf
  private:
    void store_diabolo_transform(ignition::math::Pose3d pose_);
    void publish_diabolo_frame_to_tf();
    /// \brief Function to calculate the current position of the elipse and
    /// publish it to tf
  private:
    void publish_ellipse_frame_to_tf();
    void store_ellipse_transform();
    /// \brief Function to calculate and store ellipse axes lengths
  private:
    void store_ellipse_axes_lengths();

  private:
    double get_ellipse_minor_axis_length(ignition::math::Vector3d foci_1, ignition::math::Vector3d foci_2);
  
  private:
    ignition::math::Vector3d get_pull_velocity(ignition::math::Vector3d old_position, ignition::math::Vector3d new_position);
  private:
    /// \brief This function will check if the edge case with the ellipse major axis almost equal to string length applies
    // If it does, the pull velocity is constrained to the plane containing the ellipse y axis and the world z axis
    void get_edge_case_pull_velocity(ignition::math::Vector3d& v_pull);
  private:
    double get_rot_velocity_change();
  private:
    ignition::math::Vector3d get_pull_velocity_cap();
  
  private: 
    bool pull_velocity_is_directed_inward();

  private:
    void publish_pull_velocity();
    void publish_diabolo_state();
  private:
    /// \brief To determine if the vectors from the stick tips to the diabolo point in oposite direction
    bool diabolo_between_sticks();
  private :
    ignition::math::Vector3d get_diabolo_to_ellipse_center();
  private: 
    ignition::math::Vector3d get_ellipse_velocity();
    /// \brief This function creates a visualization marker mesage from the
    /// following parameters
    /// Parameters:
    /// mesh_filename: The full path the the mesh resource, if using a mesh. else
    /// send a blank string
    /// frame_id: The frame to represent the marker in
    /// ns: The marker namespace
    /// type: visualisation_msgs::Marker::<marker type>
  private:
    visualization_msgs::Marker
    make_marker_(std::string const &mesh_filename, std::string const &frame_id,
                 std::string const &ns, int type,
                 ignition::math::Vector3<float> scale, float col[]);
    /// Current positions of frames of interest.
  private:
    ignition::math::Vector3d diabolo_current_position_, diabolo_last_position_;
    ignition::math::Quaternion<double> diabolo_current_orientation_;
    tf::Transform diabolo_transform;
    tf::Transform ellipse_transform;
    /// The corrected position of the diabolo after constraining it to the ellipse

  private:
    int marker_count_;
  
  private:
    ignition::math::Vector3d diabolo_new_position_;

  private:
    ignition::math::Vector3d diabolo_current_velocity_;
    double diabolo_current_rot_velocity_; // Current rotational velocity
    double diabolo_axis_radius_;
    /// \brief The magic number that determines how much the change in string length slipping past the diabolo axis 
    /// affects its rotational velocity
    double rot_friction_factor;
  private:
    ignition::math::Vector3d ellipse_current_velocity_;

  /// \brief These are the scaling factors for pull velocity
  private: 
    double pv_cap_scaling_factor; // Scale the pull velocity cap
    double pv_post_cap_scaling_factor; // Scale the pull velocity after capping
    double pv_pre_cap_scaling_factor; // Scale the pull velocity before capping
    double constrained_velocity_scaling_factor; // This will scale the velocity when constraining it

  private:
    // If true, the diabolo will be constrained to a 2D plane parallel to the yz plane
    // The following constraints will be enacted if true: 
    // The stick positions will be projected onto the 2D plane 
    // The initial position of the diabolo will be projected onto this plane
    // The component of the diabolo velocity normal to this plane will be removed
    bool constrain_to_2D_flag;
    // The normal to and a point lying on the plane to restrict the diabolo to, if using the constrain_to_2D_flag is true
    ignition::math::Vector3d plane_normal_2D, plane_point_2D; 
    ignition::math::Vector3d constrain_to_2D(ignition::math::Vector3d& pos, ignition::math::Vector3d& vel);
    ignition::math::Vector3d constrain_position_to_2D(ignition::math::Vector3d& pos);
    ignition::math::Vector3d constrain_velocity_to_2D(ignition::math::Vector3d& vel);
  // The following stick positions will be stored:
  // current_position: The position of the stick at the beginning of the current time step
  // last_position: The position of the stick at the last time step
  // latest_position: The postion of the stick updated by the subscriber. 
  private:
    ignition::math::Vector3d left_stick_current_position_;
    ignition::math::Vector3d left_stick_last_position_;
    ignition::math::Vector3d left_stick_latest_position_;

  private:
    ignition::math::Vector3d right_stick_current_position_;
    ignition::math::Vector3d right_stick_last_position_;
    ignition::math::Vector3d right_stick_latest_position_;
  
  };
} // namespace gazebo
