#ifndef DIABOLO_MOTION_GENERATOR_H
#define DIABOLO_MOTION_GENERATOR_H
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include "ros/ros.h"
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_srvs/SetBool.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "diabolo_gazebo/DiaboloState.h"
#include "diabolo_gazebo/DiaboloSimConfig.h" // Describes the state of the simulation
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "tf/tf.h"
#include "diabolo_play/CreateSticksTrajectory.h"
#include <diabolo_play/CreateRobotTrajectory.h>
#include "diabolo_play/SimulateDiabolo.h"
#include "diabolo_play/DiaboloMotionSplineSeeds.h"
#include "helper_functions.h"

//TEMP: REmove broadcaster after debug
#include <tf/transform_broadcaster.h>
#include <ignition/math/Pose3.hh>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class DiaboloMotionGenerator
{
/*
    Generate a stick trajectory that will produce the required diabolo goal state

*/

    public:
    DiaboloMotionGenerator();
    ~DiaboloMotionGenerator();
    //Functions
    // Service handler to create stick trajectory
    bool calculateStickTrajectory(diabolo_play::CreateSticksTrajectory::Request& request, diabolo_play::CreateSticksTrajectory::Response& response);
    // Service handler to simulate diabolo states
    bool simulateDiabolo(diabolo_play::SimulateDiabolo::Request& request, diabolo_play::SimulateDiabolo::Response& response);
    
    //Evaluate the state of a diabolo against the goal state in the service request
    double evaluate_trajectory_cost(geometry_msgs::PoseArray left_poses, geometry_msgs::PoseArray right_poses, 
                                    diabolo_gazebo::DiaboloState goal_state, diabolo_gazebo::DiaboloState state, 
                                    std::vector<diabolo_gazebo::DiaboloState>& closest_states, int motion_flag);
    // Evaluate all predicted states against the waypoint states
    // This is for when the diabolo should move through multiple goal states
    
    double evaluate_trajectory_cost(geometry_msgs::PoseArray left_poses, geometry_msgs::PoseArray right_poses,
                                    std::vector<diabolo_gazebo::DiaboloState>& waypoint_states,
                                    std::vector<diabolo_gazebo::DiaboloState>& predicted_states,
                                    std::vector<diabolo_gazebo::DiaboloState>& closest_states,
                                    int motion_flag); 
    // Remove all trajectory points starting with a point violating physical constraints
    // These include robot joint movement constraints and string length constraints (distance between stick tips)
    bool trajectory_is_safe(geometry_msgs::PoseArray& left_poses, geometry_msgs::PoseArray& right_poses, double string_length);
    // Get coefficients for a polynominal describing the stick trajectory in a direction by random walk
    // min and max determine how far the coeffs are allowed to wander
    std::vector<double>get_new_trajectory_coefficients(std::vector<double> coeffs, double min, double max);

    /// \brief Get new spline coefficients from old spline knot 
    // The value of a time step is the same across dimensions and stick positions
    // There is one vector to hold "time_coordinates" of the spline, and 
    // one vector to hold the x,y,z coordinates of both sticks at those time steps
    // Parameters:  
    // time_coords: Vector holding the time variables for the spline knots
    //              format: {t1, t2, ... tn} 
    // coord_values: Vector holding the values of the x,y,z coordinates at the times in time_coords
    //              format: {{left_x1m left_x2... left_xn, right_x1, right_x2.. },{left_y1..right_yn},{left_z1...right_zn}}
    // max_time: The duration of the trajectory
    // coord_min_step: The minimum deviation from the current knot point of the random knot point
    // coord_max_step: The maximum deviation from the current knot point of the random knot point
    // motion_flag: The type of motion to generate knots for.
    //            : If looping motion, the first and last knot points are the same
    // Returns: 
    // A vector of spline objects, in the same order as the coordinates in coord values
    //  format: {s_lx, s_rx, s_ly, s_ry, s_lz, s_rz}  
    std::vector<PiecewiseSpline>get_randomized_spline_objects(std::vector<std::vector<double>>& coord_values 
                                                               ,std::vector<double>& time_coords
                                                               ,double& max_time, double stick_time_step
                                                               ,double coord_min_step, double coord_max_step,
                                                               int motion_flag);

    // Same as above, but without randomizing
    std::vector<PiecewiseSpline>get_spline_objects(std::vector<std::vector<double>>& coord_values 
                                                               ,std::vector<double>& time_coords);
    /// \brief Initialize spline knot cantidates with "likely" values based on the type of movement desired
    // Parameters: 
    // time_coords: Vector holding the times at the spline knots
    //              format: {t1, t2, ... tn} 
    // coord_values: Vector holding the values of the x,y,z coordinates at the times in time_coords
    //              format: {{left_x1m left_x2... left_xn, right_x1, right_x2.. },{left_y1..right_yn},{left_z1...right_zn}}                                                              
    void initialize_spline_knots(diabolo_play::DiaboloMotionSplineSeeds knot_seeds,std::vector<std::vector<double>>& coord_values, std::vector<double>&time_coords, int movement_flag, double max_time);                                                               
    void publish_predicted_trajectory(std::vector<diabolo_gazebo::DiaboloState>& states, float col[], std::string ns);
    void publish_stick_traj_markers(geometry_msgs::PoseArray& left_poses, geometry_msgs::PoseArray& right_poses, float col[], std::string ns);
    visualization_msgs::Marker make_marker_(std::string const &mesh_filename, std::string const &frame_id,
                                            std::string const &ns, int type,
                                            ignition::math::Vector3<float> scale, float col[]);
    // Variables
    ros::NodeHandle n_;
    ros::ServiceServer calc_trajectories_service;
    ros::ServiceServer simulate_diabolo_service;
    ros::ServiceClient get_robot_trajectories_client;
    ros::Publisher marker_array_pub_;

    int marker_count_;
    
    // Diabolo goal state
    std::vector<diabolo_gazebo::DiaboloState> goal_states_;
};

class DiaboloPredictor
{
    public:
    //Constructor
    DiaboloPredictor(diabolo_gazebo::DiaboloSimConfig);
    ~DiaboloPredictor();
    // Run prediction trial for the provided stick poses and return the resulting diabolo state at the end of the stick poses
    // Parameters:
    // left_poses: The left stick poses 
    // right_poses:The right stick poses
    // stick_time_step: The time between the stick poses. It is assumed that the time steps are identical  
    
    diabolo_gazebo::DiaboloState predict(geometry_msgs::PoseArray left_poses, geometry_msgs::PoseArray right_poses, double stick_time_step);
    // Run prediction trial and return the resulting sim state
    // left_poses:  The left stick poses 
    // right_poses: The right stick poses
    // left_times:  The time steps for the left stick from start of the trajectory
    // right_times: The time steps for the right stick from start of the trajectory
    // predicted_states: Reference to vector in which to store predicted states
    // store_states: Store the predicted states if this is true
    diabolo_gazebo::DiaboloState predict(geometry_msgs::PoseArray left_poses, geometry_msgs::PoseArray right_poses,
                                         std::vector<double>left_times, std::vector<double> right_times,
                                         std::vector<diabolo_gazebo::DiaboloState>& predicted_states, bool store_states);
    
    // Function to constrain the diabolo to the plane described by the plane normal and a point passing through the plane
    void set_2D_constraint(ignition::math::Vector3d plane_normal, ignition::math::Vector3d plane_point);         
    void remove_2D_constraint();


    /// \brief Get the current state of the diabolo.
    diabolo_gazebo::DiaboloState get_current_state();

    /// \brief Get the current state of the simulation (including stick positions and simulation parameters)
    diabolo_gazebo::DiaboloSimConfig get_current_state_full();

    private:

    // Functions
    /// \brief Get the state of the diabolo after one time step without considering constraints
    // Applies velocity verlet integration for a single time step
    void perform_freebody_calculation();
    
    /// \brief Runs a prediction trial for a set of left and right stick poses
    // The pose arrays must have the same time step
    void run_prediction_trial(geometry_msgs::PoseArray left_poses, geometry_msgs::PoseArray right_poses, double stick_time_step);
    
    /// \brief Runs a prediction trial for a set of left and right stick poses
    // The time steps of these poses can be different, and are stored in the left times and right times arrays resp
    // These time steps are the times from start of the trajectory
    void run_prediction_trial(geometry_msgs::PoseArray left_poses, geometry_msgs::PoseArray right_poses,
                              std::vector<double>left_times, std::vector<double> right_times,
                              std::vector<diabolo_gazebo::DiaboloState>& predicted_states, bool store_states);
    /// \brief Advance the simulation by one time step
    void step();
    
    // The locus of the points reachable by the diabolo when on the string is defined by an ellipsoid with its focii at the diabolo stick tips
    // The following functions are to calculate various values associated with this ellipsoid  
    ignition::math::Vector3d get_ellipse_velocity();
    void store_ellipse_axes_lengths();
    void store_ellipse_transform();
    double get_ellipse_minor_axis_length(ignition::math::Vector3d foci_1, ignition::math::Vector3d foci_2);
    ignition::math::Vector3d get_ellipse_normal_in_world_frame(ignition::math::Vector3d world_frame_pos_vec);
    
    // Constrain the diabolo to a 2D plane
    ignition::math::Vector3d constrain_to_2D(ignition::math::Vector3d& pos, ignition::math::Vector3d& vel);
    ignition::math::Vector3d constrain_position_to_2D(ignition::math::Vector3d& pos);
    ignition::math::Vector3d constrain_velocity_to_2D(ignition::math::Vector3d& vel);

    // Get functions to determine the diabolo state
    // Returns: An int describing the diabolo as being either :
    // ON_STRING, OFF_STRING_LOOSE, FLYING or OUTSIDE_STRING 
    int get_diabolo_state();

    // A function to determine if the diabolo is between the diabolo stick tips
    // Required to determine state transition
    bool diabolo_between_sticks();

    // Functions related to pull velocity
    ignition::math::Vector3d get_pull_velocity_cap();
    ignition::math::Vector3d get_pull_velocity(ignition::math::Vector3d old_position, ignition::math::Vector3d new_position);
    bool pull_velocity_is_directed_inward();
    // Constrain the pull velocity to the vertical if the string is taut enough
    void apply_edge_case_pv_constraint(ignition::math::Vector3d& v_pull);

    // Constrain the diabolo to the ellipse if required
    ignition::math::Vector3d constrain_diabolo_position();

    //Constrain the diabolo velocity not to point out of the ellipse
    ignition::math::Vector3d constrain_diabolo_velocity(ignition::math::Vector3d diabolo_velocity);

    // Get the change in rotational velocity of the diabolo
    double get_rot_velocity_change();
    // Get any required parameters from the ros param server
    void get_ros_parameters();

    //TEMP: For debug
    void store_diabolo_transform(ignition::math::Pose3d pose_);
    void publish_diabolo_frame_to_tf();
    void publish_ellipse_frame_to_tf();
    void publish_visualization_markers();
    
    visualization_msgs::Marker make_marker_(std::string const &mesh_filename, std::string const &frame_id,
                                                         std::string const &ns, int type,
                                                         ignition::math::Vector3<float> scale, float col[]);
    // Variables
    double time_step_; // The length of a time step 
    diabolo_gazebo::DiaboloSimConfig initial_state_;
    const ignition::math::Vector3d gravity_;
    double string_length_;
    double diabolo_axle_radius_;
    // Stick state variables
    ignition::math::Vector3d right_stick_last_position_, left_stick_last_position_;
    ignition::math::Vector3d right_stick_current_position_, left_stick_current_position_;

    // Diabolo state variables
    ignition::math::Vector3d diabolo_last_position_, diabolo_current_position_;
    ignition::math::Vector3d diabolo_last_velocity_, diabolo_current_velocity_;
    double diabolo_current_rot_velocity_;
    double diabolo_state_, diabolo_last_state_;
    // Threshold for if the string is taut enought to catch the diabolo
    double catching_string_taut_tolerance_;
    // Threshold for if the string is taut enough to constrain the pull velocity to the vertical plane
    double pv_string_taut_tolerance_;
    ignition::math::Vector3d pull_velocity_;

    // Velocity scaling factors
    double pv_pre_cap_scaling_factor_, pv_post_cap_scaling_factor_, pv_cap_scaling_factor_;
    double velocity_diffusion_factor_; 

    // Friction factor between string and diabolo
    double rot_friction_factor_;
    // Ellipse variables
    ignition::math::Vector3d ellipse_current_velocity_;
    double ellipse_major_axis_length_;
    double ellipse_minor_axis_length_;
    tf::Transform ellipse_transform_;

    // Used to constrain the diabolo to 2D
    ignition::math::Vector3d plane_point_2D, plane_normal_2D;
    bool constrain_to_2D_flag;
    // ros variables
    ros::NodeHandle n_;


    //TEMP: remove after debug
    std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster_;
    tf::Transform diabolo_transform_;
    int marker_count_;
    ros::Publisher marker_array_pub_;
    visualization_msgs::MarkerArray marker_array;
    public:
    // A flag to determine whether or not to sleep between time steps (For visualization)
        bool sleep_flag_;
};
#endif
