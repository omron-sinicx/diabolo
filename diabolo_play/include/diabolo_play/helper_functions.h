#include <tf/transform_listener.h>    // Includes the TF conversions
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_datatypes.h>
#include <termios.h>  // non-blocking getchar

// Was this for logging?
#include "ros/ros.h"
#include <ros/console.h>
#include <ros/package.h>

#include <math.h>
#include <algorithm>  //For min
#include <string>
#include <fstream>
#include <sstream>

#include "Eigen/Dense"

// RPY rotations are applied in the frame of the pose.
void rotatePoseByRPY(const double roll, const double pitch, const double yaw, geometry_msgs::Pose& inpose)
{
  tf::Quaternion q;
  tf::Quaternion qrotate = tf::createQuaternionFromRPY(roll, pitch, yaw);

  tf::quaternionMsgToTF(inpose.orientation, q);

  q = q * qrotate;

  tf::quaternionTFToMsg(q, inpose.orientation);
}

// Returns the angle between two quaternions
double quaternionDistance(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2) 
{ 
  tf::Quaternion q1tf, q2tf;
  tf::quaternionMsgToTF(q1, q1tf);
  tf::quaternionMsgToTF(q2, q2tf);
  return 2*q1tf.angle(q2tf); 
}

double pointDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  tf::Point tp1, tp2;
  tf::pointMsgToTF(p1, tp1);
  tf::pointMsgToTF(p2, tp2);
  return tfDistance(tp1, tp2);
}

// (c) Salvo Virga, sankyu~~
// Transforms a stamped pose from its current reference frame (as given in its header) to referenceFrame
geometry_msgs::PoseStamped transform_pose_now(geometry_msgs::PoseStamped& pose, const std::string& referenceFrame, const tf::TransformListener& listener) 
{   
  // Check if the frames are different
  if (pose.header.frame_id != referenceFrame ) {

    bool success = false;
    tf::StampedTransform transform;
    geometry_msgs::PoseStamped result_pose;

    while (!success) {
      try {
        // ros::Time t = ros::Time::now();
        ros::Time t = ros::Time(0);
        pose.header.stamp = t;
        listener.waitForTransform(pose.header.frame_id, referenceFrame, t, ros::Duration(3.0));
        listener.lookupTransform(pose.header.frame_id, referenceFrame, t, transform);
        listener.transformPose(referenceFrame, pose, result_pose);
        success = true;
        return result_pose;
      } catch (tf::ExtrapolationException e) {
        ROS_ERROR_STREAM("Something went wrong in transform_pose_now, trying to transform from " << pose.header.frame_id << " to " << referenceFrame);
        // ROS_ERROR(e.what());
      }
      sleep(0.1);
    }
  }
  return pose;
}


geometry_msgs::PoseStamped transformTargetPoseFromTipLinkToURTCP(geometry_msgs::PoseStamped ps, std::string robot_name, std::string end_effector_link, tf::TransformListener& listener)
{
  // This transforms a pose from the end_effector_link set in MoveIt to the TCP used in the UR controller. 
  // It is used when sending commands to the UR controller directly, without MoveIt/ROS controllers.
  tf::StampedTransform st_tip_to_wrist, st_ref_to_goal;
  bool success = false;
  while (!success) {
    try {
      // ros::Time t = ros::Time::now();
      ros::Time t = ros::Time(0);
      listener.waitForTransform(end_effector_link, robot_name + "_tool0", t, ros::Duration(1.0));
      listener.lookupTransform(end_effector_link, robot_name + "_tool0", ros::Time::now(), st_tip_to_wrist);
      success = true;
    } catch (tf::ExtrapolationException e) {
      ROS_ERROR_STREAM("Something went wrong in transformTargetPoseFromTipLinkToURTCP");
      // ROS_ERROR(e.what());
    }
    sleep(0.1);
  }

  tf::Quaternion q1(ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w);
  tf::Vector3 v1(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);

  // ROS_INFO_STREAM("Received pose to transform to TCP link:");
  // ROS_INFO_STREAM(ps.pose.position.x << ", " << ps.pose.position.y  << ", " << ps.pose.position.z);
  // ROS_INFO_STREAM(ps.pose.orientation.x << ", " << ps.pose.orientation.y  << ", " << ps.pose.orientation.z  << ", " << ps.pose.orientation.w);

  st_ref_to_goal.setOrigin(v1);
  st_ref_to_goal.setRotation(q1);
  st_ref_to_goal.frame_id_ = ps.header.frame_id;
  st_ref_to_goal.child_frame_id_ = "temp_goal_pose__";
  st_ref_to_goal.stamp_ = ros::Time::now()-ros::Duration(.05);
  listener.setTransform(st_ref_to_goal);
  st_ref_to_goal.stamp_ = ros::Time::now();
  listener.setTransform(st_ref_to_goal);

  st_tip_to_wrist.frame_id_ = "temp_goal_pose__";
  st_tip_to_wrist.child_frame_id_ = "temp_wrist_pose__";
  listener.setTransform(st_tip_to_wrist);
  st_tip_to_wrist.stamp_ = ros::Time::now();
  listener.setTransform(st_tip_to_wrist);
  
  geometry_msgs::PoseStamped ps_wrist, ps_new;
  ps_wrist.header.frame_id = "temp_wrist_pose__";
  ps_wrist.pose.orientation.w = 1.0;
  listener.transformPose(ps.header.frame_id, ps_wrist, ps_new);
  
  // ROS_INFO_STREAM("New pose:");
  // ROS_INFO_STREAM(ps_new.pose.position.x << ", " << ps_new.pose.position.y  << ", " << ps_new.pose.position.z);
  // ROS_INFO_STREAM(ps_new.pose.orientation.x << ", " << ps_new.pose.orientation.y  << ", " << ps_new.pose.orientation.z  << ", " << ps_new.pose.orientation.w);

  return ps_new;
}

// This function is for tuning the gripper orientation when grasping into a bin.
// Checks if an orientation is permissible (within tolerance of target_rotation) and if not, flips it by 180 degrees
double flipGraspRotationIfNecessary(double in_rotation, double target_rotation, double tolerance)
{
  // Thank you internet!!! https://github.com/petercorke/toolbox-common-matlab/blob/master/angdiff.m
  double angdiff = in_rotation - target_rotation;
  angdiff = fmod(angdiff+M_PI, 2.0*M_PI) - M_PI;

  if (abs(angdiff) > tolerance)
  {
  // Flip rotation
    if (in_rotation <= 0.0)
    {
      return (in_rotation + M_PI);
    }
    if (in_rotation > 0.0)
    {
      return (in_rotation - M_PI);
    }
  }
  return in_rotation;
}

// Non-blocking getchar
int getch()
{
  ROS_INFO("Press any key to continue...");
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt); // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON); // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt); // apply new settings

  int c = getchar(); // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  return c;
}

// Used to restrict rotation values to a certain interval.
double restrictValueToInterval(double input_value, double allowed_min, double allowed_max)
{
  if (input_value > allowed_max)
  {
    input_value = allowed_max;
  }
  else if (input_value < allowed_min)
  {
    input_value = allowed_min;
  }
  return input_value;
}

// Returns how far the value is from the interval
double distOfValueToInterval(double input_value, double allowed_min, double allowed_max)
{
  if (input_value > allowed_max)
  {
    return abs(input_value - allowed_max);
  }
  else if (input_value < allowed_min)
  {
    return abs(input_value - allowed_min);
  }
  else
  {
    return input_value;  
  }
}

// Below are factory functions for common geometry messages.

geometry_msgs::Point makePoint(double x, double y, double z)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::Quaternion makeQuaternion(double x, double y, double z, double w)
{
  geometry_msgs::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

geometry_msgs::Pose makePose()
{
  geometry_msgs::Pose pose;
  pose.position = makePoint(0.0, 0.0, 0.0);
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::PoseStamped makePoseStamped()
{
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "world";
  ps.pose = makePose();
  return ps;
}

geometry_msgs::Pose makePose(double x, double y, double z)
{
  geometry_msgs::Pose pose;
  pose.position = makePoint(x, y, z);
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose makePose(geometry_msgs::Point p)
{
  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Pose makePose(double x, double y, double z, geometry_msgs::Quaternion q)
{
  geometry_msgs::Pose pose;
  pose.position = makePoint(x, y, z);
  pose.orientation = q;
  return pose;
}

geometry_msgs::Pose makePose(geometry_msgs::Point p, double xq, double yq, double zq, double w)
{
  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = makeQuaternion(xq, yq, zq, w);
  return pose;
}

geometry_msgs::Pose makePose(double x, double y, double z, double xq, double yq, double zq, double w)
{
  geometry_msgs::Pose pose;
  pose.position = makePoint(x, y, z);
  pose.orientation = makeQuaternion(xq, yq, zq, w);
  return pose;
}
double round_to_decimal(double val, int places)
{
  const double multiplier = pow(10.0, places);
  return round(val * multiplier)/multiplier;
}
// x_low -> smaller edge coordinate
// x_high -> higher edge coordinate
// C0 -> value at lower edge coordinate
// C1 -> value at higher edge coordinate
double interpolate1D(double x, double x_low, double x_high, double C0, double C1) 
{
  
  if(x > x_high)
    return C1;
  else if (x < x_low)
    return C0;
  
  if((x_high - x_low) != 0.0)
    return (((x-x_low)/(x_high-x_low)) * C1 + ((x_high-x)/(x_high-x_low)) * C0);

  else 
  return C0;
}
// CXY -> The value at (X, Y). e.g. C01 is the value at (x_low, y_high)
//coords[0] = x_low, coords[1] = x_high, coords[2] = y_low, coords[3] = y_high
// values[0] = C00, values[1] = C01, values[2] = C10. values[3] = C11 -> increasing in binary order
double interpolate2D(double x, double y, double coords[4], double values[4])
{
  double v1 = interpolate1D(x, coords[0], coords[1], values[0], values[2]);
  double v2 = interpolate1D(x, coords[0], coords[1], values[1], values[3]);

  return interpolate1D(y, coords[2], coords[3], v1, v2);
}

// coords[0] = x_low, coords[1] = x_high, coords[2] = y_right, coords[3] = y_left, coords[4] = z_low, coords[5] = z_high
//values[] has values stored in the same binary increasing order as for interpolate2D CXYZ NOT CZYX
// C000, C001, C010, C011, C100, C101, C110, C111 
double interpolate3D(double x, double y, double z, double coords[6], double values[8])
{
  double c[4] = {coords[0], coords[1], coords[2], coords[3]};
  double v_low[4] = {values[0], values[2], values[4], values[6]};
  double v_high[4] = {values[1], values[3], values[5], values[7]};
  double v1 = interpolate2D(x, y, c, v_low);
  double v2 = interpolate2D(x, y, c, v_high);

  return interpolate1D(z, coords[4], coords[5], v1, v2);
}
// coords => A vector of 3 vectors containing the knot coordinates for the robot's bounding box
// coords[0] -> x_coordinates
// coords[1] -> y_coordinates
// coords[2] -> z_coordinates. 
// The length of each of these is the number of coordinate points available for each dimension
// values => A 8x6 matrix of joint angle values
// Rows => values for a joint
// Columns => different joint values
std::vector<double> getInterpolatedJointAngles(double x, double y, double z,
                                               std::vector<std::vector<double>> coords, 
                                               std::vector<std::vector<double>> values) 
{
  std::vector<double> joint_angles;
  joint_angles.resize(6);
  double boundary_coords[6];
  int x_section = 0;
  int y_section = 0; // These are to determine which joint values to send to the interpolation function
  int z_section = 0; 
  // Array holding the coordinates on either side of the provided x, y and z in the format that interpolate3d expects
  // Set x boundary values
  if(x < coords[0][0])
  {
    boundary_coords[0] = coords[0][0];
    boundary_coords[1] = coords[0][1];
    x_section = 0;
  }
  else if(x > coords[0][coords[0].size()-1])
  {
    boundary_coords[0] = coords[0][coords[0].size()-2];
    boundary_coords[1] = coords[0][coords[0].size()-1];
    x_section = coords[0].size()-2;
  }
  else
  {
    if(coords[0].size() == 2)
    {
      boundary_coords[0] = coords[0][0];
      boundary_coords[1] = coords[0][1];
    }
    else
    {
      for(int i = 0; i < coords[0].size()-1; i++)
      {
        if((x <= coords[0][i+1] && x > coords[0][i]) || (x < coords[0][i+1] && x >= coords[0][i]))
        {
          boundary_coords[0] = coords[0][i];
          boundary_coords[1] = coords[0][i+1];
          x_section = i;
          break;
        }
      }
    }
    

  }
  // Set y boundaries
  if(y < coords[1][0])
  {
    boundary_coords[2] = coords[1][0];
    boundary_coords[3] = coords[1][1];
    y_section = 0;
  }
  else if(y > coords[1][coords[1].size()-1])
  {
    boundary_coords[2] = coords[1][coords[1].size()-2];
    boundary_coords[3] = coords[1][coords[1].size()-1];
    y_section = coords[1].size()-2;
  }
  else
  {
    if(coords[1].size() == 2)
    {
      boundary_coords[2] = coords[1][0];
      boundary_coords[3] = coords[1][1];
    }
    else
    {
      for(int i = 0; i < coords[1].size()-1; i++)
      {

        if((y <= coords[1][i+1] && y > coords[1][i]) || (y < coords[1][i+1] && y >= coords[1][i]))
        {
          boundary_coords[2] = coords[1][i];
          boundary_coords[3] = coords[1][i+1];
          y_section = i;
        }
      }
    }
  }
    // Set z boundaries
  if(z < coords[2][0])
  {
    boundary_coords[4] = coords[2][0];
    boundary_coords[5] = coords[2][1];
    z_section = 0;
  }
  else if(z > coords[2][coords[2].size()-1])
  {
    boundary_coords[4] = coords[2][coords[1].size()-2];
    boundary_coords[5] = coords[2][coords[1].size()-1];
    z_section = coords[2].size()-2;
  }
  else
  {
    if(coords[2].size() == 2)
    {
      boundary_coords[4] = coords[2][0];
      boundary_coords[5] = coords[2][1];
    }
    else
    {
      for(int i = 0; i < coords[2].size()-1; i++)
      {

        if((z <= coords[2][i+1] && z > coords[2][i]) || (z < coords[2][i+1] && z >= coords[2][i]))
        {
          boundary_coords[4] = coords[2][i];
          boundary_coords[5] = coords[2][i+1]; 
          z_section = i;
        }
      }
    }
  }
  int x_order = coords[0].size();
  int y_order = coords[1].size();
  int z_order = coords[2].size(); 

  std::vector<int> value_positions; // This vector should have the position numbers of the joint value in values in order

  value_positions.resize(8);
  value_positions[0] = y_order*z_order* x_section    + z_order * y_section       + z_section;
  value_positions[1] = y_order*z_order* x_section    + z_order * y_section       + z_section+1;
  value_positions[2] = y_order*z_order* x_section    + z_order * (y_section + 1) + z_section;
  value_positions[3] = y_order*z_order* x_section    + z_order * (y_section + 1) + z_section+1;
  value_positions[4] = y_order*z_order*(x_section+1) + z_order *  y_section      + z_section;
  value_positions[5] = y_order*z_order*(x_section+1) + z_order *  y_section      + z_section+1;
  value_positions[6] = y_order*z_order*(x_section+1) + z_order * (y_section + 1) + z_section;
  value_positions[7] = y_order*z_order*(x_section+1) + z_order * (y_section + 1) + z_section+1;


  for(int i = 0; i < 6; i++) // Perform interpolation for each joint angle
  {
    // Store corner values for each angle
    double current_joint_values[8]; // This is the number of knot points available for each joint angle
    for(int j = 0; j < 8; j++)
    {
      current_joint_values[j] = values[value_positions[j]][i]; // Expecting the values to be in the right order
    }
  
    joint_angles[i] = interpolate3D(x, y, z, boundary_coords, current_joint_values);

  }
  
  return joint_angles;
}
// TODO: Add a YAML file version of this function
void readCsvto2DVector(const int rows, const int cols, std::string file_path, std::vector<std::vector<double>>& joint_angles)
{
  joint_angles.clear();


  std::vector<double> joints;  // contains the values a row of the csv at a time
  std::ifstream in(file_path);
  std::string line, field;

    for (int i = 0;i < rows; i++)    // get next line in file
    {
      std::getline(in,line); 
      joints.clear();
      std::stringstream ss(line);

      for (int j = 0; j<cols; j++)  // break line into comma delimitted fields
      {
        std::getline(ss,field,',');
        joints.push_back(std::stod(field));  // add each field to the 1D array
      }

      joint_angles.push_back(joints);  // add the 1D array to the 2D array
    }


}

double calculate_polynomial(std::vector<double>coeffs, double var)
{
  // Parameters:
  // coeffs: Vector with coefficients of terms in decreasing degree (, cubic, quadratic, linear, constant)
  // var: The variable to substitute in the equation
  double degree  = coeffs.size()-1;
  double val;
  for(int i =0; i < coeffs.size(); i++)
  {
    val += pow(var, degree-i)*coeffs.at(i);
  }

  return val;
  
}

class PiecewiseSpline
{
    // Class to construct and evaluate a piecewise spline trajectory 

    public:
        // Constructor parameters:
         // knots: Array of knot points, arranged in increasing order of independent variable,format : {{variable, value}, {variable, value} ...}
        //  left_slope:  slope at the left-most knot point
        // right slope: slope at the right-most knot point
        PiecewiseSpline(std::vector<std::vector<double>> knots, double left_slope, double right_slope);
        ~PiecewiseSpline();
        /// \brief Calculate and store the piecewise spline coefficients 
        void calculate_splines();
        /// \brief Store the knot points. 
        // Parameters: 
        // knots: Array of knot points, arranged in increasing order of independent variable
        // format : {{variable, value}, {variable, value} ...}
        void set_knot_points(std::vector<std::vector<double>> knots);
        std::vector<double>get_spline_coefficients();
        // set the slopes of the spline at initial and final points 
        void set_end_point_slopes(double left_slope, double right_slope);
        double evaluate_spline(double var);
        double evaluate_first_differential(double var);
        double evaluate_second_differential(double var);
    private:
        // Vector to hold the coefficients of the piecewise spline in order from section 0 to section n-1
        std::vector<std::vector<double>> coefficients_;
        std::vector<std::vector<double>> knots_; 
        int number_of_knots_; 
        // The slopes at the first and last points   
        double left_slope_;
        double right_slope_;
        Eigen::VectorXd spline_coeffs_;

};

PiecewiseSpline::PiecewiseSpline(std::vector<std::vector<double>> knots, double left_slope, double right_slope)
{
  this->set_knot_points(knots);
  this->set_end_point_slopes(left_slope, right_slope);
  this->calculate_splines();
}
PiecewiseSpline::~PiecewiseSpline(){}
std::vector<double>PiecewiseSpline::get_spline_coefficients()
{
  std::vector<double>coeffs;
  for(int i=0; i<this->spline_coeffs_.size(); i++)
  {
    coeffs.push_back(this->spline_coeffs_(i));
  }
  return coeffs;
}
double PiecewiseSpline::evaluate_second_differential(double var)
{
  if(this->number_of_knots_ < 2)
  {
    // TODO: Add a better method to handle uninitialized spline
    std::cout << "Not enough knot points!" << std::endl;
    return 0.;
  }
  // If var is less than or in the spline's calculated range
  for(int i =0; i < this->number_of_knots_-1; i++)
  {
    if((var >= this->knots_[i][0] && var < this->knots_[i+1][0]) || (var > this->knots_[i][0] && var <= this->knots_[i+1][0]))
    {
      // Spline calculated in parametric form
      double parametric_var;
      if(this->knots_[i+1][0] - this->knots_[i][0] == 0)
      {
        parametric_var = 0;
      }
      else 
        parametric_var = (var-this->knots_[i][0])/(this->knots_[i+1][0] - this->knots_[i][0]);
      
      // std::cout << "left knot is " << knots_[i][0] << " and right knot is " << knots_[i+1][0] << "\n";
      // std::cout << "Parametric var is " << parametric_var << "\n";
      std::vector<double> coeffs = {6.0*this->spline_coeffs_(4*i), 
                                    2.0*this->spline_coeffs_(4*i+1)};   
      
      // for(double c:coeffs){std::cout <<  c << " ";}
      // std::cout << std::endl;                                                         
      return(calculate_polynomial(coeffs, parametric_var));
    }

  }
}
double PiecewiseSpline::evaluate_first_differential(double var)
{
  if(this->number_of_knots_ < 2)
  {
    // TODO: Add a better method to handle uninitialized spline
    std::cout << "Not enough knot points!" << std::endl;
    return 0.;
  }
  // If var is less than or in the spline's calculated range
  for(int i =0; i < this->number_of_knots_-1; i++)
  {
    if((var >= this->knots_[i][0] && var < this->knots_[i+1][0]) || (var > this->knots_[i][0] && var <= this->knots_[i+1][0]))
    {
      // Spline calculated in parametric form
      double parametric_var;
      if(this->knots_[i+1][0] - this->knots_[i][0] == 0)
      {
        parametric_var = 0;
      }
      else 
        parametric_var = (var-this->knots_[i][0])/(this->knots_[i+1][0] - this->knots_[i][0]);
      
      std::vector<double> coeffs = {3.0*this->spline_coeffs_(4*i), 
                                    2.0*this->spline_coeffs_(4*i+1),
                                    this->spline_coeffs_(4*i+2)};   
      
      // for(double c:coeffs){std::cout <<  c << " ";}
      // std::cout << std::endl;                                                         
      return(calculate_polynomial(coeffs, parametric_var));
    }

  }
}

double PiecewiseSpline::evaluate_spline(double var)
{
  if(this->number_of_knots_ < 2)
  {
    // TODO: Add a better method to handle uninitialized spline
    std::cout << "Not enough knot points!" << std::endl;
    return 0.;
  }
  // If var is less than or in the spline's calculated range
  for(int i =0; i < this->number_of_knots_-1; i++)
  {
    if((var >= this->knots_[i][0] && var < this->knots_[i+1][0]) || (var > this->knots_[i][0] && var <= this->knots_[i+1][0]))
    {
      // Spline calculated in parametric form
      double parametric_var;
      if(this->knots_[i+1][0] - this->knots_[i][0] == 0)
      {
        parametric_var = 0;
      }
      else 
        parametric_var = (var-this->knots_[i][0])/(this->knots_[i+1][0] - this->knots_[i][0]);
      
      std::vector<double> coeffs = {this->spline_coeffs_(4*i), 
                                    this->spline_coeffs_(4*i+1),
                                    this->spline_coeffs_(4*i+2),
                                    this->spline_coeffs_(4*i+3)};   
      
      // for(double c:coeffs){std::cout <<  c << " ";}
      // std::cout << std::endl;                                                         
      return(calculate_polynomial(coeffs, parametric_var));
    }

  }
  // If var is greater than the spline's calculated range
  if(var < this->knots_[0][0])
  {
    std::cout << "Provided var smaller than leftmost spline knot. Unreliable return value\n";
    std::vector<double> coeffs = {this->spline_coeffs_(0), 
                                    this->spline_coeffs_(1),
                                    this->spline_coeffs_(2),
                                    this->spline_coeffs_(3)};
    return(calculate_polynomial(coeffs, this->knots_[0][0]));
  }

    if(var > this->knots_[this->number_of_knots_-1][0])
  {
    std::cout << "Provided var larger than rightmost spline knot. Unreliable return value\n";
    std::vector<double> coeffs = {this->spline_coeffs_(4*(this->number_of_knots_-1)-4), 
                                    this->spline_coeffs_(4*(this->number_of_knots_-1)-3),
                                    this->spline_coeffs_(4*(this->number_of_knots_-1)-2),
                                    this->spline_coeffs_(4*(this->number_of_knots_-1)-1)};
    return(calculate_polynomial(coeffs, this->knots_[this->number_of_knots_-1][0]));
  }
  
  
}

void PiecewiseSpline::set_knot_points(std::vector<std::vector<double>> knots)
{
  for(std::vector<double> k:knots)
  {
    this->knots_.push_back(k);
              // variable      //value
    
  }
  // for(int i=0; i < 4; i++)
  // {
  //   for(int j = 0; j < 2; j++)
  //   {
  //     std::cout << this->knots_[i][j]<< " ";
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout << std::endl;

  this->number_of_knots_ = this->knots_.size();
}
void PiecewiseSpline::calculate_splines()
{
  if(this->number_of_knots_ < 2)
  {
    std::cout << "Not enough knot points!" << std::endl;
    return;
  }
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(3*(this->number_of_knots_-1),3*(this->number_of_knots_-1)); // The matrix of linear equations to solve
  Eigen::VectorXd B = Eigen::VectorXd::Zero(3*(this->number_of_knots_-1));

  Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "[", "]");
  //std::cout << M.format(CleanFmt) << "\n--------------------------\n";
  for(int i = 0; i < 3*(this->number_of_knots_-1); i=i+3)
  {
    double param_term;
    int knot_count = i/3;
    if(i != 3*(this->number_of_knots_-2)) // Not the second last point
    {
      param_term = (this->knots_[(knot_count)+1][0] - this->knots_[(knot_count)][0])/(this->knots_[(knot_count+2)][0] - this->knots_[(knot_count)+1][0]);
      M(i+1,i+0) = 1.0;
      M(i+1,i+1) = 1.0; 
      M(i+1,i+2) = 1.0;

      M(i+2,i+0) = 3.0;
      M(i+2,i+1) = 2.0; 
      M(i+2,i+2) = 1.0;
      M(i+2,i+5) = -param_term;
    
      M(i+3,i+0) = 6.0; 
      M(i+3,i+1) = 2.0;
      M(i+3,i+4) = -2*param_term*param_term; 
      // std::cout << "Knot values for this i " << i << " are = " << this->knots_[(knot_count)+1][1] << " " <<  this->knots_[(knot_count)][1]<<std::endl;  
      B(i+1) = this->knots_[(knot_count)+1][1] - this->knots_[knot_count][1];
    }
   
   else
   {
      M(i+1,i+0) = 1.0;
      M(i+1,i+1) = 1.0; 
      M(i+1,i+2) = 1.0;

      M(i+2,i+0) = 3.0;
      M(i+2,i+1) = 2.0; 
      M(i+2,i+2) = 1.0;
      B(i+1) = this->knots_[(knot_count)+1][1] - this->knots_[(knot_count)][1];
      // std::cout << "Knot values for this i " << i << " are = " << this->knots_[(knot_count)+1][1] << " " <<  this->knots_[(knot_count)][1]<<std::endl;  
      B(i+2) = this->right_slope_*(this->knots_[(knot_count)+1][0] - this->knots_[(knot_count)][0]);
   }
    //std::cout << M.format(CleanFmt) << "\n--------------------------\n";
    // std::cout << "Got to this i " << i << std::endl;
  }
  M(0,2) = 1;
  B(0) = this->left_slope_*(this->knots_[1][0] - this->knots_[0][0]);


  Eigen::VectorXd A = (M.inverse())*B;
 
  this->spline_coeffs_ = Eigen::VectorXd::Zero(4*(this->number_of_knots_-1));
  for(int i = 0; i < 4*(this->number_of_knots_-1); i=i+4)
  {
    // Copy coeffs into spline coeffs vector in the appropriate places

      this->spline_coeffs_(i) = A(i - i/4);
      this->spline_coeffs_(i+1) = A((i+1) - (i+1)/4);
      this->spline_coeffs_(i+2) = A((i+2) - (i+2)/4); 
      this->spline_coeffs_(i+3) = this->knots_[(i/4)][1];

    
  }
  //std::cout << this->spline_coeffs_.format(CleanFmt) << "\n--------------------------\n";
}
void PiecewiseSpline::set_end_point_slopes(double left_slope, double right_slope)
{
  this->left_slope_ = left_slope;
  this->right_slope_ = right_slope;
}
