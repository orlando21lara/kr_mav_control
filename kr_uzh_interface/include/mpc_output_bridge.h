#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <kr_mav_msgs/SO3Command.h>
#include <quadrotor_msgs/ControlCommand.h>

class MPCOutputBridge {
 public:
  MPCOutputBridge();
  MPCOutputBridge(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  void loadParameters(void);

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void enableMotorsCallback(const std_msgs::Bool::ConstPtr &msg);
  void controlCommandCallback(const quadrotor_msgs::ControlCommand::ConstPtr
      &msg);

  ros::NodeHandle nh_;      // ROS node handle
  ros::NodeHandle pnh_;     // Private ROS node handle

  // Subscribers to topics from the uzh_mpc_controller
  ros::Subscriber control_command_sub_;         // Control output from mpc

  // Publishers to topics to the kr_mav_control
  ros::Publisher so3_command_pub_;  // Control output to kr_mav_control pipeline

  // Need odom to use current orientation for data message conversions
  ros::Subscriber odom_sub_;
  ros::Subscriber enable_motors_sub_;
  nav_msgs::Odometry odom_;
  
  // SO3 Parameters
  double mass_;         // In kg
  double kf_correction_;
  double angle_corrections_[2];
  bool enable_motors_;
  bool use_external_yaw_;
  double kR_[3];
  double kOm_[3];
};