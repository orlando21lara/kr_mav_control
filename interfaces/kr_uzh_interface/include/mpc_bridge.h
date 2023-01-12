#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <quadrotor_msgs/Trajectory.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <kr_mav_msgs/SO3Command.h>
#include <quadrotor_msgs/ControlCommand.h>

#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



class MPCBridge {
 public:
  MPCBridge();
  MPCBridge(const ros::NodeHandle &nh);

 private:
  void enableMotorsCallback(const std_msgs::Bool::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void positionCommandCallback(const kr_mav_msgs::PositionCommand::ConstPtr  &msg);
  void controlCommandCallback(const quadrotor_msgs::ControlCommand::ConstPtr &msg);



  ros::NodeHandle nh_;                    // ROS nodehandle

  // Subscribers to topics of the kr_mav_control pipeline
  ros::Subscriber enable_motors_sub_;     // Enable motors coming from upstream
  ros::Subscriber position_command_sub_;  // Position command from trackers
  ros::Subscriber control_command_sub_;         // Control output from mpc
  ros::Subscriber odom_sub_;

  // Publishers to topics to the kr_mav_control
  ros::Publisher so3_command_pub_;  // Control output to kr_mav_control pipeline
  // Publishers to topics that will be sent to the uzh_mpc_controller
  ros::Publisher start_pub_;              // Enable motors
  ros::Publisher off_pub_;                // Disable motors
  ros::Publisher trajectory_point_pub_;   // Point trajectory for mpc controller


  nav_msgs::Odometry odom_;


 private:


  
  // SO3 Parameters
  double mass_;         // In kg
  double kf_correction_;
  double angle_corrections_[2];
  bool enable_motors_;
  bool use_external_yaw_;
  double kR_[3];
  double kOm_[3];

};