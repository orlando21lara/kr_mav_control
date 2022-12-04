#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_mav_msgs/SO3Command.h>
#include <quadrotor_msgs/ControlCommand.h>
#include <quadrotor_msgs/Trajectory.h>

class ControllerBridge {
 public:
  ControllerBridge();
  ControllerBridge(const ros::NodeHandle &nh);

 private:
  ros::NodeHandle nh_;

  // Need odom to use current orientation for data message conversions
  nav_msgs::Odometry odom_;
  ros::Subscriber odom_sub_;

  // Subscribers to topics of the kr_mav_control pipeline. These are used to
  // create messages to send to the uzh_mpc_controller
  ros::Subscriber enable_motors_sub_, position_command_sub_;

  // Subscribers to topics of the uzh_mpc_controller. These are used to create
  // messages to send to the rest of the kr_mav_control pipeline
  ros::Subscriber control_command_sub_;

  // Publishers to topics that will be sent to the uzh_mpc_controller
  ros::Publisher start_pub_, off_pub_, trajectory_pub_, trajectory_point_pub_;

  // Publishers to topics that will be sent to the rest of the kr_mav_control
  // pipeline
  ros::Publisher so3_command_pub_;

  // Member variables
  double mass_;         // In kg
  bool  enable_motors_;


  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void enableMotorsCallback(const std_msgs::Bool::ConstPtr &msg);
  void positionCommandCallback(const kr_mav_msgs::PositionCommand::ConstPtr
      &msg);
  void controlCommandCallback(const quadrotor_msgs::ControlCommand::ConstPtr
      &msg);
};