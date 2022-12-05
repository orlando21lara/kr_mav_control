#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <quadrotor_msgs/Trajectory.h>

class MPCInputBridge {
 public:
  MPCInputBridge();
  MPCInputBridge(const ros::NodeHandle &nh);

 private:
  void enableMotorsCallback(const std_msgs::Bool::ConstPtr &msg);
  void positionCommandCallback(const kr_mav_msgs::PositionCommand::ConstPtr
      &msg);

  ros::NodeHandle nh_;                    // ROS nodehandle

  // Subscribers to topics of the kr_mav_control pipeline
  ros::Subscriber enable_motors_sub_;     // Enable motors coming from upstream
  ros::Subscriber position_command_sub_;  // Position command from trackers

  // Publishers to topics that will be sent to the uzh_mpc_controller
  ros::Publisher start_pub_;              // Enable motors
  ros::Publisher off_pub_;                // Disable motors
  ros::Publisher trajectory_point_pub_;   // Point trajectory for mpc controller

  bool  enable_motors_;
};