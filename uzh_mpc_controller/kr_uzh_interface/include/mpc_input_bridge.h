#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <quadrotor_msgs/Trajectory.h>

class MPCInputBridge {
 public:
  MPCInputBridge();
  MPCInputBridge(const ros::NodeHandle &nh);

 private:
  ros::NodeHandle nh_;

  // Subscribers to topics of the kr_mav_control pipeline
  ros::Subscriber enable_motors_sub_, position_command_sub_;

  // Publishers to topics that will be sent to the uzh_mpc_controller
  ros::Publisher start_pub_, off_pub_, trajectory_pub_, trajectory_point_pub_;

  bool  enable_motors_;

  void enableMotorsCallback(const std_msgs::Bool::ConstPtr &msg);
  void positionCommandCallback(const kr_mav_msgs::PositionCommand::ConstPtr
      &msg);
};