#include "mpc_input_bridge.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

MPCInputBridge::MPCInputBridge()
  : MPCInputBridge(ros::NodeHandle()) {}

MPCInputBridge::MPCInputBridge(const ros::NodeHandle &nh)
  : nh_(nh) {
  // Subscribers
  enable_motors_sub_ = nh_.subscribe("enable_motors", 1,
      &MPCInputBridge::enableMotorsCallback, this);
  position_command_sub_ = nh_.subscribe("position_cmd", 1,
      &MPCInputBridge::positionCommandCallback ,this);

  // Publishers
  start_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1);
  off_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/off", 1);
  trajectory_point_pub_ = nh_.advertise<quadrotor_msgs::TrajectoryPoint>(
    "autopilot/reference_state", 1);
}

void MPCInputBridge::enableMotorsCallback(
    const std_msgs::Bool::ConstPtr &msg) {
  enable_motors_ = msg->data;
  std_msgs::Empty pub_msg;
  if (enable_motors_ == true) {
    start_pub_.publish(pub_msg);
  } else {
    off_pub_.publish(pub_msg);
  }
}

void MPCInputBridge::positionCommandCallback(
    const kr_mav_msgs::PositionCommand::ConstPtr &msg) {
  // Received a message of type kr_mav_msgs::PositionCommand from the
  // kr_mav_control pipeline and need to publish a message to the
  // uzh_mpc_controller of type quadrotor_msgs::TrajectoryPoint
  quadrotor_msgs::TrajectoryPoint::Ptr p_reference_traj_pt =
    boost::make_shared<quadrotor_msgs::TrajectoryPoint>();
  quadrotor_msgs::TrajectoryPoint goal_point;

  // Fill the message
  tf2::Quaternion quat_tf2;
  quat_tf2.setRPY(0, 0, msg->yaw);
  geometry_msgs::Quaternion quat_msg;
  quat_msg = tf2::toMsg(quat_tf2);

  p_reference_traj_pt->pose.orientation = quat_msg;
  p_reference_traj_pt->pose.position = msg->position;

  p_reference_traj_pt->velocity.linear = msg->velocity;
  p_reference_traj_pt->velocity.angular = geometry_msgs::Vector3();

  p_reference_traj_pt->acceleration.linear  = msg->acceleration;
  p_reference_traj_pt->acceleration.angular = geometry_msgs::Vector3();

  p_reference_traj_pt->jerk.linear = msg->jerk;
  p_reference_traj_pt->jerk.angular = geometry_msgs::Vector3();

  p_reference_traj_pt->snap.linear = geometry_msgs::Vector3();
  p_reference_traj_pt->snap.angular = geometry_msgs::Vector3();

  p_reference_traj_pt->heading = msg->yaw;
  p_reference_traj_pt->heading_rate = msg->yaw_dot;
  p_reference_traj_pt->heading_acceleration = 0.0f;

  // Publish the message
  trajectory_point_pub_.publish(p_reference_traj_pt);
}
