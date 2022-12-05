#include "mpc_input_bridge.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

MPCInputBridge::MPCInputBridge()
  : MPCInputBridge(ros::NodeHandle()) {}

MPCInputBridge::MPCInputBridge(const ros::NodeHandle &nh)
  : nh_(nh) {
  // Subscribers
  odom_sub_ = nh_.subscribe("odom", 1, &MPCInputBridge::odomCallback, this,
      ros::TransportHints().tcpNoDelay());
  enable_motors_sub_ = nh_.subscribe("enable_motors", 1,
      &MPCInputBridge::enableMotorsCallback, this);
  position_command_sub_ = nh_.subscribe("position_cmd", 1,
      &MPCInputBridge::positionCommandCallback ,this);

  // Publishers
  start_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1);
  off_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/off", 1);
  trajectory_pub_ =
      nh_.advertise<quadrotor_msgs::Trajectory>("autopilot/trajectory", 1);
  trajectory_point_pub_ = nh_.advertise<quadrotor_msgs::TrajectoryPoint>(
    "autopilot/reference_state", 1);
}

void MPCInputBridge::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_odom_ = *msg;
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
  quadrotor_msgs::Trajectory::Ptr p_reference_traj =
    boost::make_shared<quadrotor_msgs::Trajectory>();
  quadrotor_msgs::TrajectoryPoint start_point;
  quadrotor_msgs::TrajectoryPoint goal_point;

  // Before we send out the trajectory first check that it isn't a previously
  // sent trajectory, only send new trajectories
  if (std::abs((msg->position.x - last_goal_.x) < MPCInputBridge::EPSILON)
      && std::abs((msg->position.y - last_goal_.y) < MPCInputBridge::EPSILON)
      && std::abs((msg->position.z - last_goal_.z) < MPCInputBridge::EPSILON))
  {
    return;
  }
  else
  {
    last_goal_ = msg->position;
  }

  // Fill the message
  p_reference_traj->header = msg->header;

  start_point.time_from_start.sec = 0;
  start_point.time_from_start.nsec = 0;

  goal_point.time_from_start.sec = 2;
  goal_point.time_from_start.nsec = 0;

  start_point.pose.position = current_odom_.pose.pose.position;
  goal_point.pose.position = msg->position;

  start_point.heading = msg->yaw;
  start_point.heading_rate = msg->yaw_dot;
  start_point.heading_acceleration = 0.0f;

  tf2::Quaternion quat_tf2;
  quat_tf2.setRPY(0, 0, msg->yaw);
  geometry_msgs::Quaternion quat_msg;
  quat_msg = tf2::toMsg(quat_tf2);
  start_point.pose.orientation = quat_msg;
  goal_point.pose.orientation = quat_msg;

  goal_point.velocity.linear = msg->velocity;
  goal_point.velocity.angular = geometry_msgs::Vector3();

  goal_point.acceleration.linear  = msg->acceleration;
  goal_point.acceleration.angular = geometry_msgs::Vector3();

  goal_point.jerk.linear = msg->jerk;
  goal_point.jerk.angular = geometry_msgs::Vector3();

  goal_point.snap.linear = geometry_msgs::Vector3();
  goal_point.snap.angular = geometry_msgs::Vector3();

  goal_point.heading = msg->yaw;
  goal_point.heading_rate = msg->yaw_dot;
  goal_point.heading_acceleration = 0.0f;

  // Publish the message
  p_reference_traj->type = 3;
  p_reference_traj->points.push_back(start_point);
  p_reference_traj->points.push_back(goal_point);
  trajectory_pub_.publish(p_reference_traj);
}
