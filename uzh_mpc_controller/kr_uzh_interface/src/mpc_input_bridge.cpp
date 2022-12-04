#include "mpc_input_bridge.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
  trajectory_pub_ =
      nh_.advertise<quadrotor_msgs::Trajectory>("autopilot/trajectory", 1);
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
  // quadrotor_msgs::Trajectory::Ptr p_reference_traj =
  //   boost::make_shared<quadrotor_msgs::Trajectory>();
  // quadrotor_msgs::TrajectoryPoint reference_point;

  // // Fill the message
  // p_reference_traj->header = msg->header;
  // // reference_point.time_from_start.sec = msg->header.stamp.sec;
  // // reference_point.time_from_start.nsec = msg->header.stamp.nsec;
  // reference_point.time_from_start.sec = 0;
  // reference_point.time_from_start.nsec = 0;

  // reference_point.pose.position = msg->position;
  // tf2::Quaternion quat_tf2;
  // quat_tf2.setRPY(0, 0, msg->yaw);
  // geometry_msgs::Quaternion quat_msg;
  // quat_msg = tf2::toMsg(quat_tf2);
  // reference_point.pose.orientation = quat_msg;

  // reference_point.velocity.linear = msg->velocity;
  // reference_point.velocity.angular = geometry_msgs::Vector3();

  // reference_point.acceleration.linear  = msg->acceleration;
  // reference_point.acceleration.angular = geometry_msgs::Vector3();

  // reference_point.jerk.linear = msg->jerk;
  // reference_point.jerk.angular = geometry_msgs::Vector3();

  // reference_point.snap.linear = geometry_msgs::Vector3();
  // reference_point.snap.angular = geometry_msgs::Vector3();

  // reference_point.heading = msg->yaw;
  // reference_point.heading_rate = msg->yaw_dot;
  // reference_point.heading_acceleration = 0.0f;

  // // Publish the message
  // p_reference_traj->type = 3;
  // p_reference_traj->points.push_back(reference_point);
  // trajectory_pub_.publish(p_reference_traj);
  // // trajectory_point_pub_.publish(p_reference_point);

  quadrotor_msgs::Trajectory::Ptr p_reference_traj =
    boost::make_shared<quadrotor_msgs::Trajectory>();
  quadrotor_msgs::TrajectoryPoint start_point;
  quadrotor_msgs::TrajectoryPoint goal_point;

  // Fill the message
  p_reference_traj->header = msg->header;
  start_point.time_from_start.sec = 0;
  start_point.time_from_start.nsec = 0;

  goal_point.time_from_start.sec = 2;
  goal_point.time_from_start.nsec = 0;

  start_point.pose.position.x = 0;
  start_point.pose.position.y = 0;
  start_point.pose.position.z = 0.4;

  start_point.heading = -0.78;
  start_point.heading_rate = 0;
  start_point.heading_acceleration = 0;

  tf2::Quaternion quat_tf2;
  quat_tf2.setRPY(0, 0, msg->yaw);
  geometry_msgs::Quaternion quat_msg;
  quat_msg = tf2::toMsg(quat_tf2);
  goal_point.pose.orientation = quat_msg;
  start_point.pose.orientation = quat_msg;

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
