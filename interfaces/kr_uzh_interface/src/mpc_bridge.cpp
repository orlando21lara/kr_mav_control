#include "mpc_bridge.h"



MPCBridge::MPCBridge()
  : MPCBridge(ros::NodeHandle()) {}

MPCBridge::MPCBridge(const ros::NodeHandle &nh)
  : nh_(nh) {

  nh_.param("mass", mass_, 0.5);
  nh_.param("use_external_yaw", use_external_yaw_, true);

  nh_.param("corrections/kf", kf_correction_, 0.0);
  nh_.param("corrections/r", angle_corrections_[0], 0.0);
  nh_.param("corrections/p", angle_corrections_[1], 0.0);

  nh_.param("gains/rot/x", kR_[0], 1.5);
  nh_.param("gains/rot/y", kR_[1], 1.5);
  nh_.param("gains/rot/z", kR_[2], 1.0);

  nh_.param("gains/ang/x", kOm_[0], 0.13);
  nh_.param("gains/ang/y", kOm_[1], 0.13);
  nh_.param("gains/ang/z", kOm_[2], 0.1);

  
  // Subscribers
  enable_motors_sub_ = nh_.subscribe("enable_motors", 1,
      &MPCBridge::enableMotorsCallback, this);
  position_command_sub_ = nh_.subscribe("position_cmd", 1,
      &MPCBridge::positionCommandCallback ,this);

  // Publishers
  start_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/start", 1);
  off_pub_ = nh_.advertise<std_msgs::Empty>("autopilot/off", 1);
  trajectory_point_pub_ = nh_.advertise<quadrotor_msgs::TrajectoryPoint>(
    "autopilot/reference_state", 1);

  control_command_sub_ = nh_.subscribe("control_command", 1,
      &MPCBridge::controlCommandCallback, this);
  odom_sub_ = nh_.subscribe("odom", 1, &MPCBridge::odomCallback, this,
      ros::TransportHints().tcpNoDelay());
  // Publishers
  so3_command_pub_ = nh_.advertise<kr_mav_msgs::SO3Command>("so3_cmd", 1);


}

void MPCBridge::enableMotorsCallback(
    const std_msgs::Bool::ConstPtr &msg) {
  enable_motors_ = msg->data;
  std_msgs::Empty pub_msg;
  if (enable_motors_ == true) {
    start_pub_.publish(pub_msg);
  } else {
    off_pub_.publish(pub_msg);
  }
}

void MPCBridge::positionCommandCallback(
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


void MPCBridge::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  odom_ = *msg;
}


void MPCBridge::controlCommandCallback(
    const quadrotor_msgs::ControlCommand::ConstPtr &msg) {
  // Received a message of type quadrotor_msgs::ControlCommand from the
  // uzh_mpc_controller and need to publish a message to the
  // kr_mav_control pipeline of type kr_mav_msgs::SO3Command
  kr_mav_msgs::SO3Command::Ptr so3_cmd =
      boost::make_shared<kr_mav_msgs::SO3Command>();

  so3_cmd->header = msg->header;
  so3_cmd->header.frame_id = odom_.header.frame_id;

  /**
   * Now we need to compute the force. In the SO3 command, the desired force is
   * given in Newtons in the world frame. In the Control command, instead of a
   * force, we get the collective mass normalized thrust (m/s^2). To do the
   * conversion we first multply the mass by the collective thrust to get the
   * desired force in the body frame. Then we transform the force from the body
   * frame to the world frame (intertial frame). f_des = R * [0, 0, u1]^T where
   * R is a rotation matrix representing the orientation of the body frame with
   * respect to the world frame and u1 is the collective thrust.
   */
  tf2::Quaternion quat_tf2;
  tf2::fromMsg(odom_.pose.pose.orientation, quat_tf2);
  // Current orientation of body frame as a rotation matrix
  tf2::Matrix3x3 rot_cur(quat_tf2);
  double u1 = msg->collective_thrust * mass_;
  so3_cmd->force.x = u1 * rot_cur[0][2];
  so3_cmd->force.y = u1 * rot_cur[1][2];
  so3_cmd->force.z = u1 * rot_cur[2][2];

  // Next part of the message is the orientation
  so3_cmd->orientation = msg->orientation;

  // Need to check if the angular velocity should be in body frame or world
  // frame. Currently considering it to be in body frame
  so3_cmd->angular_velocity = msg->bodyrates;

  // Fill the aux portion
  so3_cmd->aux.current_yaw = tf::getYaw(odom_.pose.pose.orientation);
  so3_cmd->aux.kf_correction = kf_correction_;
  so3_cmd->aux.angle_corrections[0] = angle_corrections_[0];
  so3_cmd->aux.angle_corrections[1] = angle_corrections_[1];
  so3_cmd->aux.enable_motors = enable_motors_;
  so3_cmd->aux.use_external_yaw = use_external_yaw_;

  // Fill the kR and kOm gains
  so3_cmd->kR[0] = kR_[0];
  so3_cmd->kR[1] = kR_[1];
  so3_cmd->kR[2] = kR_[2];
  so3_cmd->kOm[0] = kOm_[0];
  so3_cmd->kOm[1] = kOm_[1];
  so3_cmd->kOm[2] = kOm_[2];

  // Finally publish the message
  so3_command_pub_.publish(so3_cmd);
}
