#include <ros/ros.h>
#include <trackers_manager/Tracker.h>

class NullTracker : public trackers_manager::Tracker
{
 public:
  void Initialize(const ros::NodeHandle &nh);
  bool Activate(void);
  void Deactivate(void);

  const quadrotor_msgs::PositionCommand::Ptr update(const nav_msgs::Odometry::ConstPtr &msg);
};

void NullTracker::Initialize(const ros::NodeHandle &nh)
{
}

bool NullTracker::Activate(void)
{
  return true;
}

void NullTracker::Deactivate(void)
{
}

const quadrotor_msgs::PositionCommand::Ptr NullTracker::update(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Return a null message (will not publish the position command)
  return quadrotor_msgs::PositionCommand::Ptr();
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(NullTracker, trackers_manager::Tracker);