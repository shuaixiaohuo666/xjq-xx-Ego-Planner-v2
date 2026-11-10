#include <cmath>
#include <string>

#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

class OrbitFaceCenterYawNode
{
public:
  OrbitFaceCenterYawNode()
    : nh_(), pnh_("~"), has_cmd_(false), last_yaw_(0.0)
  {
    pnh_.param<std::string>("cmd_topic", cmd_topic_, "/drone_0_planning/pos_cmd");
    pnh_.param<std::string>("yaw_cmd_topic", yaw_cmd_topic_, "/drone_0_ego_planner_node/planning/yaw_cmd");
    pnh_.param<std::string>("use_yaw_cmd_topic", use_yaw_cmd_topic_, "/drone_0_ego_planner_node/planning/use_yaw_cmd");
    pnh_.param("center_x", center_x_, 2.0);
    pnh_.param("center_y", center_y_, 0.0);
    pnh_.param("publish_rate", publish_rate_, 100.0);
    pnh_.param("cmd_timeout", cmd_timeout_, 0.2);
    pnh_.param("min_distance", min_distance_, 0.05);

    if (publish_rate_ <= 0.0)
      publish_rate_ = 100.0;

    cmd_sub_ = nh_.subscribe(cmd_topic_, 50, &OrbitFaceCenterYawNode::cmdCallback, this);
    yaw_pub_ = nh_.advertise<std_msgs::Float64>(yaw_cmd_topic_, 10, true);
    use_yaw_pub_ = nh_.advertise<std_msgs::Bool>(use_yaw_cmd_topic_, 10, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &OrbitFaceCenterYawNode::timerCallback, this);

    std_msgs::Bool use_msg;
    use_msg.data = true;
    use_yaw_pub_.publish(use_msg);

    ROS_INFO("[orbit_face_center_yaw_node] started, center=(%.3f, %.3f), publish_rate=%.1f Hz",
             center_x_, center_y_, publish_rate_);
  }

private:
  void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
  {
    latest_x_ = msg->position.x;
    latest_y_ = msg->position.y;
    last_cmd_stamp_ = ros::Time::now();
    has_cmd_ = true;
  }

  void timerCallback(const ros::TimerEvent &)
  {
    std_msgs::Bool use_msg;
    use_msg.data = true;
    use_yaw_pub_.publish(use_msg);

    if (!has_cmd_)
      return;

    if ((ros::Time::now() - last_cmd_stamp_).toSec() > cmd_timeout_)
      return;

    const double dx = center_x_ - latest_x_;
    const double dy = center_y_ - latest_y_;

    if (std::hypot(dx, dy) > min_distance_)
      last_yaw_ = std::atan2(dy, dx);

    std_msgs::Float64 yaw_msg;
    yaw_msg.data = last_yaw_;
    yaw_pub_.publish(yaw_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber cmd_sub_;
  ros::Publisher yaw_pub_;
  ros::Publisher use_yaw_pub_;
  ros::Timer timer_;

  std::string cmd_topic_;
  std::string yaw_cmd_topic_;
  std::string use_yaw_cmd_topic_;

  double center_x_;
  double center_y_;
  double publish_rate_;
  double cmd_timeout_;
  double min_distance_;

  bool has_cmd_;
  ros::Time last_cmd_stamp_;
  double latest_x_;
  double latest_y_;
  double last_yaw_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orbit_face_center_yaw_node");
  OrbitFaceCenterYawNode node;
  ros::spin();
  return 0;
}
