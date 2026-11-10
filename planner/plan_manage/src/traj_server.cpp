#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

using namespace Eigen;

ros::Publisher pos_cmd_pub;
ros::Publisher yaw_vis_pub;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;
double commanded_yaw_;
double yaw_dot_max_per_sec_, yaw_dot_dot_max_per_sec_;
bool use_commanded_yaw_;
bool spread_commanded_yaw_over_traj_;
bool commanded_yaw_dynamic_mode_;
double commanded_yaw_dynamic_threshold_;
double commanded_yaw_init_window_sec_;
double commanded_yaw_reach_ratio_;
double traj_start_yaw_for_cmd_;
double traj_start_cmd_yaw_;
ros::Time traj_receive_time_(0);

double normalizeAngle(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

double shortestAngleDiff(double from, double to)
{
  return normalizeAngle(to - from);
}

double interpolateYawByProgress(double start_yaw, double target_yaw, double progress)
{
  progress = std::max(0.0, std::min(1.0, progress));
  return normalizeAngle(start_yaw + shortestAngleDiff(start_yaw, target_yaw) * progress);
}

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void yawCmdCallback(const std_msgs::Float64ConstPtr &msg)
{
  const double new_cmd = normalizeAngle(msg->data);

  if (receive_traj_)
  {
    const double t_since_traj = (ros::Time::now() - traj_receive_time_).toSec();
    if (t_since_traj <= commanded_yaw_init_window_sec_)
    {
      // Allow initial command sync right after a new trajectory is received.
      traj_start_cmd_yaw_ = new_cmd;
    }
    else if (use_commanded_yaw_ && spread_commanded_yaw_over_traj_)
    {
      // If command keeps changing within one trajectory, treat it as a continuous external mode.
      const double delta_from_start_cmd = std::fabs(shortestAngleDiff(traj_start_cmd_yaw_, new_cmd));
      if (delta_from_start_cmd > commanded_yaw_dynamic_threshold_)
      {
        commanded_yaw_dynamic_mode_ = true;
      }
    }
  }

  commanded_yaw_ = new_cmd;
}

void useYawCmdCallback(const std_msgs::BoolConstPtr &msg)
{
  use_commanded_yaw_ = msg->data;
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;
  traj_receive_time_ = ros::Time::now();

  // Reset commanded-yaw scheduling state for this new trajectory.
  traj_start_yaw_for_cmd_ = last_yaw_;
  traj_start_cmd_yaw_ = commanded_yaw_;
  commanded_yaw_dynamic_mode_ = false;

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  std::pair<double, double> yaw_yawdot(0, 0);
  dt = std::max(dt, 1e-3);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = 0.0;
  if (use_commanded_yaw_)
  {
    if (spread_commanded_yaw_over_traj_ && !commanded_yaw_dynamic_mode_ && traj_duration_ > 1e-3)
    {
      // Finish commanded yaw earlier in the segment to leave margin before waypoint arrival.
      const double progress = t_cur / traj_duration_;
      const double reach_ratio = std::max(1e-3, commanded_yaw_reach_ratio_);
      const double yaw_progress = std::min(1.0, progress / reach_ratio);
      yaw_temp = interpolateYawByProgress(traj_start_yaw_for_cmd_, commanded_yaw_, yaw_progress);
    }
    else
    {
      yaw_temp = commanded_yaw_;
    }
  }
  else
  {
    yaw_temp = dir.norm() > 0.1
                   ? atan2(dir(1), dir(0))
                   : last_yaw_;
  }

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? yaw_dot_max_per_sec_ : -yaw_dot_max_per_sec_;
  const double YDDM = d_yaw >= 0 ? yaw_dot_dot_max_per_sec_ : -yaw_dot_dot_max_per_sec_;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  return yaw_yawdot;
}

void publish_yaw_marker(const Vector3d &p, double y)
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.ns = "command_yaw";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.2;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.15;
  marker.scale.y = 0.30;
  marker.scale.z = 0.35;

  geometry_msgs::Point start, end;
  start.x = p(0);
  start.y = p(1);
  start.z = p(2) + 0.15;
  end.x = p(0) + 1.5 * cos(y);
  end.y = p(1) + 1.5 * sin(y);
  end.z = p(2) + 0.15;
  marker.points.push_back(start);
  marker.points.push_back(end);

  yaw_vis_pub.publish(marker);
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);
  publish_yaw_marker(p, y);

  last_pos_ = p;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();

  if ((time_now - heartbeat_time_).toSec() > 0.5)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");

    receive_traj_ = false;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
  static bool finished = false;
#endif
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
    /*** calculate yaw ***/

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
    if (slowly_flip_yaw_target_ > M_PI)
      slowly_flip_yaw_target_ -= 2 * M_PI;
    if (slowly_flip_yaw_target_ < -M_PI)
      slowly_flip_yaw_target_ += 2 * M_PI;
    constexpr double CENTER[2] = {0.0, 0.0};
    slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

    // publish
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
    finished = false;
#endif
  }

#if FLIP_YAW_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    if (slowly_flip_yaw_target_ > 0)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }
    else
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif

#if TURN_YAW_TO_CENTER_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
    if (d_yaw >= M_PI)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ > M_PI)
        last_yaw_ -= 2 * M_PI;
    }
    else if (d_yaw <= -M_PI)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ < -M_PI)
        last_yaw_ += 2 * M_PI;
    }
    else if (d_yaw >= 0)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_turn_to_center_target_)
        finished = true;
    }
    else
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_turn_to_center_target_)
        finished = true;
    }

    yaw_yawdot.first = last_yaw_;
    time_last = time_now;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  ros::Subscriber yaw_cmd_sub = nh.subscribe("planning/yaw_cmd", 10, yawCmdCallback);
  ros::Subscriber use_yaw_cmd_sub = nh.subscribe("planning/use_yaw_cmd", 10, useYawCmdCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  yaw_vis_pub = nh.advertise<visualization_msgs::Marker>("yaw_vis", 10);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/yaw_dot_max_per_sec", yaw_dot_max_per_sec_, 2 * M_PI);
  nh.param("traj_server/yaw_dot_dot_max_per_sec", yaw_dot_dot_max_per_sec_, 5 * M_PI);
  nh.param("traj_server/spread_commanded_yaw_over_traj", spread_commanded_yaw_over_traj_, true);
  nh.param("traj_server/commanded_yaw_dynamic_threshold", commanded_yaw_dynamic_threshold_, 0.2);
  nh.param("traj_server/commanded_yaw_init_window_sec", commanded_yaw_init_window_sec_, 0.2);
  nh.param("traj_server/commanded_yaw_reach_ratio", commanded_yaw_reach_ratio_, 0.8);

  if (commanded_yaw_reach_ratio_ <= 1e-3 || commanded_yaw_reach_ratio_ > 1.0)
  {
    ROS_WARN("[traj_server] commanded_yaw_reach_ratio should be in (0, 1], got %.3f. Clamping.", commanded_yaw_reach_ratio_);
    commanded_yaw_reach_ratio_ = std::max(1e-3, std::min(1.0, commanded_yaw_reach_ratio_));
  }

  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;
  commanded_yaw_ = 0.0;
  use_commanded_yaw_ = false;
  commanded_yaw_dynamic_mode_ = false;
  traj_start_yaw_for_cmd_ = 0.0;
  traj_start_cmd_yaw_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}