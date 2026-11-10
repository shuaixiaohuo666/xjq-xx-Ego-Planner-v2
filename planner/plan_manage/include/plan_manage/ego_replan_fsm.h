#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <cstdint>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <gimbal/GimbalCmd.h>
#include <gimbal/GimbalState.h>
#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <quadrotor_msgs/VlaStatus.h>
#include <quadrotor_msgs/VlaWaypoint.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {
  public:
    EGOReplanFSM() {}
    ~EGOReplanFSM() {}

    static constexpr int kMaxWaypoints = 120;

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[kMaxWaypoints][3];
    double waypoints_yaw_[kMaxWaypoints];
    double waypoints_cam_pitch_[kMaxWaypoints];
    int waypoint_num_, wpt_id_;
    double planning_horizen_;
    double emergency_time_;
    double planning_delay_;
    bool flag_realworld_experiment_;
    bool use_rc_trigger_in_manual_;
    double last_planning_time_;
    bool enable_fail_safe_;
    bool enable_ground_height_measurement_;
    bool flag_escape_emergency_;
    bool print_trj_vaj_;
    bool auto_repeat_mission_{true};
    double waypoint_hold_velocity_thresh_{0.15};
    bool force_global_replan_once_{false};

    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
    Eigen::Vector3d final_goal_;                         // goal state
    double final_goal_yaw_{0.0};
    bool final_goal_use_yaw_{false};
    Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
    double current_yaw_rad_{0.0};
    std::vector<Eigen::Vector3d> wps_;
    std::vector<double> wps_yaw_;
    std::vector<double> wps_cam_pitch_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, vla_waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_;
    ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_;
    ros::Publisher yaw_cmd_pub_, use_yaw_cmd_pub_, gimbal_cmd_pub_, vla_status_pub_;
    ros::Subscriber gimbal_state_sub_;

    void print_trajectory_vaj(const std::string& traj_type_info);
    
    /* state machine functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    void printFSMExecState();
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

    /* safety */
    void checkCollisionCallback(const ros::TimerEvent &e);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);

    /* local planning */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj, double &planning_time);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(const int trial_times = 1);

    /* global trajectory */
    void waypointCallback(const quadrotor_msgs::GoalSetPtr &msg);
    void vlaWaypointCallback(const quadrotor_msgs::VlaWaypoint::ConstPtr &msg);
    void readGivenWpsAndPlan();
    bool planNextWaypoint(const Eigen::Vector3d next_wp, double next_yaw = 0.0, bool use_yaw = false);
    bool mondifyInCollisionFinalGoal();

    /* input-output */
    void mandatoryStopCallback(const std_msgs::Empty &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);
    void gimbalStateCallback(const gimbal::GimbalState::ConstPtr &msg);

    /* gimbal waypoint task */
    bool isGimbalActionWaypoint(int waypoint_idx) const;
    bool shouldTriggerGimbalActionForCurrentWaypoint(const Eigen::Vector3d &pos) const;
    void sendCurrentWaypointGimbalCmd();
    bool startHoldForCurrentWaypoint();
    bool advanceMissionFromCurrentWaypoint();
    void completeMission();
    void resetGimbalWaitState();
    bool isVlaBusy() const;
    void publishVlaStatus(uint8_t status, const std::string &message) const;
    double getYawErrorDeg(double target_yaw_rad) const;
    bool hasReachedVlaPositionYaw() const;
    bool shouldStartVlaHold() const;
    bool startHoldForCurrentTarget();
    void startVlaGimbalAction();
    bool isCurrentVlaCmdTimedOut() const;
    void clearVlaTask();

    /* ground height measurement */
    bool measureGroundHeight(double &height);

    std::vector<int> gimbal_action_waypoint_indices_;
    double gimbal_pitch_reach_thresh_deg_{2.0};
    double gimbal_waypoint_reach_thresh_m_{0.25};
    bool waiting_gimbal_action_{false};
    bool gimbal_cmd_ack_ok_{false};
    bool gimbal_pitch_reached_{false};
    bool gimbal_action_sent_for_current_wpt_{false};
    bool gimbal_hold_traj_sent_for_current_wpt_{false};
    int waiting_gimbal_waypoint_idx_{-1};
    double waiting_gimbal_target_pitch_deg_{0.0};

    bool vla_active_{false};
    uint32_t vla_seq_id_{0};
    Eigen::Vector3d vla_target_pos_{Eigen::Vector3d::Zero()};
    double vla_target_yaw_rad_{0.0};
    double vla_target_pitch_deg_{0.0};
    double vla_position_tolerance_{0.25};
    double vla_yaw_tolerance_deg_{8.0};
    double vla_pitch_tolerance_deg_{2.0};
    double vla_timeout_sec_{20.0};
    ros::Time vla_cmd_start_time_;
    bool vla_hold_started_{false};
    bool vla_gimbal_cmd_sent_{false};
  };

} // namespace ego_planner

#endif
