#include <cmath>
#include <iomanip>
#include <sstream>
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/use_rc_trigger_in_manual", use_rc_trigger_in_manual_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/ground_height_measurement", enable_ground_height_measurement_, false);
    nh.param("planning_delay", planning_delay_, 0.0);
    nh.param("print_trj_vaj", print_trj_vaj_, false);
    nh.param("fsm/auto_repeat_mission", auto_repeat_mission_, true);
    nh.param("fsm/waypoint_hold_velocity_thresh", waypoint_hold_velocity_thresh_, 0.15);

    last_planning_time_ = planning_delay_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    if (waypoint_num_ > kMaxWaypoints)
    {
      ROS_WARN("[Load Params] fsm/waypoint_num=%d exceeds max=%d, clamp to max.", waypoint_num_, kMaxWaypoints);
      waypoint_num_ = kMaxWaypoints;
    }
    nh.param("fsm/gimbal_pitch_reach_thresh_deg", gimbal_pitch_reach_thresh_deg_, 2.0);
    nh.param("fsm/gimbal_waypoint_reach_thresh_m", gimbal_waypoint_reach_thresh_m_, 0.25);
    ROS_WARN("[Load Params] Loaded fsm/waypoint_num with value: %d", waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_yaw", waypoints_yaw_[i], 0.0);
      nh.param("fsm/waypoint" + to_string(i) + "_cam_pitch", waypoints_cam_pitch_[i], 0.0);
    }

    std::string gimbal_action_indices_str;
    nh.param("fsm/gimbal_action_waypoint_indices", gimbal_action_indices_str, std::string(""));
    std::stringstream gimbal_indices_ss(gimbal_action_indices_str);
    std::string token;
    while (std::getline(gimbal_indices_ss, token, ','))
    {
      if (token.empty())
      {
        continue;
      }

      std::stringstream token_ss(token);
      int idx = -1;
      token_ss >> idx;
      if (!token_ss.fail() && idx >= 0 && idx < waypoint_num_)
      {
        gimbal_action_waypoint_indices_.push_back(idx);
      }
      else
      {
        ROS_WARN("[Load Params] Ignore invalid gimbal waypoint index token: %s", token.c_str());
      }
    }

    resetGimbalWaitState();

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    have_trigger_ = !flag_realworld_experiment_ && !use_rc_trigger_in_manual_;
    no_replan_thresh_ = 0.5 * emergency_time_ * planner_manager_->pp_.max_vel_;

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);
    yaw_cmd_pub_ = nh.advertise<std_msgs::Float64>("planning/yaw_cmd", 10, true);
    use_yaw_cmd_pub_ = nh.advertise<std_msgs::Bool>("planning/use_yaw_cmd", 10, true);
    gimbal_cmd_pub_ = nh.advertise<gimbal::GimbalCmd>("/gimbal/cmd", 10);
    vla_status_pub_ = nh.advertise<quadrotor_msgs::VlaStatus>("/vla_status", 10, true);
    gimbal_state_sub_ = nh.subscribe("/gimbal/state", 10, &EGOReplanFSM::gimbalStateCallback, this);
    vla_waypoint_sub_ = nh.subscribe("/vla_waypoint", 1, &EGOReplanFSM::vlaWaypointCallback, this);

    publishVlaStatus(quadrotor_msgs::VlaStatus::IDLE, "fsm initialized");

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);

      if (use_rc_trigger_in_manual_)
      {
        trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);
      }
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 2 second.");
      int count = 0;
      while (ros::ok() && count++ < 2000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("[Load Params] Entering readGivenWpsAndPlan function.");
      readGivenWpsAndPlan();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 200)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return; // return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        ROS_WARN("START to planFromGlobalTraj");
        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          ROS_WARN("SUCCESS to generate the first trajectory");
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          ROS_WARN("Failed to generate the first trajectory, keep trying");
          changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      bool success = false;
      if (force_global_replan_once_)
      {
        ROS_INFO("[FSM][REPLAN] force global replan for waypoint transition, wpt=%d", wpt_id_);
        success = planFromGlobalTraj(10);
        if (success)
        {
          force_global_replan_once_ = false;
        }
      }
      else
      {
        success = planFromLocalTraj(1);
      }

      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      t_cur = min(info->duration, t_cur);
      Eigen::Vector3d pos = info->traj.getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);

      const PtsChk_t* chk_ptr = &planner_manager_->traj_.local_traj.pts_chk;
      bool close_to_current_traj_end = (chk_ptr->size() >= 1 && chk_ptr->back().size() >= 1) ? chk_ptr->back().back().first - t_cur < emergency_time_ : 0; // In case of empty vector

      if (mondifyInCollisionFinalGoal()) // case 1: find that current goal is in obstacles
      {
        // pass
      }
      else if (vla_active_)
      {
        if (isCurrentVlaCmdTimedOut())
        {
          publishVlaStatus(quadrotor_msgs::VlaStatus::COMMAND_TIMEOUT, "vla command timeout");
          clearVlaTask();
          have_target_ = false;
          have_trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "VLA_TIMEOUT");
        }
        else if (shouldStartVlaHold())
        {
          if (!vla_hold_started_)
          {
            if (!startHoldForCurrentTarget())
            {
              publishVlaStatus(quadrotor_msgs::VlaStatus::HOLD_FAILED, "failed to generate hold trajectory for vla target");
              clearVlaTask();
              have_target_ = false;
              have_trigger_ = false;
              changeFSMExecState(WAIT_TARGET, "VLA_HOLD_FAIL");
            }
            break;
          }

          if (!vla_gimbal_cmd_sent_)
          {
            startVlaGimbalAction();
          }

          if (waiting_gimbal_action_)
          {
            if (gimbal_cmd_ack_ok_ && gimbal_pitch_reached_ && hasReachedVlaPositionYaw())
            {
              publishVlaStatus(quadrotor_msgs::VlaStatus::DONE, "vla waypoint completed");
              clearVlaTask();
              have_target_ = false;
              have_trigger_ = false;
              changeFSMExecState(WAIT_TARGET, "VLA_DONE");
            }
            else
            {
              ROS_INFO_THROTTLE(1.0,
                                "[FSM][VLA] Waiting completion seq=%u ack=%d pitch_reached=%d pos_err=%.3f yaw_err=%.3fdeg pitch_err=%.3fdeg",
                                vla_seq_id_,
                                gimbal_cmd_ack_ok_,
                                gimbal_pitch_reached_,
                                (odom_pos_ - vla_target_pos_).norm(),
                                getYawErrorDeg(vla_target_yaw_rad_),
                                std::abs(static_cast<double>(waiting_gimbal_target_pitch_deg_) / 100.0 - vla_target_pitch_deg_));
            }
          }
        }
        else if (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end))
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
               shouldTriggerGimbalActionForCurrentWaypoint(pos))
      {
        if (!gimbal_hold_traj_sent_for_current_wpt_)
        {
          if (!startHoldForCurrentWaypoint())
          {
            ROS_WARN_THROTTLE(1.0, "[FSM] Failed to generate hold trajectory at waypoint %d.", wpt_id_);
          }
          break;
        }

        if (!gimbal_action_sent_for_current_wpt_)
        {
          sendCurrentWaypointGimbalCmd();
        }

        if (waiting_gimbal_action_)
        {
          if (gimbal_cmd_ack_ok_ && gimbal_pitch_reached_)
          {
            ROS_INFO("[FSM] Gimbal action completed at waypoint %d, continue mission.", wpt_id_);
            if (!advanceMissionFromCurrentWaypoint())
            {
              ROS_WARN_THROTTLE(1.0, "[FSM] Failed to continue mission from waypoint %d.", wpt_id_);
            }
          }
          else
          {
            ROS_INFO_THROTTLE(1.0,
                              "[FSM] Waiting gimbal at waypoint %d, ack=%d pitch_reached=%d target_pitch=%.2fdeg thresh=%.2fdeg",
                              waiting_gimbal_waypoint_idx_,
                              gimbal_cmd_ack_ok_,
                              gimbal_pitch_reached_,
                              waiting_gimbal_target_pitch_deg_ / 100.0,
                              gimbal_pitch_reach_thresh_deg_);
          }
        }
      }
      else if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
               (final_goal_ - pos).norm() < no_replan_thresh_) // case 2: assign the next waypoint
      {
        if (!advanceMissionFromCurrentWaypoint())
        {
          ROS_WARN_THROTTLE(1.0, "[FSM] Failed to continue mission from waypoint %d.", wpt_id_);
        }
      }
      else if ((t_cur > info->duration - 1e-2) && touch_the_goal) // case 3: the final waypoint reached
      {
        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          completeMission();
        }
        else
        {
          have_target_ = false;
          if (!(target_type_ == TARGET_TYPE::MANUAL_TARGET && use_rc_trigger_in_manual_))
          {
            have_trigger_ = false;
          }

          /* The navigation task completed */
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
      }
      else if (t_cur > replan_thresh_ || (!touch_the_goal && close_to_current_traj_end)) // case 3: time to perform next replan
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      // ROS_ERROR("AAAA");

      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]"
         << "Drone:" << planner_manager_->pp_.drone_id << ", from " + state_str[pre_s] + " to " + state_str[int(new_state)] + ", wpt: "<< wpt_id_ << endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)] << ", Drone:" << planner_manager_->pp_.drone_id;

    // some warnings
    if (!have_odom_ || !have_target_ || !have_trigger_ || (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_))
    {
      cout << ". Waiting for ";
    }
    if (!have_odom_)
    {
      cout << "odom,";
    }
    if (!have_target_)
    {
      cout << "target,";
    }
    if (!have_trigger_)
    {
      cout << "trigger,";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << "prev traj,";
    }

    cout << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // check ground height by the way
    if (enable_ground_height_measurement_)
    {
      double height;
      measureGroundHeight(height);
    }

    /* --------- collision check data ---------- */
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    const double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    double t_temp = t_cur; // t_temp will be changed in the next function!
    int i_start = info->traj.locatePieceIdx(t_temp);

    if (i_start >= (int)pts_chk.size())
    {
      return;
    }
    size_t j_start = 0;
    for (; i_start < (int)pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    const bool touch_the_end = ((local_target_pt_ - final_goal_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    for (size_t i = i_start; i < i_end; ++i)
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)
      {

        double t = pts_chk[i][j].first;
        Eigen::Vector3d p = pts_chk[i][j].second;

        bool dangerous = false;
        dangerous |= map->getInflateOccupancy(p);

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;
          }

          double t_X = t + (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
            double dist = (p - swarm_pridicted).norm();
            double allowed_dist = planner_manager_->getSwarmClearance() + planner_manager_->traj_.swarm_traj.at(id).des_clearance;
            if (dist < allowed_dist)
            {
              ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
                       planner_manager_->pp_.drone_id, (int)id, dist);
              dangerous = true;
              break;
            }
          }
        }

        if (dangerous)
        {
          /* Handle the collided case immediately */
          if (planFromLocalTraj()) // Make a chance
          {
            ROS_INFO("Plan success when detect collision. %f", t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            if (t - t_cur < emergency_time_) // 0.8s of emergency time
            {
              ROS_WARN("Emergency stop! time=%f", t - t_cur);
              changeFSMExecState(EMERGENCY_STOP, "SAFETY");
            }
            else
            {
              ROS_WARN("current traj in collision, replan.");
              changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            }
            return;
          }
          break;
        }
      }
      j_start = 0;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);
    broadcast_ploytraj_pub_.publish(MINCO_msg);

    return true;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj, double &planning_time)
  {

    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_,
        planning_time);

    have_new_target_ = false;

    if (plan_success)
    {

      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);
      poly_traj_pub_.publish(poly_msg);
      broadcast_ploytraj_pub_.publish(MINCO_msg);
    }

    return plan_success;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    const ros::Time dbg_t0 = ros::Time::now();
    ROS_WARN("[FSM][DBG][GLOBAL] begin wpt=%d trial_times=%d exec_state=%d odom=(%.3f, %.3f, %.3f) vel=%.3f final_goal=(%.3f, %.3f, %.3f)",
             wpt_id_, trial_times, static_cast<int>(exec_state_),
             odom_pos_(0), odom_pos_(1), odom_pos_(2), odom_vel_.norm(),
             final_goal_(0), final_goal_(1), final_goal_(2));
    // ROS_WARN("planning_delay = %f", planning_delay_);
    // // 增加规划延迟补偿
    // double planning_delay = planning_delay_;
    // // 使用一阶模型预测补偿后的位置: new_pos = current_pos + current_vel * delay
    // start_pt_ = odom_pos_ + odom_vel_ * planning_delay;
    // start_pt_ = odom_pos_ + odom_vel_ * last_planning_time_;  // dynamic replanning
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    double new_planning_time = 0.0;
    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init, new_planning_time))
      {
        print_trajectory_vaj("New Global Trajectory");
        last_planning_time_ = new_planning_time;
        ROS_WARN("[FSM][DBG][GLOBAL] success wpt=%d try=%d dt=%.3f plan_time=%.3f",
                 wpt_id_, i, (ros::Time::now() - dbg_t0).toSec(), new_planning_time);
        return true;
      }
    }
    ROS_ERROR("[FSM][DBG][GLOBAL] failed wpt=%d dt=%.3f", wpt_id_, (ros::Time::now() - dbg_t0).toSec());
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {
    const ros::Time dbg_t0 = ros::Time::now();
    ROS_WARN("[FSM][DBG][LOCAL] begin wpt=%d trial_times=%d exec_state=%d last_plan=%.3f odom=(%.3f, %.3f, %.3f) vel=%.3f final_goal=(%.3f, %.3f, %.3f)",
             wpt_id_, trial_times, static_cast<int>(exec_state_), last_planning_time_,
             odom_pos_(0), odom_pos_(1), odom_pos_(2), odom_vel_.norm(),
             final_goal_(0), final_goal_(1), final_goal_(2));
    ROS_INFO("planning_delay = %f", last_planning_time_);
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    // start_pt_ = info->traj.getPos(t_cur);
    // start_vel_ = info->traj.getVel(t_cur);
    // start_acc_ = info->traj.getAcc(t_cur);

    // 增加规划延迟补偿
    // double planning_delay = planning_delay_; 
    // double planning_delay = last_planning_time_;
    double planning_delay = std::max(0.0, std::min(0.05, last_planning_time_));
    double t_advanced = t_cur + planning_delay;

    // 安全检查，确保预测时间不会超过当前轨迹的总时长
    if (t_advanced > info->duration)
    {
      // t_advanced = info->duration;
      t_advanced = t_cur;
    }
  
  start_pt_ = info->traj.getPos(t_advanced);
  start_vel_ = info->traj.getVel(t_advanced);
  start_acc_ = info->traj.getAcc(t_advanced);

    cout << "pt:  " << start_pt_.norm() << " (" << start_pt_[0] << ", " << start_pt_[1] << ", " << start_pt_[2] << ")" << endl;
    cout << "vel: " << start_vel_.norm() << " (" << start_vel_[0] << ", " << start_vel_[1] << ", " << start_vel_[2] << ")" << endl;
    cout << "acc: " << start_acc_.norm() << " (" << start_acc_[0] << ", " << start_acc_[1] << ", " << start_acc_[2] << ")" << endl;
    ROS_INFO("[FSM][LOCAL_REPLAN] t_cur=%.3f t_adv=%.3f dur=%.3f start_pt=(%.3f, %.3f, %.3f) odom=(%.3f, %.3f, %.3f) final_goal=(%.3f, %.3f, %.3f) local_target=(%.3f, %.3f, %.3f) wpt=%d",
             t_cur, t_advanced, info->duration,
             start_pt_(0), start_pt_(1), start_pt_(2),
             odom_pos_(0), odom_pos_(1), odom_pos_(2),
             final_goal_(0), final_goal_(1), final_goal_(2),
             local_target_pt_(0), local_target_pt_(1), local_target_pt_(2),
             wpt_id_);

    double new_planning_time = 0.0;
    bool success = callReboundReplan(false, false, new_planning_time);
    if (success)
    {
      last_planning_time_ = new_planning_time;
      ROS_WARN("[FSM][DBG][LOCAL] success-direct wpt=%d dt=%.3f plan_time=%.3f",
               wpt_id_, (ros::Time::now() - dbg_t0).toSec(), new_planning_time);
    }

    if (!success)
    {
      success = callReboundReplan(true, false, new_planning_time);
      if (success)
      {
        last_planning_time_ = new_planning_time;
        ROS_WARN("[FSM][DBG][LOCAL] success-global-init wpt=%d dt=%.3f plan_time=%.3f",
                 wpt_id_, (ros::Time::now() - dbg_t0).toSec(), new_planning_time);
      }

      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true, new_planning_time);
          if (success)
          {
            last_planning_time_ = new_planning_time;
            ROS_WARN("[FSM][DBG][LOCAL] success-random-init wpt=%d try=%d dt=%.3f plan_time=%.3f",
                     wpt_id_, i, (ros::Time::now() - dbg_t0).toSec(), new_planning_time);
            break;
          }
        }
        if (!success)
        {
          ROS_ERROR("[FSM][DBG][LOCAL] failed wpt=%d dt=%.3f", wpt_id_, (ros::Time::now() - dbg_t0).toSec());
          return false;
        }
      }
    }

      if (success)
    {
      print_trajectory_vaj("Replanned Local Trajectory");
    }

    return true;
  }

  bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp, double next_yaw, bool use_yaw)
  {
    ROS_INFO("[FSM][NEXT_WP] req current_wpt=%d odom=(%.3f, %.3f, %.3f) odom_vel=(%.3f, %.3f, %.3f) next_wp=(%.3f, %.3f, %.3f) next_yaw=%.3f use_yaw=%d exec_state=%d",
             wpt_id_,
             odom_pos_(0), odom_pos_(1), odom_pos_(2),
             odom_vel_(0), odom_vel_(1), odom_vel_(2),
             next_wp(0), next_wp(1), next_wp(2),
             next_yaw, static_cast<int>(use_yaw), static_cast<int>(exec_state_));

    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      ROS_INFO("[FSM][NEXT_WP] global traj success for wpt=%d -> goal=(%.3f, %.3f, %.3f)",
               wpt_id_, next_wp(0), next_wp(1), next_wp(2));
      final_goal_ = next_wp;
      final_goal_yaw_ = next_yaw;
      final_goal_use_yaw_ = use_yaw;

      std_msgs::Float64 yaw_msg;
      yaw_msg.data = final_goal_yaw_;
      yaw_cmd_pub_.publish(yaw_msg);

      std_msgs::Bool use_yaw_msg;
      use_yaw_msg.data = final_goal_use_yaw_;
      use_yaw_cmd_pub_.publish(use_yaw_msg);

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET)
      {
        const ros::Time wait_t0 = ros::Time::now();
        ROS_WARN("[FSM][DBG][NEXT_WP] entering wait-for-EXEC_TRAJ wpt=%d current_state=%d", wpt_id_, static_cast<int>(exec_state_));
        while (exec_state_ != EXEC_TRAJ)
        {
          ROS_WARN_THROTTLE(0.2, "[FSM][DBG][NEXT_WP] still waiting EXEC_TRAJ wpt=%d state=%d wait_dt=%.3f",
                            wpt_id_, static_cast<int>(exec_state_), (ros::Time::now() - wait_t0).toSec());
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        ROS_WARN("[FSM][DBG][NEXT_WP] leave wait-for-EXEC_TRAJ wpt=%d wait_dt=%.3f", wpt_id_, (ros::Time::now() - wait_t0).toSec());
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("[FSM][NEXT_WP] Unable to generate global trajectory for wpt=%d from odom=(%.3f, %.3f, %.3f) to goal=(%.3f, %.3f, %.3f)",
                wpt_id_,
                odom_pos_(0), odom_pos_(1), odom_pos_(2),
                next_wp(0), next_wp(1), next_wp(2));
    }

    return success;
  }

  bool EGOReplanFSM::mondifyInCollisionFinalGoal()
  {
    if (planner_manager_->grid_map_->getInflateOccupancy(final_goal_))
    {
      Eigen::Vector3d orig_goal = final_goal_;
      double t_step = planner_manager_->grid_map_->getResolution() / planner_manager_->pp_.max_vel_;
      for (double t = planner_manager_->traj_.global_traj.duration; t > 0; t -= t_step)
      {
        Eigen::Vector3d pt = planner_manager_->traj_.global_traj.traj.getPos(t);
        if (!planner_manager_->grid_map_->getInflateOccupancy(pt))
        {
          if (planNextWaypoint(pt, final_goal_yaw_, final_goal_use_yaw_)) // final_goal_=pt inside if success
          {
            ROS_INFO("Current in-collision waypoint (%.3f, %.3f %.3f) has been modified to (%.3f, %.3f %.3f)",
                     orig_goal(0), orig_goal(1), orig_goal(2), final_goal_(0), final_goal_(1), final_goal_(2));
            return true;
          }
        }

        if (t <= t_step)
        {
          ROS_ERROR("Can't find any collision-free point on global traj.");
        }
      }
    }

    return false;
  }

  void EGOReplanFSM::waypointCallback(const quadrotor_msgs::GoalSetPtr &msg)
  {
    if (msg->drone_id != planner_manager_->pp_.drone_id || msg->goal[2] < -0.1)
      return;

    ROS_INFO("Received goal: %f, %f, %f", msg->goal[0], msg->goal[1], msg->goal[2]);

    Eigen::Vector3d end_wp(msg->goal[0], msg->goal[1], msg->goal[2]);
    if (planNextWaypoint(end_wp, 0.0, false))
    {
      if (!use_rc_trigger_in_manual_)
      {
        have_trigger_ = true;
      }
    }
  }

  void EGOReplanFSM::vlaWaypointCallback(const quadrotor_msgs::VlaWaypoint::ConstPtr &msg)
  {
    if (msg->drone_id != planner_manager_->pp_.drone_id)
    {
      return;
    }

    if (msg->position.z < -0.1)
    {
      publishVlaStatus(quadrotor_msgs::VlaStatus::PLAN_FAILED, "reject invalid vla altitude");
      return;
    }

    if (isVlaBusy() || have_target_)
    {
      publishVlaStatus(quadrotor_msgs::VlaStatus::REJECTED_BUSY, "planner busy, reject new vla waypoint");
      return;
    }

    vla_seq_id_ = msg->seq_id;
    vla_target_pos_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    vla_target_yaw_rad_ = msg->yaw;
    vla_target_pitch_deg_ = msg->pitch;
    vla_position_tolerance_ = msg->position_tolerance > 1e-3 ? msg->position_tolerance : 0.25;
    vla_yaw_tolerance_deg_ = msg->yaw_tolerance_deg > 1e-3 ? msg->yaw_tolerance_deg : 8.0;
    vla_pitch_tolerance_deg_ = msg->pitch_tolerance_deg > 1e-3 ? msg->pitch_tolerance_deg : gimbal_pitch_reach_thresh_deg_;
    vla_timeout_sec_ = msg->timeout_sec > 1e-3 ? msg->timeout_sec : 20.0;
    vla_cmd_start_time_ = ros::Time::now();
    vla_active_ = true;
    vla_hold_started_ = false;
    vla_gimbal_cmd_sent_ = false;
    resetGimbalWaitState();

    if (planNextWaypoint(vla_target_pos_, vla_target_yaw_rad_, true))
    {
      have_trigger_ = true;
      publishVlaStatus(quadrotor_msgs::VlaStatus::BUSY, "accepted vla waypoint");
      return;
    }

    publishVlaStatus(quadrotor_msgs::VlaStatus::PLAN_FAILED, "failed to plan vla waypoint");
    clearVlaTask();
  }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    ROS_WARN("[Load Params] In readGivenWpsAndPlan() function.");
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    wps_yaw_.resize(waypoint_num_);
    wps_cam_pitch_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
      wps_yaw_[i] = waypoints_yaw_[i];
      wps_cam_pitch_[i] = waypoints_cam_pitch_[i];
    }
    ROS_WARN("[Load Params] FINISH load global waypoint");

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    ROS_WARN("START plan fist global waypoint");
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_], wps_yaw_[wpt_id_], true);
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    if (vla_active_)
    {
      publishVlaStatus(quadrotor_msgs::VlaStatus::ABORTED, "mandatory stop received");
      clearVlaTask();
    }
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    const auto &q = msg->pose.pose.orientation;
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    current_yaw_rad_ = std::atan2(siny_cosp, cosy_cosp);

    have_odom_ = true;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id) // myself
      return;

    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        msg->start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      ROS_WARN("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {

      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).toSec());
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).toSec());
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        blank.start_time = 0.0;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    if ( msg->start_time.toSec() <= planner_manager_->traj_.swarm_traj[recv_id].start_time ) // This must be called after buffer fill-up
    {
      ROS_WARN("Old traj received, ignored.");
      return;
    }

    /* Parse and store data */

    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    /* Ignore the trajectories that are far away */
    Eigen::MatrixXd cps_chk = MJO.getInitConstraintPoints(5); // K = 5, such accuracy is sufficient
    bool far_away = true;
    for (int i = 0; i < cps_chk.cols(); ++i)
    {
      if ((cps_chk.col(i) - odom_pos_).norm() < planner_manager_->pp_.planning_horizen_ * 4 / 3) // close to me that can not be ignored
      {
        far_away = false;
        break;
      }
    }
    if (!far_away || !have_recv_pre_agent_) // Accept a far traj if no previous agent received
    {
      poly_traj::Trajectory trajectory = MJO.getTraj();
      planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
      planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
      planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();
      planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
      planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);
      planner_manager_->traj_.swarm_traj[recv_id].des_clearance = msg->des_clearance;

      /* Check Collision */
      if (planner_manager_->checkCollision(recv_id))
      {
        changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
      }

      /* Check if receive agents have lower drone id */
      if (!have_recv_pre_agent_)
      {
        if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
        {
          for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
          {
            if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
            {
              break;
            }

            have_recv_pre_agent_ = true;
          }
        }
      }
    }
    else
    {
      planner_manager_->traj_.swarm_traj[recv_id].drone_id = -1; // Means this trajectory is invalid
    }
  }

  void EGOReplanFSM::gimbalStateCallback(const gimbal::GimbalState::ConstPtr &msg)
  {
    if (!waiting_gimbal_action_)
    {
      return;
    }

    gimbal_cmd_ack_ok_ = (msg->cmd_stat == 1 && msg->cmd_value == 4);
    const double current_pitch_deg = static_cast<double>(msg->cam_angle_p) / 100.0;
    const double target_pitch_deg = waiting_gimbal_target_pitch_deg_ / 100.0;
    gimbal_pitch_reached_ = std::abs(current_pitch_deg - target_pitch_deg) <= gimbal_pitch_reach_thresh_deg_;
  }

  bool EGOReplanFSM::isGimbalActionWaypoint(int waypoint_idx) const
  {
    return std::find(gimbal_action_waypoint_indices_.begin(),
                     gimbal_action_waypoint_indices_.end(),
                     waypoint_idx) != gimbal_action_waypoint_indices_.end();
  }

  bool EGOReplanFSM::shouldTriggerGimbalActionForCurrentWaypoint(const Eigen::Vector3d &pos) const
  {
    return isGimbalActionWaypoint(wpt_id_) &&
           (final_goal_ - pos).norm() < gimbal_waypoint_reach_thresh_m_;
  }

  void EGOReplanFSM::sendCurrentWaypointGimbalCmd()
  {
    if (wpt_id_ < 0 || wpt_id_ >= static_cast<int>(wps_cam_pitch_.size()))
    {
      ROS_WARN("[FSM] Invalid waypoint id %d for gimbal action.", wpt_id_);
      return;
    }

    gimbal::GimbalCmd cmd_msg;
    cmd_msg.ctrl_sw = 3;
    cmd_msg.cmd_value = 4;
    cmd_msg.op_value_p = static_cast<int16_t>(std::round(wps_cam_pitch_[wpt_id_]));
    cmd_msg.op_value_r = 0;
    cmd_msg.op_value_y = 0;
    gimbal_cmd_pub_.publish(cmd_msg);

    waiting_gimbal_action_ = true;
    gimbal_action_sent_for_current_wpt_ = true;
    gimbal_cmd_ack_ok_ = false;
    gimbal_pitch_reached_ = false;
    waiting_gimbal_waypoint_idx_ = wpt_id_;
    waiting_gimbal_target_pitch_deg_ = wps_cam_pitch_[wpt_id_];

    ROS_INFO("[FSM] Send gimbal pitch command at waypoint %d, target_pitch=%.2f deg (%d cdeg).",
             waiting_gimbal_waypoint_idx_, waiting_gimbal_target_pitch_deg_ / 100.0,
             static_cast<int>(std::round(waiting_gimbal_target_pitch_deg_)));
  }

  bool EGOReplanFSM::startHoldForCurrentWaypoint()
  {
    if (wpt_id_ < 0 || wpt_id_ >= static_cast<int>(wps_.size()))
    {
      ROS_WARN("[FSM] Invalid waypoint id %d for hold planning.", wpt_id_);
      return false;
    }

    if (odom_vel_.norm() > waypoint_hold_velocity_thresh_)
    {
      ROS_INFO_THROTTLE(1.0,
                        "[FSM] Waypoint %d reached position threshold, wait velocity decay %.3f/%.3f m/s before hold.",
                        wpt_id_, odom_vel_.norm(), waypoint_hold_velocity_thresh_);
      return true;
    }

    std::vector<Eigen::Vector3d> hold_wps;
    hold_wps.push_back(wps_[wpt_id_]);
    const bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        hold_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_WARN("[FSM] Unable to generate hold trajectory for waypoint %d.", wpt_id_);
      return false;
    }

    final_goal_ = wps_[wpt_id_];
    final_goal_yaw_ = wps_yaw_[wpt_id_];
    final_goal_use_yaw_ = true;

    std_msgs::Float64 yaw_msg;
    yaw_msg.data = final_goal_yaw_;
    yaw_cmd_pub_.publish(yaw_msg);

    std_msgs::Bool use_yaw_msg;
    use_yaw_msg.data = final_goal_use_yaw_;
    use_yaw_cmd_pub_.publish(use_yaw_msg);

    have_target_ = true;
    have_new_target_ = true;
    gimbal_hold_traj_sent_for_current_wpt_ = true;

    changeFSMExecState(REPLAN_TRAJ, "HOLD_WPT");
    ROS_INFO("[FSM] Hold trajectory generated for waypoint %d.", wpt_id_);
    return true;
  }

  bool EGOReplanFSM::advanceMissionFromCurrentWaypoint()
  {
    const int prev_wpt = wpt_id_;
    const Eigen::Vector3d prev_goal = final_goal_;
    ROS_INFO("[FSM][ADVANCE] begin prev_wpt=%d final_goal=(%.3f, %.3f, %.3f) odom=(%.3f, %.3f, %.3f) odom_vel=%.3f waiting=%d hold_sent=%d action_sent=%d",
             prev_wpt,
             prev_goal(0), prev_goal(1), prev_goal(2),
             odom_pos_(0), odom_pos_(1), odom_pos_(2),
             odom_vel_.norm(),
             static_cast<int>(waiting_gimbal_action_),
             static_cast<int>(gimbal_hold_traj_sent_for_current_wpt_),
             static_cast<int>(gimbal_action_sent_for_current_wpt_));

    resetGimbalWaitState();

    if (waypoint_num_ <= 0 || wps_.empty())
    {
      ROS_WARN("[FSM] No preset mission waypoints loaded.");
      return false;
    }

    if (wpt_id_ >= waypoint_num_ - 1)
    {
      ROS_INFO("[FSM][ADVANCE] reached mission end at wpt=%d/%d.", wpt_id_, waypoint_num_ - 1);
      completeMission();
      return true;
    }

    ++wpt_id_;
    force_global_replan_once_ = true;
    ROS_INFO("[FSM][ADVANCE] switch prev_wpt=%d -> next_wpt=%d next_goal=(%.3f, %.3f, %.3f) next_yaw=%.3f force_global=%d",
             prev_wpt, wpt_id_,
             wps_[wpt_id_](0), wps_[wpt_id_](1), wps_[wpt_id_](2),
             wps_yaw_[wpt_id_], static_cast<int>(force_global_replan_once_));
    return planNextWaypoint(wps_[wpt_id_], wps_yaw_[wpt_id_], true);
  }

  void EGOReplanFSM::completeMission()
  {
    resetGimbalWaitState();

    if (target_type_ == TARGET_TYPE::PRESET_TARGET && auto_repeat_mission_ && waypoint_num_ > 0 && !wps_.empty())
    {
      ROS_INFO("[FSM] Preset mission round completed, auto restart from waypoint 0.");
      wpt_id_ = 0;
      if (planNextWaypoint(wps_[wpt_id_], wps_yaw_[wpt_id_], true))
      {
        return;
      }
      ROS_WARN("[FSM] Failed to restart preset mission, fall back to WAIT_TARGET.");
    }

    have_target_ = false;
    if (!(target_type_ == TARGET_TYPE::MANUAL_TARGET && use_rc_trigger_in_manual_))
    {
      have_trigger_ = false;
    }
    changeFSMExecState(WAIT_TARGET, "FSM");
  }

  void EGOReplanFSM::resetGimbalWaitState()
  {
    waiting_gimbal_action_ = false;
    gimbal_cmd_ack_ok_ = false;
    gimbal_pitch_reached_ = false;
    gimbal_action_sent_for_current_wpt_ = false;
    gimbal_hold_traj_sent_for_current_wpt_ = false;
    waiting_gimbal_waypoint_idx_ = -1;
    waiting_gimbal_target_pitch_deg_ = 0.0;
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    MINCO_msg.des_clearance = planner_manager_->getSwarmClearance();
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  bool EGOReplanFSM::isVlaBusy() const
  {
    return vla_active_;
  }

  void EGOReplanFSM::publishVlaStatus(uint8_t status, const std::string &message) const
  {
    quadrotor_msgs::VlaStatus msg;
    msg.header.stamp = ros::Time::now();
    msg.drone_id = planner_manager_ ? planner_manager_->pp_.drone_id : 0;
    msg.seq_id = vla_seq_id_;
    msg.status = status;
    msg.message = message;
    msg.target_position.x = vla_target_pos_(0);
    msg.target_position.y = vla_target_pos_(1);
    msg.target_position.z = vla_target_pos_(2);
    msg.target_yaw = vla_target_yaw_rad_;
    msg.target_pitch = vla_target_pitch_deg_;
    msg.position_error = (odom_pos_ - vla_target_pos_).norm();
    msg.yaw_error_deg = getYawErrorDeg(vla_target_yaw_rad_);
    msg.pitch_error_deg = std::abs(static_cast<double>(waiting_gimbal_target_pitch_deg_) / 100.0 - vla_target_pitch_deg_);
    vla_status_pub_.publish(msg);
  }

  double EGOReplanFSM::getYawErrorDeg(double target_yaw_rad) const
  {
    double diff = target_yaw_rad - current_yaw_rad_;
    while (diff > M_PI)
      diff -= 2.0 * M_PI;
    while (diff < -M_PI)
      diff += 2.0 * M_PI;
    return std::abs(diff) * 180.0 / M_PI;
  }

  bool EGOReplanFSM::hasReachedVlaPositionYaw() const
  {
    return (odom_pos_ - vla_target_pos_).norm() <= vla_position_tolerance_ &&
           getYawErrorDeg(vla_target_yaw_rad_) <= vla_yaw_tolerance_deg_;
  }

  bool EGOReplanFSM::shouldStartVlaHold() const
  {
    return hasReachedVlaPositionYaw();
  }

  bool EGOReplanFSM::startHoldForCurrentTarget()
  {
    if (odom_vel_.norm() > waypoint_hold_velocity_thresh_)
    {
      ROS_INFO_THROTTLE(1.0,
                        "[FSM][VLA] Position/yaw reached, wait velocity decay %.3f/%.3f m/s before hold.",
                        odom_vel_.norm(), waypoint_hold_velocity_thresh_);
      return true;
    }

    std::vector<Eigen::Vector3d> hold_wps;
    hold_wps.push_back(vla_target_pos_);
    const bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        hold_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (!success)
    {
      ROS_WARN("[FSM][VLA] Unable to generate hold trajectory for current target.");
      return false;
    }

    final_goal_ = vla_target_pos_;
    final_goal_yaw_ = vla_target_yaw_rad_;
    final_goal_use_yaw_ = true;

    std_msgs::Float64 yaw_msg;
    yaw_msg.data = final_goal_yaw_;
    yaw_cmd_pub_.publish(yaw_msg);

    std_msgs::Bool use_yaw_msg;
    use_yaw_msg.data = final_goal_use_yaw_;
    use_yaw_cmd_pub_.publish(use_yaw_msg);

    have_target_ = true;
    have_new_target_ = true;
    vla_hold_started_ = true;
    changeFSMExecState(REPLAN_TRAJ, "VLA_HOLD");
    ROS_INFO("[FSM][VLA] Hold trajectory generated for seq=%u.", vla_seq_id_);
    return true;
  }

  void EGOReplanFSM::startVlaGimbalAction()
  {
    gimbal::GimbalCmd cmd_msg;
    cmd_msg.ctrl_sw = 3;
    cmd_msg.cmd_value = 4;
    cmd_msg.op_value_p = static_cast<int16_t>(std::round(vla_target_pitch_deg_ * 100.0));
    cmd_msg.op_value_r = 0;
    cmd_msg.op_value_y = 0;
    gimbal_cmd_pub_.publish(cmd_msg);

    waiting_gimbal_action_ = true;
    gimbal_cmd_ack_ok_ = false;
    gimbal_pitch_reached_ = false;
    waiting_gimbal_waypoint_idx_ = -1;
    waiting_gimbal_target_pitch_deg_ = vla_target_pitch_deg_ * 100.0;
    vla_gimbal_cmd_sent_ = true;

    ROS_INFO("[FSM][VLA] Send gimbal pitch command seq=%u target_pitch=%.2f deg.", vla_seq_id_, vla_target_pitch_deg_);
  }

  bool EGOReplanFSM::isCurrentVlaCmdTimedOut() const
  {
    return vla_active_ && vla_timeout_sec_ > 1e-3 && (ros::Time::now() - vla_cmd_start_time_).toSec() > vla_timeout_sec_;
  }

  void EGOReplanFSM::clearVlaTask()
  {
    vla_active_ = false;
    vla_hold_started_ = false;
    vla_gimbal_cmd_sent_ = false;
    resetGimbalWaitState();
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_p(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }

void EGOReplanFSM::print_trajectory_vaj(const std::string& traj_type_info)
{
  if(print_trj_vaj_){
    // 1. 从 planner_manager 获取最新的轨迹数据
    auto traj_data = &planner_manager_->traj_.local_traj;
    auto& traj = traj_data->traj;
    double duration = traj_data->duration;
    int piece_num = traj.getPieceNum();
    Eigen::VectorXd durations = traj.getDurations();

    // 2. 打印表头信息
    ROS_INFO(" ");
    ROS_INFO("====================== EGO-PLANNER TRAJECTORY DETAILS ======================");
    ROS_INFO_STREAM("Trajectory Type: " << traj_type_info << " | ID: " << traj_data->traj_id << " | Total Duration: " << std::fixed << std::setprecision(3) << duration << "s");
    ROS_INFO("--------------------------------------------------------------------------------");
    ROS_INFO(" CP ID | Time Alloc(s) | Distance(m) | Vel-X(m/s) | Acc-X(m/s^2) | Jerk-X(m/s^3)");
    ROS_INFO("--------------------------------------------------------------------------------");

    // 3. 循环遍历所有控制点 (Junctions)
    double accumulated_time = 0.0;
    for (int i = 0; i <= piece_num; ++i)
    {
        // 获取在每个控制点（Junction）的状态
        Eigen::Vector3d pos = traj.getJuncPos(i);
        Eigen::Vector3d vel = traj.getJuncVel(i);
        Eigen::Vector3d acc = traj.getJuncAcc(i);
        
        // Jerk在Junction处是连续的，可以通过获取该点作为起点时的Jerk来计算
        // 对于最后一个点，它也是轨迹的终点
        Eigen::Vector3d jer = traj.getJer(accumulated_time);

        double time_alloc = 0.0;
        double distance = 0.0;
        
        // 如果不是最后一个点，计算它与下一个点之间的分段信息
        if (i < piece_num) {
            time_alloc = durations(i);
            Eigen::Vector3d next_pos = traj.getJuncPos(i + 1);
            distance = (next_pos - pos).norm();
            accumulated_time += time_alloc;
        }

        // 使用 snprintf 进行格式化输出，确保对齐
        char buffer[256];
        if (i < piece_num) {
            snprintf(buffer, sizeof(buffer),
                     " %-5d | %-13.3f | %-11.3f | %-10.3f | %-12.3f | %-12.3f",
                     i, time_alloc, distance, vel.x(), acc.x(), jer.x());
        } else { // 最后一个点没有后续的时间分配和距离
            snprintf(buffer, sizeof(buffer),
                     " %-5d | N/A           | N/A         | %-10.3f | %-12.3f | %-12.3f",
                     i, vel.x(), acc.x(), jer.x());
        }
        ROS_INFO("%s", buffer);
    }

    // 4. 打印一个收尾，标志着本次输出结束
    ROS_INFO("--------------------------------------------------------------------------------");
    ROS_INFO(" "); // 打印一个空行
  }
}

} // namespace ego_planner
