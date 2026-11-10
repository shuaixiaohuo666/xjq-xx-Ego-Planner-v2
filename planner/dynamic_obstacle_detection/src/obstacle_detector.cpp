#include "dynamic_obstacle_detection/obstacle_detector.h"
#include <algorithm>
#include <numeric>
#include <deque>

namespace dynamic_obstacle_detection
{

ObstacleDetector::ObstacleDetector(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private), cloud_received_(false), odom_received_(false),
    last_uav_position_(Eigen::Vector3d::Zero()), last_obstacle_position_(Eigen::Vector3d::Zero()),
    current_obstacle_velocity_(Eigen::Vector3d::Zero()), obstacle_velocity_valid_(false)
{
  init();
}

ObstacleDetector::~ObstacleDetector()
{
}

void ObstacleDetector::init()
{
  // Load parameters
  nh_private_.param<std::string>("obstacle_cloud_topic", obstacle_cloud_topic_, "/obstacle_cloud");
  nh_private_.param<std::string>("filtered_cloud_topic", filtered_cloud_topic_, "/filtered_cloud");
  nh_private_.param<std::string>("static_cloud_topic", static_cloud_topic_, "/static_cloud");
  nh_private_.param<std::string>("world_frame", world_frame_, "world");

  // Sphere detection parameters (in world frame)
  nh_private_.param<double>("detection_radius_min", detection_radius_min_, 0.5);
  nh_private_.param<double>("detection_radius_max", detection_radius_max_, 10.0);

  // Dynamic obstacle filtering parameters
  nh_private_.param<bool>("is_pub_dyn_obs_filter_pcl", is_pub_dyn_obs_filter_pcl_, false);
  nh_private_.param<double>("obstacle_filter_radius", obstacle_filter_radius_, 0.5);

  // ROI parameters (Region of Interest in world frame)
  nh_private_.param<bool>("use_roi_filter", use_roi_filter_, false);
  
  std::vector<double> roi_min_vec, roi_max_vec;
  nh_private_.param<std::vector<double>>("roi_min", roi_min_vec, {5.0, -5.0, 0.8});
  nh_private_.param<std::vector<double>>("roi_max", roi_max_vec, {45.0, 5.0, 3.0});
  
  if (roi_min_vec.size() == 3 && roi_max_vec.size() == 3)
  {
    roi_min_ = Eigen::Vector3d(roi_min_vec[0], roi_min_vec[1], roi_min_vec[2]);
    roi_max_ = Eigen::Vector3d(roi_max_vec[0], roi_max_vec[1], roi_max_vec[2]);
  }
  else
  {
    ROS_WARN("[ObstacleDetector] Invalid ROI parameters, using defaults");
    roi_min_ = Eigen::Vector3d(5.0, -5.0, 0.8);
    roi_max_ = Eigen::Vector3d(45.0, 5.0, 3.0);
  }

  // Aggregation method
  nh_private_.param<std::string>("aggregation_method", aggregation_method_, "mean");

  // Trajectory parameters
  nh_private_.param<int>("max_trajectory_length", max_trajectory_length_, 500);
  nh_private_.param<double>("trajectory_sample_distance", trajectory_sample_distance_, 0.1);
  
  // Prediction parameters
  nh_private_.param<bool>("enable_prediction", enable_prediction_, true);
  nh_private_.param<double>("prediction_horizon", prediction_horizon_, 3.0);
  nh_private_.param<int>("prediction_samples", prediction_samples_, 20);
  nh_private_.param<int>("velocity_window_size", velocity_window_size_, 10);
  nh_private_.param<double>("min_obstacle_speed", min_obstacle_speed_, 0.1);

  // Debug parameters
  nh_private_.param<bool>("debug_mode", debug_mode_, false);

  // Subscribers (using remapped topics from launch file)
  cloud_sub_ = nh_private_.subscribe("cloud_topic", 10, &ObstacleDetector::cloudCallback, this);
  odom_sub_ = nh_private_.subscribe("odom_topic", 10, &ObstacleDetector::odomCallback, this);

  // Publishers
  obstacle_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(obstacle_cloud_topic_, 10);
  predicted_traj_pub_ = nh_.advertise<traj_utils::MINCOTraj>("/predicted_obstacle_traj", 10);
  
  // Debug publishers (only created in debug mode)
  if (debug_mode_)
  {
    filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_cloud_topic_, 10);
    uav_path_pub_ = nh_.advertise<nav_msgs::Path>("/uav_trajectory", 10);
    obstacle_path_pub_ = nh_.advertise<nav_msgs::Path>("/obstacle_trajectory", 10);
    ROS_INFO("[ObstacleDetector] Debug publishers created (filtered_cloud, uav_trajectory, obstacle_trajectory)");
  }
  
  // Static cloud publisher (only created if enabled)
  if (is_pub_dyn_obs_filter_pcl_)
  {
    static_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(static_cloud_topic_, 10);
    ROS_INFO("[ObstacleDetector] Static cloud publisher created on topic: %s", static_cloud_topic_.c_str());
  }

  // Initialize trajectories
  uav_trajectory_.header.frame_id = world_frame_;
  obstacle_trajectory_.header.frame_id = world_frame_;

  ROS_INFO("[ObstacleDetector] Initialized");
  ROS_INFO("[ObstacleDetector] Sphere detection (world frame): radius_min=%.2f m, radius_max=%.2f m", 
           detection_radius_min_, detection_radius_max_);
  if (use_roi_filter_)
  {
    ROS_INFO("[ObstacleDetector] ROI filter enabled: min=(%.2f, %.2f, %.2f), max=(%.2f, %.2f, %.2f)",
             roi_min_.x(), roi_min_.y(), roi_min_.z(),
             roi_max_.x(), roi_max_.y(), roi_max_.z());
  }
  ROS_INFO("[ObstacleDetector] Trajectory prediction: %s (horizon: %.2fs, samples: %d)", 
           enable_prediction_ ? "enabled" : "disabled", prediction_horizon_, prediction_samples_);
  ROS_INFO("[ObstacleDetector] Debug mode: %s (trajectory visualization: %s)", 
           debug_mode_ ? "ON" : "OFF", debug_mode_ ? "enabled" : "disabled");
  ROS_INFO("[ObstacleDetector] Static cloud publishing: %s (filter radius: %.2f m)", 
           is_pub_dyn_obs_filter_pcl_ ? "enabled" : "disabled", obstacle_filter_radius_);
}

void ObstacleDetector::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = msg;
  cloud_received_ = true;
}

void ObstacleDetector::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = msg;
  odom_received_ = true;

  // Update UAV trajectory at odom frequency for smoother trajectory (only in debug mode)
  if (debug_mode_)
  {
    Eigen::Vector3d current_uav_pos(msg->pose.pose.position.x, 
                                     msg->pose.pose.position.y, 
                                     msg->pose.pose.position.z);
    
    if (uav_trajectory_.poses.empty() || 
        (current_uav_pos - last_uav_position_).norm() > trajectory_sample_distance_)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = msg->header;
      pose.pose = msg->pose.pose;
      
      uav_trajectory_.poses.push_back(pose);
      last_uav_position_ = current_uav_pos;
      
      // Limit trajectory length
      if (uav_trajectory_.poses.size() > static_cast<size_t>(max_trajectory_length_))
      {
        uav_trajectory_.poses.erase(uav_trajectory_.poses.begin());
      }
      
      // Publish UAV trajectory
      uav_trajectory_.header.stamp = msg->header.stamp;
      uav_path_pub_.publish(uav_trajectory_);
    }
  }
}

void ObstacleDetector::run()
{
  ros::Rate rate(20.0); // 20 Hz

  while (ros::ok())
  {
    ros::spinOnce();

    if (cloud_received_ && odom_received_)
    {
      processPointCloud();
    }

    rate.sleep();
  }
}

void ObstacleDetector::processPointCloud()
{
  sensor_msgs::PointCloud2::ConstPtr cloud_msg;
  nav_msgs::Odometry::ConstPtr odom_msg;

  {
    std::lock_guard<std::mutex> lock1(cloud_mutex_);
    std::lock_guard<std::mutex> lock2(odom_mutex_);
    cloud_msg = latest_cloud_;
    odom_msg = latest_odom_;
  }

  if (!cloud_msg || !odom_msg)
    return;

  // Convert ROS PointCloud2 to PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  if (cloud->empty())
  {
    ROS_WARN_THROTTLE(5.0, "[ObstacleDetector] Received empty point cloud");
    
    // Still publish empty static cloud if enabled
    if (is_pub_dyn_obs_filter_pcl_)
    {
      sensor_msgs::PointCloud2 static_msg;
      pcl::toROSMsg(*cloud, static_msg);
      static_msg.header = cloud_msg->header;
      static_cloud_pub_.publish(static_msg);
    }
    return;
  }

  // Get current UAV position in world frame
  Eigen::Vector3d uav_position = getPositionFromOdom();

  // Filter points within ROI (in world frame) if enabled
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
  if (use_roi_filter_)
  {
    filterPointCloudInROI(cloud, cloud_roi);
    if (cloud_roi->empty())
    {
      ROS_DEBUG_THROTTLE(2.0, "[ObstacleDetector] No points in ROI");
      
      // Still publish original cloud as static cloud if enabled (no obstacles in ROI)
      if (is_pub_dyn_obs_filter_pcl_)
      {
        sensor_msgs::PointCloud2 static_msg;
        pcl::toROSMsg(*cloud, static_msg);
        static_msg.header = cloud_msg->header;
        static_cloud_pub_.publish(static_msg);
        ROS_DEBUG("[ObstacleDetector] Published static cloud: %lu points (no points in ROI)", cloud->size());
      }
      return;
    }
  }
  else
  {
    cloud_roi = cloud;
  }

  // Filter points within sphere around UAV (in world frame)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sphere(new pcl::PointCloud<pcl::PointXYZ>);
  filterPointCloudInSphere(cloud_roi, uav_position, cloud_sphere);

  if (cloud_sphere->empty())
  {
    ROS_DEBUG_THROTTLE(2.0, "[ObstacleDetector] No points in detection sphere");
    
    // Still publish original cloud as static cloud if enabled (no obstacles detected)
    if (is_pub_dyn_obs_filter_pcl_)
    {
      sensor_msgs::PointCloud2 static_msg;
      pcl::toROSMsg(*cloud, static_msg);
      static_msg.header = cloud_msg->header;
      static_cloud_pub_.publish(static_msg);
      ROS_DEBUG("[ObstacleDetector] Published static cloud: %lu points (no obstacles in sphere)", cloud->size());
    }
    return;
  }

  // Use detected points directly (already in world frame)
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloud_sphere;

  // Publish filtered cloud for visualization (only in debug mode)
  if (debug_mode_)
  {
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud_filtered, filtered_msg);
    filtered_msg.header = cloud_msg->header;
    filtered_cloud_pub_.publish(filtered_msg);
  }

  // Compute obstacle representation based on aggregation method
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (aggregation_method_ == "mean")
  {
    // Calculate mean position of all points
    if (!cloud_filtered->empty())
    {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cloud_filtered, centroid);
      
      pcl::PointXYZ mean_point;
      mean_point.x = centroid[0];
      mean_point.y = centroid[1];
      mean_point.z = centroid[2];
      obstacle_cloud->push_back(mean_point);
    }
  }
  else if (aggregation_method_ == "median")
  {
    // Calculate median position
    if (!cloud_filtered->empty())
    {
      std::vector<float> x_values, y_values, z_values;
      for (const auto& point : cloud_filtered->points)
      {
        x_values.push_back(point.x);
        y_values.push_back(point.y);
        z_values.push_back(point.z);
      }

      std::sort(x_values.begin(), x_values.end());
      std::sort(y_values.begin(), y_values.end());
      std::sort(z_values.begin(), z_values.end());

      size_t n = x_values.size();
      pcl::PointXYZ median_point;
      median_point.x = (n % 2 == 0) ? (x_values[n/2-1] + x_values[n/2]) / 2.0 : x_values[n/2];
      median_point.y = (n % 2 == 0) ? (y_values[n/2-1] + y_values[n/2]) / 2.0 : y_values[n/2];
      median_point.z = (n % 2 == 0) ? (z_values[n/2-1] + z_values[n/2]) / 2.0 : z_values[n/2];
      obstacle_cloud->push_back(median_point);
    }
  }
  else if (aggregation_method_ == "centroid" || aggregation_method_ == "all")
  {
    // Use all filtered points or just publish filtered cloud
    *obstacle_cloud = *cloud_filtered;
  }

  // Obstacle cloud is already in world frame, no transformation needed
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_world = obstacle_cloud;

  // Publish obstacle cloud
  sensor_msgs::PointCloud2 obstacle_msg;
  pcl::toROSMsg(*obstacle_world, obstacle_msg);
  obstacle_msg.header = cloud_msg->header;
  obstacle_cloud_pub_.publish(obstacle_msg);

  // Publish static cloud (original cloud with dynamic obstacles removed)
  // Always publish when enabled, even if no obstacles detected
  if (is_pub_dyn_obs_filter_pcl_)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (!obstacle_world->empty())
    {
      // Filter out dynamic obstacles
      filterDynamicObstacles(cloud, obstacle_world, static_cloud);
      ROS_DEBUG("[ObstacleDetector] Published static cloud: %lu points (removed %lu obstacle points)",
                static_cloud->size(), cloud->size() - static_cloud->size());
    }
    else
    {
      // No obstacles detected, publish original cloud as-is
      *static_cloud = *cloud;
      ROS_DEBUG("[ObstacleDetector] Published static cloud: %lu points (no obstacles detected)",
                static_cloud->size());
    }
    
    sensor_msgs::PointCloud2 static_msg;
    pcl::toROSMsg(*static_cloud, static_msg);
    static_msg.header = cloud_msg->header;
    static_cloud_pub_.publish(static_msg);
  }

  // Update obstacle trajectory and velocity estimation
  if (!obstacle_world->empty())
  {
    // Calculate centroid of all obstacle points
    Eigen::Vector4f obstacle_centroid;
    pcl::compute3DCentroid(*obstacle_world, obstacle_centroid);
    Eigen::Vector3d current_obstacle_pos(obstacle_centroid[0], obstacle_centroid[1], obstacle_centroid[2]);
    
    // Update obstacle history for velocity estimation (always needed for prediction)
    ros::Time current_time = cloud_msg->header.stamp;
    obstacle_history_.push_back(std::make_pair(current_time, current_obstacle_pos));
    
    // Keep only recent history for velocity estimation
    while (obstacle_history_.size() > static_cast<size_t>(velocity_window_size_))
    {
      obstacle_history_.pop_front();
    }
    
    // Update and publish obstacle trajectory (only in debug mode)
    if (debug_mode_)
    {
      if (obstacle_trajectory_.poses.empty() || 
          (current_obstacle_pos - last_obstacle_position_).norm() > trajectory_sample_distance_)
      {
        geometry_msgs::PoseStamped pose;
        pose.header = cloud_msg->header;
        pose.pose.position.x = current_obstacle_pos.x();
        pose.pose.position.y = current_obstacle_pos.y();
        pose.pose.position.z = current_obstacle_pos.z();
        pose.pose.orientation.w = 1.0;
        
        obstacle_trajectory_.poses.push_back(pose);
        last_obstacle_position_ = current_obstacle_pos;
        
        // Limit trajectory length
        if (obstacle_trajectory_.poses.size() > static_cast<size_t>(max_trajectory_length_))
        {
          obstacle_trajectory_.poses.erase(obstacle_trajectory_.poses.begin());
        }
      }
      
      // Publish obstacle trajectory
      obstacle_trajectory_.header.stamp = cloud_msg->header.stamp;
      if (!obstacle_trajectory_.poses.empty())
      {
        obstacle_path_pub_.publish(obstacle_trajectory_);
      }
    }
    
    // Predict and publish obstacle trajectory (always enabled if prediction is on)
    if (enable_prediction_)
    {
      predictObstacleTrajectory();
    }
  }

  ROS_DEBUG("[ObstacleDetector] Processed cloud: %lu points -> Sphere: %lu points -> Obstacle: %lu points",
            cloud->size(), cloud_sphere->size(), obstacle_world->size());
}

void ObstacleDetector::filterPointCloudInROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
  cloud_out->clear();
  cloud_out->header = cloud_in->header;

  for (const auto& point : cloud_in->points)
  {
    // Check if point is within ROI box in world frame
    if (point.x < roi_min_.x() || point.x > roi_max_.x())
      continue;
    
    if (point.y < roi_min_.y() || point.y > roi_max_.y())
      continue;
    
    if (point.z < roi_min_.z() || point.z > roi_max_.z())
      continue;

    cloud_out->push_back(point);
  }
}

void ObstacleDetector::filterPointCloudInSphere(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                                 const Eigen::Vector3d& center,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
  cloud_out->clear();
  cloud_out->header = cloud_in->header;

  for (const auto& point : cloud_in->points)
  {
    // Calculate distance from UAV position in world frame
    double dx = point.x - center.x();
    double dy = point.y - center.y();
    double dz = point.z - center.z();
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Check if distance is within detection sphere range
    if (distance < detection_radius_min_ || distance > detection_radius_max_)
      continue;

    cloud_out->push_back(point);
  }
}

void ObstacleDetector::filterDynamicObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                                const pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacles,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
  cloud_out->clear();
  cloud_out->header = cloud_in->header;

  if (obstacles->empty())
  {
    // No obstacles, return original cloud
    *cloud_out = *cloud_in;
    return;
  }

  // Build KD-Tree for obstacle points for efficient nearest neighbor search
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(obstacles);

  // Filter out points near obstacles
  for (const auto& point : cloud_in->points)
  {
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // Search for points within obstacle_filter_radius_
    int num_found = kdtree.radiusSearch(point, obstacle_filter_radius_, 
                                         pointIdxRadiusSearch, pointRadiusSquaredDistance);

    // If no obstacle points found nearby, keep this point
    if (num_found == 0)
    {
      cloud_out->push_back(point);
    }
  }
  
  ROS_DEBUG("[ObstacleDetector] Filtered %lu/%lu points near obstacles (radius: %.2fm)",
            cloud_in->size() - cloud_out->size(), cloud_in->size(), obstacle_filter_radius_);
}

Eigen::Vector3d ObstacleDetector::getPositionFromOdom()
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  
  if (!latest_odom_)
    return Eigen::Vector3d::Zero();

  const auto& p = latest_odom_->pose.pose.position;
  return Eigen::Vector3d(p.x, p.y, p.z);
}

bool ObstacleDetector::estimateObstacleVelocity(Eigen::Vector3d& velocity)
{
  if (obstacle_history_.size() < 2)
  {
    obstacle_velocity_valid_ = false;
    return false;
  }
  
  // Use linear regression or simple velocity estimation
  // For simplicity, use average velocity over the window
  Eigen::Vector3d vel_sum = Eigen::Vector3d::Zero();
  int count = 0;
  
  for (size_t i = 1; i < obstacle_history_.size(); ++i)
  {
    double dt = (obstacle_history_[i].first - obstacle_history_[i-1].first).toSec();
    if (dt > 0.001)  // Avoid division by very small numbers
    {
      Eigen::Vector3d dp = obstacle_history_[i].second - obstacle_history_[i-1].second;
      Eigen::Vector3d vel = dp / dt;
      vel_sum += vel;
      count++;
    }
  }
  
  if (count > 0)
  {
    velocity = vel_sum / count;
    double speed = velocity.norm();
    
    // Only consider as moving if speed is above threshold
    if (speed > min_obstacle_speed_)
    {
      current_obstacle_velocity_ = velocity;
      obstacle_velocity_valid_ = true;
      return true;
    }
  }
  
  obstacle_velocity_valid_ = false;
  return false;
}

void ObstacleDetector::predictObstacleTrajectory()
{
  if (obstacle_history_.empty())
    return;
  
  // Get current obstacle position
  Eigen::Vector3d start_pos = obstacle_history_.back().second;
  
  // Use current ROS time instead of historical timestamp to avoid time sync issues
  // This is especially important when using rosbag playback
  ros::Time start_time = ros::Time::now();
  
  // Estimate velocity
  Eigen::Vector3d velocity;
  if (!estimateObstacleVelocity(velocity))
  {
    // If obstacle is not moving, predict stationary trajectory
    velocity = Eigen::Vector3d::Zero();
  }
  
  // Predict end position using constant velocity model
  Eigen::Vector3d end_pos = start_pos + velocity * prediction_horizon_;
  
  // Assume zero acceleration at start and end
  Eigen::Vector3d start_vel = velocity;
  Eigen::Vector3d end_vel = velocity;  // Constant velocity assumption
  
  // Generate MINCO trajectory message
  traj_utils::MINCOTraj minco_msg;
  generateMINCOTrajectory(start_pos, start_vel, end_pos, end_vel, 
                          prediction_horizon_, minco_msg);
  
  minco_msg.drone_id = 30;  // Obstacle ID starts from 30
  minco_msg.start_time = start_time;
  
  // Publish predicted trajectory
  predicted_traj_pub_.publish(minco_msg);
  
  ROS_DEBUG("[ObstacleDetector] Predicted obstacle trajectory: vel=(%.2f, %.2f, %.2f) m/s, speed=%.2f m/s",
            velocity.x(), velocity.y(), velocity.z(), velocity.norm());
}

void ObstacleDetector::generateMINCOTrajectory(const Eigen::Vector3d& start_pos, 
                                                const Eigen::Vector3d& start_vel,
                                                const Eigen::Vector3d& end_pos,
                                                const Eigen::Vector3d& end_vel,
                                                double duration,
                                                traj_utils::MINCOTraj& minco_msg)
{
  // Set trajectory order (typically 3 or 5 for MINCO)
  minco_msg.order = 5;
  
  // Set start and end positions
  minco_msg.start_p[0] = start_pos.x();
  minco_msg.start_p[1] = start_pos.y();
  minco_msg.start_p[2] = start_pos.z();
  
  minco_msg.end_p[0] = end_pos.x();
  minco_msg.end_p[1] = end_pos.y();
  minco_msg.end_p[2] = end_pos.z();
  
  // Set start and end velocities
  minco_msg.start_v[0] = start_vel.x();
  minco_msg.start_v[1] = start_vel.y();
  minco_msg.start_v[2] = start_vel.z();
  
  minco_msg.end_v[0] = end_vel.x();
  minco_msg.end_v[1] = end_vel.y();
  minco_msg.end_v[2] = end_vel.z();
  
  // Set zero acceleration at start and end
  minco_msg.start_a[0] = 0.0;
  minco_msg.start_a[1] = 0.0;
  minco_msg.start_a[2] = 0.0;
  
  minco_msg.end_a[0] = 0.0;
  minco_msg.end_a[1] = 0.0;
  minco_msg.end_a[2] = 0.0;
  
  // Generate intermediate waypoints
  int num_inner_points = std::max(0, prediction_samples_ - 2);
  minco_msg.inner_x.clear();
  minco_msg.inner_y.clear();
  minco_msg.inner_z.clear();
  
  for (int i = 0; i < num_inner_points; ++i)
  {
    double t = (i + 1.0) / (num_inner_points + 1.0);
    Eigen::Vector3d inner_pos = start_pos + (end_pos - start_pos) * t;
    minco_msg.inner_x.push_back(inner_pos.x());
    minco_msg.inner_y.push_back(inner_pos.y());
    minco_msg.inner_z.push_back(inner_pos.z());
  }
  
  // Set duration for each segment
  minco_msg.duration.clear();
  int num_segments = num_inner_points + 1;
  double segment_duration = duration / num_segments;
  for (int i = 0; i < num_segments; ++i)
  {
    minco_msg.duration.push_back(segment_duration);
  }
  
  // Set clearance (not used for obstacle trajectory)
  minco_msg.des_clearance = 0.8;
  
  // Set trajectory ID (starting from 30 for obstacles)
  static int traj_id = 30;
  minco_msg.traj_id = traj_id++;
}

} // namespace dynamic_obstacle_detection
