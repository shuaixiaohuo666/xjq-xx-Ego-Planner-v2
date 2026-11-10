#ifndef DYNAMIC_OBSTACLE_DETECTOR_H
#define DYNAMIC_OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <traj_utils/MINCOTraj.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dynamic_obstacle_detection
{

class ObstacleDetector
{
public:
  ObstacleDetector(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~ObstacleDetector();

  void init();
  void run();

private:
  // Callback functions
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  // Processing functions
  void processPointCloud();
  void filterPointCloudInSphere(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                 const Eigen::Vector3d& center,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
  void filterPointCloudInROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
  void filterDynamicObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacles,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
  Eigen::Vector3d getPositionFromOdom();
  
  // Trajectory prediction functions
  void predictObstacleTrajectory();
  bool estimateObstacleVelocity(Eigen::Vector3d& velocity);
  void generateMINCOTrajectory(const Eigen::Vector3d& start_pos, 
                                const Eigen::Vector3d& start_vel,
                                const Eigen::Vector3d& end_pos,
                                const Eigen::Vector3d& end_vel,
                                double duration,
                                traj_utils::MINCOTraj& minco_msg);

  // ROS node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers and Publishers
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher obstacle_cloud_pub_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher uav_path_pub_;
  ros::Publisher obstacle_path_pub_;
  ros::Publisher predicted_traj_pub_;  // MINCO trajectory publisher
  ros::Publisher static_cloud_pub_;    // Static cloud publisher (without dynamic obstacles)

  // Parameters
  std::string obstacle_cloud_topic_;
  std::string filtered_cloud_topic_;
  std::string static_cloud_topic_;
  std::string world_frame_;

  // Sphere detection parameters (in world frame)
  double detection_radius_min_;   // 球形检测最小半径 (m)
  double detection_radius_max_;   // 球形检测最大半径 (m)

  // Dynamic obstacle filtering parameters
  bool is_pub_dyn_obs_filter_pcl_;  // 是否发布去除动态障碍物后的点云
  double obstacle_filter_radius_;    // 障碍物过滤半径 (m)

  // ROI parameters (Region of Interest in world frame)
  bool use_roi_filter_;     // 是否启用ROI过滤
  Eigen::Vector3d roi_min_; // ROI最小顶点 (x_min, y_min, z_min)
  Eigen::Vector3d roi_max_; // ROI最大顶点 (x_max, y_max, z_max)

  // Aggregation method
  std::string aggregation_method_; // "mean", "median", "centroid"

  // Trajectory parameters
  int max_trajectory_length_;
  double trajectory_sample_distance_;
  
  // Prediction parameters
  bool enable_prediction_;
  double prediction_horizon_;      // 预测时长（秒）
  int prediction_samples_;         // 预测采样点数
  int velocity_window_size_;       // 速度估计的窗口大小
  double min_obstacle_speed_;      // 最小障碍物速度阈值（m/s）

  // Debug parameters
  bool debug_mode_;                // 是否启用调试模式（发布轨迹可视化）

  // Data
  sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
  nav_msgs::Odometry::ConstPtr latest_odom_;
  
  bool cloud_received_;
  bool odom_received_;

  // Trajectory storage
  nav_msgs::Path uav_trajectory_;
  nav_msgs::Path obstacle_trajectory_;
  Eigen::Vector3d last_uav_position_;
  Eigen::Vector3d last_obstacle_position_;
  
  // Obstacle tracking for prediction
  std::deque<std::pair<ros::Time, Eigen::Vector3d>> obstacle_history_;  // 时间戳和位置
  Eigen::Vector3d current_obstacle_velocity_;
  bool obstacle_velocity_valid_;

  // Mutex for thread safety
  std::mutex cloud_mutex_;
  std::mutex odom_mutex_;
};

} // namespace dynamic_obstacle_detection

#endif // DYNAMIC_OBSTACLE_DETECTOR_H
