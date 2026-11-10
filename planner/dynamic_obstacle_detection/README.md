# Dynamic Obstacle Detection

基于点云的动态障碍物检测与轨迹预测ROS包，用于无人机导航系统。

## 功能描述

该包实现了基于点云和里程计信息的动态障碍物检测与轨迹预测功能：

- 订阅高质量点云话题（如 `/cloud_registered`）和里程计话题
- 使用球形视野（Spherical FOV）检测无人机周围的障碍物点云
- 支持世界坐标系下的ROI（感兴趣区域）过滤

- 障碍物速度估计与轨迹预测
- 发布MINCO格式的障碍物预测轨迹到 `/broadcast_traj_to_planner`
- 障碍物ID从30开始编号，避免与真实无人机冲突

## 使用方法

### 1. 基本使用

```bash
# Source工作空间
source devel/setup.bash

# 直接通过fast_event_uav启动
roslaunch ego_planner fastevenuav_replay.launch

# 独立启动障碍物检测节点（默认配置）
roslaunch dynamic_obstacle_detection dynamic_obstacle_detection.launch

# 启用调试模式（可视化轨迹）
roslaunch dynamic_obstacle_detection dynamic_obstacle_detection.launch debug_mode:=true

# 自定义输入话题
roslaunch dynamic_obstacle_detection dynamic_obstacle_detection.launch \
  cloud_topic:=your_cloud_topic \
  odom_topic:=your_odom_topic
```

### 2. 自定义配置

launch文件参数已按使用频率分组：

**常用参数（Common Parameters）**：
- `detection_radius_min/max`: 球形检测范围
- `roi_min/max`: ROI区域边界
- `enable_prediction`: 是否启用预测
- `prediction_horizon`: 预测时长

**高级参数（Advanced Parameters）**：
- 聚合方法、轨迹采样等细节参数

**启动参数（Launch Arguments）**：
- `cloud_topic`: 输入点云话题（默认：cloud_registered）
- `odom_topic`: 输入里程计话题（默认：ekf_quat/ekf_odom）
- `debug_mode`: 调试模式开关（默认：false）

所有参数均在 `launch/dynamic_obstacle_detection.launch` 文件中配置：

#### 检测参数（世界坐标系下的球形检测）
```xml
<param name="detection_radius_min" value="0.5" />     <!-- 球形检测最小半径（米） -->
<param name="detection_radius_max" value="40.0" />    <!-- 球形检测最大半径（米） -->
```

球形检测以无人机为中心，在世界坐标系下检测距离在 `[detection_radius_min, detection_radius_max]` 范围内的所有点云。
无需坐标系转换，直接计算欧几里得距离。

#### ROI参数（世界坐标系）
```xml
<param name="use_roi_filter" value="true" />
<rosparam param="roi_min">[5.0, -5.0, 0.8]</rosparam>    <!-- ROI最小顶点 [x, y, z] -->
<rosparam param="roi_max">[45.0, 5.0, 3.0]</rosparam>    <!-- ROI最大顶点 [x, y, z] -->
```

ROI功能用于只处理世界坐标系下特定立方体区域内的点云，排除不感兴趣区域的干扰。

#### 聚合方法
```xml
<param name="aggregation_method" value="all" />  <!-- 选项: "mean", "median", "all" -->
```

### 3. 话题说明

**订阅话题：**
- `/cloud_registered` (sensor_msgs/PointCloud2): 输入点云（世界坐标系）
- `/drone_0_visual_slam/odom` (nav_msgs/Odometry): 无人机里程计

**发布话题：**
- `/obstacle_cloud` (sensor_msgs/PointCloud2): 检测到的障碍物点云
- `/broadcast_traj_to_planner` (traj_utils/MINCOTraj): 预测的障碍物轨迹（重映射自 `/predicted_obstacle_traj`）
- `/static_cloud` (sensor_msgs/PointCloud2): 去除动态障碍物后的全局点云（可选，需设置 `is_pub_dyn_obs_filter_pcl=true`）
  - **实时发布**：无论是否检测到障碍物，只要启用该功能都会持续发布
  - **无障碍物时**：直接发布原始完整点云
  - **有障碍物时**：发布过滤掉障碍物周围点（半径由 `obstacle_filter_radius` 控制）的点云
  > ⚠ **注意：** 不建议启用/static_cloud，点云数据量大，可能影响系统性能，且大部分时间点云内容与原始点云无异。假如后续建图模块需要静态点云，可以考虑在对应模块中实现动态障碍物过滤功能，而非通过本包发布大数据量的静态点云。

**Debug话题（仅当 `debug_mode=true` 时发布）：**
- `/filtered_cloud` (sensor_msgs/PointCloud2): 球形检测范围内的点云（用于调试）
- `/uav_trajectory` (nav_msgs/Path): 无人机的飞行轨迹
- `/obstacle_trajectory` (nav_msgs/Path): 障碍物的运动轨迹

## 参数说明

### 检测参数 - 球形检测（世界坐标系）

- `detection_radius_min`: 球形检测最小半径（米），用于设置盲区以避免检测到无人机本体
- `detection_radius_max`: 球形检测最大半径（米），最大检测距离


### ROI参数（Region of Interest）

- `use_roi_filter`: 是否启用ROI过滤）
- `roi_min`: ROI立方体最小顶点坐标 [x_min, y_min, z_min]
- `roi_max`: ROI立方体最大顶点坐标 [x_max, y_max, z_max]

### 聚合方法

- `mean`: 计算所有检测点的平均位置，输出单个障碍物点
- `median`: 计算所有检测点的中位数位置，输出单个障碍物点
- `all`: 输出所有点云

### 轨迹参数

- `publish_trajectories`: 是否发布无人机和障碍物轨迹
- `max_trajectory_length`: 最大轨迹点数（默认500），超过后自动删除最早的点
- `trajectory_sample_distance`: 轨迹采样距离（米），只有移动超过此距离才记录新点（默认0.1米）

### 轨迹预测参数

- `enable_prediction`: 是否启用轨迹预测（默认true）
- `prediction_horizon`: 预测时长，单位秒（默认3.0）
- `prediction_samples`: 预测轨迹采样点数（默认20）
- `velocity_window_size`: 速度估计滑动窗口大小（默认10）
- `min_obstacle_speed`: 最小障碍物速度阈值，单位m/s（默认0.1）

**障碍物ID说明**：
- `drone_id`: 固定为 30（表示这是障碍物而非真实无人机）
- `traj_id`: 从 30 开始递增（30, 31, 32...），每次预测递增

### 调试参数

- `debug_mode`: 是否启用调试模式（默认false）
  - `true`: 发布调试可视化话题：
    - `/filtered_cloud`: 球形检测范围内的点云
    - `/uav_trajectory`: UAV飞行轨迹
    - `/obstacle_trajectory`: 障碍物运动轨迹
  - `false`: 不发布调试话题，节省计算资源和带宽



### 静态点云发布参数

- `is_pub_dyn_obs_filter_pcl`: 是否发布去除动态障碍物后的静态点云（默认false）
  - `true`: 实时发布静态点云到 `/static_cloud` 话题
  - `false`: 不发布静态点云，节省计算资源
- `obstacle_filter_radius`: 障碍物过滤半径（米，默认0.5）
  - 在检测到的障碍物周围该半径范围内的点会被过滤掉
  - 使用KDTree进行高效的半径搜索

**静态点云发布逻辑**：
- **启用时**（`is_pub_dyn_obs_filter_pcl=true`）：持续实时发布静态点云
- **无障碍物时**：发布完整的原始点云（未做任何过滤）
- **有障碍物时**：发布过滤掉动态障碍物周围点的点云


## 可视化

在RViz中添加以下显示项：

**基础显示（始终可用）：**
1. **PointCloud2**: 订阅 `/obstacle_cloud` 查看检测到的障碍物（红色）
2. **PointCloud2**: 订阅 `/cloud_registered` 查看原始点云（白色）
3. **Odometry**: 订阅 `/drone_0_visual_slam/odom` 查看无人机位置

**调试显示（需要设置 `debug_mode=true`）：**

4. **PointCloud2**: 订阅 `/filtered_cloud` 查看球形FOV内的点云（绿色）
5. **Path**: 订阅 `/uav_trajectory` 查看无人机飞行轨迹（绿色线）
6. **Path**: 订阅 `/obstacle_trajectory` 查看障碍物运动轨迹（橙色线）


## 坐标系说明

- **世界坐标系**（`camera_init`）：所有处理都在世界坐标系下进行
- 输入点云和里程计都在世界坐标系
- 球形检测直接在世界坐标系下计算距离，无需坐标转换
- 输出的障碍物点云也在世界坐标系

## 处理流程

```
原始点云 (世界坐标系)
    ↓
ROI过滤 (可选，世界坐标系)
    ↓
球形检测 (世界坐标系，以无人机为中心)
    ↓
障碍物聚合
    ↓
速度估计 + 轨迹预测
    ↓
发布 MINCO 轨迹消息
```

## 与EGO-Planner集成

该包设计用于与EGO-Planner配合使用：

- 预测的障碍物轨迹通过topic remap发布到 `/broadcast_traj_to_planner`
- EGO-Planner订阅该话题，接收障碍物的预测轨迹信息
- 障碍物ID（30）确保不与真实无人机ID冲突
- 使用MINCO轨迹格式，与EGO-Planner内部格式一致


## 依赖项

- ROS Kinetic/Melodic/Noetic
- PCL (Point Cloud Library)
- Eigen3
- `traj_utils` (MINCO消息定义)

## 注意事项

1. 确保输入点云质量良好（推荐使用FAST-LIO等高质量SLAM输出）
2. ROI参数需根据实际飞行环境调整
3. 球形FOV半径应根据传感器性能和飞行速度合理设置
4. 障碍物ID从30开始，请确保规划器支持该ID范围
5. 时间戳同步：预测轨迹使用 `ros::Time::now()` 作为起始时间，避免rosbag回放时的时间戳不同步问题

