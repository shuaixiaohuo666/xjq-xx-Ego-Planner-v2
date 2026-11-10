# 障碍物轨迹预测功能

## 概述

障碍物检测包现在支持动态障碍物轨迹预测，并以MINCO（Minimum Control）轨迹格式发布预测结果。预测算法基于恒速模型，使用历史位置数据估计障碍物速度。

## 功能特性

### 1. 速度估计
- **滑动窗口法**：使用历史位置序列计算平均速度
- **自适应窗口**：窗口大小可配置（默认10个样本）
- **速度阈值**：过滤静止或低速障碍物（默认0.1 m/s）

### 2. 轨迹预测
- **预测模型**：基于恒速假设的线性预测
- **预测时长**：可配置预测时间范围（默认3.0秒）
- **采样点数**：可配置预测轨迹采样数（默认20个点）

### 3. MINCO消息格式
预测轨迹使用`traj_utils/MINCOTraj`消息类型发布：

```msg
int16 drone_id          # 障碍物ID（设为0）
int32 traj_id           # 轨迹序列号（递增）
time start_time         # 预测起始时间
float32 des_clearance   # 期望安全距离（障碍物为0）
uint8 order             # 多项式阶数（默认5阶）
float32[3] start_p      # 起始位置
float32[3] start_v      # 起始速度
float32[3] start_a      # 起始加速度（为0）
float32[3] end_p        # 终止位置
float32[3] end_v        # 终止速度
float32[3] end_a        # 终止加速度（为0）
float32[] inner_x       # 中间航点X坐标
float32[] inner_y       # 中间航点Y坐标
float32[] inner_z       # 中间航点Z坐标
float32[] duration      # 每段持续时间
```

## 配置参数

在`config/obstacle_detection.yaml`中配置预测功能：

```yaml
# 轨迹预测参数
enable_prediction: true           # 是否启用预测
prediction_horizon: 3.0           # 预测时长（秒）
prediction_samples: 20            # 预测轨迹采样点数
velocity_window_size: 10          # 速度估计窗口大小
min_obstacle_speed: 0.1           # 最小障碍物速度（m/s）
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enable_prediction` | bool | true | 启用/禁用预测功能 |
| `prediction_horizon` | double | 3.0 | 预测未来多少秒的轨迹 |
| `prediction_samples` | int | 20 | 预测轨迹的采样点数量 |
| `velocity_window_size` | int | 10 | 计算速度时使用的历史位置数 |
| `min_obstacle_speed` | double | 0.1 | 只对速度大于此阈值的障碍物预测 |

## 发布的话题

### `/predicted_obstacle_traj` (traj_utils/MINCOTraj)
发布预测的障碍物轨迹，包含：
- 当前位置和速度
- 预测的未来位置
- 轨迹多项式参数

**发布频率**：跟随点云处理频率（约10-20 Hz）

## 工作流程

```
点云处理 → 障碍物检测 → 位置历史记录 → 速度估计 → 轨迹预测 → MINCO消息发布
    ↓                                                           ↓
障碍物位置发布                                            预测轨迹发布
```

### 详细步骤

1. **障碍物检测**：从点云中检测并聚合障碍物位置
2. **历史记录**：维护障碍物位置的时间戳队列
3. **速度估计**：
   - 计算相邻位置间的瞬时速度
   - 滑动窗口平均得到稳定速度估计
4. **轨迹预测**：
   - 使用当前位置和估计速度
   - 线性外推未来轨迹点
   - 检查速度是否超过阈值
5. **MINCO生成**：
   - 设置起止位置和速度
   - 生成中间航点
   - 计算段时间分配
6. **消息发布**：在`/predicted_obstacle_traj`话题发布

## 算法细节

### 速度估计算法

```cpp
// 伪代码
velocity_sum = 0
for i from 1 to history_size-1:
    dt = time[i] - time[i-1]
    dp = position[i] - position[i-1]
    velocity_sum += dp / dt
    
average_velocity = velocity_sum / (history_size - 1)
```

### 预测算法

```cpp
// 恒速模型
predicted_position(t) = current_position + velocity * t

// 生成采样点
for i from 0 to prediction_samples-1:
    t = (i / (prediction_samples-1)) * prediction_horizon
    waypoint[i] = current_position + velocity * t
```

## 使用示例

### 启动节点

```bash
roslaunch obstacle_detection obstacle_detection.launch
```

### 订阅预测轨迹

```python
#!/usr/bin/env python3
import rospy
from traj_utils.msg import MINCOTraj

def prediction_callback(msg):
    print(f"预测轨迹ID: {msg.traj_id}")
    print(f"起始位置: ({msg.start_p[0]}, {msg.start_p[1]}, {msg.start_p[2]})")
    print(f"终止位置: ({msg.end_p[0]}, {msg.end_p[1]}, {msg.end_p[2]})")
    print(f"速度: ({msg.start_v[0]}, {msg.start_v[1]}, {msg.start_v[2]})")
    print(f"中间航点数: {len(msg.inner_x)}")
    print(f"总时长: {sum(msg.duration)} 秒")

rospy.init_node('prediction_subscriber')
rospy.Subscriber('/predicted_obstacle_traj', MINCOTraj, prediction_callback)
rospy.spin()
```

### C++订阅示例

```cpp
#include <ros/ros.h>
#include <traj_utils/MINCOTraj.h>

void predictionCallback(const traj_utils::MINCOTraj::ConstPtr& msg)
{
    ROS_INFO("Predicted trajectory ID: %d", msg->traj_id);
    ROS_INFO("Start pos: (%.2f, %.2f, %.2f)", 
             msg->start_p[0], msg->start_p[1], msg->start_p[2]);
    ROS_INFO("Velocity: (%.2f, %.2f, %.2f)", 
             msg->start_v[0], msg->start_v[1], msg->start_v[2]);
    ROS_INFO("Prediction horizon: %.2f seconds", 
             std::accumulate(msg->duration.begin(), msg->duration.end(), 0.0f));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "prediction_subscriber");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/predicted_obstacle_traj", 
                                       10, predictionCallback);
    ros::spin();
    return 0;
}
```


## 限制与注意事项

### 当前限制

1. **恒速假设**：假设障碍物保持当前速度运动
   - 不考虑加速度变化
   - 对非线性运动预测精度较低

2. **单障碍物**：当前版本一次只预测一个障碍物
   - 检测到多个障碍物时，使用聚合位置

3. **短期预测**：适用于短时预测（3-5秒）
   - 长期预测误差会累积


## 调试与可视化

### 检查预测是否工作

```bash
# 查看预测话题
rostopic echo /predicted_obstacle_traj

# 检查发布频率
rostopic hz /predicted_obstacle_traj

# 查看消息内容
rostopic echo /predicted_obstacle_traj/start_p
rostopic echo /predicted_obstacle_traj/end_p
```

