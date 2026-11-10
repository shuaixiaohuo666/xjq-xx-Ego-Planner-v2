#!/usr/bin/env python3
"""
测试障碍物轨迹预测功能
订阅预测轨迹并打印关键信息
"""

import rospy
from traj_utils.msg import MINCOTraj
import numpy as np


class PredictionTester:
    def __init__(self):
        rospy.init_node('prediction_tester', anonymous=True)
        
        # 订阅预测轨迹话题
        self.prediction_sub = rospy.Subscriber(
            '/predicted_obstacle_traj',
            MINCOTraj,
            self.prediction_callback,
            queue_size=10
        )
        
        self.msg_count = 0
        
        rospy.loginfo("预测测试节点已启动")
        rospy.loginfo("等待预测轨迹消息...")
    
    def prediction_callback(self, msg):
        """处理预测轨迹消息"""
        self.msg_count += 1
        
        # 提取关键信息
        start_pos = np.array([msg.start_p[0], msg.start_p[1], msg.start_p[2]])
        end_pos = np.array([msg.end_p[0], msg.end_p[1], msg.end_p[2]])
        start_vel = np.array([msg.start_v[0], msg.start_v[1], msg.start_v[2]])
        
        # 计算速度大小
        speed = np.linalg.norm(start_vel)
        
        # 计算预测距离
        distance = np.linalg.norm(end_pos - start_pos)
        
        # 计算总时长
        total_duration = sum(msg.duration)
        
        # 计算中间航点数
        num_inner_points = len(msg.inner_x)
        
        # 打印信息
        rospy.loginfo("="*60)
        rospy.loginfo(f"预测消息 #{self.msg_count}")
        rospy.loginfo(f"轨迹ID: {msg.traj_id}")
        rospy.loginfo(f"多项式阶数: {msg.order}")
        rospy.loginfo("-"*60)
        rospy.loginfo(f"起始位置: ({start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f})")
        rospy.loginfo(f"终止位置: ({end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f})")
        rospy.loginfo(f"障碍物速度: ({start_vel[0]:.3f}, {start_vel[1]:.3f}, {start_vel[2]:.3f})")
        rospy.loginfo(f"速度大小: {speed:.3f} m/s")
        rospy.loginfo("-"*60)
        rospy.loginfo(f"预测距离: {distance:.3f} m")
        rospy.loginfo(f"预测时长: {total_duration:.3f} s")
        rospy.loginfo(f"中间航点数: {num_inner_points}")
        rospy.loginfo(f"段数: {len(msg.duration)}")
        
        # 验证数据一致性
        if num_inner_points + 1 != len(msg.duration):
            rospy.logwarn(f"警告：中间点数({num_inner_points}) + 1 != 段数({len(msg.duration)})")
        
        # 检查速度合理性
        if speed < 0.1:
            rospy.logwarn(f"警告：障碍物速度过低 ({speed:.3f} m/s)")
        elif speed > 10.0:
            rospy.logwarn(f"警告：障碍物速度过高 ({speed:.3f} m/s)")
        
        # 检查预测是否合理
        expected_distance = speed * total_duration
        distance_error = abs(distance - expected_distance) / expected_distance if expected_distance > 0 else 0
        if distance_error > 0.1:  # 10%误差
            rospy.logwarn(f"警告：预测距离误差较大 ({distance_error*100:.1f}%)")
        
        rospy.loginfo("="*60)
    
    def run(self):
        """运行测试节点"""
        rospy.spin()


if __name__ == '__main__':
    try:
        tester = PredictionTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass
