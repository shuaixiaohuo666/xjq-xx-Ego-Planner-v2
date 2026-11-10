#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
障碍物检测测试脚本
用于监控和显示障碍物检测节点的输出信息
"""

import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


class ObstacleMonitor:
    def __init__(self):
        rospy.init_node('obstacle_monitor', anonymous=True)
        
        # 订阅话题
        self.cloud_sub = rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_callback)
        self.odom_sub = rospy.Subscriber('/drone_0_visual_slam/odom', Odometry, self.odom_callback)
        self.obstacle_sub = rospy.Subscriber('/obstacle_cloud', PointCloud2, self.obstacle_callback)
        self.filtered_sub = rospy.Subscriber('/filtered_cloud', PointCloud2, self.filtered_callback)
        
        # 统计信息
        self.cloud_count = 0
        self.obstacle_count = 0
        self.last_cloud_time = None
        self.last_odom_time = None
        
        rospy.loginfo("障碍物监控节点已启动")
        
    def cloud_callback(self, msg):
        self.cloud_count += 1
        self.last_cloud_time = rospy.Time.now()
        
        # 计算点云数量
        point_count = msg.width * msg.height
        
        if self.cloud_count % 10 == 0:  # 每10帧打印一次
            # rospy.loginfo(f"原始点云: {point_count} 点, 帧率: {self.cloud_count}/s")
            pass
    
    def odom_callback(self, msg):
        self.last_odom_time = rospy.Time.now()
        
        pos = msg.pose.pose.position
        # rospy.loginfo(f"UAV位置: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
    
    def obstacle_callback(self, msg):
        self.obstacle_count += 1
        
        # 解析点云
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append(p)
        
        if len(points) > 0:
            rospy.loginfo("=" * 50)
            rospy.loginfo(f"检测到障碍物！点数: {len(points)}")
            
            # 计算平均位置
            avg_x = sum(p[0] for p in points) / len(points)
            avg_y = sum(p[1] for p in points) / len(points)
            avg_z = sum(p[2] for p in points) / len(points)
            
            rospy.loginfo(f"障碍物中心位置: x={avg_x:.2f}, y={avg_y:.2f}, z={avg_z:.2f}")
            
            # 计算距离
            distance = (avg_x**2 + avg_y**2 + avg_z**2)**0.5
            rospy.loginfo(f"距离无人机: {distance:.2f} 米")
            rospy.loginfo("=" * 50)
        else:
            rospy.loginfo("视野内无障碍物")
    
    def filtered_callback(self, msg):
        # 计算视野内的点数
        point_count = msg.width * msg.height
        
        if point_count > 0:
            rospy.loginfo(f"视野内点云: {point_count} 点")
    
    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            # 检查话题活跃度
            now = rospy.Time.now()
            
            if self.last_cloud_time and (now - self.last_cloud_time).to_sec() > 2.0:
                rospy.logwarn("警告: 长时间未收到点云数据！")
            
            if self.last_odom_time and (now - self.last_odom_time).to_sec() > 2.0:
                rospy.logwarn("警告: 长时间未收到里程计数据！")
            
            rate.sleep()


if __name__ == '__main__':
    try:
        monitor = ObstacleMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
