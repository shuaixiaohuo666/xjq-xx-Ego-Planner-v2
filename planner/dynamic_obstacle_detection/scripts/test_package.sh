#!/bin/bash
# 障碍物检测包 - 快速测试脚本

echo "========================================="
echo "  Obstacle Detection Package - 测试脚本"
echo "========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否在正确的目录
if [ ! -d "/home/FastEventUAV" ]; then
    echo -e "${RED}错误: 找不到工作空间目录 /home/FastEventUAV${NC}"
    exit 1
fi

cd /home/FastEventUAV

# 1. 检查编译
echo "1. 检查包编译状态..."
if [ -f "devel/lib/obstacle_detection/obstacle_detection_node" ]; then
    echo -e "${GREEN}✓ 编译成功 - 可执行文件存在${NC}"
else
    echo -e "${RED}✗ 编译失败 - 未找到可执行文件${NC}"
    echo "尝试重新编译..."
    catkin_make -DCATKIN_WHITELIST_PACKAGES="obstacle_detection"
fi

echo ""

# 2. 检查库文件
echo "2. 检查共享库..."
if [ -f "devel/lib/libobstacle_detection.so" ]; then
    echo -e "${GREEN}✓ 共享库存在${NC}"
else
    echo -e "${RED}✗ 共享库不存在${NC}"
fi

echo ""

# 3. Source环境
echo "3. Source ROS环境..."
source devel/setup.bash
echo -e "${GREEN}✓ 环境已加载${NC}"

echo ""

# 4. 检查包是否可以找到
echo "4. 检查ROS包..."
if rospack find obstacle_detection > /dev/null 2>&1; then
    PACKAGE_PATH=$(rospack find obstacle_detection)
    echo -e "${GREEN}✓ 包路径: $PACKAGE_PATH${NC}"
else
    echo -e "${RED}✗ 无法找到obstacle_detection包${NC}"
    exit 1
fi

echo ""

# 5. 列出包内容
echo "5. 包内容检查..."
echo "配置文件:"
ls -1 $PACKAGE_PATH/config/*.yaml 2>/dev/null | while read file; do
    echo -e "  ${GREEN}✓${NC} $(basename $file)"
done

echo ""
echo "启动文件:"
ls -1 $PACKAGE_PATH/launch/*.launch 2>/dev/null | while read file; do
    echo -e "  ${GREEN}✓${NC} $(basename $file)"
done

echo ""
echo "脚本文件:"
ls -1 $PACKAGE_PATH/scripts/*.py 2>/dev/null | while read file; do
    echo -e "  ${GREEN}✓${NC} $(basename $file)"
    # 检查执行权限
    if [ -x "$file" ]; then
        echo -e "    ${GREEN}(可执行)${NC}"
    else
        echo -e "    ${YELLOW}(无执行权限)${NC}"
    fi
done

echo ""

# 6. 检查依赖
echo "6. 检查话题依赖 (需要roscore运行)..."
if pgrep -x "roscore" > /dev/null || pgrep -x "rosmaster" > /dev/null; then
    echo -e "${GREEN}✓ ROS Master正在运行${NC}"
    
    echo ""
    echo "检查输入话题:"
    
    # 检查点云话题
    if rostopic list 2>/dev/null | grep -q "/cloud_registered"; then
        echo -e "  ${GREEN}✓${NC} /cloud_registered 存在"
        rostopic hz /cloud_registered -w 2 2>/dev/null || echo -e "    ${YELLOW}(无数据发布)${NC}"
    else
        echo -e "  ${YELLOW}!${NC} /cloud_registered 不存在"
    fi
    
    # 检查里程计话题
    if rostopic list 2>/dev/null | grep -q "/ekf_quat/ekf_odom"; then
        echo -e "  ${GREEN}✓${NC} /ekf_quat/ekf_odom 存在"
        rostopic hz /ekf_quat/ekf_odom -w 2 2>/dev/null || echo -e "    ${YELLOW}(无数据发布)${NC}"
    else
        echo -e "  ${YELLOW}!${NC} /ekf_quat/ekf_odom 不存在"
    fi
else
    echo -e "${YELLOW}! ROS Master未运行，跳过话题检查${NC}"
    echo "  提示: 运行 'roscore' 来启动ROS Master"
fi

echo ""

# 7. 测试节点启动
echo "7. 节点可用性测试..."
if rospack plugins --attrib=plugin obstacle_detection 2>/dev/null; then
    echo -e "${GREEN}✓ 节点插件已注册${NC}"
fi

echo ""
echo "========================================="
echo "  测试完成"
echo "========================================="
echo ""
echo "下一步操作:"
echo "1. 如果ROS Master未运行，先启动: roscore"
echo "2. 启动障碍物检测节点:"
echo "   roslaunch obstacle_detection obstacle_detection.launch"
echo ""
echo "3. 启动监控脚本 (可选):"
echo "   rosrun obstacle_detection obstacle_monitor.py"
echo ""
echo "4. RViz可视化 (可选):"
echo "   rviz -d \$(rospack find obstacle_detection)/rviz/obstacle_detection.rviz"
echo ""
echo "更多信息请查看: $PACKAGE_PATH/QUICKREF.md"
echo ""
