#!/bin/bash
# VR Hand Publisher - ROS2 端启动脚本
# 使用方法: bash start_ros.sh

set -e

echo "=== 安装 ros_tcp_endpoint ==="
cd "$(dirname "$0")/ros2-tools/ros_tcp_endpoint"
pip3 install -e . 2>/dev/null || true

echo ""
echo "=== 启动 ROS TCP Endpoint (端口 10000) ==="
echo "等待 Unity 连接..."
source /opt/ros/humble/setup.bash 2>/dev/null
python3 -c "
from ros_tcp_endpoint.default_server_endpoint import main
import sys
sys.argv = ['default_server_endpoint', '--ros-args', '-p', 'tcp_port:=10000']
main()
" &
ENDPOINT_PID=$!
echo "Endpoint PID: $ENDPOINT_PID"

sleep 2

echo ""
echo "=== 启动 VR Hand Subscriber ==="
python3 "$(dirname "$0")/ros2-tools/vr_hand_subscriber.py" &
SUBSCRIBER_PID=$!
echo "Subscriber PID: $SUBSCRIBER_PID"

echo ""
echo "=== ROS2 端已启动 ==="
echo "  - Endpoint 监听端口: 10000"
echo "  - Subscriber 监听话题: /vr/left_hand/pose, /vr/right_hand/pose, /vr/status"
echo ""
echo "按 Ctrl+C 停止"

trap "kill $ENDPOINT_PID $SUBSCRIBER_PID 2>/dev/null; exit" INT TERM
wait
