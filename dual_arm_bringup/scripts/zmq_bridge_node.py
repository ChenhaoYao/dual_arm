#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# zmq_bridge_node.py — dual_arm (ROS2) ↔ lerobot 数据桥接节点
#
# 作用：
#   把 ROS2 侧的数据通过 ZMQ 转发给 lerobot 项目，并把 lerobot 下发的动作转回 ROS2。
#
# 数据流：
#   /joint_states (sensor_msgs/JointState)  ──ZMQ PUB:5556──▶  lerobot DualArmZMQRobot
#   相机 (OpenCV 直接采集)                    ──ZMQ PUB:5555/5559/5560──▶  lerobot ZMQCamera
#   VR 动作 (ROS2 topic, TODO)               ──ZMQ PUB:5557──▶  lerobot VRZMQTeleop
#   lerobot 动作                              ──ZMQ PULL:5558──▶  /xxx_arm_controller/joint_trajectory
#
# ZMQ 端口约定（与 lerobot 侧 DualArmZMQRobotConfig / VRZMQTeleopConfig 默认值一致）：
#   5555  head_camera   (PUB)
#   5556  joint_states  (PUB)
#   5557  vr_action     (PUB)   ← VR 部分目前是占位 TODO
#   5558  lerobot_action(PULL)
#   5559  left_wrist    (PUB)
#   5560  right_wrist   (PUB)
#
# 运行（需先 source ROS2 与本工作区环境）：
#   python3 ~/dual_arm/dual_arm_bringup/scripts/zmq_bridge_node.py
#
# 依赖：rclpy, sensor_msgs, trajectory_msgs, pyzmq, opencv-python, numpy

import base64
import contextlib
import json
import threading
import time

import cv2
import numpy as np
import rclpy
import zmq
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ============================ 可调参数 ============================

# 左右臂关节顺序（需与 soem_bridge.yaml / MoveIt 保持一致）
LEFT_JOINTS = [f"laxis{i}_joint" for i in range(1, 8)]
RIGHT_JOINTS = [f"raxis{i}_joint" for i in range(1, 8)]
ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS

# ZMQ 端口
PORT_HEAD_CAMERA = 5555
PORT_JOINT_STATE = 5556
PORT_VR_ACTION = 5557
PORT_LEROBOT_ACTION = 5558
PORT_LEFT_WRIST = 5559
PORT_RIGHT_WRIST = 5560

# 相机配置：name → (device_id, port)
# device_id 为 /dev/videoX 的 X；按实际硬件修改
CAMERA_CONFIG = {
    "head_camera": {"device_id": 0, "port": PORT_HEAD_CAMERA},
    "left_wrist_camera": {"device_id": 2, "port": PORT_LEFT_WRIST},
    "right_wrist_camera": {"device_id": 4, "port": PORT_RIGHT_WRIST},
}
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
JPEG_QUALITY = 80

# lerobot 动作下发的目标 JTC 话题
LEFT_TRAJ_TOPIC = "/left_arm_controller/joint_trajectory"
RIGHT_TRAJ_TOPIC = "/right_arm_controller/joint_trajectory"
# 轨迹点到达时间（秒）；越小越跟手，但太小会被 JTC 拒绝
TRAJ_TIME_FROM_START = 0.1

# VR 动作的 ROS2 话题（TODO：等 VR 同事确定后填写）
VR_ACTION_TOPIC = "/vr_action"  # 占位


def _encode_image(image: np.ndarray, quality: int = JPEG_QUALITY) -> str:
    """RGB/BGR 图像编码为 base64 JPEG 字符串。"""
    ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return ""
    return base64.b64encode(buffer).decode("utf-8")


class CameraPublisher(threading.Thread):
    """独立线程：OpenCV 采集 + ZMQ PUB 发布单个相机。

    发布格式与 lerobot ImageServer / ZMQCamera 完全一致：
        {"timestamps": {cam_name: float}, "images": {cam_name: "<base64-jpeg>"}}
    """

    def __init__(self, context: zmq.Context, name: str, device_id: int, port: int, logger):
        super().__init__(daemon=True, name=f"cam_{name}")
        self.cam_name = name
        self.device_id = device_id
        self.port = port
        self.logger = logger
        self._stop = threading.Event()

        self.socket = context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 5)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f"tcp://*:{port}")

    def run(self):
        cap = cv2.VideoCapture(self.device_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        if not cap.isOpened():
            self.logger.error(f"[{self.cam_name}] 无法打开相机 /dev/video{self.device_id}")
            return

        self.logger.info(f"[{self.cam_name}] 采集中 → ZMQ:{self.port} (/dev/video{self.device_id})")
        period = 1.0 / CAMERA_FPS
        while not self._stop.is_set():
            t0 = time.time()
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            # OpenCV 默认 BGR，转 RGB 以匹配 lerobot ColorMode.RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            message = {
                "timestamps": {self.cam_name: time.time()},
                "images": {self.cam_name: _encode_image(frame_rgb)},
            }
            with contextlib.suppress(zmq.Again):
                self.socket.send_string(json.dumps(message), zmq.NOBLOCK)

            sleep = period - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)

        cap.release()
        self.socket.close()

    def stop(self):
        self._stop.set()


class ZmqBridgeNode(Node):
    def __init__(self):
        super().__init__("zmq_bridge_node")

        self.context = zmq.Context()

        # ---- 关节状态 PUB ----
        self.joint_pub = self.context.socket(zmq.PUB)
        self.joint_pub.setsockopt(zmq.SNDHWM, 10)
        self.joint_pub.setsockopt(zmq.LINGER, 0)
        self.joint_pub.bind(f"tcp://*:{PORT_JOINT_STATE}")

        # ---- VR 动作 PUB（占位，等 VR ROS2 topic 接入后启用）----
        self.vr_pub = self.context.socket(zmq.PUB)
        self.vr_pub.setsockopt(zmq.SNDHWM, 10)
        self.vr_pub.setsockopt(zmq.LINGER, 0)
        self.vr_pub.bind(f"tcp://*:{PORT_VR_ACTION}")

        # ---- lerobot 动作 PULL ----
        self.action_pull = self.context.socket(zmq.PULL)
        self.action_pull.setsockopt(zmq.RCVHWM, 1)
        self.action_pull.setsockopt(zmq.LINGER, 0)
        self.action_pull.bind(f"tcp://*:{PORT_LEROBOT_ACTION}")

        # ---- ROS2 订阅 /joint_states ----
        js_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, js_qos
        )

        # ---- ROS2 发布到 JTC（接收 lerobot 动作后转发）----
        self.left_traj_pub = self.create_publisher(JointTrajectory, LEFT_TRAJ_TOPIC, 10)
        self.right_traj_pub = self.create_publisher(JointTrajectory, RIGHT_TRAJ_TOPIC, 10)

        # ---- TODO: VR 动作订阅 ----
        # 等 VR 同事确定消息类型后，取消注释并实现 on_vr_action。
        # 假设 VR 端发布 sensor_msgs/JointState（name + position）：
        #
        # self.vr_sub = self.create_subscription(
        #     JointState, VR_ACTION_TOPIC, self.on_vr_action, 10
        # )
        self.get_logger().warn(
            f"VR 订阅未启用（占位）。等 VR 端确定 topic ({VR_ACTION_TOPIC}) 与消息类型后，"
            "在 zmq_bridge_node.py 中实现 on_vr_action。"
        )

        # ---- 相机采集线程 ----
        self.cam_threads: list[CameraPublisher] = []
        for name, cfg in CAMERA_CONFIG.items():
            t = CameraPublisher(self.context, name, cfg["device_id"], cfg["port"], self.get_logger())
            t.start()
            self.cam_threads.append(t)

        # ---- lerobot 动作接收定时器（100Hz 轮询）----
        self.action_timer = self.create_timer(0.01, self.poll_lerobot_action)

        self.get_logger().info("zmq_bridge_node 已启动：")
        self.get_logger().info(f"  /joint_states  → ZMQ PUB :{PORT_JOINT_STATE}")
        self.get_logger().info(f"  cameras        → ZMQ PUB :{PORT_HEAD_CAMERA}/{PORT_LEFT_WRIST}/{PORT_RIGHT_WRIST}")
        self.get_logger().info(f"  vr_action      → ZMQ PUB :{PORT_VR_ACTION} (TODO)")
        self.get_logger().info(f"  lerobot action ← ZMQ PULL:{PORT_LEROBOT_ACTION}")

    # ------------------------------------------------------------------
    # ROS2 → ZMQ：关节状态
    # ------------------------------------------------------------------
    def on_joint_state(self, msg: JointState):
        payload = {
            "names": list(msg.name),
            "positions": list(msg.position),
            "velocities": list(msg.velocity) if msg.velocity else [],
            "timestamp": time.time(),
        }
        with contextlib.suppress(zmq.Again):
            self.joint_pub.send_string(json.dumps(payload), zmq.NOBLOCK)

    # ------------------------------------------------------------------
    # ROS2 → ZMQ：VR 动作（TODO 占位）
    # ------------------------------------------------------------------
    # def on_vr_action(self, msg: JointState):
    #     joint_positions = dict(zip(msg.name, msg.position))
    #     payload = {"joint_positions": joint_positions, "timestamp": time.time()}
    #     with contextlib.suppress(zmq.Again):
    #         self.vr_pub.send_string(json.dumps(payload), zmq.NOBLOCK)

    # ------------------------------------------------------------------
    # ZMQ → ROS2：lerobot 动作 → JTC
    # ------------------------------------------------------------------
    def poll_lerobot_action(self):
        try:
            msg = self.action_pull.recv_string(zmq.NOBLOCK)
        except zmq.Again:
            return
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"接收 lerobot 动作失败: {e}")
            return

        try:
            data = json.loads(msg)
            joint_positions: dict[str, float] = data.get("joint_positions", {})
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"解析 lerobot 动作失败: {e}")
            return

        self._publish_trajectory(self.left_traj_pub, LEFT_JOINTS, joint_positions)
        self._publish_trajectory(self.right_traj_pub, RIGHT_JOINTS, joint_positions)

    def _publish_trajectory(self, publisher, joint_names: list[str], joint_positions: dict[str, float]):
        # 仅当该臂所有关节都存在于动作中才发布
        positions = [joint_positions.get(j) for j in joint_names]
        if any(p is None for p in positions):
            return

        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        sec = int(TRAJ_TIME_FROM_START)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((TRAJ_TIME_FROM_START - sec) * 1e9)
        traj.points.append(point)
        publisher.publish(traj)

    # ------------------------------------------------------------------
    # 清理
    # ------------------------------------------------------------------
    def destroy_node(self):
        for t in self.cam_threads:
            t.stop()
        for t in self.cam_threads:
            t.join(timeout=2.0)
        self.joint_pub.close()
        self.vr_pub.close()
        self.action_pull.close()
        self.context.term()
        super().destroy_node()


def main():
    rclpy.init()
    node = ZmqBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
