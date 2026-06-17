#!/usr/bin/env python3
"""
MoveIt 末端位姿控制（通过 /compute_ik + /move_action 接口）。
先调用 IK 求解器验证，再发送规划目标。

用法:
  ros2 run dual_arm_description move_to_pose.py --ros-args \
    -p x:=0.3 -p y:=0.0 -p z:=0.5 \
    -p roll:=0.0 -p pitch:=0.0 -p yaw:=0.0
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PositionConstraint,
    OrientationConstraint,
    Constraints,
    MoveItErrorCodes,
    RobotState,
)
from moveit_msgs.srv import GetPositionIK
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

# MoveIt 错误码映射表，用于将数字错误码转换为可读的字符串
# 来源：moveit_msgs/msg/MoveItErrorCodes.msg
MOVEIT_ERROR_CODES = {
    1: "SUCCESS",                                    # 成功
    0: "UNDEFINED",                                  # 未定义
    99999: "FAILURE",                                # 通用失败（未指明原因）
    -1: "PLANNING_FAILED",                           # 规划失败
    -2: "INVALID_MOTION_PLAN",                       # 无效的运动规划
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",  # 环境变化导致规划失效
    -4: "CONTROL_FAILED",                            # 控制执行失败
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",              # 无法获取传感器数据
    -6: "TIMED_OUT",                                 # 超时
    -7: "PREEMPTED",                                 # 被抢占
    -10: "START_STATE_IN_COLLISION",                 # 起始状态碰撞
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",    # 起始状态违反路径约束
    -12: "GOAL_IN_COLLISION",                        # 目标状态碰撞
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",           # 目标违反路径约束
    -14: "GOAL_CONSTRAINTS_VIOLATED",                # 目标约束不满足
    -15: "INVALID_GROUP_NAME",                       # 无效的规划组名
    -16: "INVALID_GOAL_CONSTRAINTS",                 # 无效的目标约束
    -17: "INVALID_ROBOT_STATE",                      # 无效的机器人状态
    -18: "INVALID_LINK_NAME",                        # 无效的 link 名称
    -19: "INVALID_OBJECT_NAME",                      # 无效的物体名称
    -21: "FRAME_TRANSFORM_FAILURE",                  # 坐标变换失败
    -22: "COLLISION_CHECKING_UNAVAILABLE",           # 碰撞检测不可用
    -23: "ROBOT_STATE_STALE",                        # 机器人状态过期
    -24: "SENSOR_INFO_STALE",                        # 传感器信息过期
    -25: "COMMUNICATION_FAILURE",                    # 通信失败
    -26: "START_STATE_INVALID",                      # 起始状态无效
    -27: "GOAL_STATE_INVALID",                       # 目标状态无效
    -28: "UNRECOGNIZED_GOAL_TYPE",                   # 无法识别的目标类型
    -29: "CRASH",                                    # 崩溃
    -30: "ABORT",                                    # 中止
    -31: "NO_IK_SOLUTION",                           # 无 IK 解
}


def rpy_to_quaternion(roll, pitch, yaw):
    """将欧拉角（Roll, Pitch, Yaw）转换为四元数（qx, qy, qz, qw）。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # qx
        cr * sp * cy + sr * cp * sy,  # qy
        cr * cp * sy - sr * sp * cy,  # qz
        cr * cp * cy + sr * sp * sy,  # qw
    )


def check_ik(node: Node, group: str, ee_link: str, target: PoseStamped) -> bool:
    """调用 /compute_ik 服务，直接查询 IK 求解器。

    这个函数绕过 MoveGroup 的规划管道，直接调用 IK 求解器服务，
    可以拿到 IK 求解器的原始返回值（成功/失败 + 关节角度解）。

    参数:
        node: ROS2 节点
        group: 规划组名称（如 "left_arm"）
        ee_link: 末端执行器 link 名称（如 "laxis7_link"）
        target: 目标位姿（PoseStamped）

    返回:
        True 表示 IK 求解成功，False 表示失败
    """
    # 创建 /compute_ik 服务客户端
    client = node.create_client(GetPositionIK, "/compute_ik")
    node.get_logger().info("Waiting for /compute_ik service...")
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("/compute_ik service not available")
        return False

    # 构建 IK 请求
    req = GetPositionIK.Request()
    req.ik_request.group_name = group           # 规划组名
    req.ik_request.ik_link_name = ee_link       # 末端执行器 link
    req.ik_request.pose_stamped = target        # 目标位姿
    req.ik_request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()  # IK 求解超时
    req.ik_request.avoid_collisions = False     # 不做碰撞检测（只验证运动学）

    # 异步调用服务
    node.get_logger().info("Calling IK solver...")
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if not future.done():
        node.get_logger().error("Timed out waiting for /compute_ik response")
        return False

    # 解析响应
    resp = future.result()
    code = resp.error_code.val
    name = MOVEIT_ERROR_CODES.get(code, f"UNKNOWN({code})")

    if code == MoveItErrorCodes.SUCCESS:
        # IK 成功，打印求解得到的关节角度
        node.get_logger().info(f"IK SUCCESS: {name}")
        node.get_logger().info(
            f"  Joint values: {list(resp.solution.joint_state.position)}"
        )
        return True
    else:
        # IK 失败，打印原始错误码
        node.get_logger().error(f"IK FAILED: error_code={code}: {name}")
        return False


def main():
    """主函数：先验证 IK，再发送规划目标。"""

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("move_to_pose")

    # ========== 声明 ROS 参数 ==========
    node.declare_parameter("group", "left_arm")      # 规划组名称
    node.declare_parameter("ee_link", "laxis7_link") # 末端执行器 link
    node.declare_parameter("x", 0.3)                 # 目标 X 坐标（米）
    node.declare_parameter("y", 0.0)                 # 目标 Y 坐标（米）
    node.declare_parameter("z", 0.5)                 # 目标 Z 坐标（米）
    node.declare_parameter("roll", 0.0)              # 滚转角（弧度）
    node.declare_parameter("pitch", 0.0)             # 俯仰角（弧度）
    node.declare_parameter("yaw", 0.0)               # 偏航角（弧度）

    # ========== 获取参数值 ==========
    group = node.get_parameter("group").value
    ee_link = node.get_parameter("ee_link").value
    x = node.get_parameter("x").value
    y = node.get_parameter("y").value
    z = node.get_parameter("z").value
    roll = node.get_parameter("roll").value
    pitch = node.get_parameter("pitch").value
    yaw = node.get_parameter("yaw").value

    # 欧拉角转四元数
    qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
    node.get_logger().info(
        f"Target: xyz=({x:.3f}, {y:.3f}, {z:.3f}) "
        f"quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
    )

    # ========== 构建目标位姿 ==========
    target = PoseStamped()
    target.header.frame_id = "base_link"  # 相对于 base_link 坐标系 # TODO 是否可以不用base link，抬高一点
    target.header.stamp = node.get_clock().now().to_msg()
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = z
    target.pose.orientation.x = qx
    target.pose.orientation.y = qy
    target.pose.orientation.z = qz
    target.pose.orientation.w = qw

    # ========== 第一步：直接调用 IK 求解器验证 ==========
    # 绕过 MoveGroup 规划管道，直接查询 IK 求解器是否有解
    # 失败时会打印 IK 求解器的原始错误码，便于排查
    ik_ok = check_ik(node, group, ee_link, target)
    if not ik_ok:
        node.get_logger().error("IK check failed, aborting.")
        rclpy.shutdown()
        return

    # ========== 第二步：发送规划目标 ==========
    # IK 验证通过后，才发送完整的规划+执行请求

    # 等待 /joint_states 话题可用（确认机器人状态正常）
    js_sub = node.create_subscription(JointState, "/joint_states", lambda msg: None, 10)
    node.get_logger().info("Waiting for /joint_states...")
    rclpy.spin_until_future_complete(node, rclpy.task.Future(), timeout_sec=1.0)
    node.destroy_subscription(js_sub)

    # 连接 MoveGroup Action 服务器
    client = ActionClient(node, MoveGroup, "/move_action")
    node.get_logger().info("Waiting for /move_action server...")
    client.wait_for_server()
    node.get_logger().info("Connected")

    # ========== 设置位置约束 ==========
    # 定义末端执行器必须到达的目标区域（球体）
    pc = PositionConstraint()
    pc.header = target.header
    pc.link_name = ee_link
    pc.target_point_offset = Vector3()  # 零偏移
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.05]  # 球体半径 0.05 米
    pc.constraint_region.primitives = [sphere]
    pc.constraint_region.primitive_poses = [target.pose]
    pc.weight = 1.0  # 权重 1.0，必须满足

    # ========== 设置姿态约束 ==========
    # 宽松的姿态约束，允许大范围的姿态偏差
    oc = OrientationConstraint()
    oc.header = target.header
    oc.link_name = ee_link
    oc.orientation = target.pose.orientation
    oc.absolute_x_axis_tolerance = 3.14  # 允许绕 X 轴旋转 ±180°
    oc.absolute_y_axis_tolerance = 3.14  # 允许绕 Y 轴旋转 ±180°
    oc.absolute_z_axis_tolerance = 3.14  # 允许绕 Z 轴旋转 ±180°
    oc.weight = 0.5  # 权重 0.5，低于位置约束

    # ========== 组合约束 ==========
    gc = Constraints()
    gc.position_constraints = [pc]
    gc.orientation_constraints = [oc]

    # ========== 构建运动规划请求 ==========
    req = MotionPlanRequest()
    req.group_name = group
    req.goal_constraints = [gc]
    req.num_planning_attempts = 20      # 最大规划尝试次数
    req.allowed_planning_time = 10.0    # 允许的规划时间（秒）
    req.max_velocity_scaling_factor = 0.1    # 速度缩放因子（10%）
    req.max_acceleration_scaling_factor = 0.1  # 加速度缩放因子（10%）

    # ========== 发送目标到 MoveGroup ==========
    goal = MoveGroup.Goal()
    goal.request = req
    goal.planning_options.plan_only = False  # False=规划后执行，True=仅规划

    node.get_logger().info("Sending goal...")
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done():
        node.get_logger().error("Timed out waiting for goal acceptance")
        rclpy.shutdown()
        return

    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error("Goal REJECTED by move_group")
        rclpy.shutdown()
        return

    # ========== 等待规划和执行结果 ==========
    node.get_logger().info("Goal accepted, planning...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
    if not result_future.done():
        node.get_logger().error("Timed out waiting for result (30s)")
        rclpy.shutdown()
        return

    # ========== 处理结果 ==========
    result = result_future.result()
    error_code = result.result.error_code.val
    error_name = MOVEIT_ERROR_CODES.get(error_code, f"UNKNOWN({error_code})")

    if error_code == 1:
        node.get_logger().info(f"SUCCESS! (error_code={error_code}: {error_name})")
    else:
        node.get_logger().error(f"FAILED: error_code={error_code}: {error_name}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
