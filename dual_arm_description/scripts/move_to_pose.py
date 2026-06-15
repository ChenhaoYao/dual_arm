#!/usr/bin/env python3
"""
MoveIt 末端位姿控制（通过 /move_action 接口）。
用法:
  ros2 run dual_arm_description move_to_pose.py --ros-args \
    -p x:=0.3 -p y:=0.0 -p z:=0.5 \
    -p roll:=0.0 -p pitch:=0.0 -p yaw:=0.0
"""

# 导入标准库
import math  # 数学函数，用于三角函数计算
import sys   # 系统参数，用于获取命令行参数

# 导入ROS 2 Python客户端库
import rclpy
from rclpy.action import ActionClient  # ROS 2 Action客户端，用于与MoveIt action server通信

# 导入消息类型
from geometry_msgs.msg import PoseStamped, Vector3  # 几何消息：带坐标系的位姿、三维向量
from moveit_msgs.action import MoveGroup  # MoveIt动作类型：MoveGroup动作
from moveit_msgs.msg import (           # MoveIt消息类型
    MotionPlanRequest,       # 运动规划请求
    PositionConstraint,      # 位置约束
    OrientationConstraint,   # 姿态约束
    Constraints,             # 约束集合
)
from shape_msgs.msg import SolidPrimitive  # 形状消息：用于定义约束区域（球体）
from sensor_msgs.msg import JointState     # 传感器消息：关节状态


def rpy_to_quaternion(roll, pitch, yaw):
    """将欧拉角（Roll, Pitch, Yaw）转换为四元数（qx, qy, qz, qw）。
    
    使用ZYX欧拉角顺序（先偏航，再俯仰，最后滚转）。
    四元数用于表示旋转，避免万向锁问题。
    
    参数:
        roll:  绕X轴的旋转角度（弧度）
        pitch: 绕Y轴的旋转角度（弧度）
        yaw:   绕Z轴的旋转角度（弧度）
    
    返回:
        (qx, qy, qz, qw) 四元数元组
    """
    # 计算半角的三角函数值，用于四元数公式
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    
    # 四元数计算公式（ZYX顺序）
    return (
        sr * cp * cy - cr * sp * sy,  # qx
        cr * sp * cy + sr * cp * sy,  # qy
        cr * cp * sy - sr * sp * cy,  # qz
        cr * cp * cy + sr * sp * sy,  # qw
    )


def main():
    """主函数：初始化ROS 2节点，设置MoveIt目标位姿，并发送规划请求。"""
    
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("move_to_pose")

    # ========== 声明ROS参数 ==========
    # 这些参数可以通过命令行或配置文件设置
    node.declare_parameter("group", "left_arm")      # 运动组名称（左臂/右臂）
    node.declare_parameter("ee_link", "laxis7_link") # 末端执行器链接名称
    node.declare_parameter("x", 0.3)                 # 目标X坐标（米）
    node.declare_parameter("y", 0.0)                 # 目标Y坐标（米）
    node.declare_parameter("z", 0.5)                 # 目标Z坐标（米）
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

    # 将欧拉角转换为四元数，用于ROS位姿消息
    qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

    # 打印目标位姿信息到日志
    node.get_logger().info(
        f"Target: xyz=({x:.3f}, {y:.3f}, {z:.3f}) "
        f"quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
    )

    # ========== 等待/joint_states话题 ==========
    # 确认move_group节点的状态监控正常工作
    # 创建一个临时订阅者，监听/joint_states话题
    js_sub = node.create_subscription(JointState, "/joint_states", lambda msg: None, 10)
    node.get_logger().info("Waiting for /joint_states...")
    
    # 等待1秒，让订阅者接收至少一条消息
    rclpy.spin_until_future_complete(node, rclpy.task.Future(), timeout_sec=1.0)
    
    # 销毁临时订阅者，因为我们只需要确认话题存在
    node.destroy_subscription(js_sub)

    # ========== 连接MoveIt Action Server ==========
    # 创建MoveGroup动作客户端，连接到/move_action服务器
    client = ActionClient(node, MoveGroup, "/move_action")
    node.get_logger().info("Waiting for /move_action server...")
    
    # 阻塞等待，直到服务器可用
    client.wait_for_server()
    node.get_logger().info("Connected")

    # ========== 构建目标位姿 ==========
    # 创建带时间戳和坐标系的位姿消息
    target = PoseStamped()
    target.header.frame_id = "base_link"  # 位姿在base_link坐标系中定义
    target.header.stamp = node.get_clock().now().to_msg()  # 设置当前时间戳
    
    # 设置目标位置（笛卡尔坐标）
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.position.z = z
    
    # 设置目标姿态（四元数）
    target.pose.orientation.x = qx
    target.pose.orientation.y = qy
    target.pose.orientation.z = qz
    target.pose.orientation.w = qw

    # ========== 设置位置约束 ==========
    # 位置约束定义末端执行器必须到达的目标区域
    pc = PositionConstraint()
    pc.header = target.header  # 使用相同的坐标系和时间戳
    pc.link_name = ee_link     # 被约束的链接名称
    
    # 目标点偏移（这里设为零偏移）
    pc.target_point_offset = Vector3()
    
    # 定义约束区域：半径为0.05米的球体
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE  # 球体类型
    sphere.dimensions = [0.05]           # 球体半径（米）
    
    # 将球体添加到约束区域
    pc.constraint_region.primitives = [sphere]
    pc.constraint_region.primitive_poses = [target.pose]  # 球体中心在目标位姿
    
    # 约束权重（1.0为必须满足）
    pc.weight = 1.0

    # ========== 设置姿态约束 ==========
    # 姿态约束定义末端执行器的朝向要求
    # 这里设置为宽松约束，允许一定范围内的姿态偏差
    oc = OrientationConstraint()
    oc.header = target.header
    oc.link_name = ee_link
    oc.orientation = target.pose.orientation  # 目标姿态
    
    # 姿态容差：允许绕各轴旋转π弧度（180度），非常宽松
    oc.absolute_x_axis_tolerance = 3.14
    oc.absolute_y_axis_tolerance = 3.14
    oc.absolute_z_axis_tolerance = 3.14
    
    # 姿态约束权重（0.5，低于位置约束）
    oc.weight = 0.5

    # ========== 组合约束 ==========
    # 将位置约束和姿态约束组合成一个约束集合
    gc = Constraints()
    gc.position_constraints = [pc]      # 位置约束列表
    gc.orientation_constraints = [oc]   # 姿态约束列表

    # ========== 构建运动规划请求 ==========
    req = MotionPlanRequest()
    req.group_name = group              # 运动组名称
    req.goal_constraints = [gc]         # 目标约束
    req.num_planning_attempts = 20      # 最大规划尝试次数
    req.allowed_planning_time = 10.0    # 允许的规划时间（秒）
    
    # 速度和加速度缩放因子（0.1表示最大速度/加速度的10%）
    # 降低这些值可以使运动更平稳，但会变慢
    req.max_velocity_scaling_factor = 0.1
    req.max_acceleration_scaling_factor = 0.1

    # ========== 发送目标到MoveIt ==========
    # 创建MoveGroup动作目标
    goal = MoveGroup.Goal()
    goal.request = req                  # 设置规划请求
    goal.planning_options.plan_only = False  # False表示规划后执行，True表示仅规划不执行

    # 异步发送目标
    node.get_logger().info("Sending goal...")
    future = client.send_goal_async(goal)
    
    # 等待目标被接受（超时10秒）
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done():
        node.get_logger().error("Timed out waiting for goal acceptance")
        rclpy.shutdown()
        return
    
    # 获取目标句柄（用于后续获取结果）
    goal_handle = future.result()

    # 检查目标是否被接受
    if not goal_handle.accepted:
        node.get_logger().error("Goal REJECTED by move_group")
        rclpy.shutdown()
        return

    # ========== 等待规划和执行结果 ==========
    node.get_logger().info("Goal accepted, planning...")
    
    # 异步获取结果（超时30秒）
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
    if not result_future.done():
        node.get_logger().error("Timed out waiting for result (30s)")
        rclpy.shutdown()
        return

    # ========== 处理结果 ==========
    result = result_future.result()
    error_code = result.result.error_code.val
    
    # MoveIt错误代码：1表示成功
    if error_code == 1:
        node.get_logger().info("SUCCESS!")
    else:
        node.get_logger().error(f"FAILED: error_code={error_code}")

    # 关闭ROS 2客户端
    rclpy.shutdown()


if __name__ == "__main__":
    main()
