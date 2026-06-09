#!/usr/bin/env python3
"""
把"关节绕自身轴旋转 q 角"的姿态烘焙进 URDF origin rpy，从而平移关节零点：
使新的关节角 0 对应旧的关节角 q。

原理:
    URDF origin 旋转矩阵 (固定顺序):  R_origin = Rz(yaw) * Ry(pitch) * Rx(roll)
    关节绕自身轴 a 转 q:              R_child(q) = R_origin * Rot(a, q)
    令 R_origin' = R_origin * Rot(a, q)，再反解出新的 (roll, pitch, yaw)。

使用方法: 在下方 "输入参数" 处填写原 rpy、旋转轴、偏移角度，然后运行:
    python3 utils/bake_joint_offset.py
"""
import math

import numpy as np

# ============== 输入参数 (在这里修改) ==============
RPY = [-2.2242, 0.24666, 1.8795]   # 原 origin rpy (弧度)
AXIS = [0, 1, 0]                   # 关节旋转轴 xyz
ANGLE_DEG = 164                   # 绕关节轴偏移角度 (角度制)
# =================================================


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_matrix(roll, pitch, yaw):
    """URDF 约定: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_rpy(R):
    """从 R = Rz(yaw)*Ry(pitch)*Rx(roll) 反解 (roll, pitch, yaw)."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-9:  # 非奇异
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # 万向锁 (pitch = +-90 deg)
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def axis_angle_to_matrix(axis, angle):
    """罗德里格斯公式: 绕单位轴 axis 转 angle 弧度."""
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        raise ValueError("axis 不能为零向量")
    axis = axis / n
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def main():
    angle = math.radians(ANGLE_DEG)   # 角度制 -> 弧度
    R_origin = rpy_to_matrix(*RPY)
    R_new = R_origin @ axis_angle_to_matrix(AXIS, angle)
    new_rpy = matrix_to_rpy(R_new)

    print(f"关节轴 : {AXIS}")
    print(f"偏移角 : {ANGLE_DEG:.4f} deg ({angle:.6f} rad)")
    print(f"原 rpy : {RPY[0]:.6f} {RPY[1]:.6f} {RPY[2]:.6f}")
    print(f"新 rpy : {new_rpy[0]:.6f} {new_rpy[1]:.6f} {new_rpy[2]:.6f}")
    print()
    print("可直接替换 origin:")
    print(f'    rpy="{new_rpy[0]:.6f} {new_rpy[1]:.6f} {new_rpy[2]:.6f}"')


if __name__ == "__main__":
    main()
