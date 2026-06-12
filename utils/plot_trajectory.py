#!/usr/bin/env python3
"""
轨迹日志分析工具
用法：python3 plot_trajectory.py [csv文件路径] [--joint 关节名] [--all]
"""

import sys
import os
import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# 默认CSV路径（log目录下最新的文件）
DEFAULT_LOG_DIR = "/home/dell/dual_arm/dual_arm_soem_bridge/log"

# 所有关节名
ALL_JOINTS = [
    "laxis1", "laxis2", "laxis3", "laxis4", "laxis5", "laxis6", "laxis7",
    "raxis1", "raxis2", "raxis3", "raxis4", "raxis5", "raxis6", "raxis7"
]


def get_latest_csv():
    """获取log目录下最新的CSV文件"""
    if not os.path.exists(DEFAULT_LOG_DIR):
        print(f"Error: Log directory not found: {DEFAULT_LOG_DIR}")
        sys.exit(1)
    
    csv_files = [f for f in os.listdir(DEFAULT_LOG_DIR) if f.endswith('.csv')]
    if not csv_files:
        print(f"Error: No CSV files found in {DEFAULT_LOG_DIR}")
        sys.exit(1)
    
    # 按文件名排序（时间戳格式），取最新的
    csv_files.sort(reverse=True)
    latest = os.path.join(DEFAULT_LOG_DIR, csv_files[0])
    print(f"Using latest file: {latest}")
    return latest


def load_data(csv_path):
    """加载CSV文件"""
    if csv_path is None:
        csv_path = get_latest_csv()
    
    if not os.path.exists(csv_path):
        print(f"Error: File not found: {csv_path}")
        sys.exit(1)
    
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} rows from {csv_path}")
    print(f"Columns: {list(df.columns)}")
    
    # 自动检测关节名（从列名中提取）
    for col in df.columns:
        if col.endswith('_ref_pos'):
            joint_name = col.replace('_ref_pos', '')
            print(f"Detected joint: {joint_name}")
            break
    
    return df


def get_joint_columns(df, joint_name):
    """获取指定关节的列名"""
    ref_col = f"{joint_name}_ref_pos"
    fb_col = f"{joint_name}_fb_pos"
    vel_col = f"{joint_name}_output_vel"
    
    missing = []
    for col in [ref_col, fb_col, vel_col]:
        if col not in df.columns:
            missing.append(col)
    
    if missing:
        print(f"Warning: Missing columns for {joint_name}: {missing}")
        return None, None, None
    
    return ref_col, fb_col, vel_col


def plot_single_joint(df, joint_name, ax=None):
    """绘制单个关节的轨迹"""
    ref_col, fb_col, vel_col = get_joint_columns(df, joint_name)
    if ref_col is None:
        return
    
    if ax is None:
        fig, ax = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # 归一化时间到秒
    t = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) / 1e9
    
    # 位置图
    ax[0].plot(t, df[ref_col], 'b-', label='Reference', linewidth=1.5)
    ax[0].plot(t, df[fb_col], 'r--', label='Feedback', linewidth=1.0)
    ax[0].set_ylabel('Position (rad)')
    ax[0].set_title(f'{joint_name} - Position Tracking')
    ax[0].legend()
    ax[0].grid(True, alpha=0.3)
    
    # 速度图
    ax[1].plot(t, df[vel_col], 'g-', label='Output Velocity', linewidth=1.0)
    ax[1].set_xlabel('Time (s)')
    ax[1].set_ylabel('Velocity (rad/s)')
    ax[1].set_title(f'{joint_name} - PID Output Velocity')
    ax[1].legend()
    ax[1].grid(True, alpha=0.3)
    
    # 打印统计信息
    vel_data = df[vel_col]
    print(f"\n{joint_name} velocity statistics:")
    print(f"  Max:     {vel_data.max():.4f} rad/s")
    print(f"  Min:     {vel_data.min():.4f} rad/s")
    print(f"  Mean:    {vel_data.mean():.4f} rad/s")
    print(f"  Std:     {vel_data.std():.4f} rad/s")
    
    return ax


def plot_all_joints_velocity(df, joints=None):
    """绘制所有关节的速度对比图"""
    if joints is None:
        joints = ALL_JOINTS
    
    # 过滤存在的关节
    valid_joints = []
    for j in joints:
        vel_col = f"{j}_output_vel"
        if vel_col in df.columns:
            valid_joints.append(j)
    
    if not valid_joints:
        print("No valid joints found")
        return
    
    t = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) / 1e9
    
    fig, axes = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
    
    # 左臂
    left_joints = [j for j in valid_joints if j.startswith('laxis')]
    for j in left_joints:
        axes[0].plot(t, df[f"{j}_output_vel"], label=j, linewidth=1.0)
    axes[0].set_ylabel('Velocity (rad/s)')
    axes[0].set_title('Left Arm - All Joints Velocity')
    axes[0].legend(loc='best', fontsize=8)
    axes[0].grid(True, alpha=0.3)
    
    # 右臂
    right_joints = [j for j in valid_joints if j.startswith('raxis')]
    for j in right_joints:
        axes[1].plot(t, df[f"{j}_output_vel"], label=j, linewidth=1.0)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Velocity (rad/s)')
    axes[1].set_title('Right Arm - All Joints Velocity')
    axes[1].legend(loc='best', fontsize=8)
    axes[1].grid(True, alpha=0.3)
    
    # 打印所有关节的最大速度
    print("\n=== Maximum Velocity Summary ===")
    print(f"{'Joint':<12} {'Max Vel (rad/s)':<18} {'Max Vel (deg/s)':<18}")
    print("-" * 48)
    for j in valid_joints:
        vel_col = f"{j}_output_vel"
        max_vel = df[vel_col].abs().max()
        max_vel_deg = np.degrees(max_vel)
        print(f"{j:<12} {max_vel:<18.4f} {max_vel_deg:<18.4f}")


def plot_position_tracking_all(df, joints=None):
    """绘制所有关节的位置跟踪图"""
    if joints is None:
        joints = ALL_JOINTS
    
    valid_joints = []
    for j in joints:
        ref_col = f"{j}_ref_pos"
        if ref_col in df.columns:
            valid_joints.append(j)
    
    if not valid_joints:
        print("No valid joints found")
        return
    
    t = (df['timestamp_ns'] - df['timestamp_ns'].iloc[0]) / 1e9
    
    n_joints = len(valid_joints)
    fig, axes = plt.subplots(n_joints, 1, figsize=(14, 3 * n_joints), sharex=True)
    
    if n_joints == 1:
        axes = [axes]
    
    for i, j in enumerate(valid_joints):
        ref_col = f"{j}_ref_pos"
        fb_col = f"{j}_fb_pos"
        
        axes[i].plot(t, df[ref_col], 'b-', label='Ref', linewidth=1.5)
        if fb_col in df.columns:
            axes[i].plot(t, df[fb_col], 'r--', label='Fb', linewidth=1.0)
        axes[i].set_ylabel('Pos (rad)')
        axes[i].set_title(j)
        axes[i].legend(loc='upper right', fontsize=8)
        axes[i].grid(True, alpha=0.3)
    
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Position Tracking - All Joints', fontsize=14)
    plt.tight_layout()


def main():
    parser = argparse.ArgumentParser(description='Plot trajectory log')
    parser.add_argument('csv', nargs='?', default=None, help='CSV file path (default: latest in log dir)')
    parser.add_argument('--save', '-s', type=str, help='Save figure to file')
    parser.add_argument('--show', action='store_true', default=True, help='Show figure')
    parser.add_argument('--list', '-l', action='store_true', help='List all CSV files in log dir')
    
    args = parser.parse_args()
    
    if args.list:
        # 列出所有CSV文件
        if os.path.exists(DEFAULT_LOG_DIR):
            csv_files = sorted([f for f in os.listdir(DEFAULT_LOG_DIR) if f.endswith('.csv')])
            print(f"\nCSV files in {DEFAULT_LOG_DIR}:")
            for f in csv_files:
                print(f"  {f}")
        else:
            print(f"Log directory not found: {DEFAULT_LOG_DIR}")
        return
    
    df = load_data(args.csv)
    
    # 自动检测关节名
    joint_name = None
    for col in df.columns:
        if col.endswith('_ref_pos'):
            joint_name = col.replace('_ref_pos', '')
            break
    
    if joint_name is None:
        print("Error: No joint data found in CSV")
        sys.exit(1)
    
    print(f"Plotting joint: {joint_name}")
    
    # 绘制图表
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    plot_single_joint(df, joint_name, axes)
    fig.suptitle(f'Trajectory Analysis - {joint_name}', fontsize=14)
    plt.tight_layout()
    
    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches='tight')
        print(f"Figure saved to: {args.save}")
    
    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
