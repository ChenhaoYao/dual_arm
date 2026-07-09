#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk
import threading
import math


class JointControlPanel:
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('joint_control_panel_gui')
        self.publisher_ = self.node.create_publisher(JointState, 'joint_states', 10)
        
        self.joint_names = [
            'fl_wheel', 'rl_wheel', 'rr_wheel', 'fr_wheel', 'mr_wheel', 'ml_wheel',
            'laxis1_joint', 'laxis2_joint', 'laxis3_joint', 'laxis4_joint',
            'laxis5_joint', 'laxis6_joint', 'laxis7_joint',
            'raxis1_joint', 'raxis2_joint', 'raxis3_joint', 'raxis4_joint',
            'raxis5_joint', 'raxis6_joint', 'raxis7_joint'
        ]
        
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        
        self.root = tk.Tk()
        self.root.title("关节控制面板 - dual_arm_description2")
        self.root.geometry("800x900")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        title_label = ttk.Label(main_frame, text="关节控制面板", font=("Arial", 16, "bold"))
        title_label.pack(pady=10)
        
        canvas = tk.Canvas(main_frame)
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        self.sliders = {}
        
        groups = [
            ("轮子 (Wheels)", ['fl_wheel', 'rl_wheel', 'rr_wheel', 'fr_wheel', 'mr_wheel', 'ml_wheel']),
            ("左臂 (Left Arm)", ['laxis1_joint', 'laxis2_joint', 'laxis3_joint', 'laxis4_joint',
                                 'laxis5_joint', 'laxis6_joint', 'laxis7_joint']),
            ("右臂 (Right Arm)", ['raxis1_joint', 'raxis2_joint', 'raxis3_joint', 'raxis4_joint',
                                  'raxis5_joint', 'raxis6_joint', 'raxis7_joint'])
        ]
        
        for group_name, joints in groups:
            group_frame = ttk.LabelFrame(scrollable_frame, text=group_name, padding="10")
            group_frame.pack(fill=tk.X, padx=5, pady=5)
            
            for joint_name in joints:
                self.create_slider(group_frame, joint_name)
        
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=10)
        
        reset_btn = ttk.Button(button_frame, text="重置所有关节", command=self.reset_all)
        reset_btn.pack(side=tk.LEFT, padx=5)
        
        left_home_btn = ttk.Button(button_frame, text="左臂归位", command=lambda: self.home_arm('left'))
        left_home_btn.pack(side=tk.LEFT, padx=5)
        
        right_home_btn = ttk.Button(button_frame, text="右臂归位", command=lambda: self.home_arm('right'))
        right_home_btn.pack(side=tk.LEFT, padx=5)
        
        self.running = True
        
    def create_slider(self, parent, joint_name):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=2)
        
        label = ttk.Label(frame, text=joint_name, width=20)
        label.pack(side=tk.LEFT)
        
        value_var = tk.DoubleVar(value=0.0)
        slider = ttk.Scale(frame, from_=-3.14, to=3.14, variable=value_var,
                          orient=tk.HORIZONTAL, length=400)
        slider.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        value_label = ttk.Label(frame, text="0.00", width=8)
        value_label.pack(side=tk.LEFT)
        
        deg_label = ttk.Label(frame, text="0°", width=8)
        deg_label.pack(side=tk.LEFT)
        
        self.sliders[joint_name] = {
            'slider': slider,
            'var': value_var,
            'label': value_label,
            'deg_label': deg_label
        }
        
        value_var.trace_add('write', lambda *args, jn=joint_name: self.on_slider_change(jn))
    
    def on_slider_change(self, joint_name):
        value = self.sliders[joint_name]['var'].get()
        self.joint_positions[joint_name] = value
        self.sliders[joint_name]['label'].config(text=f"{value:.2f}")
        deg_value = math.degrees(value)
        self.sliders[joint_name]['deg_label'].config(text=f"{deg_value:.1f}°")
    
    def reset_all(self):
        for joint_name in self.joint_names:
            self.sliders[joint_name]['var'].set(0.0)
            self.joint_positions[joint_name] = 0.0
    
    def home_arm(self, side):
        if side == 'left':
            prefix = 'laxis'
        else:
            prefix = 'raxis'
        
        for joint_name in self.joint_names:
            if joint_name.startswith(prefix):
                self.sliders[joint_name]['var'].set(0.0)
                self.joint_positions[joint_name] = 0.0
    
    def publish_joint_states(self):
        if not self.running:
            return
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [self.joint_positions[name] for name in self.joint_names]
        msg.velocity = []
        msg.effort = []
        self.publisher_.publish(msg)
        self.root.after(50, self.publish_joint_states)
    
    def on_closing(self):
        self.running = False
        self.root.destroy()
    
    def run(self):
        self.publish_joint_states()
        self.root.mainloop()


def main():
    panel = JointControlPanel()
    
    try:
        panel.run()
    except KeyboardInterrupt:
        pass
    finally:
        panel.running = False
        panel.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()