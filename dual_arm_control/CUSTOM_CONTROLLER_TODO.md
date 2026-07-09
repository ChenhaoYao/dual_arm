# 自定义控制器开发计划

## 目标

替换 ros2_control 的 JTC，自己实现轨迹接收、插值、实时控制。

## 架构

```
MoveIt → 你的节点 (FollowJointTrajectory action) → buffer → 实时控制 → 电机
         ↑                                                              ↑
    实现 action server                                          直接发送 EtherCAT
```

## 任务清单

### 阶段1：Action Server 基础框架

- [ ] 创建 `custom_controller` 包
- [ ] 实现 `FollowJointTrajectory` action server
- [ ] 接收轨迹并存入 buffer
- [ ] 打印接收到的轨迹信息（调试用）

### 阶段2：轨迹插值

- [ ] 实现线性插值（位置）
- [ ] 实现三次样条插值（平滑）
- [ ] 处理多关节同步
- [ ] 时间管理（1ms 周期）

### 阶段3：实时控制

- [ ] 实现 RT 线程（SCHED_FIFO）
- [ ] 从 buffer 取当前时刻的点
- [ ] 发送给 EtherCAT 电机
- [ ] 处理轨迹结束

### 阶段4：安全机制

- [ ] 速度限制（正反转）
- [ ] 加速度限制
- [ ] 位置软限位
- [ ] 启动冲击保护（enable 后先归位）

### 阶段5：反馈与监控

- [ ] 读取编码器位置
- [ ] 发布 `/joint_states`
- [ ] Action result 反馈
- [ ] 状态监控

## 关键代码参考

### Action Server 模板

```cpp
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using FollowJointTrajectory = control_msgs::action::follow_joint_trajectory;

class CustomController : public rclcpp::Node
{
public:
    CustomController() : Node("custom_controller")
    {
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "/left_arm_controller/follow_joint_trajectory",
            std::bind(&CustomController::handle_goal, this, _1, _2),
            std::bind(&CustomController::handle_cancel, this, _1),
            std::bind(&CustomController::handle_accepted, this, _1)
        );
    }

private:
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    
    // 轨迹 buffer
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_buffer_;
    rclcpp::Time trajectory_start_time_;
    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        trajectory_buffer_ = goal->trajectory.points;
        trajectory_start_time_ = this->now();
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
    {
        trajectory_buffer_.clear();
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
    {
        // 在新线程中执行
        std::thread{std::bind(&CustomController::execute, this, goal_handle)}.detach();
    }
    
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
    {
        // 轨迹执行逻辑
    }
};
```

### 实时控制循环

```cpp
// 1ms 定时器
timer_ = create_wall_timer(std::chrono::milliseconds(1), [this]() {
    if (trajectory_buffer_.empty()) return;
    
    auto elapsed = (this->now() - trajectory_start_time_).seconds();
    
    // 找到当前时刻对应的轨迹点
    auto point = interpolate(elapsed);
    
    // 速度限制
    for (auto& vel : point.velocities) {
        vel = std::clamp(vel, -max_velocity_, max_velocity_);
    }
    
    // 发送给电机
    send_to_motor(point.velocities);
});
```

## 参考资源

- [ros2_control 官方文档](https://control.ros.org/)
- [JTC 源码](https://github.com/ros-controls/ros2_controllers/tree/master/joint_trajectory_controller)
- [FollowJointTrajectory action 定义](https://github.com/ros-controls/control_msgs/tree/master/control_msgs/action)
