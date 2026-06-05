# 位置开环 → 速度闭环 改造 Todo List

## 需要修改的文件

| # | 文件 | 修改内容 | 状态 |
|---|------|----------|------|
| 1 | `ros2_controllers.yaml` | `command_interfaces: position` → `velocity` | 待做 |
| 2 | `ros2_controllers.yaml` | 添加 PID 参数 `gains.<joint>.p/i/d` | 待做 |
| 3 | `moveit_controllers.yaml` | 确认无需修改（FollowJointTrajectory 通用） | 待做 |

## 已满足的条件

| # | 文件 | 状态 |
|---|------|------|
| 4 | URDF - velocity command interface | ✅ 已定义 |
| 5 | joint_limits.yaml - max_velocity | ✅ 已定义 (2.0 rad/s) |
| 6 | soem_bridge - CSV 模式 | ✅ 已使用 velocity 指令 |

## 后续工作

| # | 任务 | 状态 |
|---|------|------|
| 7 | 编译验证 | 待做 |
| 8 | 实物测试 - PID 调参 | 待做 |

## 修改说明

### ros2_controllers.yaml

当前配置（开环）：
```yaml
command_interfaces:
  - position
```

目标配置（闭环）：
```yaml
command_interfaces:
  - velocity
gains:
  laxis1_joint: {p: 10.0, i: 0.0, d: 0.0}
  laxis2_joint: {p: 10.0, i: 0.0, d: 0.0}
  # ... 其余关节
```

### 数据流变化

```
开环：reference.pos → 直接输出到 command_interface.position
闭环：reference.pos → PID(pos_error) → 输出到 command_interface.velocity
```
