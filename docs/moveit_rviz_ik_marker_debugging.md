# MoveIt2 (Jazzy) RViz 交互式 IK Marker 调试记录

调试目标：在 RViz 的 MotionPlanning 面板里，通过拖动末端 6-DOF 交互 marker 给定末端目标位姿，让 MoveIt 做逆运动学（IK）规划。

机器人：`dual_arm_1kg`，左右各一条 7-DOF 冗余臂，规划组 `left_arm` / `right_arm`，tip link 为 `laxis7_link` / `raxis7_link`，根 link 为 `base_link`（SRDF 无 virtual joint，按 fixed 处理）。

相关文件：
- `dual_arm_moveit_config/config/dual_arm_1kg.srdf`
- `dual_arm_moveit_config/config/kinematics.yaml`
- `dual_arm_moveit_config/config/moveit.rviz`
- `dual_arm_moveit_config/launch/moveit.launch.py`（被 `dual_arm_bringup sim.launch.py` 调用）

---

## 现象演进

1. **完全看不到交互 marker**：即便把 `Interactive Marker Size` 调大也没有。
2. 修复后 marker 出现，但拖动后 `Plan & Execute` 报 `Goal constraints are already satisfied`（目标≈当前，没运动）。
3. 拖动 marker 时**橙色目标机械臂始终不跟随**，终端没有 IK 相关输出。
4. 换求解器（KDL → pick_ik）后**问题依旧**。

---

## 已确认并修复的问题

### 问题 1：规划组用 `<joint>` 列表定义，没有运动学链

原 SRDF：
```xml
<group name="left_arm">
  <joint name="laxis1_joint"/> ... <joint name="laxis7_joint"/>
</group>
```
RViz 显示可拖动 6-DOF marker 需要明确知道组的 tip link。纯 joint 列表无法可靠确定末端，marker 不会被创建。

**修复**：改为 chain 定义（MoveIt Setup Assistant 标准做法）。
```xml
<group name="left_arm">
  <chain base_link="base_link" tip_link="laxis7_link"/>
</group>
<group name="right_arm">
  <chain base_link="base_link" tip_link="raxis7_link"/>
</group>
```
已核对 URDF：`base_link → laxis1..7_link`、`base_link → raxis1..7_link` 是连续串联链，合法。

### 问题 2：`<end_effector>` 定义错误（退化定义）

原 SRDF：
```xml
<end_effector name="left_ee" parent_link="laxis7_link" group="left_arm"/>
```
`group` 指向了手臂组自身，形成循环/退化定义。RViz 的 RobotInteraction 会把 `left_arm` 当成夹爪 EE，导致**末端 6-DOF marker 根本不创建**。

**修复**：本机器人没有独立夹爪组，直接删除/注释掉 `<end_effector>`。带 IK 求解器的 chain 组会自动在 tip link 生成 marker。

> 经验：EE 的 `group` 必须指向独立的夹爪组，并用 `parent_group` 指向手臂组；没有夹爪时不要定义 `<end_effector>`，否则要么没 marker（退化定义），要么行为异常。

修复后效果：marker 正常出现（确认名为 `EE:goal_laxis7_link`，带 "Set goal state to → random/current/same as start" 菜单和 6-DOF 控件），拖动时 marker 视觉能跟随鼠标移动。

---

## 仍未解决的问题（核心症结）

**拖动 marker 后，橙色目标机械臂不跟随；marker 自由移动到目标位姿后不回弹，但目标状态没更新。**

### 已通过实验排除的因素

| 排查项 | 方法 | 结论 |
|---|---|---|
| 求解器是否加载 | `sim.launch.py` 日志 | move_group 与 rviz2 两进程都加载了 IK 插件（KDL 时 rviz 打印 `Joint weights`；pick_ik 时 rviz 触发 kdl_parser 建链） |
| 求解器能否解出 IK | `ros2 service call /compute_ik`，用 `laxis7_link` 当前位姿 | `error_code val=1`（SUCCESS），返回全 0（平凡解正确） |
| 能否解非平凡位姿 | 用用户实际拖到的位姿 `(0.377, -0.085, 0.654)` 调 `/compute_ik` | 成功，解出真实关节 `[0.559,0.355,0.797,0.907,0.626,1.320,0.524]` |
| 是否超时不够 | 同一位姿分别用 `timeout 0.05s` 和 `0.5s` | **两者都 `val=1` 成功**，pick_ik 解此位姿 <0.05s。超时不是原因 |
| marker 是否为 IK 末端 marker | `get_interactive_markers` 服务 | 是 `EE:goal_laxis7_link`，含 IK 菜单与 6-DOF 控件 |
| 目标状态显示链路 | `Joints` 标签页拖关节滑块 | 橙色目标臂**能**正常跟随（FK 路径正常） |
| 换求解器 | KDL → pick_ik（`ros-jazzy-pick-ik`） | 问题不变，说明与求解器无关 |
| RViz 配置 | `moveit.rviz` | `Fixed Frame=base_link`、`Query Goal State=true`、frame 均正确 |

### 关键结论

问题 **100% 在 RViz 进程内部"拖动反馈 → 调用 `setFromIK` → 更新 goal RobotState → 渲染橙色目标臂"这条链路**，与求解器、超时、SRDF marker 注册均无关。

- 同一个 pick_ik 插件、同一个可达位姿：`/compute_ik` 服务（move_group）能解；RViz 拖动到该位姿却不更新目标。
- `Joints` 标签页（FK，不走 IK）能更新橙色目标臂 → 目标状态渲染本身没问题。
- 所以是 RViz 的 `RobotInteraction` 拖动回调没有把 IK 结果应用到 goal 上。

### 唯一的具体错误线索

真实拖动时，move_group 打出（且仅此一条）：
```
[move_group...moveit.core.conversions]: Found empty JointState message
```
怀疑：拖动时 goal RobotState 变成了空（无关节值），故橙色臂不动，且该空状态传到 move_group 触发此告警。但根因尚未定位。

---

## 已尝试但被中断 / 待继续的方向

1. **抓 RViz 端 IK 调用日志**（最可能定位根因的手段）：
   ```bash
   ros2 service call /rviz2/set_logger_levels rcl_interfaces/srv/SetLoggerLevels \
     "{levels: [{name: 'rviz2.moveit', level: 10}, {name: 'rviz2.moveit.ros.robot_interaction', level: 10}]}"
   ```
   设为 DEBUG 后再手动拖动，观察 `robot_interaction` 是否真的调用了 `setFromIK`、返回成功还是失败、goal 是否被写空。
   - 若 logger service 不可用，改为重启 rviz2 时加 `--ros-args --log-level rviz2.moveit:=debug`。

2. **从 CLI 注入 feedback 模拟拖动**（已验证不可靠）：直接向 `.../feedback` 发 `InteractiveMarkerFeedback`（POSE_UPDATE / MENU_SELECT）marker 不响应，疑似缺少完整握手协议（MOUSE_DOWN 序列 / client 注册），此法**不可用于测试**。

3. 待查：`MoveGroup namespace changed: / -> .` 这条日志是否导致 RViz 端 kinematics 参数命名空间解析异常。

4. 待查：从全零（奇异）初始位形起步对拖动 IK 的影响——建议先用 `Joints` 把臂拨到非奇异姿态，再拖。

---

## 临时可用的替代方案（绕过拖动）

求解器本身完好，可直接用 `/compute_ik` 服务或脚本给定笛卡尔位姿求 IK，再用 JTC 执行；或用 `Joints` 标签页设定关节目标（FK）后 `Plan`。GUI 拖动 IK 的问题待用方向 1 的 DEBUG 日志定位后再修。

---

## 环境与命令备忘

```bash
# 启动仿真（mock 硬件）
source install/setup.bash
ros2 launch dual_arm_bringup sim.launch.py

# 取末端当前位姿
ros2 run tf2_ros tf2_echo base_link laxis7_link

# 验证 IK 求解器（move_group 侧）
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK \
  "{ik_request: {group_name: left_arm, pose_stamped: {header: {frame_id: base_link}, \
   pose: {position: {x: 0.21, y: -0.007, z: 0.457}, orientation: {x: 0.172, y: 0.696, z: -0.677, w: -0.168}}}, \
   timeout: {sec: 1}}}"

# 查看 RViz 创建的交互 marker
ros2 service call /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/get_interactive_markers \
  visualization_msgs/srv/GetInteractiveMarkers
```

pick_ik 安装：`sudo apt install ros-jazzy-pick-ik`
