#!/usr/bin/env python3
"""Move both real arms from a known start pose to a mirrored ready pose.

The node is intentionally a one-shot startup coordinator. It waits for fresh
encoder feedback and active JointTrajectoryControllers, opens the SOEM command
gate, then moves both arms together through FollowJointTrajectory.
Any failure after startup begins requests the SOEM software stop and exits with
a non-zero status so real.launch.py will not start Servo/VR.
"""

import math
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


class ReadyPoseError(RuntimeError):
    """Raised when a safety prerequisite or trajectory execution fails."""


class MoveToReady(Node):
    """One-shot coordinator for the real-hardware startup ready motion."""

    def __init__(self) -> None:
        super().__init__("move_to_ready_node")

        self._declare_parameters()
        self.left_names = list(self.get_parameter("left_joint_names").value)
        self.right_names = list(self.get_parameter("right_joint_names").value)
        self.left_target = self._float_list("left_ready_positions")
        self.right_target = self._float_list("right_ready_positions")
        self.trajectory_duration = float(
            self.get_parameter("trajectory_duration_sec").value
        )
        self.action_timeout_margin = float(
            self.get_parameter("action_timeout_margin_sec").value
        )
        self.allow_unknown_start = bool(
            self.get_parameter("allow_unknown_start").value
        )
        self.zero_tolerance = float(self.get_parameter("zero_tolerance_rad").value)
        self.ready_tolerance = float(
            self.get_parameter("ready_tolerance_rad").value
        )
        self.absolute_position_limit = float(
            self.get_parameter("absolute_position_limit_rad").value
        )
        self.required_stable_samples = int(
            self.get_parameter("required_stable_samples").value
        )
        self.joint_state_timeout = float(
            self.get_parameter("joint_state_timeout_sec").value
        )
        self.startup_timeout = float(self.get_parameter("startup_timeout_sec").value)
        self.final_settle_timeout = float(
            self.get_parameter("final_settle_timeout_sec").value
        )
        self.manage_command_gate = bool(
            self.get_parameter("manage_command_gate").value
        )

        self._validate_parameters()
        self.required_names = set(self.left_names + self.right_names)
        self.positions: Dict[str, float] = {}
        self.last_joint_state_time: Optional[float] = None
        self.stable_samples = 0
        self.command_gate_opened = False
        self.active_goal_handles = {}

        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.controller_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.enable_client = self.create_client(
            SetBool, "/soem_bridge_node/enable"
        )
        self.stop_client = self.create_client(Trigger, "/soem_bridge_node/stop")
        self.left_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/left_arm_controller/follow_joint_trajectory",
        )
        self.right_action = ActionClient(
            self,
            FollowJointTrajectory,
            "/right_arm_controller/follow_joint_trajectory",
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "left_joint_names", [f"laxis{index}_joint" for index in range(1, 8)]
        )
        self.declare_parameter(
            "right_joint_names", [f"raxis{index}_joint" for index in range(1, 8)]
        )
        self.declare_parameter(
            "left_ready_positions", [0.3, 0.0, 1.2, 1.2, 0.0, 0.0, 0.0]
        )
        self.declare_parameter(
            "right_ready_positions", [-0.3, 0.0, -1.2, -1.2, 0.0, 0.0, 0.0]
        )
        self.declare_parameter("trajectory_duration_sec", 12.0)
        self.declare_parameter("action_timeout_margin_sec", 10.0)
        self.declare_parameter("allow_unknown_start", False)
        self.declare_parameter("zero_tolerance_rad", 0.15)
        self.declare_parameter("ready_tolerance_rad", 0.08)
        self.declare_parameter("absolute_position_limit_rad", 3.2)
        self.declare_parameter("required_stable_samples", 10)
        self.declare_parameter("joint_state_timeout_sec", 0.5)
        self.declare_parameter("startup_timeout_sec", 60.0)
        self.declare_parameter("final_settle_timeout_sec", 5.0)
        self.declare_parameter("manage_command_gate", True)

    def _float_list(self, name: str) -> List[float]:
        return [float(value) for value in self.get_parameter(name).value]

    def _validate_parameters(self) -> None:
        arms = (
            ("left", self.left_names, self.left_target),
            ("right", self.right_names, self.right_target),
        )
        for arm, names, target in arms:
            if len(names) != 7 or len(target) != 7:
                raise ReadyPoseError(f"{arm} ready pose must contain exactly 7 joints")
            if len(set(names)) != len(names):
                raise ReadyPoseError(f"{arm} joint names contain duplicates")
            if not all(math.isfinite(value) for value in target):
                raise ReadyPoseError(f"{arm} ready pose contains a non-finite value")
            if any(abs(value) > self.absolute_position_limit for value in target):
                raise ReadyPoseError(
                    f"{arm} ready pose exceeds +/-{self.absolute_position_limit:.3f} rad"
                )
        if set(self.left_names) & set(self.right_names):
            raise ReadyPoseError("left and right joint lists overlap")
        if self.trajectory_duration <= 0.0:
            raise ReadyPoseError("trajectory_duration_sec must be positive")
        if self.action_timeout_margin <= 0.0:
            raise ReadyPoseError("action_timeout_margin_sec must be positive")
        if self.required_stable_samples <= 0:
            raise ReadyPoseError("required_stable_samples must be positive")
        if self.joint_state_timeout <= 0.0 or self.startup_timeout <= 0.0:
            raise ReadyPoseError("state and startup timeouts must be positive")

    def _on_joint_state(self, msg: JointState) -> None:
        """
        编码器反馈帧检查：
        - name 和 position 必须一一对应，并且包含左右臂全部 14 个关节。
        - 14 个位置必须是有限数值，且不能超出配置的绝对范围 +/-3.2 rad。
        - 任意一项不合格都会把连续有效帧计数清零，不会沿用之前的旧反馈。
        """
        if len(msg.name) != len(msg.position):
            self.stable_samples = 0
            return
        values = dict(zip(msg.name, msg.position))
        if not self.required_names.issubset(values):
            self.stable_samples = 0
            return
        selected = {name: float(values[name]) for name in self.required_names}
        if not all(math.isfinite(value) for value in selected.values()):
            self.stable_samples = 0
            return
        if any(abs(value) > self.absolute_position_limit for value in selected.values()):
            self.stable_samples = 0
            return

        now = time.monotonic()
        if (
            self.last_joint_state_time is None
            or now - self.last_joint_state_time > self.joint_state_timeout
        ):
            self.stable_samples = 1
        else:
            self.stable_samples += 1
        self.positions = selected
        self.last_joint_state_time = now

    def _joint_state_ready(self) -> bool:
        """
        只有连续有效帧达到 required_stable_samples，且最后一帧没有超过
        joint_state_timeout_sec，才允许把编码器状态用于自动运动。
        """
        if self.last_joint_state_time is None:
            return False
        return (
            self.stable_samples >= self.required_stable_samples
            and time.monotonic() - self.last_joint_state_time
            <= self.joint_state_timeout
        )

    def _wait_until(self, predicate, timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise ReadyPoseError(f"timed out waiting for {description}")

    def _wait_future(self, future, timeout: float, description: str):
        self._wait_until(lambda: future.done(), timeout, description)
        try:
            return future.result()
        except Exception as exc:
            raise ReadyPoseError(f"{description} failed: {exc}") from exc

    def _wait_service(self, client, description: str) -> None:
        self._wait_until(
            client.service_is_ready,
            self.startup_timeout,
            description,
        )

    def _check_controllers_active(self) -> None:
        """
        等待左右 JTC 都完成启动：
        - control_base 先启动左控制器，左侧完成后才启动右控制器。
        - 因此只看到左侧 active、右侧 missing/inactive 是正常的瞬时状态。
        - 在 startup_timeout_sec 内持续查询，只有真正超时才拒绝自动运动。
        - 两侧都 active 后才允许打开 SOEM 命令门，避免无控制器时放行速度。
        """
        self._wait_service(self.controller_client, "controller manager service")
        required = ("left_arm_controller", "right_arm_controller")
        deadline = time.monotonic() + self.startup_timeout
        last_states: Dict[str, str] = {}
        last_wait_log = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            response = self._wait_future(
                self.controller_client.call_async(ListControllers.Request()),
                min(2.0, remaining),
                "controller list response",
            )
            last_states = {
                controller.name: controller.state for controller in response.controller
            }
            if all(last_states.get(name) == "active" for name in required):
                self.get_logger().info("Both JointTrajectoryControllers are active")
                return

            now = time.monotonic()
            if now - last_wait_log >= 5.0:
                last_wait_log = now
                detail = ", ".join(
                    f"{name}={last_states.get(name, 'missing')}" for name in required
                )
                self.get_logger().info(f"Waiting for controllers: {detail}")
            rclpy.spin_once(self, timeout_sec=0.1)

        detail = ", ".join(
            f"{name}={last_states.get(name, 'missing')}" for name in required
        )
        raise ReadyPoseError(f"controllers did not become active: {detail}")

    def _pose_error(self, names: Sequence[str], target: Sequence[float]) -> float:
        return max(abs(self.positions[name] - value) for name, value in zip(names, target))

    def _classify_start(
        self, arm: str, names: Sequence[str], target: Sequence[float]
    ) -> str:
        """
        初始姿态白名单检查：
        - zero：七个关节都在全零位容差内，可以执行完整 ready 轨迹。
        - ready：已经在目标容差内，该臂不需要再次运动。
        - 其他姿态默认拒绝，防止从未知位置沿固定关节路径自动运动。
        - allow_unknown_start 仅供隔离测试或人工确认后的恢复使用，实物默认关闭。
        """
        zero_error = max(abs(self.positions[name]) for name in names)
        ready_error = self._pose_error(names, target)
        if ready_error <= self.ready_tolerance:
            state = "ready"
        elif zero_error <= self.zero_tolerance:
            state = "zero"
        elif self.allow_unknown_start:
            state = "unknown-allowed"
        else:
            raise ReadyPoseError(
                f"{arm} arm is neither near zero nor ready "
                f"(zero_error={zero_error:.3f}, ready_error={ready_error:.3f} rad)"
            )
        self.get_logger().info(
            f"{arm} start classified as {state}: "
            f"zero_error={zero_error:.3f}, ready_error={ready_error:.3f} rad"
        )
        return state

    def _open_command_gate(self) -> None:
        """
        所有前置检查通过后才调用 SOEM enable(data=true)：
        - 此服务只打开软件速度命令门，不负责 CiA402 驱动硬件使能。
        - enable 成功后才进入 FollowJointTrajectory 执行阶段。
        - 后续任何异常都会由 safe_stop() 再次关闭命令门并清除旧目标。
        """
        if not self.manage_command_gate:
            self.get_logger().warn("SOEM command gate management disabled")
            return
        self._wait_service(self.enable_client, "SOEM enable service")
        request = SetBool.Request()
        request.data = True
        response = self._wait_future(
            self.enable_client.call_async(request),
            self.startup_timeout,
            "SOEM enable response",
        )
        if not response.success:
            raise ReadyPoseError(f"SOEM command gate rejected enable: {response.message}")
        self.command_gate_opened = True
        self.get_logger().warn("SOEM command gate opened; ready motion is now armed")

    @staticmethod
    def _duration_message(seconds: float) -> DurationMsg:
        whole_seconds = int(seconds)
        nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        return DurationMsg(sec=whole_seconds, nanosec=nanoseconds)

    def _trajectory_goal(
        self, names: Sequence[str], target: Sequence[float]
    ) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.velocities = [0.0] * len(names)
        point.accelerations = [0.0] * len(names)
        point.time_from_start = self._duration_message(self.trajectory_duration)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = self._duration_message(self.final_settle_timeout)
        return goal

    def _execute_arms_together(self) -> None:
        """
        同时执行并验证左右臂 ready 轨迹：
        - 已经处于 ready 的一侧直接跳过，只移动尚未到位的手臂。
        - 先为所有待移动手臂创建 action future，再共同等待接受结果；因此两侧
          控制器并行执行同一个 12 秒时间段，而不是左、右各占 12 秒。
        - 两侧 action 必须全部成功，并且新鲜编码器反馈中的实际位置必须同时进入
          ready_tolerance_rad；任何一侧失败都会进入 safe_stop() 取消其余目标。
        """
        arm_specs = (
            ("left", self.left_names, self.left_target, self.left_action),
            ("right", self.right_names, self.right_target, self.right_action),
        )
        moving = []
        for arm, names, target, action_client in arm_specs:
            if self._pose_error(names, target) <= self.ready_tolerance:
                self.get_logger().info(f"{arm} arm is already at ready; skipping motion")
            else:
                moving.append((arm, names, target, action_client))

        if not moving:
            return
        if not self._joint_state_ready():
            raise ReadyPoseError("joint feedback became stale before ready motion")
        for arm, _, _, action_client in moving:
            if not action_client.wait_for_server(timeout_sec=self.startup_timeout):
                raise ReadyPoseError(
                    f"{arm} FollowJointTrajectory action is unavailable"
                )

        moving_names = " and ".join(arm for arm, _, _, _ in moving)
        self.get_logger().warn(
            f"Moving {moving_names} arm(s) to ready together over "
            f"{self.trajectory_duration:.1f}s"
        )
        send_futures = {
            arm: action_client.send_goal_async(self._trajectory_goal(names, target))
            for arm, names, target, action_client in moving
        }
        self._wait_until(
            lambda: all(future.done() for future in send_futures.values()),
            self.startup_timeout,
            "ready trajectory goal acceptance",
        )

        acceptance_errors = []
        for arm, future in send_futures.items():
            try:
                goal_handle = future.result()
            except Exception as exc:
                acceptance_errors.append(f"{arm}: {exc}")
                continue
            if not goal_handle.accepted:
                acceptance_errors.append(f"{arm}: rejected")
                continue
            self.active_goal_handles[arm] = goal_handle
        if acceptance_errors:
            raise ReadyPoseError(
                "ready trajectory goal acceptance failed: "
                + "; ".join(acceptance_errors)
            )

        result_futures = {
            arm: goal_handle.get_result_async()
            for arm, goal_handle in self.active_goal_handles.items()
        }
        self._wait_until(
            lambda: all(future.done() for future in result_futures.values()),
            self.trajectory_duration + self.action_timeout_margin,
            "dual-arm trajectory results",
        )

        result_errors = []
        for arm, future in result_futures.items():
            try:
                result_wrapper = future.result()
            except Exception as exc:
                result_errors.append(f"{arm}: {exc}")
                continue
            if result_wrapper.status != GoalStatus.STATUS_SUCCEEDED:
                result_errors.append(
                    f"{arm}: action status {result_wrapper.status}"
                )
            elif (
                result_wrapper.result.error_code
                != FollowJointTrajectory.Result.SUCCESSFUL
            ):
                result_errors.append(
                    f"{arm}: code={result_wrapper.result.error_code} "
                    f"message={result_wrapper.result.error_string!r}"
                )
        if result_errors:
            raise ReadyPoseError(
                "ready trajectory failed: " + "; ".join(result_errors)
            )
        self.active_goal_handles = {}

        self.stable_samples = 0
        self._wait_until(
            lambda: self._joint_state_ready()
            and all(
                self._pose_error(names, target) <= self.ready_tolerance
                for _, names, target, _ in moving
            ),
            self.final_settle_timeout,
            "moving arms to settle at ready",
        )
        for arm, names, target, _ in moving:
            self.get_logger().info(
                f"{arm} arm reached ready; "
                f"max error={self._pose_error(names, target):.3f} rad"
            )

    def safe_stop(self, reason: str) -> None:
        """
        统一失败出口：
        - 若左右 JTC action 正在执行，先逐一请求取消全部活动目标。
        - 随后调用 SOEM /stop，在 RT 线程关闭命令门并清除全部轴的旧速度目标。
        - 节点最终以非零状态退出，使 real.launch.py 不再启动 Servo/VR。
        """
        self.get_logger().error(f"Ready motion stopped: {reason}")
        for arm, goal_handle in self.active_goal_handles.items():
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to request {arm} trajectory cancel: {exc}"
                )
        if not self.manage_command_gate:
            return
        if not self.stop_client.service_is_ready():
            self.get_logger().error("SOEM stop service unavailable")
            return
        try:
            future = self.stop_client.call_async(Trigger.Request())
            deadline = time.monotonic() + 2.0
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not future.done() or not future.result().success:
                self.get_logger().error("SOEM software stop was not confirmed")
            else:
                self.command_gate_opened = False
                self.get_logger().warn("SOEM software stop confirmed")
        except Exception as exc:
            self.get_logger().error(f"SOEM software stop call failed: {exc}")

    def run(self) -> bool:
        """
        自动预备位总流程：
        - 等待完整、有限、连续且新鲜的 14 轴编码器反馈。
        - 确认每只手臂位于 zero 或 ready 白名单姿态。
        - 等待左右 JTC 都 active，然后才打开 SOEM 软件命令门。
        - 同时提交左右轨迹，并使用实际编码器共同验证两侧最终位置。
        - 本函数只有全部步骤成功才返回；异常统一进入 safe_stop()。
        """
        self.get_logger().info("Waiting for stable 14-joint encoder feedback")
        self._wait_until(
            self._joint_state_ready,
            self.startup_timeout,
            "stable 14-joint feedback",
        )
        left_state = self._classify_start("left", self.left_names, self.left_target)
        right_state = self._classify_start("right", self.right_names, self.right_target)
        self._check_controllers_active()
        self._open_command_gate()

        self._execute_arms_together()

        if left_state == "ready" and right_state == "ready":
            self.get_logger().info("Both arms were already at ready")
        self.get_logger().info("READY MOTION COMPLETE: Servo/VR may now start")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[MoveToReady] = None
    exit_code = 1
    try:
        node = MoveToReady()
        exit_code = 0 if node.run() else 1
    except KeyboardInterrupt:
        if node is not None:
            node.safe_stop("interrupted by operator")
    except Exception as exc:
        if node is not None:
            node.safe_stop(str(exc))
        else:
            print(f"move_to_ready startup failed: {exc}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
