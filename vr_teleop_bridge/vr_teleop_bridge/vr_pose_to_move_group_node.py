#!/usr/bin/env python3
"""Convert relative VR controller motion to MoveGroup end-effector pose goals."""

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


Vector3Tuple = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _norm(vector: Vector3Tuple) -> float:
    return math.sqrt(sum(component * component for component in vector))


def _normalize_quat(quat: Quat) -> Quat:
    x, y, z, w = quat
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if length <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / length, y / length, z / length, w / length


def _quat_inverse(quat: Quat) -> Quat:
    x, y, z, w = _normalize_quat(quat)
    return -x, -y, -z, w


def _quat_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_to_rotvec(quat: Quat) -> Vector3Tuple:
    x, y, z, w = _normalize_quat(quat)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    vector_norm = math.sqrt(x * x + y * y + z * z)
    if vector_norm <= 1e-9:
        return 0.0, 0.0, 0.0
    angle = 2.0 * math.atan2(vector_norm, max(-1.0, min(1.0, w)))
    scale = angle / vector_norm
    return x * scale, y * scale, z * scale


def _rotvec_to_quat(rotvec: Vector3Tuple) -> Quat:
    angle = _norm(rotvec)
    if angle <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    x, y, z = (component / angle for component in rotvec)
    half = angle * 0.5
    scale = math.sin(half)
    return _normalize_quat((x * scale, y * scale, z * scale, math.cos(half)))


@dataclass
class ArmConfig:
    name: str
    enabled: bool
    group: str
    ee_link: str
    pose_topic: str
    joints: Tuple[str, ...]


@dataclass
class ArmState:
    config: ArmConfig
    vr_reference: Optional[PoseStamped] = None
    ee_reference: Optional[Pose] = None
    current_target: Optional[Pose] = None
    last_target: Optional[Pose] = None
    goal_active: bool = False
    last_goal_time: float = 0.0


class VrPoseToMoveGroup(Node):
    def __init__(self):
        super().__init__("vr_pose_to_move_group_node")
        self._declare_parameters()

        self.planning_frame = self.get_parameter("planning_frame").value
        self.combined_group = self.get_parameter("combined_group").value
        self.position_scale = float(self.get_parameter("position_scale").value)
        self.orientation_scale = float(self.get_parameter("orientation_scale").value)
        self.follow_orientation = bool(self.get_parameter("follow_orientation").value)
        self.goal_period = float(self.get_parameter("goal_period").value)
        self.min_position_delta = float(self.get_parameter("min_position_delta").value)
        self.min_rotation_delta = float(self.get_parameter("min_rotation_delta").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.ik_timeout = float(self.get_parameter("ik_timeout").value)
        self.joint_tolerance = float(self.get_parameter("joint_tolerance").value)
        self.velocity_scale = float(self.get_parameter("velocity_scale").value)
        self.acceleration_scale = float(self.get_parameter("acceleration_scale").value)
        self.workspace_min = tuple(float(v) for v in self.get_parameter("workspace_min").value)
        self.workspace_max = tuple(float(v) for v in self.get_parameter("workspace_max").value)
        self.axis_map = list(self.get_parameter("unity_to_ros_axis_map").value)
        self.axis_sign = [float(value) for value in self.get_parameter("unity_to_ros_axis_sign").value]
        self._validate_axis_mapping()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.move_group_busy = False
        self.ik_busy = False
        self.ik_results = {}
        self.ik_arms = []
        self.latest_joint_state: Optional[JointState] = None
        self.last_goal_time = 0.0

        self.arms: Dict[str, ArmState] = {
            "left": ArmState(
                ArmConfig(
                    name="left",
                    enabled=bool(self.get_parameter("left_enabled").value),
                    group=self.get_parameter("left_group").value,
                    ee_link=self.get_parameter("left_ee_link").value,
                    pose_topic=self.get_parameter("left_pose_topic").value,
                    joints=tuple(self.get_parameter("left_joints").value),
                )
            ),
            "right": ArmState(
                ArmConfig(
                    name="right",
                    enabled=bool(self.get_parameter("right_enabled").value),
                    group=self.get_parameter("right_group").value,
                    ee_link=self.get_parameter("right_ee_link").value,
                    pose_topic=self.get_parameter("right_pose_topic").value,
                    joints=tuple(self.get_parameter("right_joints").value),
                )
            ),
        }

        for arm in self.arms.values():
            self.create_subscription(
                PoseStamped,
                arm.config.pose_topic,
                lambda msg, state=arm: self._on_pose(state, msg),
                10,
            )
        self.create_subscription(String, self.get_parameter("status_topic").value, self._on_status, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self.get_logger().info(
            f"VR MoveGroup bridge ready: relative /vr/*/pose -> {self.combined_group} /move_action goals"
        )

    def _declare_parameters(self):
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("combined_group", "dual_arm")
        self.declare_parameter("left_pose_topic", "/vr/left_hand/pose")
        self.declare_parameter("right_pose_topic", "/vr/right_hand/pose")
        self.declare_parameter("status_topic", "/vr/status")
        self.declare_parameter("left_enabled", True)
        self.declare_parameter("right_enabled", True)
        self.declare_parameter("left_group", "left_arm")
        self.declare_parameter("right_group", "right_arm")
        self.declare_parameter("left_ee_link", "laxis7_link")
        self.declare_parameter("right_ee_link", "raxis7_link")
        self.declare_parameter(
            "left_joints",
            [
                "laxis1_joint",
                "laxis2_joint",
                "laxis3_joint",
                "laxis4_joint",
                "laxis5_joint",
                "laxis6_joint",
                "laxis7_joint",
            ],
        )
        self.declare_parameter(
            "right_joints",
            [
                "raxis1_joint",
                "raxis2_joint",
                "raxis3_joint",
                "raxis4_joint",
                "raxis5_joint",
                "raxis6_joint",
                "raxis7_joint",
            ],
        )
        self.declare_parameter("position_scale", 0.5)
        self.declare_parameter("orientation_scale", 1.0)
        self.declare_parameter("follow_orientation", False)
        self.declare_parameter("goal_period", 1.0)
        self.declare_parameter("min_position_delta", 0.03)
        self.declare_parameter("min_rotation_delta", 0.25)
        self.declare_parameter("position_tolerance", 0.04)
        self.declare_parameter("orientation_tolerance", 3.14)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("ik_timeout", 0.5)
        self.declare_parameter("joint_tolerance", 0.01)
        self.declare_parameter("velocity_scale", 0.1)
        self.declare_parameter("acceleration_scale", 0.1)
        self.declare_parameter("workspace_min", [-0.6, -0.8, 0.05])
        self.declare_parameter("workspace_max", [0.8, 0.8, 1.2])
        self.declare_parameter("unity_to_ros_axis_map", [2, 0, 1])
        self.declare_parameter("unity_to_ros_axis_sign", [1.0, -1.0, 1.0])

    def _validate_axis_mapping(self):
        if sorted(self.axis_map) != [0, 1, 2] or len(self.axis_sign) != 3:
            raise ValueError("unity_to_ros_axis_map must be a permutation of [0, 1, 2]")
        if len(self.workspace_min) != 3 or len(self.workspace_max) != 3:
            raise ValueError("workspace_min and workspace_max must have three values")

    def _on_joint_state(self, msg: JointState):
        if msg.name and msg.position:
            self.latest_joint_state = msg

    def _on_status(self, msg: String):
        text = msg.data.lower()
        if "disable" in text or "stop" in text or "reset" in text:
            for arm in self.arms.values():
                arm.vr_reference = None
                arm.ee_reference = None
                arm.current_target = None
                arm.last_target = None
            self.get_logger().info("VR references reset by status message")

    def _on_pose(self, arm: ArmState, msg: PoseStamped):
        if not arm.config.enabled:
            return
        now = self.get_clock().now().nanoseconds * 1e-9

        if arm.vr_reference is None or arm.ee_reference is None:
            ee_pose = self._lookup_ee_pose(arm.config.ee_link)
            if ee_pose is None:
                return
            arm.vr_reference = msg
            arm.ee_reference = ee_pose
            arm.current_target = ee_pose
            arm.last_target = ee_pose
            self.get_logger().info(
                f"{arm.config.name} reference captured: {arm.config.ee_link} in {self.planning_frame}"
            )
            return

        target = self._make_target_pose(arm, msg)
        if target is None or not self._target_changed(arm.current_target, target):
            return
        arm.current_target = target

        if self.move_group_busy or self.ik_busy or now - self.last_goal_time < self.goal_period:
            return
        if not self.move_group_client.server_is_ready():
            if not self.move_group_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().warn("Waiting for /move_action server", throttle_duration_sec=5.0)
                return
        if not self.ik_client.service_is_ready():
            if not self.ik_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().warn("Waiting for /compute_ik service", throttle_duration_sec=5.0)
                return

        self._start_ik_requests()

    def _lookup_ee_pose(self, ee_link: str) -> Optional[Pose]:
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.planning_frame,
                ee_link,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.planning_frame} -> {ee_link}: {exc}",
                throttle_duration_sec=5.0,
            )
            return None
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def _make_target_pose(self, arm: ArmState, msg: PoseStamped) -> Pose:
        assert arm.vr_reference is not None
        assert arm.ee_reference is not None
        vr_delta = (
            msg.pose.position.x - arm.vr_reference.pose.position.x,
            msg.pose.position.y - arm.vr_reference.pose.position.y,
            msg.pose.position.z - arm.vr_reference.pose.position.z,
        )
        ros_delta = self._map_unity_vector_to_ros(vr_delta)

        target = Pose()
        target.position.x = _clamp(
            arm.ee_reference.position.x + ros_delta[0] * self.position_scale,
            self.workspace_min[0],
            self.workspace_max[0],
        )
        target.position.y = _clamp(
            arm.ee_reference.position.y + ros_delta[1] * self.position_scale,
            self.workspace_min[1],
            self.workspace_max[1],
        )
        target.position.z = _clamp(
            arm.ee_reference.position.z + ros_delta[2] * self.position_scale,
            self.workspace_min[2],
            self.workspace_max[2],
        )

        reference_quat = self._pose_quat(arm.ee_reference)
        if self.follow_orientation:
            vr_delta_quat = _quat_multiply(
                self._pose_quat(msg),
                _quat_inverse(self._pose_quat(arm.vr_reference)),
            )
            ros_rotvec = self._map_unity_vector_to_ros(_quat_to_rotvec(vr_delta_quat))
            scaled_rotvec = tuple(component * self.orientation_scale for component in ros_rotvec)
            target_quat = _quat_multiply(_rotvec_to_quat(scaled_rotvec), reference_quat)
        else:
            target_quat = reference_quat
        (
            target.orientation.x,
            target.orientation.y,
            target.orientation.z,
            target.orientation.w,
        ) = _normalize_quat(target_quat)
        return target

    def _target_changed(self, previous: Optional[Pose], current: Pose) -> bool:
        if previous is None:
            return True
        position_delta = (
            current.position.x - previous.position.x,
            current.position.y - previous.position.y,
            current.position.z - previous.position.z,
        )
        if _norm(position_delta) >= self.min_position_delta:
            return True
        previous_q = self._pose_quat(previous)
        current_q = self._pose_quat(current)
        rotation_delta = _quat_to_rotvec(_quat_multiply(current_q, _quat_inverse(previous_q)))
        return _norm(rotation_delta) >= self.min_rotation_delta

    def _make_ik_request(self, arm: ArmState, target_pose: Pose) -> GetPositionIK.Request:
        stamped = PoseStamped()
        stamped.header.frame_id = self.planning_frame
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = target_pose

        request = GetPositionIK.Request()
        request.ik_request.group_name = arm.config.group
        request.ik_request.ik_link_name = arm.config.ee_link
        request.ik_request.pose_stamped = stamped
        request.ik_request.avoid_collisions = True
        if self.latest_joint_state is not None:
            request.ik_request.robot_state.joint_state = self.latest_joint_state
        request.ik_request.timeout = Duration(seconds=self.ik_timeout).to_msg()
        return request

    def _start_ik_requests(self):
        included_arms = []
        for arm in self.arms.values():
            if not arm.config.enabled or arm.current_target is None:
                continue
            included_arms.append(arm)

        if not included_arms:
            return

        self.ik_busy = True
        self.ik_results = {}
        self.ik_arms = included_arms
        for arm in included_arms:
            future = self.ik_client.call_async(self._make_ik_request(arm, arm.current_target))
            future.add_done_callback(lambda done, state=arm: self._on_ik_result(state, done))

    def _on_ik_result(self, arm: ArmState, future):
        if not self.ik_busy:
            return
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"{arm.config.name} IK request failed: {exc}")
            self._clear_ik()
            return

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            target = arm.current_target
            target_text = "unknown"
            if target is not None:
                target_text = f"({target.position.x:.3f},{target.position.y:.3f},{target.position.z:.3f})"
            self.get_logger().warn(
                f"{arm.config.name} IK failed for {arm.config.ee_link} target {target_text}: "
                f"error_code={response.error_code.val}"
            )
            self._clear_ik()
            return

        solution = dict(zip(response.solution.joint_state.name, response.solution.joint_state.position))
        missing = [name for name in arm.config.joints if name not in solution]
        if missing:
            self.get_logger().warn(f"{arm.config.name} IK solution missing joints: {missing}")
            self._clear_ik()
            return

        self.ik_results[arm.config.name] = {name: solution[name] for name in arm.config.joints}
        if len(self.ik_results) == len(self.ik_arms):
            arms = list(self.ik_arms)
            joint_targets = dict(self.ik_results)
            self._clear_ik()
            self._send_combined_joint_goal(arms, joint_targets)

    def _clear_ik(self):
        self.ik_busy = False
        self.ik_results = {}
        self.ik_arms = []

    def _send_combined_joint_goal(self, included_arms, joint_targets):
        constraints = Constraints()
        for arm in included_arms:
            for joint_name, position in joint_targets[arm.config.name].items():
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = position
                joint_constraint.tolerance_above = self.joint_tolerance
                joint_constraint.tolerance_below = self.joint_tolerance
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)

        request = MotionPlanRequest()
        request.group_name = self.combined_group
        request.goal_constraints = [constraints]
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.allowed_planning_time
        request.max_velocity_scaling_factor = self.velocity_scale
        request.max_acceleration_scaling_factor = self.acceleration_scale

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        for arm in included_arms:
            arm.goal_active = True
            arm.last_goal_time = self.get_clock().now().nanoseconds * 1e-9
            arm.last_target = arm.current_target
        self.move_group_busy = True
        self.last_goal_time = self.get_clock().now().nanoseconds * 1e-9
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(lambda done, states=included_arms: self._on_goal_response(states, done))
        target_text = " ".join(
            f"{arm.config.name}=({arm.current_target.position.x:.3f},"
            f"{arm.current_target.position.y:.3f},{arm.current_target.position.z:.3f})"
            for arm in included_arms
        )
        self.get_logger().info(f"{self.combined_group} MoveGroup IK target: {target_text}")

    def _clear_goal_active(self, arms):
        for arm in arms:
            arm.goal_active = False
        self.move_group_busy = False

    def _on_goal_response(self, arms, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._clear_goal_active(arms)
            self.get_logger().warn(f"{self.combined_group} MoveGroup goal request failed: {exc}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self._clear_goal_active(arms)
            self.get_logger().warn(f"{self.combined_group} MoveGroup goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done, states=arms: self._on_goal_result(states, done))

    def _on_goal_result(self, arms, future):
        self._clear_goal_active(arms)
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"{self.combined_group} MoveGroup result failed: {exc}")
            return
        code = result.result.error_code.val if result is not None else 99999
        if code == 1:
            self.get_logger().info(f"{self.combined_group} MoveGroup goal succeeded")
        else:
            self.get_logger().warn(f"{self.combined_group} MoveGroup goal failed: error_code={code}")

    def _map_unity_vector_to_ros(self, vector: Vector3Tuple) -> Vector3Tuple:
        values = [vector[0], vector[1], vector[2]]
        return tuple(self.axis_sign[i] * values[self.axis_map[i]] for i in range(3))

    def _pose_quat(self, pose_or_msg) -> Quat:
        orientation = pose_or_msg.pose.orientation if hasattr(pose_or_msg, "pose") else pose_or_msg.orientation
        return (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )


def main():
    rclpy.init()
    node = VrPoseToMoveGroup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
