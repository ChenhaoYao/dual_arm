#!/usr/bin/env python3
"""Bridge dual_arm ROS 2 observations/actions to LeRobot over ZeroMQ.

The bridge is deliberately not part of the normal real/sim launch path.  It can
publish observations and demonstration targets without being allowed to command
the robot.  Here "demonstration action" means an outgoing training label, while
"policy action" means an incoming command from a trained model.  Only the latter
is controlled by ``allow_action_commands`` and ``~/enable_actions``.  The existing
SOEM ``~/enable`` service is a separate downstream motor-command gate.
"""

import base64
import contextlib
import json
import math
import threading
import time
import uuid
from typing import Any, Optional

import rclpy
import zmq
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


PROTOCOL_VERSION = 1
LEFT_JOINTS = [f"laxis{i}_joint" for i in range(1, 8)]
RIGHT_JOINTS = [f"raxis{i}_joint" for i in range(1, 8)]
ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS


def _ros_stamp_ns(msg: Any) -> int:
    """Return a ROS header stamp in nanoseconds, or zero for headerless data."""
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return 0
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _ordered_values(
    names: list[str], values: Any, required_names: list[str]
) -> Optional[list[float]]:
    """Reorder a named ROS array and reject missing or non-finite values."""
    if len(names) != len(values):
        return None
    if len(names) != len(set(names)):
        return None
    value_by_name = dict(zip(names, values))
    if any(name not in value_by_name for name in required_names):
        return None
    ordered = [float(value_by_name[name]) for name in required_names]
    if not all(math.isfinite(value) for value in ordered):
        return None
    return ordered


def _encode_rgb_jpeg(frame_bgr: Any, cv2_module: Any, quality: int) -> str:
    """Encode an OpenCV BGR frame so LeRobot's current decoder returns RGB bytes.

    LeRobot's experimental ``ZMQCamera`` decodes JPEG with ``cv2.imdecode`` but
    does not yet apply its configured RGB conversion.  Pre-swapping here keeps
    the array delivered to the dataset in RGB order.  The handoff document calls
    out that this compatibility workaround belongs in the LeRobot receiver long
    term.
    """
    frame_rgb = cv2_module.cvtColor(frame_bgr, cv2_module.COLOR_BGR2RGB)
    ok, buffer = cv2_module.imencode(
        ".jpg", frame_rgb, [int(cv2_module.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return base64.b64encode(buffer).decode("ascii")


class CameraPublisher(threading.Thread):
    """Capture one OpenCV camera and publish its latest frame on one PUB port."""

    def __init__(
        self,
        context: zmq.Context,
        *,
        camera_name: str,
        device: str,
        bind_address: str,
        port: int,
        width: int,
        height: int,
        fps: float,
        jpeg_quality: int,
        cv2_module: Any,
        session_id: str,
        logger: Any,
    ) -> None:
        super().__init__(daemon=True, name=f"zmq_camera_{camera_name}")
        self.context = context
        self.camera_name = camera_name
        self.device = device
        self.bind_address = bind_address
        self.port = port
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.cv2 = cv2_module
        self.session_id = session_id
        self.logger = logger
        self._stop_event = threading.Event()

    @staticmethod
    def _opencv_device(device: str) -> str | int:
        return int(device) if device.isdecimal() else device

    def run(self) -> None:
        """
        Camera/ZMQ thread ownership rules:
        - A ZMQ socket must only be used by the thread that created it.
        - VideoCapture is opened and released in that same worker thread.
        - SNDHWM=2 and non-blocking sends drop old frames instead of delaying control.
        """
        socket = self.context.socket(zmq.PUB)
        socket.setsockopt(zmq.SNDHWM, 2)
        socket.setsockopt(zmq.LINGER, 0)
        endpoint = f"tcp://{self.bind_address}:{self.port}"
        try:
            socket.bind(endpoint)
        except zmq.ZMQError as exc:
            self.logger.error(f"[{self.camera_name}] cannot bind {endpoint}: {exc}")
            socket.close()
            return

        capture = self.cv2.VideoCapture(self._opencv_device(self.device))
        capture.set(self.cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(self.cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(self.cv2.CAP_PROP_FPS, self.fps)
        if not capture.isOpened():
            self.logger.error(f"[{self.camera_name}] cannot open camera {self.device}")
            capture.release()
            socket.close()
            return

        self.logger.info(
            f"[{self.camera_name}] {self.device} -> {endpoint} "
            f"({self.width}x{self.height}@{self.fps:g})"
        )
        sequence = 0
        period = 1.0 / self.fps
        try:
            while not self._stop_event.is_set():
                loop_start = time.monotonic()
                ok, frame = capture.read()
                if not ok:
                    self._stop_event.wait(0.02)
                    continue
                if frame.shape[1] != self.width or frame.shape[0] != self.height:
                    frame = self.cv2.resize(frame, (self.width, self.height))
                try:
                    encoded = _encode_rgb_jpeg(frame, self.cv2, self.jpeg_quality)
                except RuntimeError as exc:
                    self.logger.warning(f"[{self.camera_name}] {exc}")
                    continue

                timestamp = time.time()
                payload = {
                    "protocol_version": PROTOCOL_VERSION,
                    "message_type": "camera_frame",
                    "session_id": self.session_id,
                    "sequence": sequence,
                    "timestamp": timestamp,
                    "color_space": "rgb",
                    "timestamps": {self.camera_name: timestamp},
                    "images": {self.camera_name: encoded},
                }
                with contextlib.suppress(zmq.Again):
                    socket.send_string(json.dumps(payload), flags=zmq.NOBLOCK)
                sequence += 1
                self._stop_event.wait(max(0.0, period - (time.monotonic() - loop_start)))
        finally:
            capture.release()
            socket.close()

    def stop(self) -> None:
        self._stop_event.set()


class ZmqBridgeNode(Node):
    """ROS 2/ZMQ bridge with observation-only startup and guarded action input."""

    def __init__(self) -> None:
        super().__init__("zmq_bridge_node")
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self.zmq_context = zmq.Context()
        self._warning_times: dict[str, float] = {}
        self.session_id = str(uuid.uuid4())
        self._joint_sequence = 0
        self._demo_sequence = 0
        self._last_joint_positions: Optional[list[float]] = None
        self._last_joint_receive_time = 0.0
        self._controller_references: dict[str, tuple[list[float], float, int]] = {}
        self._vr_enabled = {"left": False, "right": False}

        """
        Keep the two meanings of "action" and the two software gates separate:
        - demo_pub/port 5557 is output-only training data. It publishes the JTC
          reference produced by VR/Servo and is never blocked by this flag.
        - action_pull/port 5558 is an incoming trained-policy command. Only this
          path is controlled by allow_action_commands and _action_gate_open.
        - /soem_bridge_node/enable is farther downstream and decides whether JTC
          output may reach EtherCAT motors. This bridge never opens that SOEM gate.

        Data collection therefore keeps allow_action_commands=false and still
        publishes joint observations, demonstration actions, and camera images.
        """
        self._action_gate_open = False
        self._last_accepted_action_time = 0.0
        self._last_action_sequence: Optional[int] = None
        self._action_session_id: Optional[str] = None

        # Outgoing dataset streams; neither socket can command ROS or the motors.
        self.joint_pub = self._make_pub_socket(self.joint_state_port, hwm=5)
        self.demo_pub = self._make_pub_socket(self.demo_action_port, hwm=5)

        # Incoming trained-policy command stream; guarded before publishing to JTC.
        self.action_pull = self.zmq_context.socket(zmq.PULL)
        self.action_pull.setsockopt(zmq.RCVHWM, 5)
        self.action_pull.setsockopt(zmq.LINGER, 0)
        self.action_pull.setsockopt(zmq.MAXMSGSIZE, 64 * 1024)
        self.action_pull.bind(self._endpoint(self.policy_action_port))

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.joint_sub = self.create_subscription(
            JointState, self.joint_state_topic, self._on_joint_state, sensor_qos
        )
        self.left_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            self.left_controller_state_topic,
            lambda msg: self._on_controller_state("left", msg),
            10,
        )
        self.right_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            self.right_controller_state_topic,
            lambda msg: self._on_controller_state("right", msg),
            10,
        )
        self.left_enable_sub = self.create_subscription(
            Bool,
            self.left_vr_enable_topic,
            lambda msg: self._set_vr_enabled("left", msg),
            10,
        )
        self.right_enable_sub = self.create_subscription(
            Bool,
            self.right_vr_enable_topic,
            lambda msg: self._set_vr_enabled("right", msg),
            10,
        )

        self.left_traj_pub = self.create_publisher(
            JointTrajectory, self.left_trajectory_topic, 10
        )
        self.right_traj_pub = self.create_publisher(
            JointTrajectory, self.right_trajectory_topic, 10
        )
        # This service arms only port 5558 -> JTC, not the SOEM/EtherCAT command gate.
        self.enable_actions_service = self.create_service(
            SetBool, "~/enable_actions", self._on_enable_actions
        )

        self.camera_threads: list[CameraPublisher] = []
        self._start_cameras()
        self.demo_timer = self.create_timer(
            1.0 / self.demo_action_publish_rate, self._publish_demo_action
        )
        self.action_timer = self.create_timer(0.01, self._poll_policy_action)
        self.watchdog_timer = self.create_timer(0.05, self._check_action_watchdog)

        self.get_logger().info(
            f"ZMQ v{PROTOCOL_VERSION} bridge ready: joints={self._endpoint(self.joint_state_port)}, "
            f"demonstration_actions={self._endpoint(self.demo_action_port)}, "
            f"policy_actions={self._endpoint(self.policy_action_port)}"
        )
        if self.allow_action_commands:
            self.get_logger().warn(
                "Incoming policy-action capability is configured, but its bridge gate "
                "starts CLOSED. "
                "Call ~/enable_actions with data=true only after completing safety checks."
            )
        else:
            self.get_logger().info(
                "Data-output mode: joint observations, demonstration actions, and cameras "
                "remain enabled; incoming policy actions cannot command ROS controllers."
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter("bind_address", "*")
        self.declare_parameter("joint_state_port", 5556)
        self.declare_parameter("demo_action_port", 5557)
        self.declare_parameter("policy_action_port", 5558)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "left_controller_state_topic", "/left_arm_controller/controller_state"
        )
        self.declare_parameter(
            "right_controller_state_topic", "/right_arm_controller/controller_state"
        )
        self.declare_parameter("left_vr_enable_topic", "/vr/left_hand/enabled")
        self.declare_parameter("right_vr_enable_topic", "/vr/right_hand/enabled")
        self.declare_parameter(
            "left_trajectory_topic", "/left_arm_controller/joint_trajectory"
        )
        self.declare_parameter(
            "right_trajectory_topic", "/right_arm_controller/joint_trajectory"
        )
        self.declare_parameter("demo_action_publish_rate", 30.0)
        self.declare_parameter("state_timeout", 0.25)

        """
        This parameter does not disable demonstration-action recording:
        - false (normal data collection): continue publishing ports 5555/5556/5557,
          but discard incoming trained-policy commands from port 5558.
        - true: merely permits the input path to be armed later through
          ~/enable_actions; startup still leaves the runtime gate closed.
        """
        self.declare_parameter("allow_action_commands", False)
        self.declare_parameter("accept_legacy_actions", True)
        self.declare_parameter("action_watchdog_timeout", 0.30)
        self.declare_parameter("action_max_age", 0.50)
        self.declare_parameter("action_future_tolerance", 1.0)
        self.declare_parameter("max_joint_delta", 0.10)
        self.declare_parameter("joint_position_limit", 3.14)
        self.declare_parameter("trajectory_time_from_start", 0.10)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("jpeg_quality", 80)
        for name, device, port in (
            ("head_camera", "/dev/video0", 5555),
            ("left_wrist_camera", "/dev/video2", 5559),
            ("right_wrist_camera", "/dev/video4", 5560),
        ):
            self.declare_parameter(f"cameras.{name}.enabled", True)
            self.declare_parameter(f"cameras.{name}.device", device)
            self.declare_parameter(f"cameras.{name}.port", port)

    def _read_parameters(self) -> None:
        value = lambda name: self.get_parameter(name).value
        self.bind_address = str(value("bind_address"))
        self.joint_state_port = int(value("joint_state_port"))
        self.demo_action_port = int(value("demo_action_port"))
        self.policy_action_port = int(value("policy_action_port"))
        self.joint_state_topic = str(value("joint_state_topic"))
        self.left_controller_state_topic = str(value("left_controller_state_topic"))
        self.right_controller_state_topic = str(value("right_controller_state_topic"))
        self.left_vr_enable_topic = str(value("left_vr_enable_topic"))
        self.right_vr_enable_topic = str(value("right_vr_enable_topic"))
        self.left_trajectory_topic = str(value("left_trajectory_topic"))
        self.right_trajectory_topic = str(value("right_trajectory_topic"))
        self.demo_action_publish_rate = float(value("demo_action_publish_rate"))
        self.state_timeout = float(value("state_timeout"))
        self.allow_action_commands = bool(value("allow_action_commands"))
        self.accept_legacy_actions = bool(value("accept_legacy_actions"))
        self.action_watchdog_timeout = float(value("action_watchdog_timeout"))
        self.action_max_age = float(value("action_max_age"))
        self.action_future_tolerance = float(value("action_future_tolerance"))
        self.max_joint_delta = float(value("max_joint_delta"))
        self.joint_position_limit = float(value("joint_position_limit"))
        self.trajectory_time_from_start = float(value("trajectory_time_from_start"))
        self.camera_width = int(value("camera_width"))
        self.camera_height = int(value("camera_height"))
        self.camera_fps = float(value("camera_fps"))
        self.jpeg_quality = int(value("jpeg_quality"))
        self.camera_configs = []
        for name in ("head_camera", "left_wrist_camera", "right_wrist_camera"):
            self.camera_configs.append(
                {
                    "name": name,
                    "enabled": bool(value(f"cameras.{name}.enabled")),
                    "device": str(value(f"cameras.{name}.device")),
                    "port": int(value(f"cameras.{name}.port")),
                }
            )

    def _validate_parameters(self) -> None:
        ports = [self.joint_state_port, self.demo_action_port, self.policy_action_port]
        ports.extend(cfg["port"] for cfg in self.camera_configs if cfg["enabled"])
        if any(port <= 0 or port > 65535 for port in ports):
            raise ValueError("all enabled ZMQ ports must be in 1..65535")
        if len(set(ports)) != len(ports):
            raise ValueError("enabled ZMQ ports must be unique")
        positive = {
            "demo_action_publish_rate": self.demo_action_publish_rate,
            "state_timeout": self.state_timeout,
            "action_watchdog_timeout": self.action_watchdog_timeout,
            "action_max_age": self.action_max_age,
            "max_joint_delta": self.max_joint_delta,
            "joint_position_limit": self.joint_position_limit,
            "trajectory_time_from_start": self.trajectory_time_from_start,
            "camera_fps": self.camera_fps,
        }
        for name, parameter in positive.items():
            if not math.isfinite(parameter) or parameter <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.camera_width <= 0 or self.camera_height <= 0:
            raise ValueError("camera dimensions must be positive")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in 1..100")

    def _endpoint(self, port: int) -> str:
        return f"tcp://{self.bind_address}:{port}"

    def _make_pub_socket(self, port: int, hwm: int) -> zmq.Socket:
        socket = self.zmq_context.socket(zmq.PUB)
        socket.setsockopt(zmq.SNDHWM, hwm)
        socket.setsockopt(zmq.LINGER, 0)
        socket.bind(self._endpoint(port))
        return socket

    def _start_cameras(self) -> None:
        enabled_configs = [config for config in self.camera_configs if config["enabled"]]
        for config in self.camera_configs:
            if not config["enabled"]:
                self.get_logger().info(f"[{config['name']}] disabled by configuration")
        if not enabled_configs:
            return

        """
        OpenCV is optional for the joint/action bridge:
        - Import it only when at least one camera is enabled.
        - A broken OpenCV/NumPy ABI must not suppress robot proprioception.
        - All cameras are disabled together on import failure to avoid three duplicate errors.
        """
        try:
            import cv2
        except (ImportError, RuntimeError) as exc:
            self.get_logger().error(
                "all ZMQ cameras disabled: OpenCV cannot be imported; "
                f"check the OpenCV/NumPy environment ({exc})"
            )
            return

        for config in enabled_configs:
            thread = CameraPublisher(
                self.zmq_context,
                camera_name=config["name"],
                device=config["device"],
                bind_address=self.bind_address,
                port=config["port"],
                width=self.camera_width,
                height=self.camera_height,
                fps=self.camera_fps,
                jpeg_quality=self.jpeg_quality,
                cv2_module=cv2,
                session_id=self.session_id,
                logger=self.get_logger(),
            )
            thread.start()
            self.camera_threads.append(thread)

    def _warn_throttled(self, key: str, message: str, period: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._warning_times.get(key, 0.0) >= period:
            self._warning_times[key] = now
            self.get_logger().warn(message)

    def _on_joint_state(self, msg: JointState) -> None:
        positions = _ordered_values(list(msg.name), msg.position, ALL_JOINTS)
        if positions is None:
            self._warn_throttled(
                "invalid_joint_state",
                "/joint_states ignored: expected 14 named joints with finite positions",
            )
            return

        velocities = _ordered_values(list(msg.name), msg.velocity, ALL_JOINTS)
        efforts = _ordered_values(list(msg.name), msg.effort, ALL_JOINTS)
        receive_time = time.monotonic()
        self._last_joint_positions = positions
        self._last_joint_receive_time = receive_time
        payload = {
            "protocol_version": PROTOCOL_VERSION,
            "message_type": "joint_state",
            "session_id": self.session_id,
            "sequence": self._joint_sequence,
            "timestamp": time.time(),
            "ros_timestamp_ns": _ros_stamp_ns(msg),
            "names": ALL_JOINTS,
            "positions": positions,
            "velocities": velocities or [],
            "efforts": efforts or [],
        }
        self._joint_sequence += 1
        with contextlib.suppress(zmq.Again):
            self.joint_pub.send_string(json.dumps(payload), flags=zmq.NOBLOCK)

    def _on_controller_state(
        self, side: str, msg: JointTrajectoryControllerState
    ) -> None:
        required = LEFT_JOINTS if side == "left" else RIGHT_JOINTS
        reference = _ordered_values(
            list(msg.joint_names), msg.reference.positions, required
        )
        if reference is None:
            self._warn_throttled(
                f"invalid_{side}_reference",
                f"{side} controller reference ignored: incomplete or non-finite positions",
            )
            return
        self._controller_references[side] = (
            reference,
            time.monotonic(),
            _ros_stamp_ns(msg),
        )

    def _set_vr_enabled(self, side: str, msg: Bool) -> None:
        self._vr_enabled[side] = bool(msg.data)

    def _publish_demo_action(self) -> None:
        """
        Demonstration labels use JTC references, not encoder observations:
        - Encoder positions are what the robot actually did (observation).
        - JTC reference positions are the target produced by VR -> Servo (action).
        - Both arms must be fresh so every LeRobot sample has a complete 14-DOF action.
        - This output-only training label is unrelated to the policy-action input gate.
        """
        now = time.monotonic()
        if any(side not in self._controller_references for side in ("left", "right")):
            self._warn_throttled(
                "missing_controller_reference",
                "demonstration action waiting for both controller_state topics",
                period=5.0,
            )
            return
        left, left_time, left_stamp = self._controller_references["left"]
        right, right_time, right_stamp = self._controller_references["right"]
        if now - min(left_time, right_time) > self.state_timeout:
            self._warn_throttled(
                "stale_controller_reference",
                "demonstration action not published: controller_state is stale",
            )
            return

        positions = left + right
        payload = {
            "protocol_version": PROTOCOL_VERSION,
            "message_type": "demonstration_action",
            "source": "joint_trajectory_controller_reference",
            "session_id": self.session_id,
            "sequence": self._demo_sequence,
            "timestamp": time.time(),
            "ros_timestamp_ns": max(left_stamp, right_stamp),
            "joint_positions": dict(zip(ALL_JOINTS, positions)),
            "vr_enabled": dict(self._vr_enabled),
        }
        self._demo_sequence += 1
        with contextlib.suppress(zmq.Again):
            self.demo_pub.send_string(json.dumps(payload), flags=zmq.NOBLOCK)

    def _on_enable_actions(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """
        This service controls only incoming trained-policy commands (ZMQ 5558 -> JTC):
        - Configure ``allow_action_commands=true`` before bridge startup.
        - Arm this runtime gate with ``~/enable_actions``.

        It does not affect outgoing observations, demonstration labels, or cameras.
        The existing /soem_bridge_node/enable gate is independent and farther
        downstream; it remains authoritative for real EtherCAT motor commands.
        This service never opens, closes, or bypasses that SOEM gate.
        """
        if not request.data:
            self._action_gate_open = False
            response.success = True
            response.message = "LeRobot incoming policy-action gate closed"
            self.get_logger().warn(response.message)
            return response
        if not self.allow_action_commands:
            response.success = False
            response.message = (
                "incoming policy-action capability disabled by "
                "allow_action_commands=false; outgoing dataset streams remain enabled"
            )
            return response
        if not self._joint_state_is_fresh():
            response.success = False
            response.message = "cannot arm: complete joint state is missing or stale"
            return response
        if (
            self.left_traj_pub.get_subscription_count() == 0
            or self.right_traj_pub.get_subscription_count() == 0
        ):
            response.success = False
            response.message = "cannot arm: both trajectory controllers must be subscribed"
            return response

        self._action_gate_open = True
        self._last_accepted_action_time = time.monotonic()
        self._last_action_sequence = None
        self._action_session_id = None
        response.success = True
        response.message = (
            "LeRobot incoming policy-action gate OPEN; commands may reach both JTCs, "
            "but the independent SOEM motor gate is unchanged"
        )
        self.get_logger().warn(response.message)
        return response

    def _joint_state_is_fresh(self) -> bool:
        return (
            self._last_joint_positions is not None
            and time.monotonic() - self._last_joint_receive_time <= self.state_timeout
        )

    def _poll_policy_action(self) -> None:
        latest_message: Optional[str] = None
        while True:
            try:
                latest_message = self.action_pull.recv_string(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except (UnicodeDecodeError, zmq.ZMQError) as exc:
                self._warn_throttled("action_receive_error", f"action receive failed: {exc}")
                return
        if latest_message is None:
            return
        if not self.allow_action_commands or not self._action_gate_open:
            self._warn_throttled(
                "action_gate_closed",
                "LeRobot policy action received but ignored: its bridge input gate is closed; "
                "outgoing dataset streams are unaffected",
            )
            return

        try:
            positions, action_session_id, sequence = self._validate_policy_action(
                latest_message
            )
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._warn_throttled("invalid_action", f"LeRobot action rejected: {exc}")
            return

        self._publish_trajectory(self.left_traj_pub, LEFT_JOINTS, positions[:7])
        self._publish_trajectory(self.right_traj_pub, RIGHT_JOINTS, positions[7:])
        if sequence is not None:
            self._last_action_sequence = sequence
            self._action_session_id = action_session_id
        self._last_accepted_action_time = time.monotonic()

    def _validate_policy_action(
        self, message: str
    ) -> tuple[list[float], Optional[str], Optional[int]]:
        """Validate protocol, freshness, completeness, limits, and actual-relative step."""
        data = json.loads(message)
        if not isinstance(data, dict):
            raise TypeError("payload must be a JSON object")
        version = data.get("protocol_version")
        if version is None:
            if not self.accept_legacy_actions:
                raise ValueError("missing protocol_version")
            action_session_id = None
            sequence = None
        elif version != PROTOCOL_VERSION:
            raise ValueError(f"unsupported protocol_version={version}")
        else:
            if data.get("message_type") != "joint_position_action":
                raise ValueError(f"unexpected message_type={data.get('message_type')!r}")
            action_session_id = data.get("session_id")
            if not isinstance(action_session_id, str) or not action_session_id:
                raise ValueError("session_id must be a non-empty string")
            if (
                self._action_session_id is not None
                and action_session_id != self._action_session_id
            ):
                raise ValueError(
                    "policy-action session changed while armed; close and re-arm its bridge input gate"
                )
            sequence = data.get("sequence")
            if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
                raise ValueError("sequence must be a non-negative integer")
            if (
                self._last_action_sequence is not None
                and sequence <= self._last_action_sequence
            ):
                raise ValueError(
                    f"sequence {sequence} is not newer than {self._last_action_sequence}"
                )

        timestamp = float(data.get("timestamp", float("nan")))
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        age = time.time() - timestamp
        if age > self.action_max_age:
            raise ValueError(f"action is stale ({age:.3f}s)")
        if age < -self.action_future_tolerance:
            raise ValueError(f"action timestamp is in the future ({-age:.3f}s)")

        joint_positions = data.get("joint_positions")
        if not isinstance(joint_positions, dict):
            raise TypeError("joint_positions must be an object")
        if set(joint_positions) != set(ALL_JOINTS):
            missing = sorted(set(ALL_JOINTS) - set(joint_positions))
            extra = sorted(set(joint_positions) - set(ALL_JOINTS))
            raise ValueError(f"joint set mismatch; missing={missing}, extra={extra}")
        positions = [float(joint_positions[name]) for name in ALL_JOINTS]
        if not all(math.isfinite(position) for position in positions):
            raise ValueError("joint positions must be finite")
        if any(abs(position) > self.joint_position_limit for position in positions):
            raise ValueError(
                f"joint position exceeds +/-{self.joint_position_limit:.3f} rad"
            )
        if not self._joint_state_is_fresh():
            raise ValueError("joint state is missing or stale")
        assert self._last_joint_positions is not None
        largest_delta = max(
            abs(target - actual)
            for target, actual in zip(positions, self._last_joint_positions)
        )
        if largest_delta > self.max_joint_delta:
            raise ValueError(
                f"actual-relative delta {largest_delta:.3f} rad exceeds "
                f"{self.max_joint_delta:.3f} rad"
            )
        return positions, action_session_id, sequence

    def _publish_trajectory(
        self, publisher: Any, joint_names: list[str], positions: list[float]
    ) -> None:
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = list(joint_names)
        point = JointTrajectoryPoint()
        point.positions = positions
        seconds = int(self.trajectory_time_from_start)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = int(
            (self.trajectory_time_from_start - seconds) * 1_000_000_000
        )
        trajectory.points.append(point)
        publisher.publish(trajectory)

    def _check_action_watchdog(self) -> None:
        if not self._action_gate_open:
            return
        if time.monotonic() - self._last_accepted_action_time <= self.action_watchdog_timeout:
            return
        self._action_gate_open = False
        self.get_logger().error(
            "LeRobot policy-action watchdog timed out; its bridge input gate latched CLOSED. "
            "Re-arm ~/enable_actions only after checking the policy stream."
        )

    def destroy_node(self) -> None:
        self._action_gate_open = False
        for thread in self.camera_threads:
            thread.stop()
        for thread in self.camera_threads:
            thread.join(timeout=2.0)
            if thread.is_alive():
                self.get_logger().warn(f"camera thread did not stop: {thread.name}")
        self.joint_pub.close()
        self.demo_pub.close()
        self.action_pull.close()
        self.zmq_context.destroy(linger=0)
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[ZmqBridgeNode] = None
    try:
        node = ZmqBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                # ros2 launch forwards SIGINT to the child after its own SIGINT;
                # a second signal can arrive while rclpy destroys subscriptions.
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
