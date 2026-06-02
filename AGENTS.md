# dual_arm — ROS 2 Dual-Arm Robot Workspace

Colcon workspace with 5 ament_cmake packages for a dual-arm mobile robot (two 7-DOF arms on a wheeled chassis), plus a vendored SOEM library for EtherCAT.

## Build & launch

```bash
colcon build                          # build all packages
source install/setup.bash             # required before any ros2 command
ros2 launch dual_arm_bringup sim.launch.py   # simulation (mock hardware)
ros2 launch dual_arm_bringup real.launch.py  # real hardware
```

Single-package build (faster iteration):
```bash
colcon build --packages-select dual_arm_control
```

There is no test suite, lint config, or CI. No typecheck or formatter is configured.

## Package map

```
dual_arm_description   ← URDF/xacro, meshes, rviz config (standalone, has its own .git)
dual_arm_control       ← C++ ros2_control hardware interface plugin (only compiled target)
dual_arm_moveit_config ← MoveIt2 config: SRDF, kinematics, controllers, launch
dual_arm_bringup       ← top-level launch: sim.launch.py, real.launch.py
dual_arm_soem_bridge   ← ROS2 ↔ SOEM EtherCAT bridge node (PP waypoint mode)
SOEM/                  ← vendored third-party EtherCAT master library (built by soem_bridge)
```

Dependency chain: `bringup → moveit_config → description`, `bringup → control`, `soem_bridge → SOEM/`.

## Key architecture: hardware plugin switching

The xacro file (`dual_arm_description/urdf/dual_arm_1kg.urdf.xacro`) takes an `hw_plugin` arg that selects the ros2_control hardware backend:

- `mock_components/GenericSystem` — simulation passthrough (default in xacro and sim.launch.py)
- `dual_arm_control/DualArmHardware` — real hardware plugin (C++ shared lib, used by real.launch.py)

The bringup launch files set this arg and delegate to `dual_arm_moveit_config/demo.launch.py`, which starts 7 nodes: robot_state_publisher, ros2_control_node, joint_state_broadcaster, left_arm_controller, right_arm_controller, move_group, rviz2.

## dual_arm_control details

- Only package (besides soem_bridge) that compiles C++.
- Plugin registered via `pluginlib_export_plugin_description_file` in CMakeLists.txt.
- Implements `hardware_interface::SystemInterface` — currently a passthrough simulator (`read()` copies commands to states). Real hardware integration (CAN/EtherCAT) is stubbed.
- Joint count: 14 (laxis1–7, raxis1–7), each with position/velocity command and position/velocity/effort state interfaces.

## dual_arm_soem_bridge details

- ROS2 node that bridges MoveIt trajectories to SOEM EtherCAT PP (Profile Position) control.
- Subscribes to `/dual_arm/pp_waypoints` (JointTrajectory). Publishes `~/real_joint_states`.
- Services: `~/enable` (SetBool), `~/stop` (Trigger), `~/clear_fault` (Trigger).
- **Defaults to `dry_run=true`** — won't open EtherCAT hardware unless explicitly enabled.
- Launch: `ros2 launch dual_arm_soem_bridge soem_bridge.launch.py ifname:=enp0s31f6`
- SOEM is built as a CMake subdirectory from `../SOEM`; its samples are disabled.

## MoveIt2 Jazzy configuration gotchas

These are hard-won lessons from debugging `move_group` crashes. See `DEBUGGING_NOTES.md` for the full story.

- **YAML content, not paths**: `demo.launch.py` must pass parsed YAML dicts (via `yaml.safe_load()`) for `robot_description_kinematics`, `robot_description_planning`, `ompl`, and `moveit_simple_controller_manager`. Passing a file path string silently fails.
- **OMPL planning plugin field**: In MoveIt2 Jazzy, the field is `planning_plugins` (plural, list), not `planning_plugin` (singular). The ompl config must be namespaced under `ompl:` in the YAML.
- **Joint acceleration limits required**: `joint_limits.yaml` must have `has_acceleration_limits: true` and a nonzero `max_acceleration` for every joint, or `AddTimeOptimalParameterization` fails.
- **Collision geometry**: Large STL meshes (>10k vertices) crash MoveIt/FCL. Use simple box/cylinder primitives for collision; keep STLs for visual only.
- **Octomap disabled**: No 3D sensor configured, so Octomap must be explicitly disabled via empty `octomap_frame`, zero resolution, etc.
- **MoveIt controller mapping**: `moveit_controllers.yaml` must use the `moveit_simple_controller_manager` namespace with `controller_names` list and per-controller `type: FollowJointTrajectory` blocks.

## Gotchas

- `dual_arm_description` is a separate git repo inside the workspace. Changes there need a separate commit.
- `urdf.rviz` must be installed to the share directory or rviz2 defaults to `map` frame and fails. Already handled in CMakeLists.txt — do not remove that install rule.
- All arm joints are `type="revolute"` with limits ±3.14 rad, effort 20 Nm, velocity 2.0 rad/s. They were originally `type="fixed"` (see `dual_arm_description/notes.md` for migration history).
- Wheels are defined in URDF but have no functional joints or meshes.
- The `dual_arm_description/AGENTS.md` has package-specific details on layout, robot structure, and build.
- Mesh paths use `package://dual_arm_description/meshes/` — the package name must match exactly.
- `sim.launch.py` passes `mock_components/GenericSystem`; `real.launch.py` passes `dual_arm_control/DualArmHardware`. Don't mix them up.
- Standalone joint control GUI (not a launch file): `python3 dual_arm_description/joint_control_panel.py`
