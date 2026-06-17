# dual_arm — ROS 2 Dual-Arm Robot Workspace

Colcon workspace with 5 ament_cmake packages for a dual-arm mobile robot (two 7-DOF arms on a wheeled chassis), plus a vendored SOEM library for EtherCAT. Target ROS 2 distro: **Jazzy**.

## Build & launch

```bash
colcon build                          # build all packages
source install/setup.bash             # required before any ros2 command
ros2 launch dual_arm_bringup sim.launch.py   # simulation (mock hardware)
```

For real hardware:
```bash
# Terminal 1: Start MoveIt (without joint_state_broadcaster to avoid conflict)
ros2 launch dual_arm_bringup sim.launch.py use_broadcaster:=false

# Terminal 2: Start SOEM bridge (auto-connects to motors, reads encoders)
ros2 launch dual_arm_soem_bridge soem_bridge.launch.py

# Terminal 3: Enable sending velocity commands
ros2 service call /soem_bridge_node/enable std_srvs/srv/SetBool "{data: true}"
```

Single-package build (faster iteration):
```bash
colcon build --packages-select dual_arm_soem_bridge
```

There is no test suite, lint config, or CI. No typecheck or formatter is configured.

## Package map

```
dual_arm_description   ← URDF/xacro, meshes, rviz config
dual_arm_control       ← C++ ros2_control hardware interface plugin (not currently used)
dual_arm_moveit_config ← MoveIt2 config: SRDF, kinematics, controllers, launch
dual_arm_bringup       ← top-level launch: sim.launch.py (real.launch.py deprecated)
dual_arm_soem_bridge   ← ROS2 ↔ SOEM EtherCAT bridge node (CSV velocity mode)
SOEM/                  ← vendored third-party EtherCAT master library (built by soem_bridge)
```

Dependency chain: `bringup → moveit_config → description`, `soem_bridge → SOEM/`.

## Key architecture: soem_bridge_node

The soem_bridge_node handles all real hardware communication via EtherCAT:

```
MoveIt → JTC → controller_state → soem_bridge_node → EtherCAT → Motors
                /joint_states (mock)     ↓
                                    feedback_topic (real encoders)
```

### dry_run modes

- `dry_run=true`: No EtherCAT, only prints waypoint conversions
- `dry_run=false`: Auto-starts EtherCAT on node startup, reads encoders immediately

### Services

- `~/enable` (SetBool): Controls whether velocity commands are sent (not EtherCAT connection)
- `~/stop` (Trigger): Disables sending
- `~/clear_fault` (Trigger): Fault reset handled by RT thread

### Config

`dual_arm_soem_bridge/config/soem_bridge.yaml`:
- `ifname`: EtherCAT network interface (e.g., "enp0s31f6")
- `dry_run`: true/false
- `feedback_topic`: Set to "/joint_states" for RViz display
- `left_state_topic` / `right_state_topic`: JTC controller_state topics
- `axis_*`: Per-axis calibration parameters (slave, direction, zero_offset, gear_ratio, limits)

## MoveIt2 Jazzy configuration gotchas

These are hard-won lessons from debugging `move_group` crashes. See `DEBUGGING_NOTES.md` for the full story.

- **YAML content, not paths**: `moveit.launch.py` must pass parsed YAML dicts (via `yaml.safe_load()`) for `robot_description_kinematics`, `robot_description_planning`, `ompl`, and `moveit_simple_controller_manager`. Passing a file path string silently fails.
- **OMPL planning plugin field**: In MoveIt2 Jazzy, the field is `planning_plugins` (plural, list), not `planning_plugin` (singular). The ompl config must be namespaced under `ompl:` in the YAML.
- **Joint acceleration limits required**: `joint_limits.yaml` must have `has_acceleration_limits: true` and a nonzero `max_acceleration` for every joint, or `AddTimeOptimalParameterization` fails.
- **Collision geometry**: Large STL meshes (>10k vertices) crash MoveIt/FCL. base_link and laxis/raxis 1-2 already use box/cylinder primitives for collision. laxis/raxis 3-7 still use STL meshes for collision — if `move_group` crashes with "too many vertices", simplify those too.
- **Octomap disabled**: No 3D sensor configured, so Octomap must be explicitly disabled via empty `octomap_frame`, zero resolution, etc.
- **MoveIt controller mapping**: `moveit_controllers.yaml` must use the `moveit_simple_controller_manager` namespace with `controller_names` list and per-controller `type: FollowJointTrajectory` blocks.

## Gotchas

- `urdf.rviz` must be installed to the share directory or rviz2 defaults to `map` frame and fails. Already handled in CMakeLists.txt — do not remove that install rule.
- All arm joints are `type="revolute"` with limits ±3.14 rad, effort 20 Nm, velocity 2.0 rad/s. They were originally `type="fixed"` (see `dual_arm_description/notes.md` for migration history).
- Wheels are defined in URDF but have no functional joints or meshes.
- The `dual_arm_description/AGENTS.md` has package-specific details on layout, robot structure, and build.
- Mesh paths use `package://dual_arm_description/meshes/` — the package name must match exactly.
- `sim.launch.py` passes `mock_components/GenericSystem`. `real.launch.py` is deprecated — use `sim.launch.py` + `soem_bridge_node` instead.
- Standalone joint control GUI (not a launch file): `python3 dual_arm_description/joint_control_panel.py`
- `ros2_controllers.yaml` uses only `position` command interface, even though the URDF defines both position and velocity. This is intentional — velocity commands are unused.
- `README.md` says `fake_components/GenericSystem` — the actual xacro and launch files use `mock_components/GenericSystem`. Trust the code, not the README.
