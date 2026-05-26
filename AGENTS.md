# dual_arm — ROS 2 Dual-Arm Robot Workspace

Colcon workspace with 4 ament_cmake packages for a dual-arm mobile robot (two 7-DOF arms on a wheeled chassis).

## Build & launch

```bash
colcon build                          # build all packages
source install/setup.bash             # required before any ros2 command
ros2 launch dual_arm_bringup sim.launch.py   # simulation (fake hardware)
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
```

Dependency chain: `bringup → moveit_config → description`, `bringup → control`.

## Key architecture: hardware plugin switching

The xacro file (`dual_arm_description/urdf/dual_arm_1kg.urdf.xacro`) takes an `hw_plugin` arg that selects the ros2_control hardware backend:

- `fake_components/GenericSystem` — simulation passthrough (default)
- `dual_arm_control/DualArmHardware` — real hardware plugin (C++ shared lib)

The bringup launch files set this arg and delegate to `dual_arm_moveit_config/demo.launch.py`, which starts 7 nodes: robot_state_publisher, ros2_control_node, joint_state_broadcaster, left_arm_controller, right_arm_controller, move_group, rviz2.

## dual_arm_control details

- Only package that compiles C++ (`libdual_arm_control.so`).
- Plugin registered via `pluginlib_export_plugin_description_file` in CMakeLists.txt.
- Implements `hardware_interface::SystemInterface` — currently a passthrough simulator (`read()` copies commands to states). Real hardware integration (CAN/EtherCAT) is stubbed.
- Joint count: 14 (laxis1–7, raxis1–7), each with position/velocity command and position/velocity/effort state interfaces.

## Gotchas

- `dual_arm_description` is a separate git repo inside the workspace. Changes there need a separate commit.
- `urdf.rviz` must be installed to the share directory or rviz2 defaults to `map` frame and fails. Already handled in CMakeLists.txt — do not remove that install rule.
- All arm joints are `type="revolute"` with limits ±3.14 rad. They were originally `type="fixed"` (see `dual_arm_description/notes.md` for migration history).
- Wheels are defined in URDF but have no functional joints or meshes.
- The `dual_arm_description/AGENTS.md` has package-specific details on layout, robot structure, and build.
- Mesh paths use `package://dual_arm_description/meshes/` — the package name must match exactly.
