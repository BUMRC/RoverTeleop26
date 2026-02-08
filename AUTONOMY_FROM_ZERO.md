# Rover Autonomy From Zero

This document explains this rover stack from the ground up.

The goal is that someone with no ROS background can:

1. Understand the architecture.
2. Build and run teleop.
3. Run point-to-point navigation.
4. Visualize everything in Foxglove.
5. Add obstacle avoidance later.
6. Move toward SLAM + GNSS + odometry fusion for URC.

## 1. What You Are Building

You are building a ROS 2 robot software system with four layers:

1. Hardware control layer:
   - Talks to CubeMars motors over CAN.
   - Exposes wheel velocity interfaces to ROS controllers.
2. Base control layer:
   - Uses diff drive controller to convert linear/angular velocity into wheel commands.
3. Navigation layer:
   - Accepts a goal pose and plans/drives to it.
4. Operator UI layer:
   - Foxglove for teleop, visualization, map/path/odom/debug.

Current status of this stack:

- Teleop works.
- Navigation to local map coordinates works.
- Obstacle avoidance is scaffolded, not mandatory for first milestone.
- SLAM + GNSS fusion is scaffolded for later integration.

Important hardware limitation in current setup:

- Motors on a side share the same CAN ID (`111`).
- You use two CAN buses (`can0`, `can1`) to separate left and right sides.
- A command on one bus is received by both motors on that bus, so they do the same thing.

## 2. ROS Basics (No Prior Knowledge)

If you are new to ROS, these are the only concepts you must understand to use this project.

### 2.1 Node

A node is a program process that does one job.

Examples:

- `ros2_control_node` runs controllers.
- `goal_bridge` converts `/goal_pose` messages into a Nav2 action call.
- `foxglove_bridge` exposes ROS data to Foxglove.

### 2.2 Topic

A topic is a named message stream.

Examples:

- `/cmd_vel`: velocity command stream (`geometry_msgs/Twist`).
- `/odom`: odometry stream (`nav_msgs/Odometry`).
- `/goal_pose`: navigation goal input (`geometry_msgs/PoseStamped`).

### 2.3 Action

An action is for long-running tasks with feedback.

Example:

- Nav2 `NavigateToPose` action:
  - You send a goal.
  - You receive progress.
  - You receive success/fail status.

### 2.4 TF (Transforms)

TF defines coordinate frames and how they relate.

Core frames in this stack:

- `base_link`: robot body frame.
- `odom`: local drifting frame.
- `map`: global planning frame.

For first milestone, `map -> odom` is identity (0 offset), so map coordinates are effectively local coordinates.

### 2.5 Launch File

A launch file starts multiple nodes with parameters/remaps.

Example:

- `navigation_bringup.launch.py` starts controller stack + Nav2 + bridges + optional Foxglove.

## 3. Project Layout

Workspace root:

- `src/cubemars_ak_hardware/`: custom ROS 2 hardware plugin for CubeMars over SocketCAN.
- `src/rover_description/`: robot URDF/Xacro and `ros2_control` joint definitions.
- `src/rover_bringup/`: base launch files and controller YAML.
- `src/rover_autonomy/`: teleop/nav/foxglove/slam-gnss orchestration and helper nodes.

Key files for autonomy flow:

- `src/rover_autonomy/launch/teleop_bringup.launch.py`
- `src/rover_autonomy/launch/navigation_bringup.launch.py`
- `src/rover_autonomy/launch/navigation_with_obstacles.launch.py`
- `src/rover_autonomy/launch/navigation_with_slam_gnss.launch.py`
- `src/rover_autonomy/config/twist_mux.yaml`
- `src/rover_autonomy/config/nav2_no_obstacles.yaml`
- `src/rover_autonomy/config/nav2_no_obstacles_zed.yaml`
- `src/rover_autonomy/config/nav2_with_voxel_obstacles.yaml`
- `src/rover_autonomy/config/nav2_with_voxel_obstacles_zed.yaml`
- `src/rover_autonomy/rover_autonomy/goal_bridge.py`
- `src/rover_autonomy/rover_autonomy/nav_cmd_bridge.py`
- `src/rover_autonomy/rover_autonomy/odom_relay.py`
- `src/rover_autonomy/config/foxglove_layout_nav.json`
- `src/rover_autonomy/config/foxglove_layout_nav_teleop_virtual_joystick.json`

## 4. Architecture and Data Flow

### 4.1 Control path

1. Teleop or Nav2 publishes velocity command.
2. `twist_mux` chooses active command source.
3. Output goes to `/diff_drive_controller/cmd_vel`.
4. Diff drive controller computes left/right wheel targets.
5. `cubemars_ak_hardware` converts wheel rad/s to motor ERPM and sends CAN frame.

### 4.2 Navigation path

1. User publishes `PoseStamped` to `/goal_pose`.
2. `goal_bridge` sends `NavigateToPose` action request.
3. Nav2 planners/controllers compute path and command velocities.
4. `nav_cmd_bridge` routes Nav2 `/cmd_vel` to `/cmd_vel_nav`.
5. `twist_mux` forwards to base controller.

### 4.3 Visualization path

1. ROS topics + TF are exposed through `foxglove_bridge`.
2. Foxglove shows:
   - robot pose
   - odometry
   - goal
   - path
   - costmaps (when enabled)

## 5. Hardware Design Constraints (Very Important)

### 5.1 Why two CAN buses are used

Because current motors are restricted to same CAN ID, you split physical system into two buses:

- Left side motors on one bus (`can0`)
- Right side motors on second bus (`can1`)

This lets you command left and right sides independently, even with duplicate IDs on each side.

### 5.2 What this implies

- Within a single bus, motors sharing ID cannot be individually addressed.
- Both motors on the same bus execute the same command.
- Rear wheels are modeled as mimic joints for kinematics/visualization.

### 5.3 Why open-loop is used

For duplicate-ID conditions, feedback ambiguity can happen.

So this stack uses open-loop options in critical places:

- Hardware `open_loop_feedback` for same-ID URDF mode.
- Diff drive `open_loop: true` in same-ID controller mode.

This is a practical compromise until unique IDs per motor are available.

## 6. Prerequisites

## 6.1 OS and ROS baseline

Recommended baseline for this project:

- Ubuntu 22.04
- ROS 2 Humble

For Jetson Orin target:

- Prefer JetPack/Ubuntu combination compatible with ROS 2 Humble deployment strategy.
- Keep same ROS distro and package versions between dev VM and Jetson as much as possible.

## 6.2 Required ROS packages

After ROS 2 Humble is installed, install runtime dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-twist-mux \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-foxglove-bridge \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox
```

ZED2i:

1. Install ZED SDK from Stereolabs.
2. Install ROS 2 wrapper package:

```bash
sudo apt install -y ros-humble-zed-ros2-wrapper
```

## 7. Build and Run This Workspace

From workspace root:

```bash
cd /home/parallels/rover_ws
colcon build --packages-select cubemars_ak_hardware rover_description rover_bringup rover_autonomy
source install/setup.bash
```

Tip:

- Add `source /home/parallels/rover_ws/install/setup.bash` to your shell startup once stable.

## 8. Operating Modes

### 8.1 Teleop mode

```bash
ros2 launch rover_autonomy teleop_bringup.launch.py
```

What starts:

- Base control (`four_wheel.launch.py`)
- Joystick input (`joy_node`)
- Teleop conversion (`teleop_twist_joy` -> `/cmd_vel_teleop`)
- `twist_mux` -> `/diff_drive_controller/cmd_vel`
- `foxglove_bridge` (if enabled)

### 8.2 Navigation mode without ZED (bench test)

```bash
ros2 launch rover_autonomy navigation_bringup.launch.py use_zed_odom:=false
```

Use this for wheels-off-ground dry testing and bringup.

### 8.3 Navigation mode with ZED odometry

```bash
ros2 launch rover_autonomy navigation_bringup.launch.py \
  use_zed_odom:=true \
  zed_odom_topic:=/zed/zed_node/odom
```

What changes when `use_zed_odom:=true`:

- Base controller config with `enable_odom_tf: false` is used to avoid duplicate odom TF.
- `odom_relay` normalizes ZED odom to `/zed_odom` and publishes TF.
- Nav2 configs that use `/zed_odom` are selected.

### 8.4 Navigation with obstacle configs

```bash
ros2 launch rover_autonomy navigation_with_obstacles.launch.py use_zed_odom:=true
```

This enables voxel-layer costmaps fed from ZED point cloud config files.

### 8.5 Future SLAM + GNSS scaffold

```bash
ros2 launch rover_autonomy navigation_with_slam_gnss.launch.py \
  enable_slam:=true \
  enable_gnss_fusion:=true
```

This is a scaffold launch for future integration and tuning.

## 9. Sending a Goal (Coordinate Navigation)

### 9.1 Goal message format

Topic: `/goal_pose` type `geometry_msgs/PoseStamped`

Must use frame `map`.

Example command:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
```

### 9.2 How the goal is executed

- `goal_bridge` listens on `/goal_pose`.
- It sends a `NavigateToPose` action to Nav2.
- It publishes status text on `/goal_bridge/status`.

## 10. Foxglove Setup

### 10.1 Start Foxglove bridge

It is already included in launch files when `use_foxglove:=true`.

### 10.2 Import layout

Use one of:

- `src/rover_autonomy/config/foxglove_layout_nav.json`
- `src/rover_autonomy/config/foxglove_layout_nav_teleop_virtual_joystick.json`

Open Foxglove:

1. Connect to live data source.
2. `Layouts -> Import from file...`
3. Select layout JSON.

Notes are in:

- `src/rover_autonomy/config/foxglove_layouts.md`

## 10.3 What to inspect in Foxglove

For navigation validation:

- TF tree (`map`, `odom`, `base_link`)
- `/odom` or `/zed_odom`
- `/goal_pose`
- `/plan` and `/local_plan`
- `/goal_bridge/status`

For obstacle mode:

- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/zed/zed_node/point_cloud/cloud_registered`

## 11. Wheels-Off-Ground Bench Testing Procedure

This is critical for safe early validation.

1. Lift rover so wheels spin free.
2. Start navigation in non-ZED mode:

```bash
ros2 launch rover_autonomy navigation_bringup.launch.py use_zed_odom:=false
```

3. Verify topic flow:

```bash
ros2 topic echo /diff_drive_controller/cmd_vel
ros2 topic echo /goal_bridge/status
```

4. Send small goal near origin:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
```

5. Confirm:

- Path gets generated.
- Commands are published.
- Wheels spin in expected directions.

6. Test stop behavior:

- Send zero teleop command or cancel goal.
- Confirm wheels stop quickly.

## 12. From-Scratch Reimplementation Blueprint

If you had to rebuild this stack from an empty workspace, follow this order.

### 12.1 Create workspace and packages

```bash
mkdir -p ~/rover_ws/src
cd ~/rover_ws/src

ros2 pkg create --build-type ament_cmake rover_description
ros2 pkg create --build-type ament_cmake rover_bringup
ros2 pkg create --build-type ament_cmake cubemars_ak_hardware --dependencies rclcpp hardware_interface pluginlib
ros2 pkg create --build-type ament_python rover_autonomy --dependencies rclpy geometry_msgs nav_msgs nav2_msgs tf2_ros action_msgs std_msgs
```

### 12.2 Implement hardware plugin package

Package: `cubemars_ak_hardware`

Essential components:

1. SocketCAN wrapper.
2. `SystemInterface` plugin implementing:
   - `on_init`
   - `on_activate` / `on_deactivate`
   - `read`
   - `write`
3. pluginlib export file.
4. CMake install/export for plugin.

Required behavior:

- Command interface: wheel velocity in rad/s.
- Convert to ERPM using direction, gear ratio, pole pairs.
- Send CAN extended frame in speed mode with `(mode<<8)|can_id`.
- Support open-loop feedback mode for duplicate-ID limitations.

### 12.3 Implement robot description package

Package: `rover_description`

Essential files:

- `simple_4wd_rover_sameid_2bus.urdf.xacro`

Required design:

- 4-wheel geometry with fixed wheel joints.
- Two `ros2_control` hardware instances:
  - left bus on `can0`
  - right bus on `can1`
- One actuated wheel joint per side (same ID `111`).
- Rear wheels mimic front wheels.

### 12.4 Implement bringup package

Package: `rover_bringup`

Essential files:

- `launch/four_wheel.launch.py`
- `config/four_wheel_sameid_diff_controllers.yaml`
- `config/joy_teleop.yaml`
- `launch/joy_teleop.launch.py`

Required behavior:

- Start `robot_state_publisher` + `ros2_control_node` + controller spawners.
- Diff drive controller in open-loop mode for same-ID setup.
- Teleop publishes velocity commands.

### 12.5 Implement autonomy package

Package: `rover_autonomy`

Essential launch files:

- `teleop_bringup.launch.py`
- `navigation_bringup.launch.py`
- `navigation_with_obstacles.launch.py`
- `navigation_with_slam_gnss.launch.py`

Essential helper nodes:

- `nav_cmd_bridge.py`
- `goal_bridge.py`
- `odom_relay.py`

Essential configs:

- `twist_mux.yaml`
- Nav2 parameter files for no-obstacle and obstacle modes.
- EKF/navsat configs for future fusion.
- Foxglove layout JSON files.

### 12.6 Build and validate in stages

1. Build hardware + description + bringup.
2. Validate teleop.
3. Add Nav2 no-obstacle mode.
4. Validate goal-to-motion loop.
5. Add Foxglove and verify path/odom/goal visualization.
6. Add ZED odom integration.
7. Add obstacle layers.
8. Add SLAM + GNSS fusion.

## 13. Why `twist_mux` Is Used

Without muxing, you must hard-switch command remaps per launch mode.

With `twist_mux`:

- Teleop can be high priority.
- Nav can be lower priority.
- Emergency lock can force stop.

In this stack:

- `/cmd_vel_teleop` priority 100
- `/cmd_vel_nav` priority 50
- Optional lock `/cmd_vel_lock` priority 255

## 14. Coordinate Frames and Goal Semantics

You currently operate with local planning coordinates.

- Goal frame required: `map`
- In first-stage mode, `map -> odom` is static identity.

That means:

- A goal at `(x=3, y=1)` means 3 m forward and 1 m left in your local map frame at startup.

Later with SLAM/GNSS:

- `map` becomes globally meaningful and drift-corrected.

## 15. Obstacle Avoidance Later: Exact Upgrade Path

Current no-obstacle mode is for fast first success.

When enabling avoidance:

1. Use `navigation_with_obstacles.launch.py`.
2. Feed ZED point cloud into Nav2 voxel layer.
3. Tune key parameters:
   - obstacle min/max range
   - max obstacle height
   - inflation radius
   - local costmap size/resolution
4. Validate in Foxglove:
   - point cloud alignment with `base_link`
   - obstacle marking and clearing
   - planner routing around obstacles

## 16. SLAM + GNSS + Odometry Fusion Roadmap

Target URC architecture:

1. Local EKF (`robot_localization`):
   - wheel odom + IMU (+ optional VIO)
   - output stable local odom
2. `navsat_transform_node`:
   - transforms GPS to robot-local context
3. Global EKF:
   - fuses local odom + transformed GPS
   - outputs `map -> odom`
4. SLAM:
   - adds map consistency and loop closure where appropriate

Scaffold files already present:

- `src/rover_autonomy/config/ekf_local.yaml`
- `src/rover_autonomy/config/ekf_global.yaml`
- `src/rover_autonomy/config/navsat_transform.yaml`
- `src/rover_autonomy/launch/navigation_with_slam_gnss.launch.py`

## 17. Safety and Reliability Checklist

Before ground tests:

1. Verify CAN wiring and motor directions with tiny commands.
2. Confirm emergency stop path exists and works.
3. Set conservative max speeds in Nav2 and teleop.
4. Verify command timeout behavior.
5. Test goal cancel.

Before autonomous field tests:

1. Validate TF tree consistency.
2. Validate odometry quality (drift and jump checks).
3. Run repeated short missions.
4. Monitor controller saturation and dropped messages.

## 18. Common Problems and Fixes

### 18.1 Nav2 does not move

Check:

- Is goal frame `map`?
- Is `/goal_bridge/status` reporting action server available?
- Is `/diff_drive_controller/cmd_vel` receiving commands?
- Is `twist_mux` output connected correctly?

### 18.2 Robot spins wrong direction

Check:

- `direction` parameter in URDF joint config.
- Left/right wheel ordering in diff drive config.

### 18.3 Duplicate TF warnings

Check:

- In ZED odom mode, use controller config with `enable_odom_tf: false`.
- Ensure only one source publishes `odom -> base_link`.

### 18.4 Foxglove not showing data

Check:

- `foxglove_bridge` node running.
- Correct connection URL/session in Foxglove.
- Correct topic names in layout and runtime.

## 19. Jetson Orin Deployment Notes

This dev VM is for development speed. Jetson is the runtime target.

Recommended deployment approach:

1. Keep same package versions and launch files on both machines.
2. Build natively on Jetson for architecture compatibility.
3. Use conservative CPU/GPU budget first, then optimize.
4. Log all mission topics for post-run debugging.

Minimum deployment checklist:

1. Install ROS dependencies on Jetson.
2. Install ZED SDK + wrapper on Jetson.
3. Copy workspace and build on Jetson.
4. Validate teleop mode first.
5. Validate no-obstacle nav mode.
6. Enable obstacle mode after baseline stability.

## 20. Quick Command Reference

Build:

```bash
cd /home/parallels/rover_ws
colcon build --packages-select cubemars_ak_hardware rover_description rover_bringup rover_autonomy
source install/setup.bash
```

Teleop:

```bash
ros2 launch rover_autonomy teleop_bringup.launch.py
```

Navigation without ZED:

```bash
ros2 launch rover_autonomy navigation_bringup.launch.py use_zed_odom:=false
```

Navigation with ZED odom:

```bash
ros2 launch rover_autonomy navigation_bringup.launch.py use_zed_odom:=true zed_odom_topic:=/zed/zed_node/odom
```

Navigation with obstacle config:

```bash
ros2 launch rover_autonomy navigation_with_obstacles.launch.py use_zed_odom:=true
```

Goal publish example:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
```

## 21. What To Do Next

After this baseline is stable, do this order:

1. Tune controller gains and velocity limits for safe tracking.
2. Validate ZED odometry quality outdoors.
3. Enable voxel obstacle mode and tune costmaps.
4. Add SLAM map workflow for repeatable routes.
5. Integrate GNSS fusion for larger URC courses.

