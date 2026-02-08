# Foxglove Layouts

These layout files are intended for this rover stack:

- `foxglove_layout_nav.json`
- `foxglove_layout_nav_teleop_virtual_joystick.json`

## Import

1. Open Foxglove.
2. Connect to the rover data source (`foxglove_bridge`).
3. `Layouts` -> `Import from file...`
4. Select one of the JSON files above.

## Notes

- `foxglove_layout_nav.json` is extension-free and focuses on:
  - 3D visualization
  - ZED2i RGB image + depth map panels
  - `/goal_pose` publishing from 3D panel tools
  - `/plan` + `/local_plan` visualization
  - odometry and status inspection
- `foxglove_layout_nav_teleop_virtual_joystick.json` now uses the built-in `Teleop` panel (extension-free) and includes ZED2i RGB + depth map panels.
- If your ZED odometry topic differs, update topics in the 3D panel settings after import.
- The layouts default to current ZED ROS 2 wrapper RGB naming (`/zed/zed_node/rgb/color/rect/image`). If you are on an older wrapper, switch to legacy RGB topic (`/zed/zed_node/rgb/image_rect_color`) in the `Image` panel settings after import.
- For obstacle avoidance sessions, set `/global_costmap/costmap` and `/local_costmap/costmap` visible in the 3D panel topic list.
