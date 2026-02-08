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
  - `/goal_pose` publishing from 3D panel tools
  - `/plan` + `/local_plan` visualization
  - odometry and status inspection
- `foxglove_layout_nav_teleop_virtual_joystick.json` includes a virtual joystick panel and expects that panel extension to be installed in Foxglove.
- If your ZED odometry topic differs, update topics in the 3D panel settings after import.
- For obstacle avoidance sessions, set `/global_costmap/costmap` and `/local_costmap/costmap` visible in the 3D panel topic list.
