# maze_explorer

Drives the bot through the maze and runs slam_toolbox to build the occupancy
map. Two driving modes live here:

- `path_executor` — deterministic. Knows the maze layout (same seed as the
  generator), plans A\* on the cell grid, drives cell by cell to the goal.
  This is what `full_demo.launch.py` uses because it's reproducible and
  doesn't get stuck.

- `frontier_explorer` — autonomous exploration. Subscribes to the slam map,
  finds frontiers (free cells bordering unknown), drives there, repeats.
  Simpler reactive control, occasionally needs a recovery dance. Use this
  when you want to see slam-only navigation without prior knowledge of the
  maze.

## config

`config/slam_async.yaml` — slam_toolbox tuned for the indoor maze:
- `base_frame: base_link`
- `resolution: 0.05` (5 cm grid cells)
- `max_laser_range: 8.0`
- looped closure on, tight thresholds for small environments

## tf chain

```
map  ── slam_toolbox ──►  odom  ── diff_drive ──►  base_link
                                                       │
                                                       └── caster
                                                       └── lidar_link
                                                       └── left_wheel
                                                       └── right_wheel
```

`map → odom` comes from slam, `odom → base_link` from the gz diff-drive
plugin, the rest is published by `robot_state_publisher` from the URDF.
