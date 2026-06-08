# maze_worlds

Procedural maze generator plus the bot itself, packaged as a Gazebo `.sdf` world.
The maze_bot (diff-drive + 360° lidar) is baked directly into the SDF so no
separate spawn step is needed.

## features
- recursive backtracker generator, deterministic via seed
- 3 difficulty levels (easy / medium / hard)
- multi-path mode adds extra passages so there are multiple routes start→goal
- start + goal markers (green / red with glow)

## generating a maze

```bash
# medium (default), ~20% extra passages
python3 -m maze_worlds.generate_maze --size 10 --seed 7 --out worlds/maze.sdf

# easy — more shortcuts
python3 -m maze_worlds.generate_maze --size 10 --seed 7 --difficulty easy --out worlds/easy.sdf

# hard — perfect maze, exactly one route
python3 -m maze_worlds.generate_maze --size 15 --seed 42 --difficulty hard --out worlds/hard.sdf
```

## launching

```bash
ros2 launch maze_worlds maze_world.launch.py world:=/path/to/maze.sdf
```

Or via the full demo (maze + slam + planner together):

```bash
ros2 launch maze_planners full_demo.launch.py
```

## layout

```
maze_worlds/
├── maze_worlds/generate_maze.py    generator + inline bot sdf
├── urdf/maze_bot.urdf.xacro        urdf for rviz robot model
├── config/bridge.yaml              ros<->gz topic bridge
├── launch/maze_world.launch.py
└── worlds/maze.sdf                 generated world
```

## difficulty numbers (10×10, seed 7)
- easy:   ~85 walls (many alternatives)
- medium: ~105 walls (several routes)
- hard:   ~121 walls (one route only)
"# maze generator" 
