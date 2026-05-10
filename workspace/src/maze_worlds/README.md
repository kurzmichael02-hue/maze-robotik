# maze_worlds

generiert random labyrinthe und exportiert sie als gazebo `.sdf` world files.
inkl. den maze_bot (diff-drive + lidar) direkt in die welt.

## features
- recursive backtracker maze generator (deterministisch via seed)
- 3 schwierigkeitsgrade: easy/medium/hard
- multi-path mode (extra durchgaenge -> mehrere wege von start zu ziel)
- bot direkt in welt eingebaut, kein extra spawn-step
- start + ziel marker (gruen / rot mit glow)

## verwendung

**maze generieren:**
```bash
# medium (default) - ~20% extra passages
python3 -m maze_worlds.generate_maze --size 10 --seed 7 --out worlds/maze.sdf

# easy - mehr abkuerzungen
python3 -m maze_worlds.generate_maze --size 10 --seed 7 --difficulty easy --out worlds/easy.sdf

# hard - perfect maze (genau ein weg)
python3 -m maze_worlds.generate_maze --size 15 --seed 42 --difficulty hard --out worlds/hard.sdf
```

**laden in gazebo:**
```bash
ros2 launch maze_worlds maze_world.launch.py world:=/path/to/maze.sdf
```

oder via full demo (maze + slam + planner zusammen):
```bash
ros2 launch maze_planners full_demo.launch.py
```

## struktur
```
maze_worlds/
├── maze_worlds/
│   └── generate_maze.py    # generator + bot-sdf inline
├── urdf/maze_bot.urdf.xacro  # urdf fuer rviz robot model
├── config/bridge.yaml        # ros<->gz topic bridge
├── launch/maze_world.launch.py
└── worlds/maze.sdf           # generated world
```

## difficulty zahlen (10x10 seed=7)
- easy:   ~85 walls (viele wege)
- medium: ~105 walls (mehrere wege)
- hard:   ~121 walls (genau 1 weg)
