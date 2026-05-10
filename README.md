# Maze Robotik

Roboter findet selbstständig den Weg durch ein Labyrinth (Simulation).

## idee

bot fährt in unbekanntes maze, mappt mit lidar + slam, sucht danach den kürzesten weg. wir vergleichen mehrere pfadalgorithmen.

## stack

- ROS 2 Jazzy
- Gazebo Harmonic
- SLAM Toolbox
- Nav2

## aufteilung

| package | wer | inhalt |
|---|---|---|
| `maze_worlds` | schayan | maze-generator (python -> .sdf), gazebo welten, verschiedene größen |
| `maze_explorer` | bartolmay | frontier exploration, slam config, "fertig"-erkennung |
| `maze_planners` | michael | wall-follower / BFS / A* / flood-fill, benchmark, plots |

## setup

### voraussetzung
- docker desktop (windows/mac) oder docker engine (linux)
- **windows**: zusätzlich VcXsrv für gui (https://sourceforge.net/projects/vcxsrv/)
  - xlaunch -> multiple windows -> start no client -> **disable access control** ✓

### einmalig
```
# windows
.\docker-build.ps1

# linux
./docker-build.sh
```
dauert ~5 min. baut ros2 + gazebo + slam + nav2 ins image.

### jedes mal
```
# windows
.\docker-run.ps1

# linux
./docker-run.sh
```
öffnet bash im container. workspace ist gemounted, deine änderungen bleiben.

## test ob alles tut

im container:
```
cd /root/ros2_ws/src/maze_worlds
python3 -m maze_worlds.generate_maze --size 10 --seed 7 --out worlds/maze_demo.sdf
gz sim -v 4 -r worlds/maze_demo.sdf
```

→ gazebo geht auf, du siehst ein 10x10 maze mit grünem start- und rotem ziel-marker.

## struktur

```
.
├── Dockerfile
├── docker-build.{ps1,sh}
├── docker-run.{ps1,sh}
└── workspace/
    └── src/
        ├── maze_worlds/      <- maze-generierung + worlds
        ├── maze_explorer/    <- slam + frontier
        └── maze_planners/    <- wegfindung + benchmark
```
