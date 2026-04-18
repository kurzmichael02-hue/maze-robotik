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
| `maze_worlds` | person 1 | maze-generator (python -> .sdf), gazebo welten, verschiedene größen |
| `maze_explorer` | person 2 | frontier exploration, slam config, "fertig"-erkennung |
| `maze_planners` | person 3 | wall-follower / BFS / A* / flood-fill, benchmark, plots |

## setup

work in progress. derzeit: docker / WSL2 setup.

## struktur

```
.
├── Dockerfile
├── docker-build.ps1
├── docker-run.ps1
└── workspace/
    └── src/
        ├── maze_worlds/
        ├── maze_explorer/
        └── maze_planners/
```
