# Maze Robotik

Autonomer roboter findet den optimalen weg durch ein labyrinth.
Simulation in **ROS 2 Jazzy** + **Gazebo Harmonic** + **SLAM Toolbox**.

![Status](https://img.shields.io/badge/status-running-green)
![ROS](https://img.shields.io/badge/ROS_2-Jazzy-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)

---

## was das projekt macht

1. **labyrinth** wird prozedural generiert (8×8 zellen, mehrere wege)
2. **roboter** mit lidar spawnt im start (grüner marker)
3. **slam_toolbox** mappt während der bot fährt (live in rviz)
4. **path_executor** plant A* von start zu ziel und fährt deterministisch hin
5. **planner_node** vergleicht alle 4 wegfindungs-algos auf der gemappten karte:
   - **wall-follower** (links-hand-regel) → scheitert bei multi-path
   - **BFS** → findet kürzesten weg, viele expansions
   - **A*** → findet kürzesten weg, weniger expansions (smart)
   - **flood-fill** → micromouse-stil, optimal
6. ergebnisse als rote linien in rviz + zahlen-tabelle

---

## quickstart

**voraussetzungen:** docker desktop, vcxsrv (windows) oder X11 (linux), git.

```bash
# 1. repo holen
git clone https://github.com/kurzmichael02-hue/maze-robotik.git
cd maze-robotik

# 2. base image vom dozent bauen (einmalig, ~5min)
git clone https://gitlab.com/MarkGeiger/robotik.git tmp_dozent
docker build -t ros2 tmp_dozent/exercises/aktuell/docker

# 3. unser image (~3min)
docker build -t maze-robotik .
```

**windows:** xlaunch starten → multiple windows → start no client → **disable access control ✓**

**container starten:**
```powershell
.\docker-run.ps1     # windows
./docker-run.sh      # linux
```

**im container** (jedes mal):
```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_planners full_demo.launch.py
```

→ gazebo + rviz öffnen, bot fährt autonom los, erreicht ziel nach ~2min.

**zum planner-vergleich** (parallel im 2. fenster):
```bash
docker exec -it maze_robotik bash
source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash
ros2 service call /run_planners std_srvs/srv/Trigger
```

---

## architektur

```
┌─ maze_worlds ─────────────────┐    ┌─ maze_explorer ──────────────┐
│ generate_maze.py (procedural) │    │ path_executor.py  (drives bot)│
│ urdf + lidar + diff_drive     │    │ frontier_explorer.py (slam)   │
│ → maze.sdf with bot inside    │    │ slam_toolbox config           │
└────────────┬──────────────────┘    └──────────────┬───────────────┘
             │                                       │
             v                                       v
       ┌──────────────┐    /scan, /odom    ┌─────────────────┐
       │   gazebo     │ ◄─────────────────►│  ros 2 graph    │
       │  (physics)   │     /cmd_vel        │  (slam, rviz)   │
       └──────────────┘                     └────────┬────────┘
                                                     │
                                              /map   v
                                          ┌─ maze_planners ─────┐
                                          │ planner_node        │
                                          │  ├─ wall_follower   │
                                          │  ├─ BFS             │
                                          │  ├─ A*              │
                                          │  └─ flood-fill      │
                                          │ benchmark.py        │
                                          │ visualize.py        │
                                          └─────────────────────┘
```

## packages

| package | inhalt | wer |
|---|---|---|
| **maze_worlds** | maze-generator, urdf, gazebo welt, ros↔gz bridge | schayan |
| **maze_explorer** | autonomes fahren (path_executor, frontier_explorer), slam config | bartolmay |
| **maze_planners** | 4 wegfindungs-algos, ros2 service node, benchmark + plots | michael |

---

## doku

- [`docs/ALGOS.md`](docs/ALGOS.md) — mathematische erklärung jedes algos (für die mündliche prüfung)
- [`docs/benchmark/benchmark.png`](docs/benchmark/benchmark.png) — vergleichs-plots auf 5×5 bis 25×25 mazes
- [`docs/benchmark/benchmark.csv`](docs/benchmark/benchmark.csv) — rohdaten

## ergebnisse (8×8 maze, seed=7)

| algo | weg-länge | expansions | drehungen | erfolg |
|---|---|---|---|---|
| **A\*** | 14 | 54 | 1 | ✅ optimal + 16% smarter als BFS |
| **BFS** | 14 | 64 | 1 | ✅ optimal |
| **flood-fill** | 14 | 64 | 1 | ✅ optimal |
| **wall-follower** | 512 | 512 | 511 | ❌ scheitert (multi-path) |

→ wall-follower bricht in mazes mit mehreren wegen, alle anderen finden den optimum mit unterschiedlicher effizienz.

---

## standalone benchmark (offline, ohne ros)

```bash
cd workspace/src/maze_planners
python3 -m maze_planners.benchmark --sizes 5,10,15,20 --seeds 5 --plot --out results/
python3 -m maze_planners.visualize --size 12 --seed 7 --algo astar --out astar.png
```

generiert csv + plots im `results/` ordner.

---

## troubleshooting

| problem | fix |
|---|---|
| `.\docker-run.ps1` blocked | `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass` |
| `daemon not running` | docker desktop neu starten |
| `Frame map does not exist` in rviz | warten bis slam initialisiert (~10sek) |
| bot bewegt sich nicht | container fresh starten: `docker rm -f maze_robotik` |
