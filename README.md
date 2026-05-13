# maze robotik

A simulated robot finds the shortest path through a maze.
Built on ROS 2 Jazzy + Gazebo Harmonic + slam_toolbox.

## what it does

The bot spawns at the start cell, knows the maze layout (same seed as generator), plans an A\* path and drives cell by cell to the goal. slam_toolbox runs in parallel and maps the maze live with the lidar — the map grows in rviz as the bot moves. Once the bot reaches the goal you call a ROS service, which runs all four pathfinding algorithms on the slam-built map and prints a comparison.

The algo comparison is the interesting part:
- **wall-follower** breaks on multi-path mazes (loops, islands)
- **BFS** finds the shortest path but expands a lot
- **A\*** with manhattan heuristic finds the same path with fewer expansions
- **flood-fill** (micromouse style) is optimal and replan-friendly

## who does what

| package | owner | content |
|---|---|---|
| `maze_worlds` | schayan | procedural maze generator, urdf + lidar, gazebo world |
| `maze_explorer` | bartolmay | slam config, frontier exploration, path executor |
| `maze_planners` | michael | the 4 algos, benchmark, ros2 comparison service |

## setup (one time)

```bash
git clone https://github.com/kurzmichael02-hue/maze-robotik.git
cd maze-robotik

# base image from the lecturer's repo (~5min)
git clone https://gitlab.com/MarkGeiger/robotik.git tmp
docker build -t ros2 tmp/exercises/aktuell/docker

# our image
docker build -t maze-robotik .
```

On Windows you also need VcXsrv. Start XLaunch: multiple windows, start no client, **tick disable access control**. Linux already has X11.

## running it

```powershell
.\docker-run.ps1     # windows
./docker-run.sh      # linux
```

Inside the container:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_planners full_demo.launch.py
```

Gazebo and rviz pop up. The bot starts driving autonomously after a few seconds, maps as it goes, and reaches the goal in roughly two minutes.

For the algo comparison, open a second powershell:

```bash
docker exec -it maze_robotik bash
source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash
ros2 service call /run_planners std_srvs/srv/Trigger
```

Four colored paths show up in rviz, plus the table in the terminal.

## sample output

25×25 maze, seed 7:

```
wall_follower  len=536  exp=564   turns=330
bfs            len=200  exp=386   turns=125
astar          len=200  exp=342   turns=125
floodfill      len=200  exp=625   turns=125
```

BFS, A\* and flood-fill all find length 200 — that's the proof it's the shortest. A\* expands ~11% fewer cells than BFS thanks to the heuristic. wall-follower takes 2.7× more steps and outright fails on mazes with islands.

Plots and csv: `docs/benchmark/`.

## offline benchmark

No ROS required, pure python:

```bash
cd workspace/src/maze_planners
python3 -m maze_planners.benchmark --sizes 5,10,15,20,25 --seeds 8 --plot --out results/
python3 -m maze_planners.visualize --size 15 --seed 7 --algo astar --out astar.png
```

## layout

```
.
├── Dockerfile
├── docker-run.{ps1,sh}
├── workspace/src/
│   ├── maze_worlds/      maze + bot + gazebo world
│   ├── maze_explorer/    slam + autonomous driving
│   └── maze_planners/    algos + benchmark + service
└── docs/benchmark/       plots + csv
```

## troubleshooting

| problem | fix |
|---|---|
| `docker-run.ps1` blocked | `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass` |
| docker daemon offline | restart docker desktop |
| rviz says `Frame map does not exist` | wait ~10s for slam to initialize |
| bot won't move | `docker rm -f maze_robotik` and start fresh |
