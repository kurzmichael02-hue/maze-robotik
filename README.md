# maze robotik

Simulated robot finds the shortest path through a maze.
ROS 2 Jazzy + Gazebo Harmonic + slam_toolbox.

## what it does

Bot spawns at the start, knows the maze layout (same seed as the generator), plans A\* on the cell grid and drives cell by cell to the goal. slam_toolbox runs in parallel and builds the map live with the lidar — you watch it grow in rviz. Once at the goal you call a service that runs four pathfinding algorithms on the slam-built map and prints a comparison.

The algos:
- **wall-follower** — fails when the maze has loops
- **BFS** — finds the shortest path, lots of expansions
- **A\*** with manhattan heuristic — same path, fewer expansions
- **flood-fill** (micromouse style) — optimal, good for replanning

## who does what

| package | owner | content |
|---|---|---|
| `maze_worlds` | schayan | maze generator, urdf, gazebo world |
| `maze_explorer` | bartolmay | slam config, exploration, path executor |
| `maze_planners` | michael | the 4 algos + benchmark + comparison service |

## setup (one time)

```bash
git clone https://github.com/kurzmichael02-hue/maze-robotik.git
cd maze-robotik

# lecturer's base image (~5min)
git clone https://gitlab.com/MarkGeiger/robotik.git tmp
docker build -t ros2 tmp/exercises/aktuell/docker

# ours
docker build -t maze-robotik .
```

Windows needs VcXsrv. Start XLaunch: multiple windows, start no client, **tick disable access control**.

## running

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

Gazebo and rviz open. Bot drives off after a few seconds, reaches the goal in ~2 min.

Algo comparison in a second powershell:

```bash
docker exec -it maze_robotik bash
source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash
ros2 service call /run_planners std_srvs/srv/Trigger
```

Four colored paths appear in rviz, plus the table in the terminal.

## sample output (25×25, seed 7)

```
wall_follower  len=536  exp=564   turns=330
bfs            len=200  exp=386   turns=125
astar          len=200  exp=342   turns=125
floodfill      len=200  exp=625   turns=125
```

BFS, A\* and flood-fill all return length 200 — that's the proof it's the shortest. A\* expands ~11% fewer cells than BFS. wall-follower takes 2.7× more steps and outright fails on mazes with islands.

Plots and csv: `docs/benchmark/`.

## offline benchmark (pure python, no ROS)

```bash
cd workspace/src/maze_planners
python3 -m maze_planners.benchmark --sizes 5,10,15,20,25 --seeds 8 --plot --out results/
python3 -m maze_planners.visualize --size 15 --seed 7 --algo astar --out astar.png
```

## troubleshooting

| problem | fix |
|---|---|
| `docker-run.ps1` blocked | `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass` |
| docker daemon offline | restart docker desktop |
| rviz says `Frame map does not exist` | wait ~10s for slam |
| bot won't move | `docker rm -f maze_robotik` and start fresh |
