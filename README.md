# maze robotik

simulated robot solves a maze. ros 2 jazzy + gazebo harmonic + slam_toolbox.

bot spawns at the start, drives cell-by-cell to the goal via A\*, slam_toolbox maps live with the lidar. once at the goal you call a service that runs 4 pathfinding algos on the slam map and prints a comparison.

algos: **wall-follower** (fails on multi-path mazes), **BFS** (optimal, many expansions), **A\*** (optimal, fewer expansions), **flood-fill** (optimal, micromouse style).

## who does what

- `maze_worlds` — schayan — generator, urdf, gazebo world
- `maze_explorer` — bartolmay — slam, exploration, path executor
- `maze_planners` — michael — the 4 algos + benchmark + service

## setup

```bash
git clone https://github.com/kurzmichael02-hue/maze-robotik.git
cd maze-robotik

# lecturer's base image
git clone https://gitlab.com/MarkGeiger/robotik.git tmp
docker build -t ros2 tmp/exercises/aktuell/docker

# ours
docker build -t maze-robotik .
```

windows: install vcxsrv, start xlaunch with **disable access control** ticked.

## run

```powershell
.\docker-run.ps1     # windows, or ./docker-run.sh on linux
```

then inside the container:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_planners full_demo.launch.py
```

bot reaches the goal in ~2 min. for the algo comparison open a second powershell:

```bash
docker exec -it maze_robotik bash
source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash
ros2 service call /run_planners std_srvs/srv/Trigger
```

four colored paths in rviz + table in the terminal.

## sample output (25×25, seed 7)

```
wall_follower  len=536  exp=564   turns=330
bfs            len=200  exp=386   turns=125
astar          len=200  exp=342   turns=125
floodfill      len=200  exp=625   turns=125
```

bfs / a\* / flood-fill all return 200 — proof it's the shortest. a\* expands ~11% fewer cells than bfs. wall-follower needs 2.7× more steps and outright fails on mazes with islands. plots in `docs/benchmark/`.

## quick fixes

- `docker-run.ps1` blocked → `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass`
- daemon offline → restart docker desktop
- rviz says `Frame map does not exist` → wait ~10s for slam
- bot stuck → `docker rm -f maze_robotik` and start fresh
