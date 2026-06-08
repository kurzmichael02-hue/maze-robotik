# maze robotik

simulated robot finds the shortest path through a maze. ros 2 jazzy + gazebo harmonic + slam_toolbox.

```
  maze_worlds              maze_explorer              maze_planners
  ───────────              ─────────────              ─────────────
  gazebo maze world        slam_toolbox  ──► /map ──► bfs · a*
  bot + 360° lidar  ─scan─► builds the map            flood-fill
         ▲                  path_executor (a*)        wall-follower
         │                  drives the bot                  │
         └──── /cmd_vel ──────────┘                         ▼
                                                     compare the paths
                                                     on the slam map
```

the bot drives cell by cell to the goal while slam maps in parallel. at the end four algorithms compare their paths on the mapped grid.

## components

- **maze_worlds** — maze generator + robot (urdf with lidar) + gazebo world — *schayan*
- **maze_explorer** — slam_toolbox config + the controller that drives the bot — *bartolmay*
- **maze_planners** — the 4 pathfinding algos (wall-follower, bfs, a*, flood-fill) + comparison service + benchmark — *michael*
- **docker** — wraps ros2 jazzy + gazebo + slam so it runs the same for everyone

## usage

needs docker desktop. on windows also vcxsrv for the graphics (start xlaunch, tick "disable access control").

**once** build the image. we build on top of the lecturer's ros2 image, so that one has to exist first:

```bash
git clone https://gitlab.com/MarkGeiger/robotik.git tmp
docker build -t ros2 tmp/exercises/aktuell/docker
docker build -t maze-robotik .
```

first time takes about 10 min (downloads ros2 + gazebo).

**every time** to start:

```bash
.\docker-run.ps1          # windows, or ./docker-run.sh on linux
```

you land inside the container. there:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_planners full_demo.launch.py
```

gazebo + rviz open, the bot drives off on its own and reaches the goal after ~2 min.

run the algo comparison in a **second terminal** (the first one is busy with the launch):

```bash
docker exec -it maze_robotik bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 service call /run_planners std_srvs/srv/Trigger
```

the 4 paths then show up as colored lines in rviz + the table in the terminal.

generating your own maze goes straight through the generator:

```bash
ros2 run maze_worlds generate_maze --size 12 --seed 3 --difficulty easy
```

## numbers (25×25 maze)

```
wall_follower  len=536  ← fails (loops)
bfs            len=200
astar          len=200  exp=342  ← fewer than bfs
floodfill      len=200
```

all the smart algos find 200, a* with fewer expansions. plots in `docs/benchmark/`.

## quick fixes

- powershell blocks the script → `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass`
- bot gets stuck → `docker rm -f maze_robotik` and restart

## project

team project at dhbw mannheim, built over 3 months. graded 96/100. weekly meeting notes in [docs/meetings.md](docs/meetings.md).
