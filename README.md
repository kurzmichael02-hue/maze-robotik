# maze robotik

simulierter roboter findet den kürzesten weg durchs labyrinth. ros 2 jazzy + gazebo harmonic + slam_toolbox.

```
   maze (gazebo)  ─►  lidar  ─►  slam_toolbox  ─►  /map
        ▲                                            │
        │                                            ▼
       bot  ◄─────────── /cmd_vel ◄──── path_executor (A*)
                                              │
                                              ▼
                                       planner_node  ──►  bfs / a* / floodfill / wall-follower
                                                                   │
                                                                   ▼
                                                              vergleich
```

bot fährt cell-by-cell zum ziel, slam mappt parallel. am ende vergleichen vier algos den weg auf der gemappten karte.

## wer macht was

- **schayan** — maze_worlds (generator, gazebo welt, urdf)
- **bartolmay** — maze_explorer (slam, controller)
- **michael** — maze_planners (die 4 algos + benchmark + service)

## setup

einmalig: docker desktop + (windows) vcxsrv. dann:

```bash
docker build -t ros2 .   # base image vom dozent in tmp/ vorher clonen
docker build -t maze-robotik .
```

starten: `.\docker-run.ps1` (oder `.sh`), drin:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_planners full_demo.launch.py
```

algo-vergleich im 2. terminal: `ros2 service call /run_planners std_srvs/srv/Trigger`

## zahlen (25×25 maze)

```
wall_follower  len=536  ← scheitert (loopt)
bfs            len=200
astar          len=200  exp=342  ← weniger als bfs
floodfill      len=200
```

alle smarten algos finden 200, a* mit weniger expansions. plots in `docs/benchmark/`.

## noch offen

- nav2 statt eigener controller (sauberer aber config-aufwand)
- benchmark auf 50×50 mazes (slam braucht länger zum mappen)
- evtl 5. algo: theta* oder D* lite
- texturen am maze noch n bisschen aufpolieren

## quick fixes

- powershell blockt das script → `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass`
- bot bleibt hängen → `docker rm -f maze_robotik` und neu starten
