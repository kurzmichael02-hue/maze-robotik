# maze robotik

simulierter roboter findet selbständig den kürzesten weg durch ein labyrinth.
ROS 2 jazzy + gazebo harmonic + slam_toolbox.

## was passiert

bot spawnt im start, kennt sein maze (gleiche seed wie generator), plant A* und fährt cell-by-cell zum ziel. parallel mappt slam_toolbox live mit dem lidar — die map wächst in rviz mit. nach ankunft am ziel triggert man einen service, der alle 4 wegfindungs-algos auf die gemappte karte loslässt und vergleicht.

vergleich der algos ist das wo's interessant wird:
- **wall-follower** scheitert in mazes mit mehreren wegen (loops)
- **BFS** findet den kürzesten weg, expandiert aber viel
- **A\*** mit manhattan-heuristik findet den selben weg, expandiert weniger
- **flood-fill** (micromouse-stil) ist optimal und re-planning-freundlich

## aufteilung

| package | wer | inhalt |
|---|---|---|
| `maze_worlds` | schayan | prozeduraler maze-generator, urdf + lidar, gazebo welt |
| `maze_explorer` | bartolmay | slam config, frontier exploration, path executor |
| `maze_planners` | michael | die 4 algos, benchmark, ros2 service für vergleich |

## setup

einmalig:

```bash
git clone https://github.com/kurzmichael02-hue/maze-robotik.git
cd maze-robotik

# base image vom dozent (~5min)
git clone https://gitlab.com/MarkGeiger/robotik.git tmp
docker build -t ros2 tmp/exercises/aktuell/docker

# unser image
docker build -t maze-robotik .
```

windows: xlaunch starten (multiple windows, start no client, **disable access control** ankreuzen). linux braucht das nicht.

## jedes mal

```powershell
# windows
.\docker-run.ps1

# linux
./docker-run.sh
```

im container:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_planners full_demo.launch.py
```

gazebo + rviz gehen auf. bot fährt nach paar sekunden autonom los, mappt während er fährt, ist nach ~2min am ziel.

für den algo-vergleich in einem zweiten powershell:

```bash
docker exec -it maze_robotik bash
source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash
ros2 service call /run_planners std_srvs/srv/Trigger
```

→ vier farbige linien in rviz + zahlen-tabelle im terminal.

## ergebnisse

beispiel-output auf einem 25×25 maze, seed=7:

```
wall_follower  len=536  exp=564   turns=330
bfs            len=200  exp=386   turns=125
astar          len=200  exp=342   turns=125
floodfill      len=200  exp=625   turns=125
```

BFS / A* / flood-fill finden alle die selbe optimale länge — das ist der mathematische beweis dass 200 der kürzeste ist. A* expandiert ~11% weniger zellen als BFS (heuristik wirkt). wall-follower braucht 2.7× so viele schritte und scheitert ganz in mazes mit inseln.

plots + csv: `docs/benchmark/`.

## offline benchmark

ohne ros, nur python:

```bash
cd workspace/src/maze_planners
python3 -m maze_planners.benchmark --sizes 5,10,15,20,25 --seeds 8 --plot --out results/
python3 -m maze_planners.visualize --size 15 --seed 7 --algo astar --out astar.png
```

## struktur

```
.
├── Dockerfile
├── docker-run.{ps1,sh}
├── workspace/
│   └── src/
│       ├── maze_worlds/      maze + bot + gazebo
│       ├── maze_explorer/    slam + autonomes fahren
│       └── maze_planners/    algos + benchmark + service
└── docs/benchmark/           plots + csv
```

## troubleshooting

| problem | fix |
|---|---|
| `docker-run.ps1` blocked | `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass` |
| docker daemon offline | docker desktop neu starten |
| rviz `Frame map does not exist` | 10 sek warten, slam initialisiert noch |
| bot bewegt sich nicht | `docker rm -f maze_robotik` + neu starten |
