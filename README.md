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

## komponenten

- **maze_worlds** — labyrinth-generator + roboter (urdf mit lidar) + gazebo welt — *schayan*
- **maze_explorer** — slam_toolbox config + der controller der den bot fährt — *bartolmay*
- **maze_planners** — die 4 wegfindungs-algos (wall-follower, bfs, a*, flood-fill) + vergleichs-service + benchmark — *michael*
- **docker** — kapselt ros2 jazzy + gazebo + slam, damit's bei allen gleich läuft

## benutzung

braucht docker desktop. unter windows zusätzlich vcxsrv für die grafik (xlaunch starten, "disable access control" anhaken).

**einmalig** das image bauen. wir bauen auf dem ros2-image vom dozent auf, das muss also zuerst da sein:

```bash
git clone https://gitlab.com/MarkGeiger/robotik.git tmp
docker build -t ros2 tmp/exercises/aktuell/docker
docker build -t maze-robotik .
```

dauert beim ersten mal so 10 min (lädt ros2 + gazebo runter).

**jedes mal** zum starten:

```bash
.\docker-run.ps1          # windows, bzw ./docker-run.sh auf linux
```

man landet dann im container. dort:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch maze_planners full_demo.launch.py
```

gazebo + rviz gehen auf, der bot fährt von selbst los und ist nach ~2 min am ziel.

den algo-vergleich startet man in nem **zweiten terminal** (erstes ist ja vom launch belegt):

```bash
docker exec -it maze_robotik bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 service call /run_planners std_srvs/srv/Trigger
```

dann erscheinen die 4 pfade als farbige linien in rviz + die tabelle im terminal.

eigenes labyrinth generieren geht über den generator direkt:

```bash
ros2 run maze_worlds generate_maze --size 12 --seed 3 --difficulty easy
```

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
