# maze_planners

Four pathfinding algorithms on a cell-based maze representation. Each one
implemented from scratch with the same `Planner.plan(grid, start, goal)`
interface so they're directly comparable.

## the algos

| algo | optimal? | typical expansions | uses heuristic? |
|---|---|---|---|
| wall_follower (left-hand rule) | no | sometimes O(n²), can loop | no |
| BFS | yes (unweighted graph) | many | no |
| A\* (manhattan) | yes | fewer than BFS | yes |
| flood-fill (micromouse) | yes | scans the whole grid once | no |

Why wall-follower can fail: it works in simply connected mazes (perfect
mazes, one route only). Once there's a loop or an inner island it can walk
around the same wall forever without ever reaching the goal.

## quickstart

Inside the container or any environment with python + matplotlib + pytest:

```bash
cd /root/ros2_ws/src/maze_planners

# visualize one algo on one maze
python3 -m maze_planners.visualize --size 10 --seed 7 --algo astar --out /tmp/astar.png

# full benchmark across sizes/seeds
python3 -m maze_planners.benchmark --sizes 5,10,15,20 --seeds 5 --plot --out /tmp/results/

# unit tests
pytest tests/
```

## ros2 service

The `planner_node` listens for an `/run_planners` trigger, takes the current
slam map, converts it to the cell grid, runs all four algos, publishes each
path as `nav_msgs/Path` and as line markers in rviz.

```bash
ros2 service call /run_planners std_srvs/srv/Trigger
```

Returns success + a message with one line per algo (length, expansions,
turns, runtime).

## layout

```
maze_planners/
├── grid.py                    GridMap representation (cells + walls)
├── algorithms/
│   ├── base.py                abstract Planner + PlanResult
│   ├── wall_follower.py
│   ├── bfs.py
│   ├── astar.py
│   └── floodfill.py
├── benchmark.py               sweep all algos over multiple sizes/seeds
├── visualize.py               render a single maze + path
├── map_converter.py           slam occupancy grid → cell GridMap
└── planner_node.py            ros2 service that runs all algos
```
