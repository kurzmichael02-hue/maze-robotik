# maze_planners

wegfindungs-algos auf der gemappten karte. 4 algos, alle vergleichbar gemacht.

## algos

| algo | optimal? | expansions | nutzt heuristik? |
|---|---|---|---|
| wall_follower (left-hand) | nein | ja im worst case | nein |
| BFS | ja (unweighted) | viel | nein |
| A* (manhattan) | ja | weniger | ja |
| flood-fill (micromouse) | ja | wie BFS, aber re-usable map | nein |

## quickstart

im container, mit installierter `matplotlib`:

```
pip install matplotlib pytest
cd /root/ros2_ws/src/maze_planners
```

**single algo visualisieren:**
```
python3 -m maze_planners.visualize --size 10 --seed 7 --algo astar --out /tmp/astar.png
```

**alle algos benchmarken:**
```
python3 -m maze_planners.benchmark --sizes 5,10,15,20 --seeds 5 --plot --out /tmp/results/
```
liefert csv + plots.

**tests:**
```
pytest tests/
```

## struktur

```
maze_planners/
├── grid.py                    # GridMap representation
├── algorithms/
│   ├── base.py                # abstract Planner + PlanResult
│   ├── wall_follower.py
│   ├── bfs.py
│   ├── astar.py
│   └── floodfill.py
├── benchmark.py               # alle algos auf verschiedenen mazes
└── visualize.py               # einzelnes maze + path zeichnen
```

## todo
- [ ] ros2 node wrapper (subscribe /map, publish /planned_path)
- [ ] integration mit nav2 actions (bot fährt den pfad)
- [ ] evtl 5. algo: theta\* oder D\* lite (innovation factor)
