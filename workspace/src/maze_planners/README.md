# maze_planners

verschiedene wegfindungs-algos auf der vom explorer gemappten karte.

## algos
- [ ] **wall-follower** (left-hand-rule) - baseline, scheitert bei inseln
- [ ] **BFS** - kürzester pfad in zellen (unweighted)
- [ ] **A\*** - mit manhattan heuristic
- [ ] **flood-fill** - micromouse-stil, optimal für grids

## benchmark
csv mit:
- pfadlänge (zellen)
- fahrzeit (sek)
- anzahl drehungen
- energie (cmd_vel integral)

am ende: matplotlib plots zum vergleich.

## interface

jeder planner = ein ros2 node der hört auf `/map` (occupancy grid) und published auf `/planned_path` (nav_msgs/Path).
