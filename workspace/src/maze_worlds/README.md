# maze_worlds

generiert random labyrinthe und exportiert sie als gazebo `.sdf` world files.

## todo
- [ ] generator funktioniert (recursive backtracker)
- [ ] mehrere schwierigkeitsgrade (10x10, 20x20, mit inseln)
- [ ] launch file zum starten der welt
- [ ] entry/exit markierung für planner

## test
```bash
python -m maze_worlds.generate_maze --size 10 --out worlds/maze_10.sdf
```
