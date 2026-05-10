"""breadth-first search. liefert kürzesten pfad in zellen (unweighted)."""
import time
from collections import deque
from .base import Planner, PlanResult


class BFS(Planner):
    name = 'bfs'

    def plan(self, grid, start=None, goal=None):
        t0 = time.perf_counter()
        start = start or grid.start
        goal = goal or grid.goal

        q = deque([start])
        came_from = {start: None}
        expanded = 0

        while q:
            cur = q.popleft()
            expanded += 1
            if cur == goal:
                # rekonstruieren
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = came_from[cur]
                path.reverse()
                t1 = time.perf_counter()
                return PlanResult(path=path, expanded=expanded,
                                  runtime_ms=(t1 - t0) * 1000.0, success=True)

            for nx, ny, _d in grid.neighbors(*cur):
                if (nx, ny) not in came_from:
                    came_from[(nx, ny)] = cur
                    q.append((nx, ny))

        t1 = time.perf_counter()
        return PlanResult(expanded=expanded, runtime_ms=(t1 - t0) * 1000.0, success=False)
