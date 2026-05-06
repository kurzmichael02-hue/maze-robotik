"""flood-fill (micromouse style).
schritt 1: distance map vom goal aus rückwärts (BFS).
schritt 2: vom start aus dem gradient folgen (immer richtung niedrigerer distanz).

der vorteil gegenüber BFS: distance map ist re-usable wenn der bot
unterwegs neue infos kriegt (dynamic re-planning). hier nutzen wir nur
schritt 1 zum optimalen pfad finden.
"""
import time
from collections import deque
from .base import Planner, PlanResult


class FloodFill(Planner):
    name = 'floodfill'

    def plan(self, grid, start=None, goal=None):
        t0 = time.perf_counter()
        start = start or grid.start
        goal = goal or grid.goal

        # 1. distance map vom goal aus
        dist = {goal: 0}
        q = deque([goal])
        expanded = 0
        while q:
            cur = q.popleft()
            expanded += 1
            for nx, ny, _d in grid.neighbors(*cur):
                if (nx, ny) not in dist:
                    dist[(nx, ny)] = dist[cur] + 1
                    q.append((nx, ny))

        if start not in dist:
            t1 = time.perf_counter()
            return PlanResult(expanded=expanded,
                              runtime_ms=(t1 - t0) * 1000.0, success=False)

        # 2. von start dem gradient folgen
        path = [start]
        cur = start
        while cur != goal:
            best = None
            best_d = dist[cur]
            for nx, ny, _ in grid.neighbors(*cur):
                if dist.get((nx, ny), float('inf')) < best_d:
                    best = (nx, ny)
                    best_d = dist[(nx, ny)]
            if best is None:
                # festsitzen, sollte nicht passieren wenn dist map stimmt
                break
            cur = best
            path.append(cur)

        t1 = time.perf_counter()
        return PlanResult(path=path, expanded=expanded,
                          runtime_ms=(t1 - t0) * 1000.0,
                          success=(path[-1] == goal))
