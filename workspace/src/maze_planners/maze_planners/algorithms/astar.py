"""A* mit manhattan heuristic.
in 4-connected grids ist manhattan admissible -> optimal.
"""
import time
import heapq
from .base import Planner, PlanResult


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class AStar(Planner):
    name = 'astar'

    def __init__(self, heuristic=manhattan):
        self.heuristic = heuristic

    def plan(self, grid, start=None, goal=None):
        t0 = time.perf_counter()
        start = start or grid.start
        goal = goal or grid.goal

        # open set als min-heap: (f, counter, node)
        # counter um ties zu brechen (heapq vergleicht sonst tuples)
        counter = 0
        open_heap = [(self.heuristic(start, goal), counter, start)]
        came_from = {start: None}
        g_score = {start: 0}
        expanded = 0

        while open_heap:
            f, _, cur = heapq.heappop(open_heap)
            expanded += 1
            if cur == goal:
                path = []
                while cur is not None:
                    path.append(cur)
                    cur = came_from[cur]
                path.reverse()
                t1 = time.perf_counter()
                return PlanResult(path=path, expanded=expanded,
                                  runtime_ms=(t1 - t0) * 1000.0, success=True)

            for nx, ny, _d in grid.neighbors(*cur):
                tentative = g_score[cur] + 1
                if tentative < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = cur
                    g_score[(nx, ny)] = tentative
                    f_new = tentative + self.heuristic((nx, ny), goal)
                    counter += 1
                    heapq.heappush(open_heap, (f_new, counter, (nx, ny)))

        t1 = time.perf_counter()
        return PlanResult(expanded=expanded, runtime_ms=(t1 - t0) * 1000.0, success=False)
