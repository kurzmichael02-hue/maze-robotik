"""left-hand-rule wall follower.
bot bleibt mit linker hand an der wand. simpel, scheitert bei inseln.
"""
import time
from .base import Planner, PlanResult
from ..grid import DIRS, LEFT, RIGHT, OPP


class WallFollower(Planner):
    name = 'wall_follower'

    def plan(self, grid, start=None, goal=None):
        t0 = time.perf_counter()
        start = start or grid.start
        goal = goal or grid.goal

        # initial richtung: nach norden
        x, y = start
        d = 'N'
        path = [(x, y)]
        visited_count = 0
        max_steps = grid.size * grid.size * 8  # safety - bei inseln endlosschleife möglich

        for _ in range(max_steps):
            visited_count += 1
            if (x, y) == goal:
                t1 = time.perf_counter()
                return PlanResult(path=path, expanded=visited_count,
                                  runtime_ms=(t1 - t0) * 1000.0, success=True)

            # priorität: links - geradeaus - rechts - umdrehen
            left_d = LEFT[d]
            right_d = RIGHT[d]

            if not grid.has_wall(x, y, left_d):
                d = left_d
            elif not grid.has_wall(x, y, d):
                pass  # geradeaus
            elif not grid.has_wall(x, y, right_d):
                d = right_d
            else:
                d = OPP[d]
                # bei dead-end: umdrehen, dann nochmal in nächster iteration
                continue

            dx, dy = DIRS[d]
            x, y = x + dx, y + dy
            path.append((x, y))

        # nicht gefunden in max_steps
        t1 = time.perf_counter()
        return PlanResult(path=path, expanded=visited_count,
                          runtime_ms=(t1 - t0) * 1000.0, success=False)
