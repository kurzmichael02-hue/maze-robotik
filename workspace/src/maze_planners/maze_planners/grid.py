"""grid representation für die planner.
maze ist NxN cells, jede cell hat 4 wände (N/E/S/W).
start = (0,0) bottom-left, goal = (N-1, N-1) top-right.

import passend zum maze_worlds.generate_maze format.
"""
import random


# directions: dx, dy, opposite
DIRS = {
    'N': (0, 1),
    'E': (1, 0),
    'S': (0, -1),
    'W': (-1, 0),
}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
LEFT = {'N': 'W', 'W': 'S', 'S': 'E', 'E': 'N'}
RIGHT = {'N': 'E', 'E': 'S', 'S': 'W', 'W': 'N'}


class GridMap:
    def __init__(self, size, walls=None, start=(0, 0), goal=None):
        self.size = size
        self.start = start
        self.goal = goal if goal else (size - 1, size - 1)
        # walls: dict {(x,y): set of dirs that have walls}
        if walls is None:
            # default: alle wände vorhanden (kein durchgang)
            self.walls = {(x, y): {'N', 'E', 'S', 'W'} for x in range(size) for y in range(size)}
        else:
            self.walls = walls

    def has_wall(self, x, y, direction):
        return direction in self.walls.get((x, y), set())

    def neighbors(self, x, y):
        """gibt erreichbare nachbarn zurück (durch offene wände)."""
        out = []
        for d, (dx, dy) in DIRS.items():
            if self.has_wall(x, y, d):
                continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.size and 0 <= ny < self.size:
                out.append((nx, ny, d))
        return out

    def in_bounds(self, x, y):
        return 0 <= x < self.size and 0 <= y < self.size

    @classmethod
    def random(cls, size, seed=None):
        """recursive backtracker - selbe logik wie maze_worlds.generate_maze."""
        if seed is not None:
            random.seed(seed)
        walls = {(x, y): {'N', 'E', 'S', 'W'} for x in range(size) for y in range(size)}
        visited = {(0, 0)}
        stack = [(0, 0)]
        while stack:
            x, y = stack[-1]
            nbrs = []
            for d, (dx, dy) in DIRS.items():
                nx, ny = x + dx, y + dy
                if 0 <= nx < size and 0 <= ny < size and (nx, ny) not in visited:
                    nbrs.append((d, nx, ny))
            if not nbrs:
                stack.pop()
                continue
            d, nx, ny = random.choice(nbrs)
            walls[(x, y)].discard(d)
            walls[(nx, ny)].discard(OPP[d])
            visited.add((nx, ny))
            stack.append((nx, ny))
        return cls(size, walls=walls, start=(0, 0), goal=(size - 1, size - 1))


def path_length(path):
    """anzahl moves im pfad."""
    return max(0, len(path) - 1)


def count_turns(path):
    """anzahl richtungswechsel im pfad. straight = nicht zählen."""
    if len(path) < 3:
        return 0
    turns = 0
    for i in range(1, len(path) - 1):
        dx1 = path[i][0] - path[i - 1][0]
        dy1 = path[i][1] - path[i - 1][1]
        dx2 = path[i + 1][0] - path[i][0]
        dy2 = path[i + 1][1] - path[i][1]
        if (dx1, dy1) != (dx2, dy2):
            turns += 1
    return turns
