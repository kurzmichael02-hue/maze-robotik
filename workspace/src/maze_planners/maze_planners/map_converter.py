"""occupancy grid (slam_toolbox output) -> GridMap (cell-basiert).

annahme: maze cells sind exact `cell_size` m breit, das maze ist axis-aligned
und start ist bei (0,0).

algo: für jede cell schau dir das pixel-feld an, das die zelle abdeckt.
für jede der 4 wände teste ob die occupancy-werte entlang der wand-linie
free oder occupied sind.
"""
import math
from .grid import GridMap, DIRS, OPP


# occupancy grid values
UNKNOWN = -1
OCC_THRESH = 50  # >= 50 = occupied


def occgrid_to_gridmap(occ_data, occ_w, occ_h, occ_res, occ_origin_x, occ_origin_y,
                       cell_size, maze_size, start_world=(0.5, 0.5)):
    """occupancy grid -> GridMap.

    occ_data: 1D array, w*h, values [-1, 0..100]
    occ_origin: world-koords der ecke (0,0) der occ_grid
    cell_size: maze cell width in meters
    maze_size: NxN cells
    start_world: world-position der maze-cell (0,0) center
    """
    walls = {(x, y): set() for x in range(maze_size) for y in range(maze_size)}

    def occ_at(px, py):
        if px < 0 or py < 0 or px >= occ_w or py >= occ_h:
            return UNKNOWN
        return occ_data[py * occ_w + px]

    def world_to_pixel(wx, wy):
        px = int(round((wx - occ_origin_x) / occ_res))
        py = int(round((wy - occ_origin_y) / occ_res))
        return px, py

    def line_blocked(wx1, wy1, wx2, wy2, samples=12):
        """sample punkte entlang der linie, true wenn ein punkt occupied ist."""
        for i in range(samples + 1):
            t = i / samples
            wx = wx1 + (wx2 - wx1) * t
            wy = wy1 + (wy2 - wy1) * t
            px, py = world_to_pixel(wx, wy)
            v = occ_at(px, py)
            if v >= OCC_THRESH:
                return True
        return False

    sx, sy = start_world
    for cy in range(maze_size):
        for cx in range(maze_size):
            # cell center in world
            wx_c = sx + cx * cell_size
            wy_c = sy + cy * cell_size
            half = cell_size / 2.0

            # 4 wand-linien (cell-edges)
            edges = {
                'N': (wx_c - half, wy_c + half, wx_c + half, wy_c + half),
                'S': (wx_c - half, wy_c - half, wx_c + half, wy_c - half),
                'E': (wx_c + half, wy_c - half, wx_c + half, wy_c + half),
                'W': (wx_c - half, wy_c - half, wx_c - half, wy_c + half),
            }
            for d, (x1, y1, x2, y2) in edges.items():
                if line_blocked(x1, y1, x2, y2):
                    walls[(cx, cy)].add(d)

    # symmetrie: wenn cell A wand nach E hat, hat cell rechts daneben wand nach W
    # (sollte schon aus dem occ_grid rauskommen, aber zur sicherheit)
    for (x, y), w in list(walls.items()):
        for d in list(w):
            dx, dy = DIRS[d]
            nx, ny = x + dx, y + dy
            if 0 <= nx < maze_size and 0 <= ny < maze_size:
                walls[(nx, ny)].add(OPP[d])

    return GridMap(maze_size, walls=walls, start=(0, 0),
                   goal=(maze_size - 1, maze_size - 1))
