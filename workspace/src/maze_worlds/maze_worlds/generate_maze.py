"""generate a random maze and export it as a gazebo .sdf world.

usage:
    python -m maze_worlds.generate_maze --size 10 --cell 1.0 --out worlds/maze_10.sdf

algo: recursive backtracker (iterativ, mit stack).
maze ist ein grid von cells, jede cell hat 4 wände (N/E/S/W).
beim besuchen brechen wir die wand zur nächsten cell auf.
am ende exportieren wir alle übriggebliebenen wände als boxen ins .sdf.
"""
import argparse
import random
from pathlib import Path


def generate_grid(size, seed=None):
    """recursive backtracker. liefert dict {(x,y): {'N','E','S','W'} mit verbleibenden wänden}."""
    if seed is not None:
        random.seed(seed)

    cells = {(x, y): {'N', 'E', 'S', 'W'} for x in range(size) for y in range(size)}
    visited = set()
    stack = [(0, 0)]
    visited.add((0, 0))

    opp = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
    delta = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}

    while stack:
        x, y = stack[-1]
        nbrs = []
        for d, (dx, dy) in delta.items():
            nx, ny = x + dx, y + dy
            if 0 <= nx < size and 0 <= ny < size and (nx, ny) not in visited:
                nbrs.append((d, nx, ny))
        if not nbrs:
            stack.pop()
            continue
        d, nx, ny = random.choice(nbrs)
        cells[(x, y)].discard(d)
        cells[(nx, ny)].discard(opp[d])
        visited.add((nx, ny))
        stack.append((nx, ny))

    # entry / exit aufmachen
    cells[(0, 0)].discard('S')
    cells[(size - 1, size - 1)].discard('N')

    return cells


def cells_to_walls(cells, size, cell_size):
    """jede wand wird zu einem (x, y, length, orientation) tupel.
    duplikate (eine wand zwischen 2 cells) werden gemerged.
    """
    walls = set()
    for (x, y), w in cells.items():
        cx = x * cell_size
        cy = y * cell_size
        if 'N' in w:
            walls.add((cx, cy + cell_size, cell_size, 'h'))
        if 'S' in w:
            walls.add((cx, cy, cell_size, 'h'))
        if 'E' in w:
            walls.add((cx + cell_size, cy, cell_size, 'v'))
        if 'W' in w:
            walls.add((cx, cy, cell_size, 'v'))
    return walls


def walls_to_sdf(walls, size, cell_size, wall_height=0.5, wall_thickness=0.05):
    boxes = []
    for i, (x, y, length, orient) in enumerate(walls):
        if orient == 'h':
            sx, sy, sz = length, wall_thickness, wall_height
            px, py, pz = x + length / 2, y, wall_height / 2
        else:
            sx, sy, sz = wall_thickness, length, wall_height
            px, py, pz = x, y + length / 2, wall_height / 2

        boxes.append(f"""
    <model name="wall_{i}">
      <static>true</static>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.4 0.4 0.45 1</diffuse></material>
        </visual>
      </link>
      <pose>{px} {py} {pz} 0 0 0</pose>
    </model>""")

    floor_size = size * cell_size + 1
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="maze">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>{floor_size} {floor_size}</size></plane></geometry></collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>{floor_size} {floor_size}</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
{''.join(boxes)}
  </world>
</sdf>
"""
    return sdf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=int, default=10, help='maze size in cells (NxN)')
    ap.add_argument('--cell', type=float, default=1.0, help='cell size in meters')
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--out', type=str, default='worlds/maze.sdf')
    args = ap.parse_args()

    cells = generate_grid(args.size, args.seed)
    walls = cells_to_walls(cells, args.size, args.cell)
    sdf = walls_to_sdf(walls, args.size, args.cell)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(sdf, encoding='utf-8')
    print(f"wrote {out} ({len(walls)} walls, {args.size}x{args.size} grid)")


if __name__ == '__main__':
    main()
