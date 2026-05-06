"""visualisiert ein maze + den geplanten pfad.

usage:
    python -m maze_planners.visualize --size 10 --seed 7 --algo astar --out path.png
"""
import argparse
from pathlib import Path

from .grid import GridMap
from .algorithms import ALL_PLANNERS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=int, default=10)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--algo', type=str, default='astar', choices=list(ALL_PLANNERS.keys()))
    ap.add_argument('--out', type=str, default='path.png')
    args = ap.parse_args()

    grid = GridMap.random(args.size, seed=args.seed)
    planner = ALL_PLANNERS[args.algo]()
    res = planner.plan(grid)

    print(f"{args.algo}: success={res.success} len={res.length} "
          f"expanded={res.expanded} runtime={res.runtime_ms:.2f}ms")

    try:
        draw(grid, res.path, args.algo, args.out)
        print(f"plot -> {args.out}")
    except ImportError:
        print("matplotlib fehlt, kein plot")


def draw(grid, path, title, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(8, 8))
    s = grid.size

    # boden
    ax.add_patch(Rectangle((0, 0), s, s, facecolor='#f5f5f5', edgecolor='none'))

    # wände als linien zeichnen
    for (x, y), ws in grid.walls.items():
        if 'N' in ws:
            ax.plot([x, x + 1], [y + 1, y + 1], 'k-', linewidth=2)
        if 'S' in ws:
            ax.plot([x, x + 1], [y, y], 'k-', linewidth=2)
        if 'E' in ws:
            ax.plot([x + 1, x + 1], [y, y + 1], 'k-', linewidth=2)
        if 'W' in ws:
            ax.plot([x, x], [y, y + 1], 'k-', linewidth=2)

    # start, goal
    sx, sy = grid.start
    gx, gy = grid.goal
    ax.add_patch(Rectangle((sx + 0.15, sy + 0.15), 0.7, 0.7,
                           facecolor='#28a745', edgecolor='none', alpha=0.85))
    ax.add_patch(Rectangle((gx + 0.15, gy + 0.15), 0.7, 0.7,
                           facecolor='#dc3545', edgecolor='none', alpha=0.85))
    ax.text(sx + 0.5, sy + 0.5, 'S', ha='center', va='center',
            fontsize=14, fontweight='bold', color='white')
    ax.text(gx + 0.5, gy + 0.5, 'G', ha='center', va='center',
            fontsize=14, fontweight='bold', color='white')

    # pfad
    if path:
        xs = [p[0] + 0.5 for p in path]
        ys = [p[1] + 0.5 for p in path]
        ax.plot(xs, ys, '-', color='#0066cc', linewidth=3, alpha=0.7)
        ax.plot(xs, ys, 'o', color='#0066cc', markersize=4)

    ax.set_xlim(-0.2, s + 0.2)
    ax.set_ylim(-0.2, s + 0.2)
    ax.set_aspect('equal')
    ax.set_title(f"{title} on {s}x{s} maze | path={max(0, len(path) - 1)} cells")
    ax.set_xticks([])
    ax.set_yticks([])

    plt.tight_layout()
    plt.savefig(out, dpi=120)


if __name__ == '__main__':
    main()
