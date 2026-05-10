"""benchmark runner.
generiert mazes verschiedener größen, fährt alle algos durch, sammelt metriken,
schreibt csv, malt vergleichs-plots.

usage:
    python -m maze_planners.benchmark --sizes 5,10,15 --seeds 5 --out results/
"""
import argparse
import csv
import os
from pathlib import Path

from .grid import GridMap, count_turns
from .algorithms import ALL_PLANNERS


def run_one(grid, planner_cls):
    p = planner_cls()
    res = p.plan(grid)
    return {
        'algo': p.name,
        'success': res.success,
        'path_length': res.length,
        'expanded': res.expanded,
        'runtime_ms': round(res.runtime_ms, 3),
        'turns': count_turns(res.path) if res.success else 0,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sizes', type=str, default='5,10,15',
                    help='comma-separated maze sizes (NxN cells)')
    ap.add_argument('--seeds', type=int, default=5,
                    help='wie viele random mazes pro size')
    ap.add_argument('--out', type=str, default='results/')
    ap.add_argument('--plot', action='store_true', help='matplotlib plots erzeugen')
    args = ap.parse_args()

    sizes = [int(s) for s in args.sizes.split(',') if s.strip()]
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    print(f"benchmarking on sizes={sizes}, {args.seeds} seeds each, "
          f"{len(ALL_PLANNERS)} algos -> {len(sizes) * args.seeds * len(ALL_PLANNERS)} runs")

    for size in sizes:
        for seed in range(args.seeds):
            grid = GridMap.random(size, seed=seed)
            for name, cls in ALL_PLANNERS.items():
                r = run_one(grid, cls)
                r['size'] = size
                r['seed'] = seed
                rows.append(r)
                ok = 'OK' if r['success'] else 'FAIL'
                print(f"  {ok}  size={size:2d} seed={seed} {r['algo']:14s} "
                      f"len={r['path_length']:4d} exp={r['expanded']:5d} "
                      f"turns={r['turns']:3d} t={r['runtime_ms']:8.3f}ms")

    # csv
    csv_path = out_dir / 'benchmark.csv'
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['size', 'seed', 'algo', 'success',
                                          'path_length', 'expanded', 'turns',
                                          'runtime_ms'])
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"\ncsv -> {csv_path}")

    if args.plot:
        try:
            make_plots(rows, out_dir)
        except ImportError as e:
            print(f"matplotlib nicht installiert, skip plots ({e})")


def make_plots(rows, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # gruppieren: {algo: {size: [values]}}
    def group(metric):
        out = {}
        for r in rows:
            if not r['success']:
                continue
            out.setdefault(r['algo'], {}).setdefault(r['size'], []).append(r[metric])
        return out

    metrics = [
        ('path_length', 'pfadlänge (zellen)'),
        ('expanded', 'expandierte zellen'),
        ('turns', 'anzahl drehungen'),
        ('runtime_ms', 'laufzeit (ms, log)'),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    axes = axes.flatten()

    for ax, (metric, title) in zip(axes, metrics):
        data = group(metric)
        for algo, by_size in data.items():
            xs = sorted(by_size.keys())
            ys = [sum(by_size[s]) / len(by_size[s]) for s in xs]
            ax.plot(xs, ys, marker='o', label=algo)
        ax.set_xlabel('maze größe (NxN)')
        ax.set_ylabel(title)
        ax.set_title(title)
        if metric == 'runtime_ms':
            ax.set_yscale('log')
        ax.grid(True, alpha=0.3)
        ax.legend()

    plt.tight_layout()
    out_path = out_dir / 'benchmark.png'
    plt.savefig(out_path, dpi=120)
    print(f"plot -> {out_path}")


if __name__ == '__main__':
    main()
