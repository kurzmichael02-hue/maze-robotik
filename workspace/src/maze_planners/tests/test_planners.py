"""smoke tests + korrektheits-checks für die planner."""
import pytest
from maze_planners.grid import GridMap
from maze_planners.algorithms import ALL_PLANNERS, BFS, AStar, FloodFill, WallFollower


def make_open_grid(n):
    """grid wo alle wände weg sind (komplett offen)."""
    walls = {(x, y): set() for x in range(n) for y in range(n)}
    return GridMap(n, walls=walls)


def test_grid_random_solvable():
    """random grid sollte solvable sein (BFS findet pfad)."""
    g = GridMap.random(10, seed=42)
    res = BFS().plan(g)
    assert res.success
    assert res.path[0] == g.start
    assert res.path[-1] == g.goal


@pytest.mark.parametrize("name,cls", list(ALL_PLANNERS.items()))
def test_all_planners_solve_random_maze(name, cls):
    g = GridMap.random(8, seed=123)
    res = cls().plan(g)
    assert res.success, f"{name} failed on random 8x8 maze seed=123"
    assert res.path[0] == g.start
    assert res.path[-1] == g.goal


def test_bfs_optimal_on_open_grid():
    """auf komplett offenem grid: optimaler pfad = manhattan distance."""
    g = make_open_grid(5)
    res = BFS().plan(g)
    assert res.length == 8  # 4 + 4 moves


def test_astar_matches_bfs_length():
    """A* sollte selbe pfadlänge wie BFS liefern (beide optimal in unweighted grids)."""
    for seed in range(5):
        g = GridMap.random(10, seed=seed)
        a = BFS().plan(g)
        b = AStar().plan(g)
        assert a.success and b.success
        assert a.length == b.length, f"seed {seed}: bfs={a.length}, astar={b.length}"


def test_floodfill_optimal():
    """flood-fill via gradient ist auch optimal in 4-connected grids."""
    for seed in range(5):
        g = GridMap.random(10, seed=seed)
        bfs = BFS().plan(g)
        ff = FloodFill().plan(g)
        assert ff.success
        assert ff.length == bfs.length


def test_astar_expands_less_than_bfs():
    """A* mit guter heuristic sollte ≤ BFS expansions haben."""
    g = GridMap.random(15, seed=0)
    a = BFS().plan(g)
    b = AStar().plan(g)
    assert b.expanded <= a.expanded


def test_wall_follower_reaches_goal_in_simple_maze():
    """wall follower findet ziel in random maze (recursive backtracker = simply connected)."""
    g = GridMap.random(8, seed=1)
    res = WallFollower().plan(g)
    assert res.success
