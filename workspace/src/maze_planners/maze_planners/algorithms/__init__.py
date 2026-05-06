from .base import Planner, PlanResult
from .wall_follower import WallFollower
from .bfs import BFS
from .astar import AStar
from .floodfill import FloodFill

ALL_PLANNERS = {
    'wall_follower': WallFollower,
    'bfs': BFS,
    'astar': AStar,
    'floodfill': FloodFill,
}
