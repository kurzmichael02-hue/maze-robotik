"""path_executor — bot faehrt cell-by-cell zum ziel.
plant A* auf der bekannten cell-grid (gleicher seed wie generator).
slam mappt parallel mit fuer rviz.

TODO: irgendwann durch nav2 ersetzen — sauberer aber halt riesen-config.
"""
import math
import heapq
import random

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


DELTA = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'W': (-1, 0)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}


def regen_maze_cells(size, seed, extra_factor=0.35):
    """rebuild same maze cells as generate_maze.py (deterministic via seed)."""
    if seed is not None:
        random.seed(seed)
    cells = {(x, y): {'N', 'E', 'S', 'W'} for x in range(size) for y in range(size)}
    visited = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        x, y = stack[-1]
        nbrs = []
        for d, (dx, dy) in DELTA.items():
            nx, ny = x + dx, y + dy
            if 0 <= nx < size and 0 <= ny < size and (nx, ny) not in visited:
                nbrs.append((d, nx, ny))
        if not nbrs:
            stack.pop()
            continue
        d, nx, ny = random.choice(nbrs)
        cells[(x, y)].discard(d)
        cells[(nx, ny)].discard(OPP[d])
        visited.add((nx, ny))
        stack.append((nx, ny))

    inner_walls = []
    for (x, y), w in cells.items():
        for d in list(w):
            dx, dy = DELTA[d]
            nx, ny = x + dx, y + dy
            if 0 <= nx < size and 0 <= ny < size:
                inner_walls.append(((x, y), d, (nx, ny)))
    seen = set()
    unique = []
    for a, d, b in inner_walls:
        key = tuple(sorted([a, b]))
        if key in seen:
            continue
        seen.add(key)
        unique.append((a, d, b))
    n_remove = int(extra_factor * len(unique))
    if n_remove > 0:
        random.shuffle(unique)
    for (x, y), d, (nx, ny) in unique[:n_remove]:
        cells[(x, y)].discard(d)
        cells[(nx, ny)].discard(OPP[d])
    return cells


def astar_cells(cells, start, goal, size):
    """A* auf cell-grid. liefert liste von cells inklusive start+goal."""
    def heur(c):
        return abs(c[0] - goal[0]) + abs(c[1] - goal[1])

    open_h = [(heur(start), 0, start)]
    came = {start: None}
    g = {start: 0}
    counter = 0

    while open_h:
        _, _, cur = heapq.heappop(open_h)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = came[cur]
            path.reverse()
            return path
        for d, (dx, dy) in DELTA.items():
            if d in cells.get(cur, set()):
                continue  # wand zwischen cur und nb
            nb = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nb[0] < size and 0 <= nb[1] < size):
                continue
            tg = g[cur] + 1
            if tg < g.get(nb, float('inf')):
                g[nb] = tg
                came[nb] = cur
                counter += 1
                heapq.heappush(open_h, (tg + heur(nb), counter, nb))
    return []


# direction zwischen 2 nachbar-cells
def cells_to_dir(c1, c2):
    dx = c2[0] - c1[0]
    dy = c2[1] - c1[1]
    if dx == 1: return 'E'
    if dx == -1: return 'W'
    if dy == 1: return 'N'
    if dy == -1: return 'S'
    return None


# yaw fuer eine direction
DIR_YAW = {'N': math.pi / 2, 'S': -math.pi / 2, 'E': 0, 'W': math.pi}


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class PathExecutor(Node):
    def __init__(self):
        super().__init__('path_executor')

        self.declare_parameter('maze_size', 8)
        self.declare_parameter('cell_size', 1.2)
        self.declare_parameter('seed', 7)
        self.declare_parameter('difficulty', 'easy')
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.5)

        size = self.get_parameter('maze_size').value
        cs = self.get_parameter('cell_size').value
        seed = self.get_parameter('seed').value
        diff = self.get_parameter('difficulty').value
        self.size = size
        self.cs = cs
        self.lin_speed = self.get_parameter('linear_speed').value
        self.ang_speed = self.get_parameter('angular_speed').value

        # rebuild maze cells (same seed = same maze als generator)
        extra = {'easy': 0.35, 'medium': 0.20, 'hard': 0.0}.get(diff, 0.20)
        cells = regen_maze_cells(size, seed, extra)

        # plan A* von (0,0) zu (size-1, size-1)
        start = (0, 0)
        goal = (size - 1, size - 1)
        cell_path = astar_cells(cells, start, goal, size)
        if not cell_path:
            self.get_logger().error('cannot plan path!')
            self.cell_path = []
        else:
            self.cell_path = cell_path
            self.get_logger().info(f'A* path: {len(cell_path)} cells')

        # convert cell-path zu ODOM-frame waypoints
        # spawn world pose: (start_x, start_y, yaw=pi/2)
        # odom origin = spawn -> world_to_odom: rotate by -pi/2, translate
        # ox = wy - sy
        # oy = sx - wx
        sx = 0.5 * cs
        sy = 0.5 * cs

        def world_to_odom(wx, wy):
            return (wy - sy, sx - wx)

        self.waypoints = []
        for c in cell_path:
            wx = 0.5 * cs + c[0] * cs
            wy = 0.5 * cs + c[1] * cs
            self.waypoints.append(world_to_odom(wx, wy))

        # extra waypoint: durch den exit raus zum goal_marker
        last_cell = cell_path[-1]
        exit_wx = 0.5 * cs + last_cell[0] * cs
        exit_wy = size * cs + 0.4  # ausserhalb maze beim goal_marker
        self.waypoints.append(world_to_odom(exit_wx, exit_wy))
        self.wp_idx = 0
        self.pose = None  # (x, y, yaw)
        self.state = 'WAIT_POSE'  # WAIT_POSE -> TURN -> DRIVE -> DONE
        self.target_yaw = None
        self.target_xy = None

        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path_executor', 10)
        self.pub_marker = self.create_publisher(Marker, '/executor_target', 10)

        # publish path einmal als preview
        self.timer_path = self.create_timer(1.0, self.publish_path)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f'path_executor up. {size}x{size} maze, start->goal in {len(cell_path)} steps')

    def cb_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.pose = (p.x, p.y, yaw)

    def tick(self):
        if self.pose is None or not self.waypoints:
            return

        if self.state == 'DONE':
            self.pub_cmd.publish(Twist())
            return

        if self.state == 'WAIT_POSE':
            # erster waypoint ist start - skip auf zweiten
            self.wp_idx = 1
            if self.wp_idx >= len(self.waypoints):
                self.state = 'DONE'
                return
            self.start_to_next()
            return

        x, y, yaw = self.pose

        if self.state == 'TURN':
            yaw_err = wrap(self.target_yaw - yaw)
            if abs(yaw_err) < 0.08:
                self.pub_cmd.publish(Twist())
                self.state = 'DRIVE'
                return
            cmd = Twist()
            cmd.angular.z = self.ang_speed * (1 if yaw_err > 0 else -1)
            self.pub_cmd.publish(cmd)
            return

        if self.state == 'DRIVE':
            tx, ty = self.target_xy
            dist = math.hypot(tx - x, ty - y)
            if dist < 0.1:
                self.pub_cmd.publish(Twist())
                # naechster waypoint
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.state = 'DONE'
                    self.get_logger().info('GOAL REACHED!')
                    return
                self.start_to_next()
                return
            # PD: geradeaus mit minimal-correction
            cmd = Twist()
            target_yaw = math.atan2(ty - y, tx - x)
            yaw_err = wrap(target_yaw - yaw)
            cmd.linear.x = self.lin_speed * max(0.3, 1 - abs(yaw_err))
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 2.0 * yaw_err))
            self.pub_cmd.publish(cmd)
            self.publish_target_marker()
            return

    def start_to_next(self):
        # target_yaw aus odom-coords berechnen (kein world-frame issue)
        self.target_xy = self.waypoints[self.wp_idx]
        x, y, _ = self.pose
        tx, ty = self.target_xy
        self.target_yaw = math.atan2(ty - y, tx - x)
        self.state = 'TURN'

    def publish_path(self):
        if not self.waypoints:
            return
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in self.waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)

    def publish_target_marker(self):
        if self.target_xy is None:
            return
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'target'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(self.target_xy[0])
        m.pose.position.y = float(self.target_xy[1])
        m.pose.position.z = 0.2
        m.pose.orientation.w = 1.0
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    n = PathExecutor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.pub_cmd.publish(Twist())
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
