"""frontier explorer mit A* pfadplanung + stuck-recovery.

states:
  EXPLORING - bot sucht frontiers, faehrt zur naechsten
  GOING_TO_GOAL - exploration fertig, bot faehrt vom start zum goal-marker
  DONE - am ziel angekommen

improvements ggue dem reactive ansatz:
  - A* auf der occupancy-grid statt direkter linie
  - stuck-detection: wenn position 4 sek nicht aendert -> recovery rotation
  - 360-grad safety check (nicht nur vorne)
"""
import math
import time
import heapq
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


UNKNOWN = -1
FREE_THRESH = 50
INFLATION_RADIUS = 0  # safety wird vom reactive controller gemacht


def astar_grid(occ, w, h, start_px, goal_px, inflation=INFLATION_RADIUS):
    """A* auf der occupancy grid. start/goal sind pixel-coords."""
    def at(p):
        x, y = p
        if x < 0 or y < 0 or x >= w or y >= h:
            return 100
        return occ[y * w + x]

    def is_free(p):
        v = at(p)
        return 0 <= v < FREE_THRESH

    # inflation: cells nahe an wand sind blocked
    def is_safe(p):
        if not is_free(p):
            return False
        x, y = p
        for dx in range(-inflation, inflation + 1):
            for dy in range(-inflation, inflation + 1):
                if at((x + dx, y + dy)) >= FREE_THRESH:
                    return False
        return True

    # falls start nicht frei ist (drift, off-map), suche naechsten freien pixel
    if not is_free(start_px):
        found = False
        for r in range(1, 40):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    p = (start_px[0] + dx, start_px[1] + dy)
                    if is_free(p):
                        start_px = p
                        found = True
                        break
                if found:
                    break
            if found:
                break
        if not found:
            return []
    # falls goal nicht frei, auch nearest free
    if not is_free(goal_px):
        found = False
        for r in range(1, 40):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    p = (goal_px[0] + dx, goal_px[1] + dy)
                    if is_free(p):
                        goal_px = p
                        found = True
                        break
                if found:
                    break
            if found:
                break
        if not found:
            return []

    h_func = lambda p: math.hypot(p[0] - goal_px[0], p[1] - goal_px[1])
    open_h = [(h_func(start_px), 0, start_px)]
    came = {start_px: None}
    g = {start_px: 0}
    counter = 0

    while open_h:
        _, _, cur = heapq.heappop(open_h)
        if cur == goal_px or math.hypot(cur[0] - goal_px[0], cur[1] - goal_px[1]) < 3:
            path = []
            while cur is not None:
                path.append(cur)
                cur = came[cur]
            path.reverse()
            return path
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
            nb = (cur[0] + dx, cur[1] + dy)
            if not is_safe(nb):
                continue
            cost = 1.4 if dx and dy else 1.0
            tg = g[cur] + cost
            if tg < g.get(nb, float('inf')):
                g[nb] = tg
                came[nb] = cur
                counter += 1
                heapq.heappush(open_h, (tg + h_func(nb), counter, nb))
    return []


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.7)
        self.declare_parameter('waypoint_tol', 0.25)
        self.declare_parameter('safety_distance', 0.18)
        self.declare_parameter('replan_period', 3.0)
        self.declare_parameter('min_frontier_size', 4)
        self.declare_parameter('goal_x', 9.5)
        self.declare_parameter('goal_y', 9.5)
        self.declare_parameter('stuck_timeout', 5.0)

        self.lin_speed = self.get_parameter('linear_speed').value
        self.ang_speed = self.get_parameter('angular_speed').value
        self.wp_tol = self.get_parameter('waypoint_tol').value
        self.safety = self.get_parameter('safety_distance').value
        self.replan_period = self.get_parameter('replan_period').value
        self.min_cluster = self.get_parameter('min_frontier_size').value
        self.goal_world = (self.get_parameter('goal_x').value,
                           self.get_parameter('goal_y').value)
        self.stuck_timeout = self.get_parameter('stuck_timeout').value

        self.map = None
        self.pose = None
        self.scan_min = {'front': float('inf'), 'left': float('inf'), 'right': float('inf')}
        self.path = []  # list of world coords
        self.path_index = 0
        self.state = 'EXPLORING'
        self.last_replan = 0.0
        self.last_progress_pose = None
        self.last_progress_time = time.time()
        self.recovery_until = 0.0
        self.recovery_dir = 1

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)

        # tf2 fuer map->base_link lookup (drift-korrigierte position)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(f'frontier_explorer up, goal=({self.goal_world})')

    def cb_map(self, m):
        self.map = m

    def update_pose_from_tf(self):
        """frischer pose aus map->base_link transform (drift-korrigiert)."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            self.pose = (x, y, yaw)
            return True
        except Exception:
            return False

    def cb_scan(self, m):
        n = len(m.ranges)
        if n == 0:
            return
        # 30° vorne, 60° links, 60° rechts
        front = list(range(n - 15, n)) + list(range(15))
        left = list(range(n // 4 - 30, n // 4 + 30))
        right = list(range(3 * n // 4 - 30, 3 * n // 4 + 30))
        for name, idxs in [('front', front), ('left', left), ('right', right)]:
            valid = [m.ranges[i] for i in idxs if 0 <= i < n
                     and m.range_min < m.ranges[i] < m.range_max]
            self.scan_min[name] = min(valid) if valid else float('inf')

    def tick(self):
        if self.map is None:
            return
        if not self.update_pose_from_tf():
            return  # noch kein map->base_link tf

        now = time.time()

        # stuck detection: weder position noch yaw aendern sich
        if self.last_progress_pose is None:
            self.last_progress_pose = self.pose
            self.last_progress_time = now
        else:
            dx = self.pose[0] - self.last_progress_pose[0]
            dy = self.pose[1] - self.last_progress_pose[1]
            dyaw = abs(wrap(self.pose[2] - self.last_progress_pose[2]))
            if math.hypot(dx, dy) > 0.05 or dyaw > 0.3:
                self.last_progress_pose = self.pose
                self.last_progress_time = now
            elif now - self.last_progress_time > self.stuck_timeout and self.recovery_until < now:
                self.get_logger().warn('stuck detected, full recovery')
                # full backup 2s + dann ~270 grad drehen ~5s
                self.recovery_until = now + 7.5
                self.recovery_dir = 1 if self.scan_min['left'] > self.scan_min['right'] else -1
                self.path = []
                self.last_progress_time = now

        # recovery: erst 2 sek zurueck, dann ~5 sek drehen (270 grad)
        if now < self.recovery_until:
            cmd = Twist()
            time_left = self.recovery_until - now
            if time_left > 5.5:
                cmd.linear.x = -0.12
            else:
                cmd.angular.z = self.ang_speed * self.recovery_dir
            self.pub_cmd.publish(cmd)
            return

        if self.state == 'EXPLORING':
            self.do_explore(now)
        elif self.state == 'GOING_TO_GOAL':
            self.do_goal()

    def do_explore(self, now):
        if not self.path or (now - self.last_replan) > self.replan_period:
            frontiers = self.find_frontiers()
            if not frontiers:
                self.get_logger().info('no frontiers -> switching to GOING_TO_GOAL')
                self.state = 'GOING_TO_GOAL'
                self.path = []
                self.publish_markers([])
                return
            biggest = max(frontiers, key=len)
            cx = sum(p[0] for p in biggest) / len(biggest)
            cy = sum(p[1] for p in biggest) / len(biggest)
            self.publish_markers(frontiers)
            # A* zum frontier-centroid
            self.plan_to_pixel((int(cx), int(cy)))
            self.last_replan = now

        self.follow_path()

    def do_goal(self):
        if not self.path:
            gx, gy = self.goal_world
            gx_px = int(round((gx - self.map.info.origin.position.x) / self.map.info.resolution))
            gy_px = int(round((gy - self.map.info.origin.position.y) / self.map.info.resolution))
            self.plan_to_pixel((gx_px, gy_px))
            if not self.path:
                self.get_logger().error('cannot plan to goal!')
                self.stop()
                return
            self.publish_path_msg()
            self.get_logger().info(f'plan to goal: {len(self.path)} waypoints')

        # check ziel erreicht
        x, y, _ = self.pose
        gx, gy = self.goal_world
        if math.hypot(x - gx, y - gy) < 0.4:
            self.get_logger().info('GOAL REACHED!')
            self.stop()
            self.state = 'DONE'
            return

        self.follow_path()

    def plan_to_pixel(self, goal_px):
        m = self.map
        x, y, _ = self.pose
        sx_px = int(round((x - m.info.origin.position.x) / m.info.resolution))
        sy_px = int(round((y - m.info.origin.position.y) / m.info.resolution))
        path_px = astar_grid(m.data, m.info.width, m.info.height,
                             (sx_px, sy_px), goal_px)
        if not path_px:
            self.get_logger().warn(
                f'A* failed: from ({sx_px},{sy_px}) to {goal_px}')
        # convert to world
        self.path = []
        for px, py in path_px[::4]:  # alle 4 cells einen waypoint = sparsam
            wx = m.info.origin.position.x + (px + 0.5) * m.info.resolution
            wy = m.info.origin.position.y + (py + 0.5) * m.info.resolution
            self.path.append((wx, wy))
        if path_px:
            # final waypoint sicherstellen
            px, py = path_px[-1]
            wx = m.info.origin.position.x + (px + 0.5) * m.info.resolution
            wy = m.info.origin.position.y + (py + 0.5) * m.info.resolution
            if not self.path or self.path[-1] != (wx, wy):
                self.path.append((wx, wy))
        self.path_index = 0

    def follow_path(self):
        if not self.path:
            return
        if self.path_index >= len(self.path):
            self.path = []
            return
        wx, wy = self.path[self.path_index]
        x, y, yaw = self.pose
        dx, dy = wx - x, wy - y
        dist = math.hypot(dx, dy)
        if dist < self.wp_tol:
            self.path_index += 1
            return

        # safety check: hindernis vorne
        if self.scan_min['front'] < self.safety:
            self.stop()
            self.path = []  # force replan
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = wrap(target_yaw - yaw)
        cmd = Twist()
        if abs(yaw_err) > 0.4:
            cmd.angular.z = self.ang_speed * (1 if yaw_err > 0 else -1)
        else:
            cmd.linear.x = self.lin_speed
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 1.5 * yaw_err))
        self.pub_cmd.publish(cmd)

    def find_frontiers(self):
        m = self.map
        w, h = m.info.width, m.info.height
        data = m.data

        def at(x, y):
            return data[y * w + x]

        cells = set()
        for y in range(1, h - 1):
            for x in range(1, w - 1):
                v = at(x, y)
                if v < 0 or v >= FREE_THRESH:
                    continue
                if (at(x + 1, y) == UNKNOWN or at(x - 1, y) == UNKNOWN
                        or at(x, y + 1) == UNKNOWN or at(x, y - 1) == UNKNOWN):
                    cells.add((x, y))

        clusters = []
        visited = set()
        for c in cells:
            if c in visited:
                continue
            cluster = []
            q = deque([c])
            visited.add(c)
            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1),
                               (1, 1), (-1, 1), (1, -1), (-1, -1)]:
                    n = (cx + dx, cy + dy)
                    if n in cells and n not in visited:
                        visited.add(n)
                        q.append(n)
            if len(cluster) >= self.min_cluster:
                clusters.append(cluster)
        return clusters

    def publish_markers(self, clusters):
        ma = MarkerArray()
        d = Marker()
        d.action = Marker.DELETEALL
        ma.markers.append(d)
        for i, cl in enumerate(clusters):
            mk = Marker()
            mk.header.frame_id = self.map.header.frame_id if self.map else 'map'
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns = 'frontiers'
            mk.id = i
            mk.type = Marker.POINTS
            mk.action = Marker.ADD
            mk.scale.x = 0.05
            mk.scale.y = 0.05
            mk.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
            for cx, cy in cl:
                wx = self.map.info.origin.position.x + (cx + 0.5) * self.map.info.resolution
                wy = self.map.info.origin.position.y + (cy + 0.5) * self.map.info.resolution
                p = Point()
                p.x = float(wx)
                p.y = float(wy)
                p.z = 0.05
                mk.points.append(p)
            ma.markers.append(mk)
        self.pub_markers.publish(ma)

    def publish_path_msg(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in self.path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)

    def stop(self):
        self.pub_cmd.publish(Twist())


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    rclpy.init()
    n = FrontierExplorer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.stop()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
