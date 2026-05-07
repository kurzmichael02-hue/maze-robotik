"""frontier explorer — bot fährt selbständig durch unbekanntes maze.

algo:
1. abonniere /map (slam_toolbox output)
2. find frontiers: freie zellen die an unknown grenzen
3. cluster, pick größtes / nächstes
4. simpler reactive controller: rotate to goal, drive forward, stop near
5. wenn keine frontiers mehr -> finished

KEIN nav2 dependency - eigener controller. simpler aber für ein offenes maze
gut genug.
"""
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


# occupancy grid values
UNKNOWN = -1
FREE_THRESH = 50  # < 50 = free, >= 50 = occupied


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # params
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('goal_tol', 0.25)
        self.declare_parameter('safety_distance', 0.25)
        self.declare_parameter('replan_period', 2.0)
        self.declare_parameter('min_frontier_size', 4)

        self.lin_speed = self.get_parameter('linear_speed').value
        self.ang_speed = self.get_parameter('angular_speed').value
        self.goal_tol = self.get_parameter('goal_tol').value
        self.safety = self.get_parameter('safety_distance').value
        self.replan_period = self.get_parameter('replan_period').value
        self.min_cluster = self.get_parameter('min_frontier_size').value

        # state
        self.map = None
        self.pose = None  # (x, y, yaw) in map frame, simplified
        self.scan_min_front = float('inf')
        self.current_goal = None  # (x, y) in world
        self.exploring = True
        self.last_replan = 0.0

        # qos: map ist transient_local
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.cb_map, map_qos)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/frontier_markers', 10)

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('frontier_explorer up')

    # --- callbacks ---
    def cb_map(self, msg: OccupancyGrid):
        self.map = msg

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # quaternion -> yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (p.x, p.y, yaw)

    def cb_scan(self, msg: LaserScan):
        # min range im 60° kegel vorne
        n = len(msg.ranges)
        if n == 0:
            return
        # vorne ist normalerweise index 0 oder n/2 - hängt vom lidar frame ab
        # bei 360-grad lidar mit -pi..pi: vorne = mittlere index (yaw=0)
        # sicherheits-fallback: nimm min in den ersten 30 + letzten 30 indices
        front_idxs = list(range(30)) + list(range(n - 30, n))
        valid = [r for i, r in enumerate(msg.ranges) if i in front_idxs and msg.range_min < r < msg.range_max]
        self.scan_min_front = min(valid) if valid else float('inf')

    # --- main loop ---
    def tick(self):
        if self.map is None or self.pose is None:
            return

        # safety: hindernis vor uns
        if self.scan_min_front < self.safety:
            self.stop()
            # replan sofort
            self.replan()
            return

        now = time.time()
        # periodisch oder wenn kein goal
        if self.current_goal is None or (now - self.last_replan) > self.replan_period:
            self.replan()
            self.last_replan = now

        if self.current_goal is None:
            # keine frontier mehr => fertig
            if self.exploring:
                self.get_logger().info('no frontiers -> exploration finished')
                self.exploring = False
                self.stop()
            return

        # drive zum goal
        gx, gy = self.current_goal
        x, y, yaw = self.pose
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tol:
            self.current_goal = None
            self.stop()
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = wrap(target_yaw - yaw)

        cmd = Twist()
        if abs(yaw_err) > 0.3:
            # erst drehen
            cmd.angular.z = self.ang_speed * (1.0 if yaw_err > 0 else -1.0)
        else:
            cmd.linear.x = self.lin_speed
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 1.5 * yaw_err))
        self.pub_cmd.publish(cmd)

    # --- frontier detection ---
    def replan(self):
        frontiers = self.find_frontiers()
        if not frontiers:
            self.current_goal = None
            self.publish_markers([])
            return

        # nimm größtes cluster
        biggest = max(frontiers, key=len)
        # centroid
        cx = sum(p[0] for p in biggest) / len(biggest)
        cy = sum(p[1] for p in biggest) / len(biggest)
        # zelle -> world
        gx, gy = self.cell_to_world(cx, cy)
        self.current_goal = (gx, gy)
        self.publish_markers(frontiers)
        self.get_logger().info(
            f'new goal: ({gx:.2f}, {gy:.2f}) from cluster size {len(biggest)} '
            f'({len(frontiers)} clusters total)')

    def find_frontiers(self):
        m = self.map
        w, h = m.info.width, m.info.height
        data = m.data

        def at(x, y):
            return data[y * w + x]

        # frontier zelle = free, mit mindestens einem unknown nachbarn
        frontier_cells = set()
        for y in range(1, h - 1):
            for x in range(1, w - 1):
                v = at(x, y)
                if v < 0 or v >= FREE_THRESH:
                    continue
                # check 4-nachbarn auf unknown
                if (at(x + 1, y) == UNKNOWN or at(x - 1, y) == UNKNOWN
                    or at(x, y + 1) == UNKNOWN or at(x, y - 1) == UNKNOWN):
                    frontier_cells.add((x, y))

        # cluster via flood fill
        clusters = []
        visited = set()
        for cell in frontier_cells:
            if cell in visited:
                continue
            cluster = []
            q = deque([cell])
            visited.add(cell)
            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)]:
                    n = (cx + dx, cy + dy)
                    if n in frontier_cells and n not in visited:
                        visited.add(n)
                        q.append(n)
            if len(cluster) >= self.min_cluster:
                clusters.append(cluster)
        return clusters

    def cell_to_world(self, cx, cy):
        m = self.map
        wx = m.info.origin.position.x + (cx + 0.5) * m.info.resolution
        wy = m.info.origin.position.y + (cy + 0.5) * m.info.resolution
        return wx, wy

    def publish_markers(self, clusters):
        ma = MarkerArray()
        # delete-all
        d = Marker()
        d.action = Marker.DELETEALL
        ma.markers.append(d)

        for i, cluster in enumerate(clusters):
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
            for cx, cy in cluster:
                wx, wy = self.cell_to_world(cx, cy)
                p = Point()
                p.x = float(wx)
                p.y = float(wy)
                p.z = 0.05
                mk.points.append(p)
            ma.markers.append(mk)
        self.pub_markers.publish(ma)

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
