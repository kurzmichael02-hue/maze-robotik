"""ros2 node: subscribed /map, läuft alle 4 planner, published paths.

trigger via service /run_planners (std_srvs/Trigger).
nach trigger: konvertiere die aktuelle map -> GridMap, lass alle 4 algos
laufen, publiziere /planned_path/<algo> + marker mit farben.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_srvs.srv import Trigger
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .map_converter import occgrid_to_gridmap
from .algorithms import ALL_PLANNERS
from .grid import count_turns


# colors für die 4 algos
ALGO_COLORS = {
    'wall_follower': (1.0, 0.4, 0.4),
    'bfs':           (1.0, 1.0, 0.2),
    'astar':         (0.2, 1.0, 0.4),
    'floodfill':     (0.4, 0.6, 1.0),
}


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('cell_size', 1.0)
        self.declare_parameter('maze_size', 10)
        self.declare_parameter('start_world_x', 0.5)
        self.declare_parameter('start_world_y', 0.5)
        self.declare_parameter('frame_id', 'map')

        self.cell_size = self.get_parameter('cell_size').value
        self.maze_size = self.get_parameter('maze_size').value
        self.start_wx = self.get_parameter('start_world_x').value
        self.start_wy = self.get_parameter('start_world_y').value
        self.frame_id = self.get_parameter('frame_id').value

        self.map_msg = None

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.cb_map, map_qos)

        # path publisher pro algo
        self.path_pubs = {
            name: self.create_publisher(Path, f'/planned_path/{name}', 10)
            for name in ALL_PLANNERS
        }
        self.marker_pub = self.create_publisher(MarkerArray, '/planned_path_markers', 10)

        self.srv = self.create_service(Trigger, '/run_planners', self.cb_run)

        self.get_logger().info(
            f'planner_node up. maze={self.maze_size}x{self.maze_size}, '
            f'cell={self.cell_size}m. trigger: ros2 service call /run_planners std_srvs/srv/Trigger')

    def cb_map(self, msg):
        self.map_msg = msg

    def cb_run(self, req, resp):
        if self.map_msg is None:
            resp.success = False
            resp.message = 'no map yet'
            return resp

        m = self.map_msg
        try:
            grid = occgrid_to_gridmap(
                m.data, m.info.width, m.info.height, m.info.resolution,
                m.info.origin.position.x, m.info.origin.position.y,
                self.cell_size, self.maze_size,
                start_world=(self.start_wx, self.start_wy),
            )
        except Exception as e:
            resp.success = False
            resp.message = f'conversion failed: {e}'
            return resp

        ma = MarkerArray()
        # delete previous
        d = Marker()
        d.action = Marker.DELETEALL
        d.header.frame_id = self.frame_id
        ma.markers.append(d)

        results = []
        for i, (name, cls) in enumerate(ALL_PLANNERS.items()):
            r = cls().plan(grid)
            results.append((name, r))
            # publish path
            self.publish_path(name, r.path)
            # marker line strip
            self.add_marker(ma, name, r.path, i)
            self.get_logger().info(
                f'  {name:14s} success={r.success} len={r.length} '
                f'expanded={r.expanded} turns={count_turns(r.path)} '
                f'time={r.runtime_ms:.2f}ms')

        self.marker_pub.publish(ma)

        # report als string
        lines = [f"{n}: len={r.length}, exp={r.expanded}, turns={count_turns(r.path)}, "
                 f"t={r.runtime_ms:.1f}ms" for n, r in results]
        resp.success = True
        resp.message = '\n'.join(lines)
        return resp

    def cell_to_world(self, cx, cy):
        wx = self.start_wx + cx * self.cell_size
        wy = self.start_wy + cy * self.cell_size
        return wx, wy

    def publish_path(self, name, cells):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for c in cells:
            wx, wy = self.cell_to_world(*c)
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pubs[name].publish(path)

    def add_marker(self, ma, name, cells, idx):
        if not cells:
            return
        mk = Marker()
        mk.header.frame_id = self.frame_id
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = name
        mk.id = idx
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        # höhe nach algo gestaffelt damit linien sichtbar sind
        z = 0.1 + idx * 0.05
        mk.scale.x = 0.04
        r, g, b = ALGO_COLORS.get(name, (1.0, 1.0, 1.0))
        mk.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
        for cx, cy in cells:
            wx, wy = self.cell_to_world(cx, cy)
            p = Point()
            p.x = float(wx)
            p.y = float(wy)
            p.z = z
            mk.points.append(p)
        ma.markers.append(mk)


def main():
    rclpy.init()
    n = PlannerNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
