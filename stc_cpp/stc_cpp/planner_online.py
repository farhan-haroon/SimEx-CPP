import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from collections import defaultdict, deque
import math

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('stc_planner')

        self.cell_size = 0.8  # 2D × 2D where D = 0.4m → cell_size = 0.8m
        self.robot_position = (0.0, 0.0)  # assume starting at odom (0,0)

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.odom_callback, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/grid_centers', 10)
        self.get_logger().info('Initialized online grid & spanning tree node.')

    def odom_callback(self, msg: Odometry):
        self.robot_position = (
            msg.pose.pose.position.x, msg.pose.pose.position.y
        )

    def map_callback(self, msg: OccupancyGrid):
        resolution = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        w, h = msg.info.width, msg.info.height
        data = msg.data

        full_w = w * resolution
        full_h = h * resolution

        half_x = math.floor((full_w / self.cell_size) / 2)
        half_y = math.floor((full_h / self.cell_size) / 2)

        grid_origin_x = ox + full_w / 2.0 - (half_x + 0.5) * self.cell_size
        grid_origin_y = oy + full_h / 2.0 - (half_y + 0.5) * self.cell_size

        num_x = math.ceil(full_w / self.cell_size)
        num_y = math.ceil(full_h / self.cell_size)
        samples_per_cell = max(1, int(self.cell_size / resolution))

        marker_array = MarkerArray()
        blue_cells = []
        blue_set = set()
        cell_positions = {}

        mid_robot_x, mid_robot_y = self.robot_position

        for cx in range(num_x):
            for cy in range(num_y):
                cx_world = grid_origin_x + (cx + 0.5) * self.cell_size
                cy_world = grid_origin_y + (cy + 0.5) * self.cell_size

                free_cnt = total_cnt = 0
                for dx in range(-samples_per_cell//2, samples_per_cell//2 + 1):
                    for dy in range(-samples_per_cell//2, samples_per_cell//2 + 1):
                        mx = int((cx_world + dx * resolution - ox) / resolution)
                        my = int((cy_world + dy * resolution - oy) / resolution)
                        if 0 <= mx < w and 0 <= my < h:
                            val = data[my * w + mx]
                            total_cnt += 1
                            if val == 0:
                                free_cnt += 1

                if free_cnt == total_cnt and total_cnt > 0:
                    color = (0.0, 0.0, 1.0)  # Blue
                    blue_cells.append((cx_world, cy_world))
                    blue_set.add((round(cx_world,3), round(cy_world,3)))
                    cell_positions[(round(cx_world,3), round(cy_world,3))] = (cx_world, cy_world)
                elif free_cnt > 0:
                    color = (0.0, 1.0, 0.0)  # Green
                else:
                    color = (1.0, 0.0, 0.0)  # Red

                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "cell_centers"
                m.id = cx * num_y + cy
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = cx_world
                m.pose.position.y = cy_world
                m.pose.position.z = 0.1
                m.scale.x = m.scale.y = m.scale.z = 0.3
                m.color.r, m.color.g, m.color.b = color
                m.color.a = 1.0
                marker_array.markers.append(m)

        # Build adjacency for blue
        adjacency = defaultdict(list)
        for x,y in blue_cells:
            p = (round(x,3), round(y,3))
            for dx, dy in [(self.cell_size,0),(-self.cell_size,0),(0,self.cell_size),(0,-self.cell_size)]:
                nb = (round(x+dx,3), round(y+dy,3))
                if nb in blue_set:
                    adjacency[p].append(nb)

        # Choose root closest to robot position
        start = None
        min_d = float('inf')
        for p in blue_set:
            dist = math.hypot(p[0] - mid_robot_x, p[1] - mid_robot_y)
            if dist < min_d:
                min_d = dist
                start = p

        # BFS Tree
        tree_edges = []
        if start:
            vis = {start}
            dq = deque([start])
            while dq:
                cur = dq.popleft()
                for nb in adjacency[cur]:
                    if nb not in vis:
                        vis.add(nb)
                        tree_edges.append((cur, nb))
                        dq.append(nb)

        # Publish TREE lines as orange LINE_LIST
        tree_marker = Marker()
        tree_marker.header.frame_id = "map"
        tree_marker.header.stamp = self.get_clock().now().to_msg()
        tree_marker.ns = "spanning_tree"
        tree_marker.id = 0
        tree_marker.type = Marker.LINE_LIST
        tree_marker.action = Marker.ADD
        tree_marker.scale.x = 0.05
        tree_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)  # orange
        for a,b in tree_edges:
            pt1 = Point(x=a[0], y=a[1], z=0.1)
            pt2 = Point(x=b[0], y=b[1], z=0.1)
            tree_marker.points.append(pt1)
            tree_marker.points.append(pt2)
        marker_array.markers.append(tree_marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published grid + spanning tree with root at {start}')

def main(args=None):
    rclpy.init(args=args)
    node = MapSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
