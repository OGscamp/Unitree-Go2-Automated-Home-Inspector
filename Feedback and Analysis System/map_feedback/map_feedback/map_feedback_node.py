import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray


class MapFeedbackNode(Node):
    def __init__(self):
        super().__init__('map_feedback_node')

        # listen to SLAM map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # publish coverage as a number
        self.coverage_pub = self.create_publisher(
            Float32,
            '/map_coverage',
            10
        )

        # human-readable text
        self.text_pub = self.create_publisher(
            String,
            '/map_feedback_text',
            10
        )

        # publish floating text marker
        self.marker_pub = self.create_publisher(
            Marker,
            '/map_feedback_marker',
            10
        )

        # publish unknown cells to visualize unmapped borders
        self.frontier_pub = self.create_publisher(
            MarkerArray,
            '/map_frontiers',
            10
        )

        self.get_logger().info('map_feedback_node started, waiting for /map')

    def map_callback(self, msg: OccupancyGrid):
        data = msg.data
        total = len(data)
        if total == 0:
            return

        known = 0
        free = 0
        occ = 0
        for v in data:
            if v == -1:
                continue
            known += 1
            if v == 0:
                free += 1
            else:
                occ += 1

        coverage = (known / total) * 100.0

        # 1) publish numeric
        cov_msg = Float32()
        cov_msg.data = coverage
        self.coverage_pub.publish(cov_msg)

        # 2) publish text
        line = (
            f"coverage={coverage:.1f}% "
            f"free={free/total*100.0:.1f}% "
            f"occ={occ/total*100.0:.1f}% "
            f"cells={total}"
        )
        txt_msg = String()
        txt_msg.data = line
        self.text_pub.publish(txt_msg)

        # terminal log
        self.get_logger().info(line)

        # 3) floating text marker
        marker = Marker()
        marker.header = msg.header
        marker.ns = "map_feedback"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = msg.info.origin.position.x + 0.5
        marker.pose.position.y = msg.info.origin.position.y + 0.5
        marker.pose.position.z = 1.5
        marker.scale.z = 0.4  # text height
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"Coverage {coverage:.1f}%"
        self.marker_pub.publish(marker)

        # 4) unknown cells next to known cells
        self.publish_frontiers(msg)

    def publish_frontiers(self, msg: OccupancyGrid):
        data = msg.data
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        origin = msg.info.origin

        markers = MarkerArray()
        marker_id = 0

        for y in range(height):
            for x in range(width):
                i = y * width + x

                # only unknown cells
                if data[i] != -1:
                    continue

                # check 4 neighbors to see if any is known
                neighbor_known = False
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx = x + dx
                    ny = y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        j = ny * width + nx
                        if data[j] >= 0:
                            neighbor_known = True
                            break

                if not neighbor_known:
                    continue

                # cell -> world
                wx = origin.position.x + (x + 0.5) * res
                wy = origin.position.y + (y + 0.5) * res

                m = Marker()
                m.header = msg.header
                m.ns = "map_frontiers"
                m.id = marker_id
                marker_id += 1
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = wx
                m.pose.position.y = wy
                m.pose.position.z = 0.05
                m.scale.x = res
                m.scale.y = res
                m.scale.z = 0.02
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.7
                markers.markers.append(m)

        self.frontier_pub.publish(markers)


def main():
    rclpy.init()
    node = MapFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()