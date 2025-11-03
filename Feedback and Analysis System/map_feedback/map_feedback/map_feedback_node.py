import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, String

class MapFeedbackNode(Node):
    def __init__(self):
        super().__init__('map_feedback_node')

        # listen to SLAM/map
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

        # publish human-readable text
        self.text_pub = self.create_publisher(
            String,
            '/map_feedback_text',
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
                # unknown cell
                continue
            known += 1
            if v == 0:
                free += 1
            else:
                occ += 1

        coverage = (known / total) * 100.0
        free_pct = (free / total) * 100.0
        occ_pct = (occ / total) * 100.0

        # 1) publish numeric coverage
        cov_msg = Float32()
        cov_msg.data = coverage
        self.coverage_pub.publish(cov_msg)

        # 2) publish readable text
        line = (
            f"coverage={coverage:.1f}% "
            f"free={free_pct:.1f}% "
            f"occ={occ_pct:.1f}% "
            f"cells={total}"
        )
        txt_msg = String()
        txt_msg.data = line
        self.text_pub.publish(txt_msg)

        # 3) log to terminal
        self.get_logger().info(line)


def main():
    rclpy.init()
    node = MapFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()