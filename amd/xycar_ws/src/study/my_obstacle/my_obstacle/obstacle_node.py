#!/usr/bin/env python3
"""라이다 장애물 노드.

/scan 을 섹터로 나눠 최근접 거리를 낸다. 카메라(YOLO)가 "무엇"을 알려주고
여기서 "얼마나 먼가"를 알려주는 상보 관계다.

발행:
    /obstacle  Float32MultiArray [front_dist, left_free, right_free]
               단위 m. 비어 있으면 range_max 값이 들어간다.

판단(정지할지 말지)은 여기서 하지 않는다. my_driver 의 몫이다.
여기는 관측만 낸다 — 층별 책임 분리 원칙.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from my_obstacle.sectors import sector_min


class ObstacleNode(Node):

    def __init__(self):
        super().__init__("obstacle_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("scan_topic", "/scan"),
                # 섹터 각도 (도). 라이다 정면 0, 좌측 +
                ("front_lo_deg", -15.0),
                ("front_hi_deg", 15.0),
                ("left_lo_deg", 25.0),
                ("left_hi_deg", 70.0),
                ("right_lo_deg", -70.0),
                ("right_hi_deg", -25.0),
                # 유효 거리
                ("range_min", 0.05),
                ("range_max", 10.0),
                ("min_points", 3),
                ("log_period_sec", 2.0),
            ],
        )

        g = self.get_parameter
        self.front = (g("front_lo_deg").value, g("front_hi_deg").value)
        self.left = (g("left_lo_deg").value, g("left_hi_deg").value)
        self.right = (g("right_lo_deg").value, g("right_hi_deg").value)
        self.range_min = g("range_min").value
        self.range_max = g("range_max").value
        self.min_points = g("min_points").value
        self._log_period = g("log_period_sec").value

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(Float32MultiArray, "obstacle", reliable_qos)
        self.create_subscription(
            LaserScan, g("scan_topic").value, self.on_scan, sensor_qos
        )

        self._last_log = self.get_clock().now()
        self.get_logger().info("obstacle_node 시작")

    def on_scan(self, msg):
        def sec(lo_hi):
            return sector_min(
                msg.ranges, msg.angle_min, msg.angle_increment,
                lo_hi[0], lo_hi[1],
                max(self.range_min, msg.range_min),
                min(self.range_max, msg.range_max),
                self.min_points,
            )

        front = sec(self.front)
        left = sec(self.left)
        right = sec(self.right)

        self.pub.publish(Float32MultiArray(data=[float(front), float(left), float(right)]))

        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds / 1e9 >= self._log_period:
            self.get_logger().info(
                f"front={front:5.2f}m  left={left:5.2f}m  right={right:5.2f}m"
            )
            self._last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
