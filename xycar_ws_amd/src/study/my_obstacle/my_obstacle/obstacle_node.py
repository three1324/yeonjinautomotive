#!/usr/bin/env python3
"""라이다 장애물 노드.

/scan 을 섹터로 나눠 최근접 거리를 낸다. 카메라(YOLO)가 "무엇"을 알려주고
여기서 "얼마나 먼가"를 알려주는 상보 관계다.

발행:
    /obstacle  Float32MultiArray [front_dist, left_free, right_free]
               단위 m. 비어 있으면 range_max 값이 들어간다.
    /corridor  Float32MultiArray [offset_near, offset_far, valid, quality]
               라바콘이 이루는 복도의 중앙. **단위는 픽셀** — /lane 과 같은
               형식이라 driver 가 두 기준을 그대로 섞을 수 있다.

판단(정지할지 말지, 차선 대신 복도를 따를지)은 여기서 하지 않는다.
my_driver 의 몫이다. 여기는 관측만 낸다 — 층별 책임 분리 원칙.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from my_obstacle.corridor import CorridorEstimator
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
                # --- 라바콘 복도 추정 ---
                ("corridor.enabled", True),
                ("corridor.x_min", 0.25),
                ("corridor.x_max", 2.2),
                ("corridor.max_lateral", 1.5),
                ("corridor.min_lateral", 0.06),
                ("corridor.bin_size", 0.15),
                ("corridor.min_bins", 4),
                ("corridor.min_span_m", 0.5),
                ("corridor.min_points_per_side", 1),
                ("corridor.eval_near_m", 0.6),
                ("corridor.eval_far_m", 1.5),
                ("corridor.px_per_meter", 300.0),
                ("corridor.nominal_half_width_m", 0.35),
                ("corridor.min_both_bins", 3),
                ("corridor.hold_frames", 10),
                ("corridor.max_jump_px", 250.0),
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
        self.pub_corridor = self.create_publisher(
            Float32MultiArray, "corridor", reliable_qos)
        self.create_subscription(
            LaserScan, g("scan_topic").value, self.on_scan, sensor_qos
        )

        self.corridor_enabled = g("corridor.enabled").value
        self.corridor = CorridorEstimator(
            x_min=g("corridor.x_min").value,
            x_max=g("corridor.x_max").value,
            max_lateral=g("corridor.max_lateral").value,
            min_lateral=g("corridor.min_lateral").value,
            bin_size=g("corridor.bin_size").value,
            min_bins=g("corridor.min_bins").value,
            min_span_m=g("corridor.min_span_m").value,
            min_points_per_side=g("corridor.min_points_per_side").value,
            eval_near_m=g("corridor.eval_near_m").value,
            eval_far_m=g("corridor.eval_far_m").value,
            px_per_meter=g("corridor.px_per_meter").value,
            nominal_half_width_m=g("corridor.nominal_half_width_m").value,
            min_both_bins=g("corridor.min_both_bins").value,
            hold_frames=g("corridor.hold_frames").value,
            max_jump_px=g("corridor.max_jump_px").value,
        )

        self._last_log = self.get_clock().now()
        self.get_logger().info(
            f"obstacle_node 시작 (복도추정 {'on' if self.corridor_enabled else 'off'})")

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

        cor = None
        if self.corridor_enabled:
            cor = self.corridor.update(
                msg.ranges, msg.angle_min, msg.angle_increment,
                max(self.range_min, msg.range_min),
                min(self.range_max, msg.range_max),
            )
            self.pub_corridor.publish(Float32MultiArray(data=[
                float(cor.offset_near),
                float(cor.offset_far),
                1.0 if cor.valid else 0.0,
                float(cor.quality),
            ]))

        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds / 1e9 >= self._log_period:
            line = f"front={front:5.2f}m  left={left:5.2f}m  right={right:5.2f}m"
            if cor is not None:
                line += (f" | 복도 {'OK ' if cor.valid else 'HOLD'}"
                         f" off={cor.offset_near:+6.0f}/{cor.offset_far:+6.0f}px"
                         f" q{cor.quality:.2f} 폭{cor.width_m:.2f}m 구간{cor.n_bins}")
            self.get_logger().info(line)
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
