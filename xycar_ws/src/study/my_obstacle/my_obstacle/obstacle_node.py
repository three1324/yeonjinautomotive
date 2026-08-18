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
    /corridor_path  nav_msgs/Path      복도 중앙선을 라이다 좌표로 찍은 경로.
    /cone_walls     MarkerArray        좌/우 벽으로 채택된 점 + 중앙선.
               ↑ 둘 다 **시각화·진단 전용**이다 (publish_viz=true 일 때만).
                 제어는 위 /corridor 픽셀 오프셋만 쓴다 — 경로 추종으로 바꾸면
                 차선 추종과 제어기가 갈라지기 때문이다.

판단(정지할지 말지, 차선 대신 복도를 따를지)은 여기서 하지 않는다.
my_driver 의 몫이다. 여기는 관측만 낸다 — 층별 책임 분리 원칙.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

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
                ("corridor.wall_min_bins", 4),
                ("corridor.refine_iters", 2),
                ("corridor.refine_samples", 14),
                ("corridor.path_samples", 12),
                ("publish_viz", True),
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

        # 시각화 토픽. 라이다는 10Hz 라 점 수십 개를 내는 비용은 무시할 만하다.
        self.publish_viz = g("publish_viz").value
        self.pub_path = None
        self.pub_walls = None
        if self.publish_viz:
            self.pub_path = self.create_publisher(Path, "corridor_path", 1)
            self.pub_walls = self.create_publisher(MarkerArray, "cone_walls", 1)
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
            wall_min_bins=g("corridor.wall_min_bins").value,
            refine_iters=g("corridor.refine_iters").value,
            refine_samples=g("corridor.refine_samples").value,
            path_samples=g("corridor.path_samples").value,
            px_per_meter=g("corridor.px_per_meter").value,
            nominal_half_width_m=g("corridor.nominal_half_width_m").value,
            min_both_bins=g("corridor.min_both_bins").value,
            hold_frames=g("corridor.hold_frames").value,
            max_jump_px=g("corridor.max_jump_px").value,
        )

        self._last_log = self.get_clock().now()
        self.get_logger().info(
            f"obstacle_node 시작 (복도추정 {'on' if self.corridor_enabled else 'off'})")

    def _marker(self, header, ns, mid, mtype, scale, rgb):
        m = Marker()
        m.header = header
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = 1.0
        return m

    def _publish_viz(self, header, cor):
        """복도 중앙선(Path) + 벽으로 채택된 점(MarkerArray). 진단 전용.

        숫자만 보고는 "왜 저 오프셋이 나왔는가"를 알 수 없다. 어느 점을 벽으로
        골랐는지 눈으로 봐야 x_max / bin_size / max_lateral 을 튜닝할 수 있다.
        """
        path = Path()
        path.header = header
        for x, y in cor.centerline:
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub_path.publish(path)

        arr = MarkerArray()
        for name, pts, rgb, mid in (
                ('left_wall', cor.left_pts, (0.2, 1.0, 0.2), 0),
                ('right_wall', cor.right_pts, (1.0, 0.3, 0.2), 1)):
            m = self._marker(header, name, mid, Marker.SPHERE_LIST, 0.07, rgb)
            # 벽 점이 비어 있으면(=walls 경로가 아니었으면) 빈 마커를 내서 지운다.
            for x, y in pts:
                p = PoseStamped().pose.position
                p.x, p.y = x, y
                m.points.append(p)
            arr.markers.append(m)

        line = self._marker(header, "centerline", 2, Marker.LINE_STRIP, 0.03,
                            (1.0, 1.0, 0.2))
        for x, y in cor.centerline:
            p = PoseStamped().pose.position
            p.x, p.y = x, y
            line.points.append(p)
        arr.markers.append(line)
        self.pub_walls.publish(arr)

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
            if self.publish_viz:
                self._publish_viz(msg.header, cor)

        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds / 1e9 >= self._log_period:
            line = f"front={front:5.2f}m  left={left:5.2f}m  right={right:5.2f}m"
            if cor is not None:
                line += (f" | 복도 {'OK ' if cor.valid else 'HOLD'}"
                         f" off={cor.offset_near:+6.0f}/{cor.offset_far:+6.0f}px"
                         f" q{cor.quality:.2f} 폭{cor.width_m:.2f}m 구간{cor.n_bins}"
                         f" [{cor.method}]")
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
