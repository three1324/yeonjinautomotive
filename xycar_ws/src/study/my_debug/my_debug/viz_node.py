#!/usr/bin/env python3
"""RViz2 피더 노드 — 주행 중 상태를 RViz2 가 그릴 수 있는 표준 메시지로 바꾼다.

pipeline_view_node 가 **카메라 화면**(YOLO 검출 + 차선/판단 텍스트)을 담당한다면,
이 노드는 **공간(top-down) 화면**을 담당한다. 둘은 보는 축이 다르므로 둘 다 띄운다.

    ros2 run my_debug viz_node

발행 (전부 /viz 아래):
    /viz/scan_cloud   PointCloud2   /scan 을 3D 포인트로 변환 (RViz PointCloud2 패널용)
    /viz/ref_path     Path          my_obstacle 이 낸 라바콘 복도 중앙선. corridor 토픽의
                                    픽셀 오프셋을 px_per_meter 로 **미터로 되돌린** 것이라
                                    "변환 정보"를 눈으로 검증하는 용도다.
    /viz/plan_path    Path          현재 조향/속도 명령을 자전거 모델로 적분한 예측 경로.
                                    차가 지금 명령대로 가면 어디로 가는지 = 계획 경로.
    /viz/driven_path  Path          지금까지 지나온 궤적 (odom 프레임).
    /viz/odom         Odometry      odom 토픽이 없을 때만. 아래 '오도메트리' 참고.
    /viz/markers      MarkerArray   FSM 상태·조향·속도 텍스트, 전방거리 표식, 차체,
                                    그리고 **현재 추종 기준을 색으로 보여주는 선**
                                    (lane=노랑 / blend=주황 / corridor=빨강).

구독:
    /scan        LaserScan            (param scan_topic)
    /corridor    Float32MultiArray    my_obstacle — [off_near, off_far, valid, quality] px
    /obstacle    Float32MultiArray    my_obstacle — [front, left_free, right_free] m
    /debug_state String(JSON)         my_driver  — 판단/제어 상태 전체
    /odom        Odometry             (param odom_topic) 있으면 그대로 쓴다

오도메트리:
    이 차량은 기본 구성에 오도메트리가 없다(SLAM 을 켜야 /odom_rf2o 가 생긴다).
    그래서 odom 토픽이 `odom_timeout_sec` 동안 안 오면 **직접 발행한 조향/속도
    명령을 자전거 모델로 적분해** /viz/odom 과 odom->base_link TF 를 낸다.
    추측항법이라 시간이 지나면 반드시 어긋난다 — RViz 에서 경로 모양을 보기 위한
    것이지 측위가 아니다. 진짜 측위가 필요하면 slam:=true 로 띄우고
    odom_topic:=/odom_rf2o 를 주면 이 노드는 추측항법을 끄고 그쪽을 따라간다.
"""

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


# 지금 누가 차를 몰고 있는지 -> 색. (driver_node 의 debug_state.cone_zone)
# 숫자보다 색이 먼저 눈에 들어온다 — 콘 구간 진입/이탈이 한눈에 보여야
# 튜닝 중 판단이 빨라진다.
_LANE_COLOR = (1.0, 1.0, 0.2)      # 노랑 — 카메라 차선 (driver_node)
_CONE_COLOR = (1.0, 0.2, 0.2)      # 빨강 — 라이다 (rubbercone_node)


def _yaw_to_quat(yaw):
    """2D 이므로 z/w 만 채운다."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class VizNode(Node):

    def __init__(self):
        super().__init__("viz_node")

        g = self.declare_parameters(
            namespace="",
            parameters=[
                ("scan_topic", "/scan"),
                ("odom_topic", "/odom"),
                ("odom_frame", "odom"),
                ("base_frame", "base_link"),
                ("laser_frame", "laser_frame"),
                ("rate_hz", 20.0),
                # --- 명령(임의 단위) -> 물리량 변환 ---
                # xycar_motor 의 [angle, speed] 는 단위가 없는 값이다(§my_driver/control.py).
                # 예측 경로를 미터로 그리려면 환산이 필요하고, 그 환산값은 차량마다 다르다.
                # 여기 기본값은 젯슨 차량 실측 기준(drive_params.yaml 주석):
                #   speed 12 -> 약 0.96 m/s  =>  speed_to_mps 0.08
                ("angle_limit", 50.0),        # driver_node 의 steer.angle_limit 과 맞출 것
                ("max_steer_deg", 19.5),      # [실측] angle_limit 일 때의 실제 앞바퀴 조향각
                ("speed_to_mps", 0.08),
                ("wheelbase_m", 0.333),       # [실측] 축거 33.3cm
                # angle 부호 -> 차량 회전 방향. +angle 이 우회전이면 -1
                # (RViz 에서 예측 경로가 실제와 반대로 휘면 이 부호를 뒤집을 것)
                ("angle_sign", -1.0),
                # --- 예측 경로 ---
                ("plan_horizon_sec", 2.0),
                ("plan_dt", 0.1),
                ("plan_min_speed_mps", 0.3),  # 정지 중에도 조향 방향은 보이게 하는 최소 속도
                # --- 지나온 궤적 ---
                ("driven_path_max_poses", 2000),
                ("driven_path_min_step_m", 0.05),
                # --- 복도 중앙선 (my_obstacle 과 같은 값을 넣어야 미터 환산이 맞다) ---
                ("corridor.px_per_meter", 300.0),
                ("corridor.eval_near_m", 0.6),
                ("corridor.eval_far_m", 1.5),
                # odom 토픽이 이 시간 동안 안 오면 추측항법으로 전환
                ("odom_timeout_sec", 1.0),
            ],
        )
        del g

        p = self.get_parameter
        self.odom_frame = p("odom_frame").value
        self.base_frame = p("base_frame").value
        self.laser_frame = p("laser_frame").value
        self.angle_limit = float(p("angle_limit").value)
        self.max_steer = math.radians(float(p("max_steer_deg").value))
        self.speed_to_mps = float(p("speed_to_mps").value)
        self.wheelbase = float(p("wheelbase_m").value)
        self.angle_sign = float(p("angle_sign").value)
        self.plan_horizon = float(p("plan_horizon_sec").value)
        self.plan_dt = float(p("plan_dt").value)
        self.plan_min_speed = float(p("plan_min_speed_mps").value)
        self.driven_max = int(p("driven_path_max_poses").value)
        self.driven_step = float(p("driven_path_min_step_m").value)
        self.px_per_meter = float(p("corridor.px_per_meter").value)
        self.eval_near_m = float(p("corridor.eval_near_m").value)
        self.eval_far_m = float(p("corridor.eval_far_m").value)
        self.odom_timeout = float(p("odom_timeout_sec").value)

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

        # --- 상태 ---
        self._state = None            # /debug_state JSON
        self._corridor = None         # [off_near, off_far, valid, quality]
        self._obstacle = None         # [front, left_free, right_free]
        self._corridor_path = None    # my_obstacle 이 낸 복도 중앙선
        self._last_odom_time = None   # 외부 odom 수신 시각 (None 이면 추측항법)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_tick = self.get_clock().now()
        self._driven = []             # [(x, y)]

        # --- 구독 ---
        self.create_subscription(
            LaserScan, p("scan_topic").value, self.on_scan, sensor_qos)
        self.create_subscription(
            Odometry, p("odom_topic").value, self.on_odom, sensor_qos)
        self.create_subscription(String, "debug_state", self.on_state, 10)
        self.create_subscription(
            Float32MultiArray, "corridor", self.on_corridor, reliable_qos)
        self.create_subscription(
            Float32MultiArray, "obstacle", self.on_obstacle, reliable_qos)
        # my_obstacle 이 낸 복도 중앙선. 여기서는 색만 입혀 다시 그린다
        # (기하는 그쪽이 계산한 것을 그대로 쓴다 — 판단/계산 중복 금지).
        self.create_subscription(Path, "corridor_path", self.on_corridor_path, 1)

        # --- 발행 ---
        self.pub_cloud = self.create_publisher(PointCloud2, "/viz/scan_cloud", sensor_qos)
        self.pub_ref = self.create_publisher(Path, "/viz/ref_path", 1)
        self.pub_plan = self.create_publisher(Path, "/viz/plan_path", 1)
        self.pub_driven = self.create_publisher(Path, "/viz/driven_path", 1)
        self.pub_odom = self.create_publisher(Odometry, "/viz/odom", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/viz/markers", 1)
        self.tf = TransformBroadcaster(self)

        self.create_timer(1.0 / float(p("rate_hz").value), self._tick)
        self.get_logger().info(
            f"viz_node 시작 — odom_topic={p('odom_topic').value} "
            f"(없으면 {self.odom_timeout:.1f}s 후 추측항법으로 전환)")

    # ------------------------------------------------------------------ 구독

    def on_scan(self, msg):
        """LaserScan -> PointCloud2. RViz 의 PointCloud2 패널에 등록해서 본다."""
        n = len(msg.ranges)
        if n == 0:
            return
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        ok = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not ok.any():
            return
        r = ranges[ok]
        a = angles[ok]
        pts = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
        header = Header(stamp=msg.header.stamp,
                        frame_id=msg.header.frame_id or self.laser_frame)
        self.pub_cloud.publish(point_cloud2.create_cloud_xyz32(header, pts))

    def on_odom(self, msg):
        """외부 오도메트리가 있으면 그쪽이 진실이다 — 추측항법을 끈다."""
        self._last_odom_time = self.get_clock().now()
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_state(self, msg):
        try:
            self._state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

    def on_corridor(self, msg):
        self._corridor = list(msg.data)

    def on_obstacle(self, msg):
        self._obstacle = list(msg.data)

    def on_corridor_path(self, msg):
        self._corridor_path = msg

    # ------------------------------------------------------------------ 주기

    def _dead_reckoning(self):
        """외부 odom 이 없을 때만 참. 명령값을 적분한 추정이라 오차가 누적된다."""
        if self._last_odom_time is None:
            return True
        age = (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
        return age > self.odom_timeout

    def _cmd(self):
        """현재 조향각(rad, 좌+)과 속도(m/s). /debug_state 가 없으면 (0, 0)."""
        if self._state is None:
            return 0.0, 0.0
        angle = float(self._state.get('angle') or 0.0)
        speed = float(self._state.get('speed') or 0.0)
        ratio = max(-1.0, min(1.0, angle / self.angle_limit))
        return self.angle_sign * ratio * self.max_steer, speed * self.speed_to_mps

    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds / 1e9
        self._last_tick = now
        steer, v = self._cmd()

        if self._dead_reckoning() and 0.0 < dt < 1.0:
            # 자전거 모델: 뒷바퀴축 기준
            self._yaw += v / self.wheelbase * math.tan(steer) * dt
            self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))
            self._x += v * math.cos(self._yaw) * dt
            self._y += v * math.sin(self._yaw) * dt
            self._publish_odom_and_tf(now)

        self._publish_driven_path(now)
        self._publish_plan_path(now, steer, v)
        self._publish_ref_path(now)
        self._publish_markers(now)

    # ------------------------------------------------------------------ 발행

    def _publish_odom_and_tf(self, now):
        qx, qy, qz, qw = _yaw_to_quat(self._yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.pub_odom.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)

    def _path(self, now, frame, points):
        """(x, y, yaw) 목록 -> Path."""
        path = Path()
        path.header.stamp = now.to_msg()
        path.header.frame_id = frame
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            qx, qy, qz, qw = _yaw_to_quat(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)
        return path

    def _publish_driven_path(self, now):
        if not self._driven or math.hypot(self._x - self._driven[-1][0],
                                          self._y - self._driven[-1][1]) >= self.driven_step:
            self._driven.append((self._x, self._y))
            if len(self._driven) > self.driven_max:
                del self._driven[0:len(self._driven) - self.driven_max]
        self.pub_driven.publish(
            self._path(now, self.odom_frame, [(x, y, 0.0) for x, y in self._driven]))

    def _publish_plan_path(self, now, steer, v):
        """지금 명령대로 갔을 때의 예측 궤적 (base_link 기준)."""
        speed = max(abs(v), self.plan_min_speed) * (1.0 if v >= 0.0 else -1.0)
        x = y = yaw = 0.0
        pts = [(0.0, 0.0, 0.0)]
        steps = max(1, int(self.plan_horizon / self.plan_dt))
        for _ in range(steps):
            yaw += speed / self.wheelbase * math.tan(steer) * self.plan_dt
            x += speed * math.cos(yaw) * self.plan_dt
            y += speed * math.sin(yaw) * self.plan_dt
            pts.append((x, y, yaw))
        self.pub_plan.publish(self._path(now, self.base_frame, pts))

    def _publish_ref_path(self, now):
        """복도 중앙선. 픽셀 오프셋을 미터로 되돌린다(부호 규약은 corridor.py 와 동일).

        corridor.py:  offset_px = -y_center_m * px_per_meter   =>  y = -px / ppm
        """
        c = self._corridor
        if not c or len(c) < 3 or c[2] < 0.5 or self.px_per_meter <= 0.0:
            self.pub_ref.publish(self._path(now, self.base_frame, []))
            return
        y_near = -c[0] / self.px_per_meter
        y_far = -c[1] / self.px_per_meter
        pts = [(0.0, 0.0, 0.0),
               (self.eval_near_m, y_near, 0.0),
               (self.eval_far_m, y_far, 0.0)]
        self.pub_ref.publish(self._path(now, self.base_frame, pts))

    def _marker(self, now, ns, mid, mtype, frame=None):
        m = Marker()
        m.header.stamp = now.to_msg()
        m.header.frame_id = frame or self.base_frame
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.a = 1.0
        return m

    def _publish_markers(self, now):
        arr = MarkerArray()

        # 1) 상태 텍스트 — pipeline_view_node 하단 바와 같은 정보를 RViz 안에도 띄운다
        text = self._marker(now, "state", 0, Marker.TEXT_VIEW_FACING)
        text.pose.position.x = 0.0
        text.pose.position.z = 0.8
        text.scale.z = 0.12
        text.color.r = text.color.g = text.color.b = 1.0
        s = self._state
        if s is None:
            text.text = "waiting /debug_state ..."
        else:
            text.text = (
                f"{s.get('state', '?')}  angle={s.get('angle', 0):+.1f} "
                f"speed={s.get('speed', 0):.1f}\n"
                f"{'CONE(lidar)' if s.get('cone_zone') else 'LANE(cam)'} "
                f"{'OK' if s.get('valid') else 'HOLD'}\n"
                f"light={s.get('light', '?')} "
                f"cone={s.get('cone_n', 0)}\n"
                + (f"AVOID {'RIGHT' if s.get('ot_dir', 0) > 0 else 'LEFT'} "
                   f"{s.get('ot_amount', 0):.0f}px "
                   f"(half={s.get('half_near', 0):.0f}) "
                   f"{s.get('ot_reason', '')}\n"
                   if s.get('overtake', '-') != '-' else "")
                +
                # reason 이 "왜 안 움직이는가"의 답이다 (disabled / wait /
                # no_lane_yet / stale / ref lost ...). 빼면 화면만 보고는 알 수 없다.
                f"why: {s.get('reason', '-')}"
            )
        arr.markers.append(text)

        # 2) 전방 최근접 거리 표식
        front = self._marker(now, "front", 1, Marker.SPHERE)
        front.scale.x = front.scale.y = front.scale.z = 0.12
        front.color.r = 1.0
        front.color.g = 0.2
        front.color.b = 0.2
        d = self._obstacle[0] if self._obstacle else 0.0
        front.pose.position.x = float(d)
        arr.markers.append(front)

        # 3) 차체 외형 (대략) — 스케일 감을 잡기 위한 것
        body = self._marker(now, "body", 2, Marker.CUBE)
        body.scale.x = 0.50
        body.scale.y = 0.28
        body.scale.z = 0.15
        body.pose.position.x = self.wheelbase * 0.5
        body.pose.position.z = 0.075
        body.color.r = 0.2
        body.color.g = 0.6
        body.color.b = 1.0
        body.color.a = 0.4
        arr.markers.append(body)

        # 4) 지금 무엇을 따르고 있는지 — 복도 중앙선을 **기준별 색**으로 덧그린다.
        #    숫자(w=0.62)보다 색이 먼저 눈에 들어온다. 콘 구간 진입/이탈이
        #    한눈에 보여야 튜닝 중 판단이 빨라진다.
        mode = self._marker(now, "mode", 3, Marker.LINE_STRIP,
                            frame=(self._corridor_path.header.frame_id
                                   if self._corridor_path else None))
        mode.scale.x = 0.04
        r, g, b = (_CONE_COLOR if (s or {}).get("cone_zone")
                   else _LANE_COLOR)
        mode.color.r, mode.color.g, mode.color.b = r, g, b
        if self._corridor_path is not None:
            for pose in self._corridor_path.poses:
                mode.points.append(pose.pose.position)
        arr.markers.append(mode)

        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C / SIGTERM 은 정상 종료 경로다. 종료 중 타이머가 한 번 더 돌면
        # publish 가 무효 컨텍스트로 던지는데, 그것까지 여기서 삼킨다.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
