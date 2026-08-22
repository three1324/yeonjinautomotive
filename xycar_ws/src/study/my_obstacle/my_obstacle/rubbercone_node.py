#!/usr/bin/env python3
"""라바콘 구간 전담 주행 노드 — 라이다로 콘 복도를 찾아 /cone_cmd 를 낸다.

driver_node 가 콘 구간(카메라 YOLO 판정)일 때 이 노드의 명령을 xycar_motor 로
**그대로 통과**시킨다. 이 노드는 모터에 직접 쏘지 않는다.

────────────────────────────────────────────────────────────────────────
2026-08-22 개편 — 무엇을 왜 바꿨나

실측 치수가 확정되면서(my_obstacle/geometry.py 상단) 기존 구현의 전제가
S자 코스에서 성립하지 않는다는 것이 드러났다.

  1) ★ Pure Pursuit 기준점이 틀렸다 (가장 큰 결함)
     옛 _steer_to_target() 은 **라이다 원점을 뒤축으로 취급**했다. 라이다는
     뒤축보다 0.41m 앞에 있으므로 alpha 가 과대평가돼 **약 2배 과조향**이
     났다(목표 (1.0,0.3): 10.4도 vs 올바른 5.5도). 좌우 여유가 0.20m 인
     복도에서 2배 과조향은 그대로 지그재그 접촉이다. steer_gain 을 아무리
     만져도 안 없어진다 — 기준점 자체가 틀렸기 때문.

  2) 좌우를 y 부호로 나눌 수 없다
     S자 커브에서 **바깥 벽 콘이 r=1.1m 부터 y>0 으로 넘어온다.**
     옛 `y > 0.1 / y < -0.1` 로는 반대편 줄로 들어가 목표가 뒤집힌다.
     -> 콘 사이 거리로 잇는 **사슬**로 교체 (geometry.build_chains).

  3) 부채꼴 ROI 가 커브 안쪽 벽을 잘라낸다
     옛 `max_angle_deg=100` 은 사실상 반원이라 옆·뒤 벽을 다 끌어왔고,
     그렇다고 직사각형으로만 자르면 커브에서 y 가 가파르게 오르는 **안쪽
     벽이 통째로 탈락**한다(검산: 좌벽 1개만 남음). 직사각형(|y|)에
     **거리 상한**을 얹어 모서리를 깎은 형태로 바꿨다.

  4) 콘 페어 1쌍이 아니라 중심선 전체를 만든다
     옛 코드는 "가장 가까운 좌콘 + x 가 비슷한 우콘" 한 쌍의 중점만 썼다.
     lookahead 가 콘 배치에 따라 프레임마다 달라져 응답이 예측 불가였다.
     이제 좌우 벽의 **최근접 대응**으로 중심선을 만들고, 그 위에서 뒤축
     기준 고정 거리(lookahead_dist_m)의 점을 목표로 삼는다.

  5) 편측 폴백이 커브에서 0.26m 틀렸다
     옛 코드는 y 축으로만 밀었다(`ly - offset`). 폴리라인 **법선**으로
     밀도록 고쳤다 (geometry.offset_from_single_wall 참고).

  6) 복도 폭 상한이 없었다
     `min_gap_m` 만 있어서, 한쪽 벽이 비면 반대편 트랙 콘과 짝지어 폭 1.5m
     짜리 중심선이 생길 수 있었다. `max_gap_m` 을 추가했다.

★ 롤백: `pairing_mode: nearest_pair` 로 두면 옛 페어링 경로가 그대로 돈다
  (실차에서 검증됐던 코드다). 재시작 없이 바꿀 수 있다:
      ros2 param set /rubbercone_node pairing_mode nearest_pair
  단 위 1)번(뒤축 변환)은 **스위치 없이 항상 적용**한다 — 옛 경로도 그
  오차 위에서 돌고 있었기 때문이다.

────────────────────────────────────────────────────────────────────────
코스 형상 (2026-08-22 확인) — 파라미터의 근거

  S자: 진입 -> 좌 -> 우 -> 좌.  반파장(좌로 갔다 중앙 복귀) 경로장 약 1.5m.
  역산하면 중심선 반경 R ~ 1.0~1.5m (횡변위 0.37~0.54m).

  그래서 lookahead 를 길게 잡으면 **변곡점 너머를 겨냥해 S자를 가로지른다.**
  0.85m(뒤축 기준)는 반파장의 28% 앞을 본다. 코너 컷은 lookahead^2/(8R),
  좌우 여유는 0.20m:
                 R=1.0     R=0.8     R=0.6     R=0.5
      Ld 0.75    0.070     0.088     0.117     0.141
      Ld 0.80    0.080     0.100     0.133     0.160
      Ld 0.85    0.090     0.113     0.151     0.181
      Ld 1.10    0.151     0.189     0.252     0.303   <- 채택 (2026-08-22)
  하한은 축거의 2배(0.67m) — 그 아래는 Pure Pursuit 이 진동한다.

  ★ [사용자 결정 2026-08-22] 0.85 -> 1.10. 위 표대로 **급커브에서는 코너
    컷이 여유(0.20m)를 넘는다.** 그럼에도 올린 이유는 조향 첨두를 낮춰
    VESC fault 2(UNDER_VOLTAGE)를 피하려는 것이다 — delta 가 ld 에 거의
    반비례하므로 0.85->1.10 은 조향을 약 23% 줄인다. 콘을 스치면 다시
    내리는 것이 아니라, angle_limit 이나 base_speed 쪽으로 옮겨 잡을 것.

  [2026-08-22 실차] 급커브에서 못 꺾고 안쪽 콘을 쳐서 0.85 -> 0.80 으로
  줄여봤으나, **0.85 고정으로 되돌렸다(사용자 결정).** 짧게 볼수록 직선
  응답이 나빠지고, 같은 건에서 고친 목표점 clamp 순서 버그(scan_callback
  의 ★ 주석) 쪽 영향이 훨씬 컸기 때문이다.
  곡률 적응 lookahead 는 **쓰지 않는다** — Ld 는 어떤 경우에도 코드가
  바꾸지 않는 고정값이다. 커브에서 남는 오차는 다른 수로 잡는다.

"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from my_obstacle import geometry as geo


class RubberconeNode(Node):
    def __init__(self):
        super().__init__('rubbercone_node')

        # ---- 공통 ----
        self.declare_parameter('scan_topic', 'scan')
        # ★ 기본값은 절대 'xycar_motor' 로 두지 않는다 (2026-08-21).
        #   원본 기본값이 모터 토픽이었다. params 파일이 한 번이라도 안 실리면
        #   이 노드가 driver_node 와 같이 모터에 쏘게 되고, 콘 구간이 아닌
        #   곳에서도 차가 라이다 명령대로 움직인다.
        self.declare_parameter('drive_topic', 'cone_cmd')
        self.declare_parameter('debug_topic', '/rubbercone/debug_image')
        # RViz 용 콘 마커. 콘 하나 = 큰 점 하나 (좌=파랑 / 우=빨강).
        self.declare_parameter('cone_marker_topic', '/rubbercone/cones')
        self.declare_parameter('cone_marker_size_m', 0.12)
        # 디버그 영상에 ROI 스캔점(회색)을 함께 그릴지. 기본은 끈다 —
        # 점이 깔리면 콘이 어디로 배정됐는지가 안 보인다.
        self.declare_parameter('debug_show_scan_points', False)
        self.declare_parameter('zone_topic', '/cone_zone_active')

        self.declare_parameter('angle_offset_deg', 0.0)

        # ---- ROI: 직사각형 + 거리 상한 ----
        # 부채꼴(max_angle_deg)을 버린 이유는 모듈 docstring 3) 참고.
        self.declare_parameter('forward_min', 0.15)
        # ★ [2026-08-22] 1.00 -> 2.50, 1.40 -> 2.50.
        #   깊은 굽이(R≈0.50m)에서 바깥쪽 벽이 거리 상한 밖으로 밀려나
        #   **한쪽 벽이 통째로 안 보였다.** 근거와 되돌리는 순서는
        #   drive_params.yaml 의 같은 항목에 적어 두었다.
        #   side_half_m 은 range_max_m 과 같아서 아무것도 안 자른다(무효).
        self.declare_parameter('side_half_m', 2.50)
        self.declare_parameter('range_max_m', 2.50)

        # ---- 클러스터링 + 콘 모양 필터 ----
        # 콘 표면 간 실제 간격은 0.425 - 0.10 = 0.325m 라 0.15 는 안전하다.
        self.declare_parameter('cluster_gap_m', 0.15)
        self.declare_parameter('cluster_min_points', 2)
        # 콘 하나의 span 은 0.10m. 옛 0.4 는 벽 조각을 콘으로 통과시켰다.
        self.declare_parameter('cluster_max_span_m', 0.20)
        # 반드시 콘 간격 0.425 보다 작아야 한다 (안 그러면 이웃 콘이 병합된다).
        self.declare_parameter('cone_merge_dist_m', 0.20)

        # ---- 좌/우 사슬 ----
        # 0.425(콘 간격) < 0.60 < 0.80(복도 폭). 곡률과 무관하게 성립한다.
        self.declare_parameter('cone_chain_max_dist_m', 0.60)
        self.declare_parameter('chain_extend_m', 0.55)
        self.declare_parameter('chain_reassign_dist_m', 0.30)
        self.declare_parameter('chain_min_cones', 2)

        # ---- 중심선 폭 검증 ----
        self.declare_parameter('min_gap_m', 0.70)
        self.declare_parameter('max_gap_m', 0.82)
        # 좌우 대응이 벽 접선과 이루는 |cos| 상한. 같은 벽 콘끼리 짝지어지는
        # 것을 거리가 아니라 **방향**으로 막는다 (geometry.centerline 참고).
        self.declare_parameter('centerline_max_tangent_cos', 0.5)
        self.declare_parameter('single_side_offset_m', 0.40)   # = 복도폭/2

        # ---- Pure Pursuit ----
        self.declare_parameter('wheelbase_m', 0.333)
        self.declare_parameter('lidar_to_rear_axle_m', 0.41)
        self.declare_parameter('lookahead_dist_m', 1.10)
        self.declare_parameter('lookahead_min_m', 0.3)
        # rad -> degree 변환 상수 그 자체다 (임의의 튜닝값이 아니다).
        # xycar_motor 의 angle 명령이 실제 조향각(도)과 1:1 임이 실측됐다.
        self.declare_parameter('steer_gain', 57.29578)
        self.declare_parameter('angle_limit', 32.0)   # [2026-08-22] 35.0 -> 32.0. 근거는 drive_params.yaml
        self.declare_parameter('invert_steer', False)

        # ---- 목표점 안정화 ----
        self.declare_parameter('target_smoothing_alpha', 0.5)
        self.declare_parameter('max_target_step_m', 0.20)

        # ---- 속도 ----
        self.declare_parameter('base_speed', 6.0)
        self.declare_parameter('min_speed_ratio', 0.4)
        # ★ 절대 하한 (2026-08-22). 비율만으로는 **모터 데드밴드 아래**로
        #   떨어진다 — 아래 scan_callback 의 clamp 주석 참고.
        self.declare_parameter('min_speed', 5.0)
        self.declare_parameter('lost_speed_ratio', 0.7)

        # ---- Follow-the-Gap 폴백 ----
        self.declare_parameter('car_width_m', 0.3)
        self.declare_parameter('max_steering_angle_rad', 0.4)

        # ---- 롤백 스위치 ----
        # 직전 목표점을 몇 프레임까지 들고 갈지 (10Hz 기준 5 = 0.5s).
        self.declare_parameter('prev_target_max_age', 5)
        self.declare_parameter('pairing_mode', 'chain')   # chain | nearest_pair
        self.declare_parameter('pair_max_x_diff_m', 0.5)  # nearest_pair 전용

        # ---- 구간 진입/이탈 감지 (진단 전용. driver_node 는 카메라로 판정한다) ----
        self.declare_parameter('zone_near_m', 0.2)
        self.declare_parameter('zone_far_m', 1.5)
        self.declare_parameter('zone_half_angle_deg', 55.0)
        self.declare_parameter('zone_point_threshold', 20)
        self.declare_parameter('zone_enter_frames', 5)
        self.declare_parameter('zone_exit_frames', 5)

        p = self.get_parameter
        self.scan_topic = p('scan_topic').value
        self.drive_topic = p('drive_topic').value
        self.debug_topic = p('debug_topic').value
        self.zone_topic = p('zone_topic').value

        self.angle_offset = math.radians(p('angle_offset_deg').value)

        self.forward_min = p('forward_min').value
        self.side_half = p('side_half_m').value
        self.range_max = p('range_max_m').value

        self.cluster_gap_m = p('cluster_gap_m').value
        self.cluster_min_points = p('cluster_min_points').value
        self.cluster_max_span_m = p('cluster_max_span_m').value
        self.cone_merge_dist_m = p('cone_merge_dist_m').value

        self.chain_max_dist = p('cone_chain_max_dist_m').value
        self.chain_extend = p('chain_extend_m').value
        self.chain_reassign = p('chain_reassign_dist_m').value
        self.chain_min_cones = p('chain_min_cones').value

        self.min_gap_m = p('min_gap_m').value
        self.centerline_max_tangent_cos = p('centerline_max_tangent_cos').value
        self.max_gap_m = p('max_gap_m').value
        self.single_side_offset_m = p('single_side_offset_m').value

        self.wheelbase = p('wheelbase_m').value
        self.axle_offset = p('lidar_to_rear_axle_m').value
        self.lookahead_dist = p('lookahead_dist_m').value
        self.lookahead_min = p('lookahead_min_m').value
        self.steer_gain = p('steer_gain').value
        self.angle_limit = p('angle_limit').value
        self.invert_steer = p('invert_steer').value

        self.target_smoothing_alpha = p('target_smoothing_alpha').value
        self.max_target_step_m = p('max_target_step_m').value

        self.base_speed = p('base_speed').value
        self.min_speed_ratio = p('min_speed_ratio').value
        self.min_speed = p('min_speed').value
        self.lost_speed_ratio = p('lost_speed_ratio').value

        self.car_width = p('car_width_m').value
        self.max_steering_angle_rad = p('max_steering_angle_rad').value

        self.pair_max_x_diff_m = p('pair_max_x_diff_m').value
        self.prev_target_max_age = p('prev_target_max_age').value

        self.zone_near_m = p('zone_near_m').value
        self.zone_far_m = p('zone_far_m').value
        self.zone_half_angle_rad = math.radians(p('zone_half_angle_deg').value)
        self.zone_point_threshold = p('zone_point_threshold').value
        self.zone_enter_frames = p('zone_enter_frames').value
        self.zone_exit_frames = p('zone_exit_frames').value

        self.cone_marker_size = p('cone_marker_size_m').value
        self.debug_show_scan_points = p('debug_show_scan_points').value
        # 마커를 스캔과 **같은 프레임**으로 낸다. 그러면 /viz/scan_cloud 가
        # 제대로 보이는 환경이면 새 TF 없이 그대로 겹쳐 보인다.
        self._scan_frame = 'laser_frame'

        self.bridge = CvBridge()
        self.smoothed_y = None
        self._prev_target = None
        self._prev_target_age = 0
        # 직전 프레임의 목표점. 편측 폴백에서 미는 방향을 시간 연속성으로
        # 가르는 데 쓴다 (geo.resolve_side). 오도메트리가 없어 프레임 간
        # 좌표 이동을 보정하지 않지만, 10Hz·저속에서 이동은 0.06m 수준이고
        # 두 후보는 0.80m 떨어져 있어 판정에는 충분하다.
        #
        # ★ 경로를 못 낸 프레임에도 **바로 버리지 않는다.** 버리면 다음
        #   프레임도 근거가 없어 또 거부하는 **연쇄**가 생긴다(합성검증에서
        #   경로없음이 15% -> 55% 로 폭증했다). prev_target_max_age 프레임
        #   까지는 들고 간다 — 그동안 차는 0.3m 남짓 움직이는데 두 후보는
        #   0.80m 떨어져 있어 여전히 충분히 가른다.
        self.zone_active = False
        self.dense_count = 0
        self.sparse_count = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos)

        self.drive_pub = self.create_publisher(Float32MultiArray, self.drive_topic, 1)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)
        self.zone_pub = self.create_publisher(Bool, self.zone_topic, 10)
        self.cone_marker_pub = self.create_publisher(
            MarkerArray, p('cone_marker_topic').value, 10)

        self.get_logger().info(
            'rubbercone_node started. scan: %s -> drive: %s (mode=%s, axle=%.2fm)'
            % (self.scan_topic, self.drive_topic,
               p('pairing_mode').value, self.axle_offset))

    # ============ 라이다 -> 차량 로컬 좌표 (전방=+x, 좌측=+y) ============
    def _scan_to_points(self, msg: LaserScan):
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            a = angle
            angle += msg.angle_increment
            if math.isinf(r) or math.isnan(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            a_corrected = a + self.angle_offset
            points.append((r * math.cos(a_corrected), r * math.sin(a_corrected)))
        return points

    def _apply_roi(self, points):
        """직사각형(전방 x, 폭 |y|) + 거리 상한. 모서리를 깎은 형태다.

        거리 상한이 왜 필요한가: 순수 직사각형이면 커브에서 **안쪽 벽이
        잘린다.** 안쪽 벽은 y 가 가파르게 올라가기 때문이다(검산에서 좌벽이
        1개만 남았다). 커브에서는 "얼마나 앞"이 아니라 "얼마나 가까이"가
        올바른 경계다.

        ★ [2026-08-22] 폭 상한(|y|)은 **지금 아무것도 안 자른다.**
          side_half_m 을 range_max_m 과 같게 뒀기 때문이다. 깊은 굽이에서
          폭 제한이 바깥쪽 벽 콘을 먼저 잘라내, 거리를 아무리 늘려도 각 벽
          3개가 안 되는 지점이 대부분이 됐다(154개 중 127개). 조건문은
          되돌릴 수 있게 남겨 둔다 — 근거와 복구 순서는 drive_params.yaml.
        """
        out = []
        for (x, y) in points:
            if x < self.forward_min:
                continue
            if abs(y) > self.side_half:
                continue
            if math.hypot(x, y) > self.range_max:
                continue
            out.append((x, y))
        return out

    # ==================== 클러스터링 + 콘 모양 필터 ====================
    def _cluster_and_filter(self, points):
        if not points:
            return []
        pts = sorted(points, key=lambda q: math.atan2(q[1], q[0]))

        clusters = []
        current = [pts[0]]
        for pt in pts[1:]:
            if math.hypot(pt[0] - current[-1][0],
                          pt[1] - current[-1][1]) <= self.cluster_gap_m:
                current.append(pt)
            else:
                clusters.append(current)
                current = [pt]
        clusters.append(current)

        cones = []
        for c in clusters:
            if len(c) < self.cluster_min_points:
                continue
            xs = [q[0] for q in c]
            ys = [q[1] for q in c]
            span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            if span > self.cluster_max_span_m:
                continue  # 벽/사람 등 큰 물체는 콘이 아니다
            cones.append((sum(xs) / len(xs), sum(ys) / len(ys)))
        return self._merge_nearby_cones(cones)

    def _merge_nearby_cones(self, cones):
        merged = list(cones)
        changed = True
        while changed and len(merged) > 1:
            changed = False
            for i in range(len(merged)):
                for j in range(i + 1, len(merged)):
                    d = math.hypot(merged[i][0] - merged[j][0],
                                   merged[i][1] - merged[j][1])
                    if d <= self.cone_merge_dist_m:
                        mx = (merged[i][0] + merged[j][0]) / 2.0
                        my = (merged[i][1] + merged[j][1]) / 2.0
                        merged.pop(j)
                        merged.pop(i)
                        merged.append((mx, my))
                        changed = True
                        break
                if changed:
                    break
        return merged

    # ==================== 경로 생성 ====================
    def _plan_path(self, cones):
        """콘 -> (중심선, 좌사슬, 우사슬, 출처). 중심선은 라이다 좌표."""
        left, right = geo.build_chains(
            cones, self.chain_max_dist, self.chain_extend,
            self.chain_reassign, self.chain_min_cones,
            self.min_gap_m, self.max_gap_m)

        if left and right:
            path = geo.centerline(left, right, self.min_gap_m,
                                  self.max_gap_m, self.chain_extend,
                                  self.centerline_max_tangent_cos)
            if path:
                return path, left, right, 'center'

        # 한쪽 벽만 — 커브 안쪽 벽은 콘이 원래 성기므로 **정상 경로**다.
        # 어느 쪽으로 밀지는 수직 판정 + 직전 목표점(시간 연속성)으로 정한다.
        # y 부호로 정하면 벽이 차에서 방사 방향으로 보이는 프레임에서 반대로
        # 밀려 목표점이 0.80m — 복도 폭만큼 — 어긋난다 (geo.resolve_side 참고).
        for chain in (left, right):
            if len(chain) >= 2:
                cs, conf = geo.corridor_side(chain)
                side = geo.resolve_side(chain, self.single_side_offset_m,
                                        cs, conf, self._prev_target)
                if side is None:
                    # 미는 방향의 근거가 없다. 추측하면 복도 폭만큼 반대로
                    # 간다 — 이 벽은 쓰지 않고 FTG 로 넘긴다.
                    continue
                path = geo.offset_from_single_wall(
                    chain, self.single_side_offset_m, side)
                if path:
                    return path, left, right, 'wall'
        return [], left, right, 'none'

    # ==================== 옛 페어링 (pairing_mode=nearest_pair) ====================
    def _find_pair_target(self, cones):
        """2026-08-22 이전의 구현. 실차 검증은 됐지만 S자에서 좌우가 뒤집힌다.

        롤백 경로로만 남긴다 — 모듈 docstring 의 '롤백' 참고.
        """
        left = sorted([(x, y) for (x, y) in cones if y > 0.1], key=lambda q: q[0])
        right = sorted([(x, y) for (x, y) in cones if y < -0.1], key=lambda q: q[0])

        if left and right:
            lc = left[0]
            rc = min(right, key=lambda q: abs(q[0] - lc[0]))
            if (abs(lc[0] - rc[0]) <= self.pair_max_x_diff_m
                    and (lc[1] - rc[1]) >= self.min_gap_m):
                return [((lc[0] + rc[0]) / 2.0, (lc[1] + rc[1]) / 2.0)], left, right, 'pair'
        if left:
            lx, ly = left[0]
            return [(lx, ly - self.single_side_offset_m)], left, right, 'pair'
        if right:
            rx, ry = right[0]
            return [(rx, ry + self.single_side_offset_m)], left, right, 'pair'
        return [], left, right, 'none'

    # ============ Follow-the-Gap 폴백 (콘을 하나도 못 찾았을 때) ============
    def _gap_follow_angle(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isnan(ranges), 0.0, ranges)
        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        ranges = np.clip(ranges, 0.0, msg.range_max)

        valid_mask = ranges > self.forward_min
        if not valid_mask.any():
            return 0.0
        masked = np.where(valid_mask, ranges, np.inf)
        closest_idx = int(np.argmin(masked))
        closest_dist = ranges[closest_idx]

        bubble_angle_rad = math.atan2(self.car_width / 2.0, max(closest_dist, 0.05))
        bubble_radius_idx = int(bubble_angle_rad / msg.angle_increment)
        b_start = max(0, closest_idx - bubble_radius_idx)
        b_end = min(len(ranges) - 1, closest_idx + bubble_radius_idx)
        processed = ranges.copy()
        processed[b_start:b_end + 1] = 0.0

        occ = (processed > self.forward_min).astype(np.int8)
        diff = np.diff(np.concatenate(([0], occ, [0])))
        gap_starts, gap_ends = np.where(diff == 1)[0], np.where(diff == -1)[0]
        if len(gap_starts) == 0:
            return 0.0
        largest = int(np.argmax(gap_ends - gap_starts))
        gs, ge = int(gap_starts[largest]), int(gap_ends[largest])
        if gs >= ge:
            best_idx = len(ranges) // 2
        else:
            seg = ranges[gs:ge]
            idxs = np.arange(gs, ge)
            best_idx = int(np.average(idxs, weights=seg)) if seg.sum() > 0 else (gs + ge) // 2

        steering_angle_rad = msg.angle_min + best_idx * msg.angle_increment
        mapped = -(steering_angle_rad / self.max_steering_angle_rad) * self.angle_limit
        return float(np.clip(mapped, -self.angle_limit, self.angle_limit))

    # ============ 구간 진입/이탈 감지 — 진단 전용 ============
    def _update_zone(self, all_points):
        """★ 이 판정은 **제어에 쓰이지 않는다.**

        ROI/클러스터 필터를 안 거친 원본 점을 전방 부채꼴에서 세기 때문에
        벽·기둥·사람이면 무엇이든 문턱을 넘는다. 실제로 콘이 없는 곳에서
        참이 돼 제어권이 넘어간 적이 있다(2026-08-19 2차 실차). 지금은
        driver_node 가 **카메라 YOLO 콘 개수**로 구간을 판정하고, 이 토픽은
        카메라 판정과 얼마나 어긋나는지 보려는 진단 로그로만 남는다.
        """
        if not all_points:
            sector_count = 0
        else:
            xs = np.array([q[0] for q in all_points])
            ys = np.array([q[1] for q in all_points])
            dist = np.hypot(xs, ys)
            ang = np.arctan2(ys, xs)
            sector = (dist >= self.zone_near_m) & (dist <= self.zone_far_m) & \
                     (np.abs(ang) <= self.zone_half_angle_rad)
            sector_count = int(np.sum(sector))

        if sector_count >= self.zone_point_threshold:
            self.dense_count += 1
            self.sparse_count = 0
        else:
            self.sparse_count += 1
            self.dense_count = 0

        if not self.zone_active and self.dense_count >= self.zone_enter_frames:
            self.zone_active = True
            self.get_logger().info('CONE_ZONE entered (diagnostic only)')
        elif self.zone_active and self.sparse_count >= self.zone_exit_frames:
            self.zone_active = False
            self.get_logger().info('CONE_ZONE exited (diagnostic only)')

        self.zone_pub.publish(Bool(data=self.zone_active))

    # ==================== 메인 콜백 ====================
    def scan_callback(self, msg: LaserScan):
        self._scan_frame = msg.header.frame_id or self._scan_frame
        all_points = self._scan_to_points(msg)
        roi_points = self._apply_roi(all_points)
        cones = self._cluster_and_filter(roi_points)

        mode = self.get_parameter('pairing_mode').value
        if mode == 'nearest_pair':
            path, left, right, source = self._find_pair_target(cones)
        else:
            path, left, right, source = self._plan_path(cones)

        target = None
        eff_ld = 0.0
        clamped = False
        if path:
            target, eff_ld, clamped = geo.target_at_lookahead(
                path, self.lookahead_dist, self.axle_offset)

        if target is not None:
            tx, ty = target
            # 목표점 급변 방지: 약한 스무딩 + 프레임당 최대 이동량 clamp.
            #
            # ★ 순서가 바뀌었다 (2026-08-22). 예전에는 clamp 를 **먼저** 걸고
            #   EMA 를 나중에 걸었는데, 그러면 실제 상한이 표기값의 (1-a)배가
            #   된다:
            #       ty        <= smoothed + step
            #       smoothed'  = a*smoothed + (1-a)*ty  <= smoothed + (1-a)*step
            #   a=0.5 에서 max_target_step_m=0.12 는 실제로 0.06 m/frame 이었다.
            #   10Hz 기준 목표 y 가 초당 0.6m 밖에 못 따라간다. 급커브(R=0.6)에
            #   들어가면 목표 y 가 Ld^2/2R = 0.60m 튀는데 따라잡는 데 10프레임
            #   (1.0s) — 그 사이 차는 0.48m 를 직진한다.
            #   **급커브에서 못 꺾고 안쪽 콘을 치던 원인이 이것이다.**
            #   EMA 를 먼저 걸고 그 결과를 clamp 하면 파라미터가 표기 그대로
            #   프레임당 상한이 된다.
            if self.smoothed_y is None:
                self.smoothed_y = ty
            else:
                a = self.target_smoothing_alpha
                sm = a * self.smoothed_y + (1.0 - a) * ty
                delta = sm - self.smoothed_y
                if abs(delta) > self.max_target_step_m:
                    sm = self.smoothed_y + math.copysign(self.max_target_step_m, delta)
                self.smoothed_y = sm
            ty = self.smoothed_y
            target = (tx, ty)

            angle = geo.steer_pure_pursuit(
                target, self.axle_offset, self.wheelbase, self.steer_gain,
                self.lookahead_min, self.angle_limit)
            speed = self.base_speed
            self._prev_target = target
            self._prev_target_age = 0
        else:
            angle = self._gap_follow_angle(msg)
            speed = self.base_speed * self.lost_speed_ratio
            source = 'gap_fallback'
            self.smoothed_y = None
            # 직전 목표점은 **바로 버리지 않고 나이만 먹인다** (위 주석 참고).
            self._prev_target_age += 1
            if self._prev_target_age > self.prev_target_max_age:
                self._prev_target = None

        if self.invert_steer:
            angle = -angle
        angle = float(np.clip(angle, -self.angle_limit, self.angle_limit))

        # 조향각이 클수록 감속. angle_limit 에서 min_speed_ratio 까지 선형.
        steer_ratio = abs(angle) / self.angle_limit if self.angle_limit > 0 else 0.0
        speed = speed * max(self.min_speed_ratio, 1.0 - steer_ratio)

        # ★ 절대 하한 (2026-08-22) — **"멈춘 뒤 다시 안 나가던" 원인.**
        #
        #   비율 감속만 두면 명령이 센서리스 BLDC 의 데드밴드 아래로 내려간다.
        #   이 차량 상수(speed_weight 0.08 x speed_to_erpm_gain 4614)로
        #   speed 4.06 = 1500 ERPM 이 그 경계인데:
        #
        #       조향  0도 -> 6.00 (2215 ERPM)  돈다
        #       조향 10도 -> 4.00 (1476 ERPM)  ← 여기서부터 안 돈다
        #       조향 20도 -> 2.40 ( 886 ERPM)  안 돈다
        #       FTG 폴백이면 x0.7 이라 더 낮다 (조향 5도에 이미 1292)
        #
        #   라바콘 복도에서 조향 10도는 예사다. 그래서 구간 대부분을
        #   데드밴드 안에서 명령하고 있었다. 게다가 **한 번 서면 스캔이
        #   안 바뀌므로 조향도 그대로 -> 속도도 그대로**라 영영 못 나간다.
        #   소프트웨어 래치가 아니라 물리적 래치였다.
        #
        #   driver_node 의 speed.min(7.0)은 **차선 주행 경로에만** 걸린다
        #   (longitudinal.py). 콘 구간 명령은 SpeedLimiter 를 그냥 통과하고,
        #   그 kick 도 min(target, kick) 이라 목표가 낮으면 못 끌어올린다.
        #   그래서 하한은 여기, 명령을 만드는 자리에 있어야 한다.
        #
        #   ⚠️ 이 하한이 걸리면 min_speed_ratio 는 사실상 죽는다
        #      (base 6.0 / min_speed 5.0 이면 비율은 1.00~0.833 구간만 산다).
        #      데드밴드 아래로는 "천천히"가 존재하지 않으므로 이게 맞다 —
        #      2.4 를 명령하는 것은 느리게 가는 게 아니라 서는 것이다.
        #   ⚠️ 5.0 = 1846 ERPM. 조향 부하까지 걸린 상태에서 이것으로도
        #      끊기면 7.0(2584 ERPM, 2026-08-19 차선 곡선에서 검증된 값)까지
        #      올려야 하고, 그러면 base_speed 도 그 이상이어야 의미가 있다.
        if speed > 0.0:
            speed = max(speed, self.min_speed)

        self.drive_pub.publish(Float32MultiArray(data=[angle, float(speed)]))

        if clamped:
            # 중심선이 차에서 멀리서 시작해 요청 lookahead 를 못 맞췄다.
            # 코너 컷이 eff_ld^2/(8R) 로 커지므로 그냥 넘기면 안 된다.
            self.get_logger().warn(
                'lookahead clamp: %.2f -> %.2f m (콘이 멀다/적다)'
                % (self.lookahead_dist, eff_ld),
                throttle_duration_sec=2.0)

        self._update_zone(all_points)
        self._publish_cone_markers(left, right)
        self._publish_debug(roi_points, left, right, path, target, angle,
                            source, eff_ld)

    # ==================== RViz 콘 마커 ====================
    def _publish_cone_markers(self, left, right):
        """콘 하나 = 큰 점 하나. 좌측 사슬 파랑 / 우측 사슬 빨강.

        원본 스캔점은 내지 않는다 — 라바콘 구간에서 알고 싶은 건
        "점이 어디 있나"가 아니라 **"어느 콘이 어느 벽으로 배정됐나"** 다.
        점이 깔려 있으면 그 배정이 안 보인다. (전체 스캔을 보고 싶으면
        viz_node 의 /viz/scan_cloud 를 켜면 된다 — 구간 밖에서는 계속 나온다.)

        SPHERE_LIST 를 쓰는 이유: 콘 개수만큼 마커를 만들면 개수가 줄었을 때
        옛 마커가 화면에 남는다(id 관리 필요). 리스트 하나면 점 목록만
        갈아끼우면 되고, 비면 DELETE 한 번으로 깨끗이 사라진다.

        프레임은 스캔과 같은 것을 쓴다 — 새 TF 를 요구하지 않는다.
        """
        arr = MarkerArray()
        size = self.cone_marker_size
        for mid, chain, ns, rgb in ((0, left, 'cones_left', (0.1, 0.4, 1.0)),
                                    (1, right, 'cones_right', (1.0, 0.1, 0.1))):
            m = Marker()
            m.header.frame_id = self._scan_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = mid
            m.type = Marker.SPHERE_LIST
            m.pose.orientation.w = 1.0
            if chain:
                m.action = Marker.ADD
                m.scale.x = m.scale.y = m.scale.z = size
                m.color.r, m.color.g, m.color.b = rgb
                m.color.a = 1.0
                for (x, y) in chain:
                    pt = Point()
                    pt.x, pt.y, pt.z = float(x), float(y), 0.0
                    m.points.append(pt)
            else:
                m.action = Marker.DELETE
            arr.markers.append(m)
        self.cone_marker_pub.publish(arr)

    # ==================== 디버그 시각화 ====================
    def _publish_debug(self, roi_points, left, right, path, target, angle,
                       source, eff_ld):
        size = 500
        # [2026-08-22] range_max 가 2.5m 로 늘면서 150 px/m 로는 ROI 원이
        # 화면 밖으로 나갔다. 100 px/m 면 좌우 ±2.5m 가 정확히 들어온다.
        scale = 100.0
        origin = (size // 2, size - 30)
        img = np.zeros((size, size, 3), dtype=np.uint8)

        for radius_m in (0.5, 1.0, 1.5, 2.0, 2.5):
            cv2.circle(img, origin, int(radius_m * scale), (40, 40, 40), 1)

        def to_px(x, y):
            return int(origin[0] - y * scale), int(origin[1] - x * scale)

        # ROI 경계 (직사각형 + 거리 상한)
        cv2.circle(img, origin, int(self.range_max * scale), (60, 60, 0), 1)

        # ROI 스캔점은 기본으로 **그리지 않는다** (debug_show_scan_points).
        # 점이 깔리면 콘이 어느 벽으로 배정됐는지가 안 보인다 — 이 화면의
        # 목적은 스캔이 아니라 그 배정이다.
        if self.debug_show_scan_points:
            for (x, y) in roi_points:
                px, py = to_px(x, y)
                if 0 <= px < size and 0 <= py < size:
                    img[py, px] = (90, 90, 90)

        def draw_chain(chain, color):
            # 콘 하나 = 큰 점 하나. RViz 마커(_publish_cone_markers)와 같은
            # 색 규약을 쓴다 — 두 화면을 번갈아 봐도 좌우가 안 헷갈리게.
            for k, (x, y) in enumerate(chain):
                cv2.circle(img, to_px(x, y), 11, color, -1)
                if k:
                    cv2.line(img, to_px(*chain[k - 1]), to_px(x, y), color, 2)

        draw_chain(left, (255, 0, 0))     # 파랑 = 좌측 사슬 (BGR)
        draw_chain(right, (0, 0, 255))    # 빨강 = 우측 사슬

        for k, (x, y) in enumerate(path):  # 초록 = 중심선
            cv2.circle(img, to_px(x, y), 3, (0, 200, 0), -1)
            if k:
                cv2.line(img, to_px(*path[k - 1]), to_px(x, y), (0, 200, 0), 1)

        cv2.circle(img, origin, 6, (0, 255, 0), -1)                    # 라이다
        cv2.circle(img, to_px(-self.axle_offset, 0.0), 5, (0, 140, 255), -1)  # 뒤축

        if target is not None:
            cv2.circle(img, to_px(*target), 8, (255, 0, 255), -1)      # 목표점
            cv2.line(img, to_px(-self.axle_offset, 0.0), to_px(*target),
                     (255, 0, 255), 1)

        cv2.putText(img, 'src=%s angle=%.1f ld=%.2f' % (source, angle, eff_ld),
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, 'cones L%d R%d  zone=%s' % (len(left), len(right),
                                                     self.zone_active),
                    (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = RubberconeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
