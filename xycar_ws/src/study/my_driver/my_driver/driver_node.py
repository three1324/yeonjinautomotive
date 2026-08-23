#!/usr/bin/env python3
"""주행 판단·제어 노드.

인지 결과를 받아 상태를 관리하고, 목표(횡위치·속도)를 정한 뒤, 실제 명령을 만든다.

구독:
    /lane      Float32MultiArray [offset_near, offset_far, valid, quality]
    /light     Int32             0=NONE 1=RED 2=YELLOW 3=GREEN 4=LEFT
    /objects   Float32MultiArray [cone_n, cone_near_y, car_present, car_cx, car_bottom_y,
                                  cone_max_h, car_h]
    /cone_cmd  Float32MultiArray [angle, speed]  라바콘 구간 전담 노드의 명령
    /cone_zone_active Bool                       그 노드의 구간 판정 (진단용, 제어 미사용)

  ※ /corridor, /obstacle 은 **구독하지 않는다** (2026-08-21 제거). 아래 참고.
발행:
    xycar_motor Float32MultiArray [angle, speed]
    debug_state String            JSON. my_debug/pipeline_view_node 시각화 전용.

제어 루프는 콜백이 아니라 **고정 주기 타이머**로 돈다. 센서마다 도착 주기가 달라서
콜백 구동으로 짜면 제어 주기가 들쭉날쭉해지고 미분항(K_damp)이 불안정해진다.

안전 설계:
    - 시작 시 /drive_enable 을 받기 전까지 speed 0 을 계속 발행한다 (f1tenth 방식)
    - 인지 데이터가 끊기면(stale) 정지한다
    - 차선을 못 보면 조향을 서서히 중립으로 되돌리며 감속, 오래가면 정지

────────────────────────────────────────────────────────────────────────
센서 역할 분리 (2026-08-19 실차 결정)

    카메라  차선 추종 · 신호등 · **방해차량 회피**
    라이다  **라바콘 구간에서만** — my_obstacle/rubbercone_node 가 전담

라이다는 트랙 밖 벽·기둥·관중과 실제 장애물을 구분하지 못한다. 상시
사용하면 곡선에서 벽이 섹터에 들어올 때마다 정지가 걸리고, 차선을 잠깐
놓칠 때마다 엉뚱한 것을 주행 기준으로 삼는다.

────────────────────────────────────────────────────────────────────────
라바콘 구간은 통째로 넘긴다 (mux)

`rubbercone_node` 가 조향·속도를 스스로 다 만든다(콘 클러스터링 -> 좌/우 사슬
-> 중심선 -> 뒤축 기준 Pure Pursuit, 실패 시 Follow-the-Gap).
원래는 팀원이 실차에서 성공시킨 구현을 그대로 가져온 것이었고, 2026-08-22 에
실측 치수(콘 간격 43.4cm / 복도 폭 80cm / 라이다-뒤축 41cm)를 넣어 기하를
개편했다. 옛 경로는 `pairing_mode: nearest_pair` 로 남아 있다.

    perception(YOLO cone_n) --> cone_zone.ConeZoneDetector --> mux 판단
    rubbercone_node         --/cone_cmd------------------------> 구간이면 그대로 통과

★ 구간 판정만은 **카메라가 한다** (2026-08-19 2차 실차).
  rubbercone_node 의 /cone_zone_active 는 전방 부채꼴 안 점 개수만 세고 콘
  모양을 보지 않아서, 콘이 없는 곳의 벽·기둥에도 참이 됐다. 그러면 제어권이
  통째로 라이다로 넘어가 차가 차선을 무시하고 달린다. 지금은 **콘 8개 이상 +
  그 콘이 충분히 가까울 때** 진입하고 **2개 이하**면 이탈한다.
  /cone_zone_active 는 진단 로그로만 남긴다.

우리가 중간에서 손대면 검증된 동작이 깨지므로 **조향은 건드리지 않는다.**
속도만 SpeedLimiter 를 통과시키는데, 알고리즘이 아니라 VESC 저전압 fault
방지(하드웨어 보호) 때문이다 — _drive_cone_zone() 주석 참고.

────────────────────────────────────────────────────────────────────────
라이다는 이 노드에 **한 경로로만** 들어온다 (2026-08-21 정리)

    콘 구간 + rubbercone 살아있음  ->  라이다 주행 (_drive_cone_zone)
    콘 구간 + rubbercone 죽음      ->  **정지**
    콘 구간 아님                    ->  차선 단독. 라이다 값을 읽지도 않는다

이전에는 두 개의 비상 경로가 더 있었다: /corridor(복도 추정)를 차선과
섞는 fusion, 그리고 /obstacle 의 front_dist 로 거는 전방 정지 상한.
둘 다 **정상 주행에서는 도달조차 못 하는 코드**였다 — 위 첫 줄이 return
하기 때문이다. 오직 "콘 구간인데 rubbercone 이 죽은" 경우에만 실행됐다.

그런데 그 경로가 안전하지도 않았다. /corridor 는 합성 데이터로만 검증됐고
px_per_meter(300.0)는 실측 전 값이라, 라이다가 죽었을 때 **검증 안 된
추정치로 콘 사이를 계속 달리는** 구조였다. 콘 구간에서 멈추는 벌점보다
콘을 치거나 코스를 이탈하는 쪽이 나쁘다고 판단해 통째로 들어냈다.

obstacle_node 자체를 지웠다 (2026-08-21). 시각화만을 위해 라이다 복도 추정을
상시 돌릴 이유가 없다. 라이다를 쓰는 노드는 rubbercone_node 하나뿐이다.
"""

import json
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Int32, String

from my_driver.control import SpeedLimiter, SteeringController
from my_driver.cone_zone import ConeZoneDetector
from my_driver.fsm import LIGHT_LEFT, DriveFSM, ShortcutPhase, State
from my_driver.lateral import LateralPlanner, OvertakeBehavior
from my_driver.longitudinal import LongitudinalPlanner

LIGHT_NAME = {0: "NONE", 1: "RED", 2: "YELLOW", 3: "GREEN", 4: "LEFT"}


@dataclass
class LaneRef:
    """차선 기준 횡방향 목표. 로그·시각화가 쓰는 형식이기도 하다.

    예전에는 여기에 복도(라이다) 융합 결과가 들어와서 corridor_weight/source
    필드가 있었다. 융합을 없앤 뒤로는 출처가 항상 차선이라 그 필드도 뺐다
    (모듈 docstring '라이다는 한 경로로만' 참고).
    """

    offset_near: float = 0.0
    offset_far: float = 0.0
    valid: bool = False
    quality: float = 0.0


@dataclass
class Obs:
    """최신 관측 묶음."""

    # 차선 (카메라)
    offset_near: float = 0.0
    offset_far: float = 0.0
    lane_valid: bool = False
    quality: float = 0.0
    # 좌회전(지름길) 전용 — 가장 왼쪽 노란픽셀 밴드만으로 만든 추정.
    # SHORTCUT 상태에서만 쓴다. 평소 주행은 위의 offset_* 를 그대로 쓴다.
    offset_near_left: float = 0.0
    offset_far_left: float = 0.0
    lane_valid_left: bool = False
    # 학습된 트랙 반폭(px). 회피 목표를 "트랙 반쪽의 중앙"으로 잡는 데 쓴다.
    # 0.0 이면 인지가 아직 좌우 흰선을 동시에 본 적이 없다는 뜻 -> 고정값 폴백.
    half_near: float = 0.0
    half_far: float = 0.0

    light: int = 0

    cone_n: int = 0
    cone_near_y: float = 0.0
    car_present: bool = False
    car_cx: float = 0.0
    car_bottom_y: float = 0.0   # 진단용. 회피 트리거에는 안 쓴다
    car_h: float = 0.0          # bbox 높이(px) — 회피 트리거의 거리 판단

    cone_max_h: float = 0.0   # 가장 큰 콘 bbox 높이(px). 구간 진입 거리 판단용

    # 라이다 관측(cor_*, front_dist, left_free, right_free)은 여기 없다.
    # 이 노드는 라바콘 구간에서 rubbercone_node 의 완성된 명령(/cone_cmd)만
    # 받아 통과시키고, 그 밖에서는 라이다를 아예 보지 않는다 (2026-08-21).


class DriverNode(Node):

    def __init__(self):
        super().__init__("driver_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("control_hz", 30.0),
                # 젯슨 실기기 카메라는 640x480 (2026-08-16 확인).
                # 튜닝에 쓴 테스트 영상은 632였으나 실차 기준으로 맞춘다.
                ("image_width", 640),
                ("require_enable", True),
                # 라이다 마스터 스위치. false 면 rubbercone_node 명령을 무시하고
                # 카메라 단독으로 달린다. **매 tick 읽으므로 재시작 없이 바뀐다:**
                #     ros2 param set /driver_node use_lidar false
                # 실차에서 "이상한 게 라이다 때문인가"를 그 자리에서 가르는 용도.
                ("use_lidar", True),
                ("stale_timeout_sec", 0.5),
                ("lane_lost_stop_sec", 2.0),
                ("log_period_sec", 1.0),
                # FSM
                ("fsm.start_confirm_frames", 5),
                # 좌회전(지름길) 진입 — LEFT 확정 즉시 SHORTCUT 으로 들어가
                # SHIFT(왼쪽으로 붙기) -> FOLLOW(좌측밴드 노란선 추종) 로
                # shortcut_total_sec 동안 간다. 고정 조향은 쓰지 않는다.
                #
                # ★ [2026-08-23] 기본값을 False -> **True** 로 바꿨다.
                #   좌회전 구현이 이것 하나뿐이라(하드코딩 TimedLeftDrive 는
                #   삭제됨) yaml 이 안 실렸을 때 좌회전이 통째로 죽는 것보다
                #   켜져 있는 편이 낫다.
                ("fsm.enable_shortcut", True),
                # 출발 신호. **둘 다 켠다** — 초록불이든 좌회전 화살표든
                # 확정되면 출발한다. 하나만 켜면 그 신호를 놓쳤을 때 차가
                # 영영 출발하지 않는다.
                ("fsm.start_on_green", True),
                ("fsm.start_on_left", True),
                ("fsm.shortcut_total_sec", 15.0),
                ("fsm.shortcut_turn_sec", 1.5),
                ("fsm.shortcut_confirm_frames", 5),
                ("fsm.shortcut_lost_frames", 3),
                ("fsm.shortcut_arm_timeout_sec", 5.0),
                # 꺾는 동안 낼 **원시** 조향/속도. 인지를 안 보고 이 값만 낸다.
                # 부호: **양수가 좌회전**이다 (steer.invert=true + 8/21 실측).
                ("shortcut.shift_px", 90.0),
                ("shortcut.turn_angle", 35.0),
                ("shortcut.turn_speed", 10.0),
                # 좌회전을 끝낸 뒤 이만큼은 회피를 하지 않는다. 분기 직후는
                # 화면이 평소와 달라 YOLO 가 차량을 오검출하기 쉬운데, 그때
                # 회피가 걸리면 갓 들어온 길에서 옆으로 벌린 채 달린다.
                ("shortcut.overtake_block_sec", 20.0),
                ("fsm.auto_start", False),
                # 라바콘 구간 — **판정은 카메라(YOLO 콘 개수), 주행은 rubbercone_node**.
                # 판정을 라이다에 맡겼더니 콘이 없는 곳(벽·기둥)에서도 구간으로
                # 잡혀 제어권이 넘어갔다 (2026-08-19 2차 실차). cone_zone.py 참고.
                ("cone_zone.enter_n", 9),      # 콘 이 개수 이상 보이면 진입 후보
                ("cone_zone.enter_min_size_px", 0.0),
                # ↑ 그 콘이 **충분히 가까워야** 진입한다 (가장 큰 콘 bbox 높이).
                #   개수만 보면 직선 끝에서 콘 무리가 보이자마자 전환된다.
                ("cone_zone.exit_n", 3),       # 이 개수 이하로 떨어지면 이탈 후보
                ("cone_zone.exit_hold_sec", 1.5),  # 이탈 후보가 이만큼 지속돼야 실제 이탈
                # rubbercone_node 의 명령이 얼마나 오래되면 "죽었다"고 볼지의 기준.
                # 그 노드는 /scan 주기(약 10Hz)로 발행하므로 0.5s 면 5프레임 여유.
                ("cone_cmd_timeout_sec", 0.5),
                # 횡방향 (회피는 카메라 전용 — 라이다 파라미터 없음)
                ("lateral.enable_overtake", True),
                ("lateral.trigger_height_px", 55.0),
                ("lateral.shift_left_px", 90.0),
                ("lateral.shift_right_px", 90.0),
                ("lateral.lost_hold_sec", 2.0),
                ("lateral.cooldown_sec", 1.0),
                # 종방향
                ("speed.base", 12.0),
                ("speed.min", 4.0),
                ("speed.limit", 27.0),
                ("speed.accel_per_sec", 20.0),
                ("speed.decel_per_sec", 60.0),
                ("speed.curve_px_lo", 30.0),
                ("speed.curve_px_hi", 150.0),
                ("speed.curve_factor_min", 0.45),
                ("speed.quality_lo", 0.4),
                ("speed.quality_factor_min", 0.5),
                ("speed.cone_n_lo", 2.0),
                ("speed.cone_n_hi", 8.0),
                ("speed.cone_factor_min", 0.6),
                ("speed.overtake_factor", 0.7),
                ("speed.kick", 7.0),
                # 조향
                ("steer.k_lat", 0.10),
                ("steer.k_curve", 0.06),
                ("steer.k_damp", 0.004),
                ("steer.angle_limit", 50.0),
                ("steer.rate_limit_per_sec", 180.0),
                ("steer.lpf_alpha", 0.45),
                ("steer.speed_gain_ref", 0.0),
                ("steer.invert", False),
            ],
        )

        g = self.get_parameter
        self.image_width = g("image_width").value
        self.stale_timeout = g("stale_timeout_sec").value
        self.lane_lost_stop = g("lane_lost_stop_sec").value
        self.require_enable = g("require_enable").value

        self.fsm = DriveFSM(
            start_confirm_frames=g("fsm.start_confirm_frames").value,
            enable_shortcut=g("fsm.enable_shortcut").value,
            start_on_green=g("fsm.start_on_green").value,
            start_on_left=g("fsm.start_on_left").value,
            shortcut_total_sec=g("fsm.shortcut_total_sec").value,
            shortcut_turn_sec=g("fsm.shortcut_turn_sec").value,
            shortcut_confirm_frames=g("fsm.shortcut_confirm_frames").value,
            shortcut_lost_frames=g("fsm.shortcut_lost_frames").value,
            shortcut_arm_timeout_sec=g("fsm.shortcut_arm_timeout_sec").value,
            auto_start=g("fsm.auto_start").value,
        )
        self.cone_zone = ConeZoneDetector(
            enter_n=g("cone_zone.enter_n").value,
            enter_min_size_px=g("cone_zone.enter_min_size_px").value,
            exit_n=g("cone_zone.exit_n").value,
            exit_hold_sec=g("cone_zone.exit_hold_sec").value,
        )
        self.cone_cmd_timeout = g("cone_cmd_timeout_sec").value
        self.lateral = LateralPlanner(
            OvertakeBehavior(
                trigger_height_px=g("lateral.trigger_height_px").value,
                shift_left_px=g("lateral.shift_left_px").value,
                shift_right_px=g("lateral.shift_right_px").value,
                lost_hold_sec=g("lateral.lost_hold_sec").value,
                cooldown_sec=g("lateral.cooldown_sec").value,
            ),
            enable_overtake=g("lateral.enable_overtake").value,
        )
        self.shortcut_shift_px = g("shortcut.shift_px").value
        self.shortcut_turn_angle = g("shortcut.turn_angle").value
        self.shortcut_turn_speed = g("shortcut.turn_speed").value
        self.shortcut_overtake_block = g("shortcut.overtake_block_sec").value
        # 남은 회피 차단 시간(초). 좌회전이 끝나는 순간 채워지고 매 tick 준다.
        self._overtake_block_left = 0.0
        self._prev_fsm_state = self.fsm.state
        self.longitudinal = LongitudinalPlanner(
            base_speed=g("speed.base").value,
            min_speed=g("speed.min").value,
            curve_px_lo=g("speed.curve_px_lo").value,
            curve_px_hi=g("speed.curve_px_hi").value,
            curve_factor_min=g("speed.curve_factor_min").value,
            quality_lo=g("speed.quality_lo").value,
            quality_factor_min=g("speed.quality_factor_min").value,
            cone_n_lo=g("speed.cone_n_lo").value,
            cone_n_hi=g("speed.cone_n_hi").value,
            cone_factor_min=g("speed.cone_factor_min").value,
            overtake_factor=g("speed.overtake_factor").value,
        )
        self.steering = SteeringController(
            k_lat=g("steer.k_lat").value,
            k_curve=g("steer.k_curve").value,
            k_damp=g("steer.k_damp").value,
            angle_limit=g("steer.angle_limit").value,
            rate_limit_per_sec=g("steer.rate_limit_per_sec").value,
            lpf_alpha=g("steer.lpf_alpha").value,
            speed_gain_ref=g("steer.speed_gain_ref").value,
            invert=g("steer.invert").value,
        )
        self.speed_limiter = SpeedLimiter(
            accel_per_sec=g("speed.accel_per_sec").value,
            decel_per_sec=g("speed.decel_per_sec").value,
            speed_limit=g("speed.limit").value,
            kick=g("speed.kick").value,
        )

        self.obs = Obs()
        # 마지막 횡방향 기준. 로그·시각화가 읽는다.
        # (융합을 걷어낸 뒤로는 항상 차선이 출처라 LaneRef 하나면 된다.)
        self._ref = LaneRef()
        # _pub_debug 가 on_tick 의 조기 return 경로(disabled / stale /
        # 신호대기)에서도 불리므로 반드시 먼저 초기화해야 한다.
        self._use_left_band = False
        self._enabled = not self.require_enable
        self._last_lane_time = None
        self._lane_lost_since = None
        self._target_offset = 0.0
        # 라바콘 노드 mux 상태
        self._cone_cmd = (0.0, 0.0)      # (angle, speed)
        self._last_cone_cmd_time = None
        # 라이다 쪽 구간 판정. **제어에는 쓰지 않는다** — 진단 비교용으로만
        # 받아서 디버그에 싣는다 (카메라 판정과 어긋나는 빈도를 보려는 것).
        self._cone_zone_from_node = False
        self._last_source = "lane"        # 전환 로그를 한 번만 찍기 위한 기억
        self._last_tick = self.get_clock().now()
        self._last_log = self._last_tick

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_motor = self.create_publisher(Float32MultiArray, "xycar_motor", qos)
        # my_debug/pipeline_view_node 가 이걸 받아 시각화 패널 하단에 텍스트로 합성한다.
        # 문자열 JSON 하나로 끝내는 이유: 이 디버그 채널만을 위해 커스텀 .msg 를
        # 새로 만들 정도의 가치가 없다 (디버그 전용, 프로토콜에 안 얹힘).
        self.pub_debug = self.create_publisher(String, "debug_state", 10)
        self.create_subscription(Float32MultiArray, "lane", self.on_lane, qos)
        self.create_subscription(Int32, "light", self.on_light, qos)
        self.create_subscription(Float32MultiArray, "objects", self.on_objects, qos)
        self.create_subscription(Bool, "drive_enable", self.on_enable, qos)
        # 라바콘 구간 전담 노드(my_obstacle/rubbercone_node)의 출력.
        # 그 노드가 스스로 구간을 판정하고 조향/속도까지 다 만든다. 우리는
        # 구간일 때 그 명령을 **그대로 통과**시키기만 한다 (§ 센서 역할 분리).
        self.create_subscription(Float32MultiArray, "cone_cmd", self.on_cone_cmd, qos)
        self.create_subscription(Bool, "cone_zone_active", self.on_cone_zone, qos)

        hz = g("control_hz").value
        self.create_timer(1.0 / hz, self.on_tick)
        self._log_period = g("log_period_sec").value

        if self.require_enable:
            self.get_logger().warn(
                "주행 대기중. 시작하려면: "
                "ros2 topic pub --once /drive_enable std_msgs/msg/Bool '{data: true}'"
            )
        self.get_logger().info(f"driver_node 시작 ({hz:.0f}Hz)")

    # ---------- 구독 콜백: 값 저장만 ----------

    def on_lane(self, msg):
        if len(msg.data) < 4:
            return
        self.obs.offset_near = msg.data[0]
        self.obs.offset_far = msg.data[1]
        self.obs.lane_valid = msg.data[2] > 0.5
        self.obs.quality = msg.data[3]
        # 반폭은 뒤에 붙은 확장 필드다. 길이로 판정해 옛 perception_node 와도
        # 섞어 쓸 수 있게 한다 (없으면 0.0 -> 고정값 폴백 경로로 간다).
        if len(msg.data) >= 6:
            self.obs.half_near = msg.data[4]
            self.obs.half_far = msg.data[5]
        # 좌회전 전용 좌측밴드 추정. 없으면 lane_valid_left=False 라 SHORTCUT 이
        # 평소 차선 추정으로 폴백한다 (옛 perception_node 와 섞였을 때).
        if len(msg.data) >= 9:
            self.obs.offset_near_left = msg.data[6]
            self.obs.offset_far_left = msg.data[7]
            self.obs.lane_valid_left = msg.data[8] > 0.5
        self._last_lane_time = self.get_clock().now()


    def on_light(self, msg):
        self.obs.light = int(msg.data)

    def on_objects(self, msg):
        if len(msg.data) < 5:
            return
        self.obs.cone_n = int(msg.data[0])
        self.obs.cone_near_y = msg.data[1]
        self.obs.car_present = msg.data[2] > 0.5
        self.obs.car_cx = msg.data[3]
        self.obs.car_bottom_y = msg.data[4]
        # 콘 크기는 뒤에 붙은 확장 필드. 없으면 0.0 이라 크기 조건이 통과되지
        # 않는다 — 옛 perception_node 와 섞어 쓰면 구간에 아예 못 들어간다.
        if len(msg.data) >= 6:
            self.obs.cone_max_h = msg.data[5]
        # 차량 bbox 높이도 뒤에 붙은 확장 필드다. 없으면 0.0 이라 회피가
        # **아예 안 걸린다** — 옛 perception_node 와 섞어 쓰면 그렇게 된다.
        if len(msg.data) >= 7:
            self.obs.car_h = msg.data[6]


    def on_cone_cmd(self, msg):
        if len(msg.data) < 2:
            return
        self._cone_cmd = (float(msg.data[0]), float(msg.data[1]))
        self._last_cone_cmd_time = self.get_clock().now()

    def on_cone_zone(self, msg):
        self._cone_zone_from_node = bool(msg.data)

    def on_enable(self, msg):
        self._enabled = bool(msg.data)
        self.get_logger().warn(f"drive_enable = {self._enabled}")
        if not self._enabled:
            self._halt()

    # ---------- 제어 루프 ----------

    def _publish(self, angle, speed):
        self.pub_motor.publish(Float32MultiArray(data=[float(angle), float(speed)]))

    def _halt(self):
        self.steering.reset()
        self.speed_limiter.stop_now()
        self._publish(0.0, 0.0)

    def on_tick(self):
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds / 1e9
        self._last_tick = now
        if dt <= 0.0:
            return

        # 라바콘 구간 판정은 **주행 여부와 무관하게 항상 돌린다.**
        # 아래의 조기 return 들(disabled / stale / 신호 대기) 뒤에 두면,
        # /drive_enable 을 켜기 전에는 판정이 아예 안 돌아 벤치·영상 테스트에서
        # cone_zone 이 영영 false 로 보인다. 실제로 그렇게 만들어 놓고 한참
        # 헤맸다(2026-08-19). 판정은 순수한 관측이라 여기서 돌아도 안전하다 —
        # 차를 움직이는 것은 아래 _drive_cone_zone() 뿐이다.
        in_cone_zone = self.cone_zone.update(
            dt, self.obs.cone_n, self.obs.cone_max_h)

        # 회피 차단 시간도 여기서 준다. 아래 조기 return(정지·대기) 뒤에 두면
        # 서 있는 동안 시간이 안 흘러, 신호 대기 한 번에 차단이 무한정
        # 늘어난다. 벽시계 20초가 맞다.
        if self._overtake_block_left > 0.0:
            self._overtake_block_left = max(0.0, self._overtake_block_left - dt)

        if not self._enabled:
            self._publish(0.0, 0.0)
            self._pub_debug(self.fsm.state.value, 0.0, 0.0, "disabled")
            return

        # 인지가 끊겼으면 무조건 정지. 오래된 값으로 달리는 게 가장 위험하다.
        if self._last_lane_time is None:
            self._publish(0.0, 0.0)
            self._pub_debug(self.fsm.state.value, 0.0, 0.0, "no_lane_yet")
            return
        age = (now - self._last_lane_time).nanoseconds / 1e9
        if age > self.stale_timeout:
            self.get_logger().warn(f"인지 데이터 끊김 ({age:.2f}s) — 정지", throttle_duration_sec=1.0)
            self._halt()
            self._pub_debug(self.fsm.state.value, 0.0, 0.0, f"stale({age:.2f}s)")
            return

        # dt 를 넘겨야 SHORTCUT 위상 시간(SHIFT/FOLLOW)이 흘러간다.
        state = self.fsm.update(self.obs.light, self.obs.lane_valid, dt)

        # 좌회전을 막 끝냈다 -> 회피 차단 시작.
        if self._prev_fsm_state is State.SHORTCUT and state is not State.SHORTCUT:
            self._overtake_block_left = self.shortcut_overtake_block
            self.get_logger().info(
                f"좌회전 완료 — 회피 {self.shortcut_overtake_block:.0f}초 차단")
        self._prev_fsm_state = state

        if not self.fsm.should_drive:
            self._publish(0.0, 0.0)
            self._log(state, 0.0, 0.0, "wait")
            self._pub_debug(state.value, 0.0, 0.0, "wait")
            return

        # ── 구간 판정은 카메라, 주행은 rubbercone_node (2026-08-19 2차 실차) ──
        # "지금이 라바콘 구간인가"는 카메라가 정한다 — 콘 8개 이상이면서 가장 큰
        # 콘이 충분히 가까우면(bbox 높이) 진입, 2개 이하면 이탈.
        # 크기 조건이 없으면 직선 끝에서 콘 무리가 보이자마자 전환된다.
        # 라이다 점밀도 판정은 콘이 없는 곳에서도 참이 돼서
        # 제어권이 넘어갔다 — cone_zone.py 의 "왜 되살렸나" 참고.
        # 구간 안에서의 조향·속도는 여전히 rubbercone_node 가 만든다. 그 출력은
        # **그대로** 통과시킨다 — 중간에서 손대면 검증된 동작이 깨진다.
        # (in_cone_zone 은 on_tick 맨 앞에서 이미 갱신했다 — 주행 여부와
        #  무관하게 판정이 돌아야 하기 때문. 그 주석 참고.)
        cone_cmd_fresh = (
            self._last_cone_cmd_time is not None
            and (now - self._last_cone_cmd_time).nanoseconds / 1e9 <= self.cone_cmd_timeout
        )

        # ── 라이다 마스터 스위치 ──
        # 매 tick 읽는다. 재시작 없이 끄고 켤 수 있어야 실차에서 "지금 이상한
        # 게 라이다 때문인가"를 그 자리에서 가를 수 있다:
        #     ros2 param set /driver_node use_lidar false
        # 구간 판정(cone_zone)은 계속 돌려둔다 — 로그·시각화에 근거가 남아야
        # 나중에 "그때 콘 구간이긴 했나"를 확인할 수 있다. 제어에 쓰는 값만 끊는다.
        # ⚠️ false 면 라바콘 구간을 라이다 없이 지나가게 된다. 콘 벽이 차선보다
        #    안쪽이라 차선만으로는 콘을 칠 수 있다 — 진단용이지 주행 모드가 아니다.
        use_lidar = self.get_parameter("use_lidar").value

        if use_lidar and in_cone_zone and cone_cmd_fresh:
            self._drive_cone_zone(dt, state)
            return

        if in_cone_zone and not use_lidar:
            # 라이다를 **의도적으로** 껐다. 이때는 정지시키지 않는다 —
            # 아래 차선 주행으로 그대로 흘려보낸다. 끈 사람이 그걸 원한 것이다.
            self.get_logger().warn(
                "CONE_ZONE 이지만 use_lidar=false — 차선만으로 통과 시도 "
                "(콘 접촉 위험)", throttle_duration_sec=2.0)

        elif in_cone_zone:
            # 콘 구간인데 명령이 안 온다 = rubbercone_node 가 죽었거나 라이다가 끊겼다.
            # **정지한다.** 차선으로 되돌아가면 콘을 친다 — 우측 콘 벽이 흰 실선보다
            # 안쪽이라 차선 중심을 따라가는 것 자체가 콘으로 들어가는 길이다.
            # 콘 사이에서 멈추는 벌점이, 콘을 치거나 코스를 이탈하는 것보다 낫다.
            self.get_logger().error(
                "CONE_ZONE 인데 /cone_cmd 가 끊겼다 — 정지 "
                "(라이다/rubbercone_node 점검)", throttle_duration_sec=1.0)
            self._halt()
            self._log(state, 0.0, 0.0, "cone_cmd lost")
            self._pub_debug(state.value, 0.0, 0.0, "cone_cmd lost")
            return

        # ── 좌회전 강제 회전 (TURN_IN) ──
        # 신호등을 지난 뒤 shortcut_turn_sec 동안, **좌측밴드가 탐지되면**
        # 고정 조향으로 밀어넣는다. 분기 초입은 곡선이 급해서 인지 추종만
        # 으로는 못 꺾고 지나칠 수 있기 때문이다.
        #
        # ★ 좌측밴드가 안 잡히면 **걸지 않는다.** 근거 없이 꺾으면 분기가
        #   없는 곳에서 LEFT 오검출 한 번에 트랙을 벗어난다. 그때는 아래
        #   평소 차선 주행으로 그냥 흘려보낸다.
        if (self.fsm.shortcut_phase is ShortcutPhase.TURN_IN
                and self.obs.lane_valid_left):
            self._drive_shortcut_turn(dt, state)
            return

        if self._last_source != "lane":
            self.get_logger().info(
                f"CONE_ZONE 이탈 — 차선 주행으로 복귀 ({self.cone_zone.last_reason})")
            self._last_source = "lane"

        # 여기 도달했다 = 콘 구간이 아니거나, 콘 구간이지만 use_lidar 를 껐다.
        # 어느 쪽이든 횡방향 기준은 **차선 단독**이다.
        # 라이다(복도)를 섞던 fusion 은 제거했다 — 모듈 docstring 참고.
        # ── 좌회전(지름길)이면 **좌측밴드 노란선**을 기준으로 쓴다 ──
        # 분기점에서는 dashed 마스크에 직진 가지와 좌회전 가지가 같이 들어와,
        # 전부 평균낸 offset 은 그 사이를 겨냥해 어느 쪽도 못 탄다.
        # 좌측밴드는 행별로 가장 왼쪽 픽셀에서 lane.left_band_px 안쪽만 남긴
        # 추정이라 좌회전 가지가 선택되고, 좌회전을 마친 뒤에는 합류 차선을
        # 그대로 따라간다. (인지가 계산해 /lane 에 실어 보낸 값이다.)
        #
        # 좌측밴드가 무효면(노란선을 아예 못 봄, 또는 옛 perception_node)
        # 평소 추정으로 폴백한다 — 갈 곳이 없는 것보다 낫다.
        # 좌측밴드는 TURN_IN/FOLLOW 에서만 쓴다.
        # SHIFT (신호등을 지나치기 전, 안쪽으로 붙는 중) 는 평소 추정
        # 그대로다 — 아직 분기점이 아니라 노란선이 하나뿐이고, 좌측밴드를
        # 쓰면 이득 없이 왼쪽으로 편향만 생긴다.
        use_left_band = (
            state is State.SHORTCUT
            and self.fsm.shortcut_phase is not ShortcutPhase.SHIFT
            and self.obs.lane_valid_left)
        self._use_left_band = use_left_band
        if use_left_band:
            ref = LaneRef(self.obs.offset_near_left, self.obs.offset_far_left,
                          True, self.obs.quality)
        else:
            ref = LaneRef(self.obs.offset_near, self.obs.offset_far,
                          self.obs.lane_valid, self.obs.quality)
        self._ref = ref

        # 차선을 오래 못 보면 정지.
        if ref.valid:
            self._lane_lost_since = None
        elif self._lane_lost_since is None:
            self._lane_lost_since = now

        if self._lane_lost_since is not None:
            lost = (now - self._lane_lost_since).nanoseconds / 1e9
            if lost > self.lane_lost_stop:
                self.get_logger().warn(
                    f"횡방향 기준 소실 {lost:.1f}s — 정지", throttle_duration_sec=1.0)
                self._halt()
                self._log(state, 0.0, 0.0, "ref lost")
                self._pub_debug(state.value, 0.0, 0.0, "ref lost")
                return

        target_offset = self.lateral.update(
            dt, self.obs, self.image_width,
            allow_overtake=(self._overtake_block_left <= 0.0))

        # ── 좌회전 진입 직후: 1차선으로 붙는다 ──
        # SHIFT 위상 동안 목표를 왼쪽으로 shortcut_shift_px 만큼 옮긴다.
        # 회피 기동과 **같은 방식**이다(목표만 계단으로 옮기고, 실제 거동은
        # 조향 rate limit / LPF 가 만든다).
        #
        # 목적은 회피가 아니라 **시야**다. 트랙 중앙에 있으면 좌회전 차선의
        # 노란선이 화면 왼쪽 끝에 걸리거나 아예 안 잡힌다. 1차선으로 붙어야
        # 좌측밴드 추정이 안정적으로 잡힌다.
        #
        # 부호: target_offset 이 **양수면 차가 트랙 왼쪽**에 선다
        # (lateral.py _offset 주석의 부호 규약과 같다). 그래서 왼쪽으로
        # 옮기려면 **더한다**.
        if self.fsm.shortcut_phase is ShortcutPhase.SHIFT:
            target_offset += self.shortcut_shift_px

        self._target_offset = target_offset
        # lateral 을 먼저 돌려야 이번 tick 의 회피 상태가 정해진다.
        # 종방향은 그 결과(기동 중인지)를 받아 감속한다 — 순서가 중요하다.
        target_speed = self.longitudinal.update(
            ref.valid, ref.offset_near, ref.offset_far,
            ref.quality, self.obs.cone_n,
            overtake_active=self.lateral.overtake.active,
        )
        speed = self.speed_limiter.update(dt, target_speed)

        if ref.valid:
            angle = self.steering.update(
                dt, ref.offset_near, ref.offset_far, target_offset, speed
            )
        else:
            angle = self.steering.hold(dt)

        self._publish(angle, speed)
        self._log(state, angle, speed, self.longitudinal.last_reason)
        self._pub_debug(state.value, angle, speed, self.longitudinal.last_reason)

    def _drive_shortcut_turn(self, dt, state):
        """좌회전 강제 회전 — 고정 조향각 + 고정 속도. 인지는 안 본다.

        진입 조건(좌측밴드 탐지)은 호출부가 이미 확인했다. 여기서는 정해진
        각도만 낸다.

        왜 고정값인가: 분기 초입은 곡선이 급해서, 좌측밴드를 보고 있어도
        인지 추종만으로는 못 꺾고 지나칠 수 있다. 그 1.5초만 밀어넣고
        궤도에 올라선 뒤(FOLLOW)부터 다시 인지로 넘긴다.

        **속도도 같이 고정해야** 회전 반경이 재현된다 — 같은 조향각이라도
        속도가 다르면 반경이 달라진다. 다만 SpeedLimiter 는 통과시킨다.
        알고리즘이 아니라 하드웨어 보호다(급가속이 배터리 전압을 끌어내려
        VESC 가 UNDER_VOLTAGE 로 멈춘다).

        조향은 follow_external() 로 낸다 — 변화율 제한(rate_limit)만 걸린다.
        sync() 처럼 즉시 꽂으면 서보에 계단 입력이 그대로 간다. 180도/s 면
        35도까지 0.19초라 1.5초 창에서는 충분히 도달한다.
        """
        angle = self.steering.follow_external(dt, self.shortcut_turn_angle)
        speed = self.speed_limiter.update(dt, self.shortcut_turn_speed)

        if self._last_source != "shortcut_turn":
            self.get_logger().info(
                f"SHORTCUT/TURN_IN — 좌측밴드 탐지, 고정 조향 {angle:+.1f} 로 "
                f"강제 회전 ({self.fsm.shortcut_turn_sec:.1f}s)")
            self._last_source = "shortcut_turn"

        self._publish(angle, speed)
        self._log(state, angle, speed, "shortcut(turn_in)")
        self._pub_debug(state.value, angle, speed, "shortcut(turn_in)")

    def _drive_cone_zone(self, dt, state):
        """라바콘 구간 — rubbercone_node 명령을 그대로 통과시킨다.

        조향은 **손대지 않는다.** 그 노드가 이미 목표점 평활화(프레임당 12cm
        clamp + EMA)를 하고 있고, 거기에 우리 rate limit 을 또 씌우면 검증된
        응답이 느려진다.

        속도만 SpeedLimiter 를 통과시킨다 — 알고리즘이 아니라 **하드웨어 보호**
        때문이다. 가속을 제한하지 않으면 기동 전류가 배터리 전압을 끌어내려
        VESC 가 fault code 2(UNDER_VOLTAGE)로 멈춘다(2026-08-19 실측으로
        accel_per_sec 을 20->8 로 낮춘 근거와 같다). 이 노드는 자체 가속 제한이
        없으므로 여기서 씌운다.
        """
        angle, cone_speed = self._cone_cmd
        speed = self.speed_limiter.update(dt, cone_speed)

        # 조향기 내부 상태를 실제 출력에 맞춰둔다. 구간을 빠져나가 우리 제어로
        # 돌아가는 순간 steering 이 0 에서 시작하면 그만큼 각도가 튄다.
        self.steering.sync(angle)

        if self._last_source != "cone":
            self.get_logger().info(
                f"CONE_ZONE — rubbercone_node 로 전환 ({self.cone_zone.last_reason})")
            self._last_source = "cone"

        self._publish(angle, speed)
        self._log(state, angle, speed, "cone_zone(rubbercone)")
        self._pub_debug(state.value, angle, speed, "cone_zone(rubbercone)")

    def _pub_debug(self, state_name, angle, speed, reason):
        """my_debug/pipeline_view_node 용 실시간 상태 스냅샷. 매 tick 발행 —
        문자열 하나 만드는 비용이라 30Hz 로 던져도 무시할 만하다."""
        ref = self._ref
        ov = self.lateral.overtake.phase.value if self.lateral.overtake.active else "-"
        payload = {
            "state": state_name,
            "angle": round(float(angle), 1),
            "speed": round(float(speed), 1),
            "off_near": round(ref.offset_near, 1),
            "off_far": round(ref.offset_far, 1),
            "quality": round(ref.quality, 2),
            "valid": bool(ref.valid),
            "light": LIGHT_NAME.get(self.obs.light, "?"),
            "cone_n": self.obs.cone_n,
            # 구간 진입 트리거의 거리 조건. cone_n 은 찼는데 이게 작아서 안
            # 들어가는 상황을 화면에서 바로 구분하려면 같이 보여야 한다.
            "cone_h": round(self.obs.cone_max_h, 0),
            "zone_why": self.cone_zone.last_reason,
            # 좌회전(지름길) 위상과 그 위상의 남은 시간. 위상이 "-" 면 그
            # 구간이 아니다. SHIFT(왼쪽으로 붙는 중)인지 FOLLOW(좌측밴드
            # 추종)인지가 화면에서 바로 구분돼야 한다.
            "shortcut_phase": (self.fsm.shortcut_phase.value
                               if self.fsm.shortcut_phase else "-"),
            "shortcut_remain": round(self.fsm.shortcut_remain, 1),
            # 좌측밴드가 지금 쓰이고 있는가.
            #   lb_valid=false 면 SHORTCUT 이어도 평소 추정으로
            #   폴백한다 — 그러면 좌회전이 "1.5초 깔짝하고 말는"
            #   것처럼 보인다. 예: perception 을 안 받아
            #   /lane 이 6필드로 오면 항상 false 다.
            "lb_valid": bool(self.obs.lane_valid_left),
            "lb_used": bool(self._use_left_band),
            "lb_near": round(self.obs.offset_near_left, 1),
            # 라이다(rubbercone_node)가 몰고 있는 구간인지. 화면에서 바로
            # 구분돼야 "지금 누가 운전 중인가"를 오해하지 않는다.
            "cone_zone": bool(self.cone_zone.active),
            "use_lidar": bool(self.get_parameter("use_lidar").value),
            # 라이다 쪽 판정. 제어에는 안 쓰지만, 카메라 판정과 얼마나
            # 어긋나는지 화면에서 바로 보이게 같이 싣는다.
            "zone_lidar": bool(self._cone_zone_from_node),
            "overtake": ov,
            # 회피 기동의 근거를 그대로 노출한다. 화면만 보고 "왜 저쪽으로
            # 피했는가 / 왜 안 피하는가"를 알 수 있어야 튜닝이 가능하다.
            "ot_dir": self.lateral.overtake.direction,
            # 회피의 **원인**: 방해차량이 어느 쪽에 있다고 봤는가.
            # ot_dir(피하는 쪽)과 부호가 반대라 둘을 같이 봐야 판단이 옳았는지
            # 화면에서 가를 수 있다 (8/21 부호 버그가 바로 이 지점이었다).
            "ot_car_side": self.lateral.overtake.car_side,
            "ot_car_cx": round(self.lateral.overtake.car_cx, 1),
            "ot_amount": round(self.lateral.overtake.amount, 1),
            "ot_reason": self.lateral.overtake.last_reason,
            # 지금 이 순간의 차량 관측(기동 여부와 무관). 기동이 안 걸릴 때
            # "차를 아예 못 본 건지, 봤는데 아직 먼 건지"를 가른다.
            "car_present": bool(self.obs.car_present),
            "car_cx": round(float(self.obs.car_cx), 1),
            "car_bottom_y": round(float(self.obs.car_bottom_y), 1),
            "car_h": round(float(self.obs.car_h), 1),
            # 좌회전 직후 회피 차단이 걸려 있는지. 0 이면 평소대로 회피한다.
            # 이게 안 보이면 "왜 안 피하지"를 화면에서 못 가른다.
            "ot_block": round(self._overtake_block_left, 1),
            "half_near": round(self.obs.half_near, 1),
            "target_off": round(float(self._target_offset), 1),
            "reason": reason,
        }
        self.pub_debug.publish(String(data=json.dumps(payload)))

    def _log(self, state, angle, speed, reason):
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds / 1e9 < self._log_period:
            return
        self._last_log = now
        ov = self.lateral.overtake.phase.value if self.lateral.overtake.active else "-"
        ref = self._ref
        self.get_logger().info(
            f"[{state.value}] angle={angle:+6.1f} speed={speed:5.1f} | "
            f"off={ref.offset_near:+6.1f}/{ref.offset_far:+6.1f} "
            f"q={ref.quality:.2f} {'OK' if ref.valid else 'HOLD'} | "
            f"light={LIGHT_NAME.get(self.obs.light, '?')}"
            f"{f' LEFT/{self.fsm.shortcut_phase.value}{self.fsm.shortcut_remain:.1f}s' if self.fsm.shortcut_phase else ''} "
            f"cone={self.obs.cone_n}{'[ZONE]' if self.cone_zone.active else ''} "
            f"ov={ov} | {reason}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._halt()
        except Exception:  # noqa: BLE001 - 종료 경로에서는 무엇도 막지 않는다
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
