#!/usr/bin/env python3
"""주행 판단·제어 노드.

인지 결과를 받아 상태를 관리하고, 목표(횡위치·속도)를 정한 뒤, 실제 명령을 만든다.

구독:
    /lane      Float32MultiArray [offset_near, offset_far, valid, quality]
    /corridor  Float32MultiArray [offset_near, offset_far, valid, quality]
               라바콘 복도 중앙 (라이다). /lane 과 같은 형식·단위(픽셀)라
               그대로 섞을 수 있다. 섞는 정책은 fusion.py 참고.
    /light     Int32             0=NONE 1=RED 2=YELLOW 3=GREEN 4=LEFT
    /objects   Float32MultiArray [cone_n, cone_near_y, car_present, car_cx, car_bottom_y]
    /obstacle  Float32MultiArray [front_dist, left_free, right_free]
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

    카메라  차선 추종 · 신호등 · **방해차량 회피** · 라바콘 구간 진입 판정
    라이다  **라바콘 구간에서만** — 콘 복도 중앙(/corridor) + 전방 장애물 상한

라이다는 트랙 밖 벽·기둥·관중과 실제 장애물을 구분하지 못한다. 상시
사용하면 곡선에서 벽이 섹터에 들어올 때마다 정지가 걸리고, 차선을 잠깐
놓칠 때마다 엉뚱한 것을 주행 기준으로 삼는다. 콘 구간은 통로가 좁아
섹터 안이 곧 콘 벽이므로 그 구간에서만 신뢰한다.

구간 판정은 cone_zone.ConeZoneDetector 한 곳에서만 하고, 그 결과를
fusion / longitudinal 두 소비처가 공유한다 (켜지는 시점이 어긋나지 않게).
"""

import json
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Int32, String

from my_driver.cone_zone import ConeZoneDetector
from my_driver.control import SpeedLimiter, SteeringController
from my_driver.fsm import DriveFSM, State
from my_driver.fusion import FusedResult, LateralFusion, LateralRef
from my_driver.lateral import LateralPlanner, OvertakeBehavior
from my_driver.longitudinal import LongitudinalPlanner

LIGHT_NAME = {0: "NONE", 1: "RED", 2: "YELLOW", 3: "GREEN", 4: "LEFT"}


@dataclass
class Obs:
    """최신 관측 묶음."""

    # 차선 (카메라)
    offset_near: float = 0.0
    offset_far: float = 0.0
    lane_valid: bool = False
    quality: float = 0.0
    # 학습된 트랙 반폭(px). 회피 목표를 "트랙 반쪽의 중앙"으로 잡는 데 쓴다.
    # 0.0 이면 인지가 아직 좌우 흰선을 동시에 본 적이 없다는 뜻 -> 고정값 폴백.
    half_near: float = 0.0
    half_far: float = 0.0

    # 라바콘 복도 (라이다). 같은 단위(픽셀)
    cor_near: float = 0.0
    cor_far: float = 0.0
    cor_valid: bool = False
    cor_quality: float = 0.0

    light: int = 0

    cone_n: int = 0
    cone_near_y: float = 0.0
    car_present: bool = False
    car_cx: float = 0.0
    car_bottom_y: float = 0.0

    front_dist: float = 99.0
    left_free: float = 99.0
    right_free: float = 99.0


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
                ("stale_timeout_sec", 0.5),
                ("lane_lost_stop_sec", 2.0),
                ("log_period_sec", 1.0),
                # FSM
                ("fsm.start_confirm_frames", 5),
                ("fsm.enable_shortcut", False),
                ("fsm.auto_start", False),
                # 라바콘 구간 판정 — 라이다를 쓸 구간을 여기서 정한다 (cone_zone.py)
                ("cone_zone.enter_n", 3),
                ("cone_zone.exit_n", 1),
                ("cone_zone.exit_hold_sec", 1.5),
                # 차선 <-> 복도 전환 (구간에 따라 센서가 통째로 바뀐다)
                ("fusion.weight_rate_per_sec", 1.5),
                ("fusion.min_corridor_quality", 0.4),
                # 횡방향 (회피는 카메라 전용 — 라이다 파라미터 없음)
                ("lateral.enable_overtake", True),
                ("lateral.shift_px", 120.0),
                ("lateral.trigger_bottom_y", 300.0),
                ("lateral.shift_sec", 0.8),
                ("lateral.pass_sec", 1.5),
                ("lateral.return_sec", 1.0),
                ("lateral.cooldown_sec", 1.0),
                ("lateral.pass_exit_ratio", 0.85),
                ("lateral.pass_exit_cx_ratio", 0.85),
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
                ("speed.stop_dist", 0.35),
                ("speed.slow_dist", 1.2),
                ("speed.overtake_factor", 0.7),
                ("speed.obstacle_cap_in_cone_only", True),
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
            auto_start=g("fsm.auto_start").value,
        )
        # 라이다를 쓸 구간(라바콘)을 정하는 유일한 판정자. 카메라 콘 개수로 판단한다.
        self.cone_zone = ConeZoneDetector(
            enter_n=g("cone_zone.enter_n").value,
            exit_n=g("cone_zone.exit_n").value,
            exit_hold_sec=g("cone_zone.exit_hold_sec").value,
        )
        self.fusion = LateralFusion(
            weight_rate_per_sec=g("fusion.weight_rate_per_sec").value,
            min_corridor_quality=g("fusion.min_corridor_quality").value,
        )
        self.lateral = LateralPlanner(
            OvertakeBehavior(
                shift_px=g("lateral.shift_px").value,
                trigger_bottom_y=g("lateral.trigger_bottom_y").value,
                shift_sec=g("lateral.shift_sec").value,
                pass_sec=g("lateral.pass_sec").value,
                return_sec=g("lateral.return_sec").value,
                cooldown_sec=g("lateral.cooldown_sec").value,
                pass_exit_ratio=g("lateral.pass_exit_ratio").value,
                pass_exit_cx_ratio=g("lateral.pass_exit_cx_ratio").value,
            ),
            enable_overtake=g("lateral.enable_overtake").value,
        )
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
            stop_dist=g("speed.stop_dist").value,
            slow_dist=g("speed.slow_dist").value,
            overtake_factor=g("speed.overtake_factor").value,
            obstacle_cap_in_cone_only=g("speed.obstacle_cap_in_cone_only").value,
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
        )

        self.obs = Obs()
        # 마지막 융합 결과. 로그에서 어느 기준을 쓰고 있는지 보여주는 용도.
        self._ref = FusedResult()
        self._enabled = not self.require_enable
        self._last_lane_time = None
        self._last_corridor_time = None
        self._lane_lost_since = None
        self._target_offset = 0.0
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
        self.create_subscription(Float32MultiArray, "corridor", self.on_corridor, qos)
        self.create_subscription(Int32, "light", self.on_light, qos)
        self.create_subscription(Float32MultiArray, "objects", self.on_objects, qos)
        self.create_subscription(Float32MultiArray, "obstacle", self.on_obstacle, qos)
        self.create_subscription(Bool, "drive_enable", self.on_enable, qos)

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
        self._last_lane_time = self.get_clock().now()

    def on_corridor(self, msg):
        if len(msg.data) < 4:
            return
        self.obs.cor_near = msg.data[0]
        self.obs.cor_far = msg.data[1]
        self.obs.cor_valid = msg.data[2] > 0.5
        self.obs.cor_quality = msg.data[3]
        self._last_corridor_time = self.get_clock().now()

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

    def on_obstacle(self, msg):
        if len(msg.data) < 3:
            return
        self.obs.front_dist = msg.data[0]
        self.obs.left_free = msg.data[1]
        self.obs.right_free = msg.data[2]

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

        state = self.fsm.update(self.obs.light, self.obs.lane_valid)

        if not self.fsm.should_drive:
            self._publish(0.0, 0.0)
            self._log(state, 0.0, 0.0, "wait")
            self._pub_debug(state.value, 0.0, 0.0, "wait")
            return

        # ── 라이다를 쓸 구간인가 (2026-08-19 결정: 라바콘 구간에서만) ──
        # 판정은 **카메라** 콘 개수로 한다. 라이다를 언제 쓸지를 라이다로 정하면
        # 순환이기 때문이다. 아래 두 소비처(fusion, longitudinal)가 이 값 하나를
        # 공유해야 켜지고 꺼지는 시점이 어긋나지 않는다. (cone_zone.py 참고)
        in_cone_zone = self.cone_zone.update(dt, self.obs.cone_n)

        # 차선(카메라)과 콘 복도(라이다)를 섞어 최종 횡방향 기준을 만든다.
        # 라바콘 구간에서는 콘 벽이 실제 주행 가능 경계이므로 복도 쪽 비중이 커진다.
        # obstacle_node 가 죽어도 마지막 /corridor 값은 남아 있다. 낡은 복도를
        # 계속 따라가면 콘 벽이 이미 지나갔는데도 그쪽으로 조향한다.
        # 차선과 같은 기준(stale_timeout_sec)으로 무효 처리한다.
        cor_fresh = (
            self._last_corridor_time is not None
            and (now - self._last_corridor_time).nanoseconds / 1e9 <= self.stale_timeout
        )

        ref = self.fusion.update(
            dt,
            LateralRef(self.obs.offset_near, self.obs.offset_far,
                       self.obs.lane_valid, self.obs.quality),
            LateralRef(self.obs.cor_near, self.obs.cor_far,
                       self.obs.cor_valid and cor_fresh, self.obs.cor_quality),
            cone_zone=in_cone_zone,
        )
        self._ref = ref

        # 기준을 아예 못 만드는 상태가 오래가면 정지.
        # 차선만 보고 판단하면 안 된다 — 콘 구간에서 차선이 안 보여도
        # 복도가 살아있으면 계속 갈 수 있어야 한다.
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

        target_offset = self.lateral.update(dt, self.obs, self.image_width)
        self._target_offset = target_offset
        # lateral 을 먼저 돌려야 이번 tick 의 회피 상태가 정해진다.
        # 종방향은 그 결과(기동 중인지)를 받아 감속한다 — 순서가 중요하다.
        target_speed = self.longitudinal.update(
            ref.valid, ref.offset_near, ref.offset_far,
            ref.quality, self.obs.cone_n, self.obs.front_dist,
            overtake_active=self.lateral.overtake.active,
            cone_zone=in_cone_zone,
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
            "source": ref.source,
            "corridor_weight": round(ref.corridor_weight, 2),
            "light": LIGHT_NAME.get(self.obs.light, "?"),
            "front_dist": round(self.obs.front_dist, 2),
            "cone_n": self.obs.cone_n,
            # 라이다를 쓰고 있는 구간인지. 화면에서 바로 구분돼야 "지금 복도를
            # 따라가는 중인가, 차선을 따라가는 중인가"를 오해하지 않는다.
            "cone_zone": bool(self.cone_zone.active),
            "overtake": ov,
            # 회피 기동의 근거를 그대로 노출한다. 화면만 보고 "왜 저쪽으로
            # 피했는가 / 왜 안 피하는가"를 알 수 있어야 튜닝이 가능하다.
            "ot_dir": self.lateral.overtake.direction,
            "ot_amount": round(self.lateral.overtake.amount, 1),
            "ot_reason": self.lateral.overtake.last_reason,
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
            f"q={ref.quality:.2f} {'OK' if ref.valid else 'HOLD'} "
            f"[{ref.source} w{ref.corridor_weight:.2f}] | "
            f"light={LIGHT_NAME.get(self.obs.light, '?')} front={self.obs.front_dist:.2f}m "
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
