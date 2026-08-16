#!/usr/bin/env python3
"""주행 판단·제어 노드.

인지 결과를 받아 상태를 관리하고, 목표(횡위치·속도)를 정한 뒤, 실제 명령을 만든다.

구독:
    /lane      Float32MultiArray [offset_near, offset_far, valid, quality]
    /light     Int32             0=NONE 1=RED 2=YELLOW 3=GREEN 4=LEFT
    /objects   Float32MultiArray [cone_n, cone_near_y, car_present, car_cx, car_bottom_y]
    /obstacle  Float32MultiArray [front_dist, left_free, right_free]
발행:
    xycar_motor Float32MultiArray [angle, speed]

제어 루프는 콜백이 아니라 **고정 주기 타이머**로 돈다. 센서마다 도착 주기가 달라서
콜백 구동으로 짜면 제어 주기가 들쭉날쭉해지고 미분항(K_damp)이 불안정해진다.

안전 설계:
    - 시작 시 /drive_enable 을 받기 전까지 speed 0 을 계속 발행한다 (f1tenth 방식)
    - 인지 데이터가 끊기면(stale) 정지한다
    - 차선을 못 보면 조향을 서서히 중립으로 되돌리며 감속, 오래가면 정지
"""

from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Int32

from my_driver.control import SpeedLimiter, SteeringController
from my_driver.fsm import DriveFSM, State
from my_driver.lateral import LateralPlanner, OvertakeBehavior
from my_driver.longitudinal import LongitudinalPlanner

LIGHT_NAME = {0: "NONE", 1: "RED", 2: "YELLOW", 3: "GREEN", 4: "LEFT"}


@dataclass
class Obs:
    """최신 관측 묶음."""

    offset_near: float = 0.0
    offset_far: float = 0.0
    lane_valid: bool = False
    quality: float = 0.0

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
                # 횡방향
                ("lateral.enable_overtake", True),
                ("lateral.shift_px", 120.0),
                ("lateral.trigger_bottom_y", 300.0),
                ("lateral.trigger_front_dist", 1.5),
                ("lateral.side_clearance", 0.6),
                ("lateral.shift_sec", 0.8),
                ("lateral.pass_sec", 1.5),
                ("lateral.return_sec", 1.0),
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
        self.lateral = LateralPlanner(
            OvertakeBehavior(
                shift_px=g("lateral.shift_px").value,
                trigger_bottom_y=g("lateral.trigger_bottom_y").value,
                trigger_front_dist=g("lateral.trigger_front_dist").value,
                side_clearance=g("lateral.side_clearance").value,
                shift_sec=g("lateral.shift_sec").value,
                pass_sec=g("lateral.pass_sec").value,
                return_sec=g("lateral.return_sec").value,
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
        self._enabled = not self.require_enable
        self._last_lane_time = None
        self._lane_lost_since = None
        self._last_tick = self.get_clock().now()
        self._last_log = self._last_tick

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_motor = self.create_publisher(Float32MultiArray, "xycar_motor", qos)
        self.create_subscription(Float32MultiArray, "lane", self.on_lane, qos)
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
                "ros2 topic pub --once /drive_enable std_msgs/Bool '{data: true}'"
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
            return

        # 인지가 끊겼으면 무조건 정지. 오래된 값으로 달리는 게 가장 위험하다.
        if self._last_lane_time is None:
            self._publish(0.0, 0.0)
            return
        age = (now - self._last_lane_time).nanoseconds / 1e9
        if age > self.stale_timeout:
            self.get_logger().warn(f"인지 데이터 끊김 ({age:.2f}s) — 정지", throttle_duration_sec=1.0)
            self._halt()
            return

        state = self.fsm.update(self.obs.light, self.obs.lane_valid)

        if not self.fsm.should_drive:
            self._publish(0.0, 0.0)
            self._log(state, 0.0, 0.0, "wait")
            return

        # 차선을 오래 못 보면 정지
        if self.obs.lane_valid:
            self._lane_lost_since = None
        elif self._lane_lost_since is None:
            self._lane_lost_since = now

        if self._lane_lost_since is not None:
            lost = (now - self._lane_lost_since).nanoseconds / 1e9
            if lost > self.lane_lost_stop:
                self.get_logger().warn(f"차선 소실 {lost:.1f}s — 정지", throttle_duration_sec=1.0)
                self._halt()
                self._log(state, 0.0, 0.0, "lane lost")
                return

        target_offset = self.lateral.update(dt, self.obs, self.image_width)
        target_speed = self.longitudinal.update(
            self.obs.lane_valid, self.obs.offset_near, self.obs.offset_far,
            self.obs.quality, self.obs.cone_n, self.obs.front_dist,
        )
        speed = self.speed_limiter.update(dt, target_speed)

        if self.obs.lane_valid:
            angle = self.steering.update(
                dt, self.obs.offset_near, self.obs.offset_far, target_offset, speed
            )
        else:
            angle = self.steering.hold(dt)

        self._publish(angle, speed)
        self._log(state, angle, speed, self.longitudinal.last_reason)

    def _log(self, state, angle, speed, reason):
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds / 1e9 < self._log_period:
            return
        self._last_log = now
        ov = self.lateral.overtake.phase.value if self.lateral.overtake.active else "-"
        self.get_logger().info(
            f"[{state.value}] angle={angle:+6.1f} speed={speed:5.1f} | "
            f"off={self.obs.offset_near:+6.1f}/{self.obs.offset_far:+6.1f} "
            f"q={self.obs.quality:.2f} {'OK' if self.obs.lane_valid else 'HOLD'} | "
            f"light={LIGHT_NAME.get(self.obs.light, '?')} front={self.obs.front_dist:.2f}m "
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
