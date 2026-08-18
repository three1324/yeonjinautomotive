"""종방향 목표 결정 — "얼마나 빨리 갈까".

ROS 의존성 없음.

조향보다 속도 결정이 기록을 좌우한다. 상황별 감속 요인을 곱으로 합성하고,
마지막에 장애물 거리로 상한을 씌운다.

    v = base * f_curve * f_quality * f_cone       (감속 요인은 곱)
    v = min(v, f_obstacle(front_dist))            (장애물은 상한)

f1tenth pure_pursuit 도 같은 발상으로 조향각에 비례해 감속한다
(speed = max - (max-min) * |angle|/steer_max).

2단계 확장 지점:
    waypoint 곡률을 받으면 curvature_limit() 에서 "아직 안 보이는 코너"까지
    미리 감속할 수 있다. 지금은 카메라에 보이는 만큼만 안다.
"""


def _lerp_factor(x, x0, x1, f0, f1):
    """x 가 x0->x1 로 갈 때 f0->f1 로 선형 보간. 범위 밖은 클램프."""
    if x1 <= x0:
        return f1
    t = (x - x0) / (x1 - x0)
    t = max(0.0, min(1.0, t))
    return f0 + (f1 - f0) * t


class LongitudinalPlanner:

    def __init__(
        self,
        base_speed,
        min_speed,
        curve_px_lo, curve_px_hi, curve_factor_min,
        quality_lo, quality_factor_min,
        cone_n_lo, cone_n_hi, cone_factor_min,
        stop_dist, slow_dist,
    ):
        self.base_speed = base_speed
        self.min_speed = min_speed

        # 곡률: |offset_far - offset_near| 가 클수록 앞이 휘고 있다는 뜻
        self.curve_px_lo = curve_px_lo
        self.curve_px_hi = curve_px_hi
        self.curve_factor_min = curve_factor_min

        # 차선 품질이 낮으면 보수적으로
        self.quality_lo = quality_lo
        self.quality_factor_min = quality_factor_min

        # 라바콘 구간
        self.cone_n_lo = cone_n_lo
        self.cone_n_hi = cone_n_hi
        self.cone_factor_min = cone_factor_min

        # 장애물 거리 상한
        self.stop_dist = stop_dist    # 이 거리 이하면 정지
        self.slow_dist = slow_dist    # 이 거리부터 감속 시작

        self.last_reason = ""

    def curvature_limit(self, waypoint_curvature=None):
        """2단계 확장 지점 — waypoint 곡률 기반 선행 감속.

        아직 쓰지 않는다. 지도가 생기면 여기서 "앞 코너가 얼마나 급한가"를
        속도 상한으로 바꾼다.
        """
        return None

    def update(self, lane_valid, offset_near, offset_far, quality,
               cone_n, front_dist):
        """목표 속도를 반환한다 (xycar_motor 의 speed 단위)."""
        v = self.base_speed
        reasons = []

        # 1) 곡률 감속 — 전방주시행과 근거리행의 차이가 곧 앞의 휘어짐
        bend = abs(offset_far - offset_near)
        f = _lerp_factor(bend, self.curve_px_lo, self.curve_px_hi, 1.0, self.curve_factor_min)
        if f < 0.99:
            reasons.append(f"curve({bend:.0f}px)")
        v *= f

        # 2) 차선 품질 — 못 보고 있으면 느리게
        if not lane_valid:
            v *= self.quality_factor_min
            reasons.append("lane_invalid")
        else:
            f = _lerp_factor(quality, self.quality_lo, 1.0, self.quality_factor_min, 1.0)
            if f < 0.99:
                reasons.append(f"quality({quality:.2f})")
            v *= f

        # 3) 라바콘 구간 — 트랙이 좁아지는 구간이므로 여유를 둔다
        f = _lerp_factor(cone_n, self.cone_n_lo, self.cone_n_hi, 1.0, self.cone_factor_min)
        if f < 0.99:
            reasons.append(f"cone(x{cone_n})")
        v *= f

        # 4) 장애물 상한 — 곱이 아니라 상한. 앞이 막히면 다른 요인과 무관하게 느려야 한다
        if front_dist <= self.stop_dist:
            v = 0.0
            reasons.append(f"STOP({front_dist:.2f}m)")
        elif front_dist < self.slow_dist:
            cap = _lerp_factor(front_dist, self.stop_dist, self.slow_dist,
                               self.min_speed, self.base_speed)
            if cap < v:
                v = cap
                reasons.append(f"obstacle({front_dist:.2f}m)")

        if v > 0.0:
            v = max(v, self.min_speed)

        self.last_reason = ", ".join(reasons) if reasons else "clear"
        return v
