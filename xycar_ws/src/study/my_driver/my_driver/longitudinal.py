"""종방향 목표 결정 — "얼마나 빨리 갈까".

ROS 의존성 없음.

조향보다 속도 결정이 기록을 좌우한다. 상황별 감속 요인을 곱으로 합성한다.

    v = base * f_curve * f_quality * f_cone * f_overtake   (감속 요인은 곱)

**입력이 전부 카메라다.** 예전에는 라이다 전방거리(front_dist)로 정지/감속
상한을 씌웠는데 2026-08-21 제거했다 — 정상 주행에서는 도달조차 못 하는
경로였고(라바콘 구간은 rubbercone_node 가 통째로 몰기 때문), 콘 구간 밖에서
켜면 라이다가 트랙 밖 벽을 잡아 곡선마다 정지가 걸렸다.
전방 장애물(방해차량)은 카메라 회피(lateral.py)가 담당한다.

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
        overtake_factor=0.7,
        car_detect_factor=0.7,
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

        # 회피 기동 중 감속.
        # 트랙 반쪽에 붙어 달리는 중이라 좌우 여유가 평소보다 없다. 게다가
        # 이 구간에서 우리가 가진 유일한 전방 정보는 카메라 bbox 뿐이라
        # (라이다 상한을 제거했다 — 모듈 docstring 참고) 명시적으로 눌러둔다.
        self.overtake_factor = overtake_factor
        # 방해차량을 **본 순간**부터 감속한다 (기동 시작 전).
        # [2026-08-21 실차] 회피가 잘 안 됐다. 원인은 접근 속도다 — 트리거가
        # 걸릴 때(bbox 하단 y >= 300px)는 이미 꽤 가까운데, 그 속도로는
        # shift_sec(0.8s) 동안 옆으로 벌리기 전에 차에 닿는다.
        # 탐지 시점부터 늦추면 벌릴 시간과 거리가 생긴다.
        self.car_detect_factor = car_detect_factor

        self.last_reason = ""

    def curvature_limit(self, waypoint_curvature=None):
        """2단계 확장 지점 — waypoint 곡률 기반 선행 감속.

        아직 쓰지 않는다. 지도가 생기면 여기서 "앞 코너가 얼마나 급한가"를
        속도 상한으로 바꾼다.
        """
        return None

    def update(self, lane_valid, offset_near, offset_far, quality,
               cone_n, overtake_active=False, car_ahead=False):
        """목표 속도를 반환한다 (xycar_motor 의 speed 단위).

        car_ahead: 방해차량이 **보이는가** (기동 중인지와 무관). 탐지만으로도
                   감속한다 — car_detect_factor 주석 참고.
        """
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

        # 4) 방해차량 관련 감속.
        #    두 요인을 곱하지 않고 **더 센 쪽 하나만** 쓴다. 곱하면
        #    0.7 x 0.7 = 0.49 로 과하게 느려져 랩타임만 잃는다.
        f = 1.0
        why = None
        if overtake_active:
            f = self.overtake_factor
            why = "overtake"
        if car_ahead and self.car_detect_factor < f:
            f = self.car_detect_factor
            why = "car ahead"
        if why is not None:
            v *= f
            reasons.append(why)

        if v > 0.0:
            v = max(v, self.min_speed)

        self.last_reason = ", ".join(reasons) if reasons else "clear"
        return v
