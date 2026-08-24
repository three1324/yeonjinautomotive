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
        vehicle_seen_factor=1.0,
        curve_preview_lo=60.0,
        curve_preview_hi=260.0,
        curve_preview_factor_min=0.55,
        curve_preview_release_lo=40.0,
        curve_preview_release_hi=100.0,
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
        # 방해차량이 **보이기만 하면** 걸리는 감속. 회피 기동 여부와
        # 무관하다 — 앞에 차가 있다는 것 자체로 여유를 둔다. 1.0 = 끔다.
        self.vehicle_seen_factor = vehicle_seen_factor
        # 선행 곱률 감속 — 곱선 **진입 전**에 미리 줄이는 항.
        self.curve_preview_lo = curve_preview_lo
        self.curve_preview_hi = curve_preview_hi
        self.curve_preview_factor_min = curve_preview_factor_min
        # 선행 감속을 **푸는** 문턱(bend 기준). bend 는 지금 눈앞의
        # 휘어짐이므로 이게 커졌다 = **이미 곱선 안**이라는 뜻이다.
        self.curve_preview_release_lo = curve_preview_release_lo
        self.curve_preview_release_hi = curve_preview_release_hi

        self.last_reason = ""

    def curvature_limit(self, waypoint_curvature=None):
        """2단계 확장 지점 — waypoint 곡률 기반 선행 감속.

        아직 쓰지 않는다. 지도가 생기면 여기서 "앞 코너가 얼마나 급한가"를
        속도 상한으로 바꾼다.
        """
        return None

    def update(self, lane_valid, offset_near, offset_far, quality,
               cone_n, overtake_active=False, curve_px=0.0,
               vehicle_seen=False):
        """목표 속도를 반환한다 (xycar_motor 의 speed 단위)."""
        v = self.base_speed
        reasons = []

        # 1) 곡률 감속 — 전방주시행과 근거리행의 차이가 곧 앞의 휘어짐
        # 1-a) **선행** 곱률 감속 (2026-08-24 신규)
        #   curve_px 는 2차피팅 a 계수 기반이라 횟편차·헤딩에 불변이고,
        #   y_lo~y_hi 전구간(약 0.46~3.75m)을 본다. bend(전방 1.25m 한 점)
        #   보다 훨씬 먼저 오르므로 **곱선에 들어가기 전**에 감속한다.
        #   그러면 아래 curve_factor_min 을 높게 두고도 안전해진다
        #   = **미리 줄이고 곱선 안에서는 빨리** 통과.
        bend0 = abs(offset_far - offset_near)
        pv = abs(curve_px)
        f = _lerp_factor(pv, self.curve_preview_lo, self.curve_preview_hi,
                         1.0, self.curve_preview_factor_min)
        # ★ 곱선 **안**에 들어오면 선행 감속을 푸는다.
        #   안 풀면 선행과 곱률 감속이 **곱해져** 곱선 안이 오히려
        #   접근보다 느려진다 — 원하는 것의 정반대다.
        #   curve_px 는 곱선 안에서도 크게 남아 있으므로 그것만으로는
        #   "접근 중"과 "안에 있음"을 구분할 수 없다. bend(눈앞의
        #   휘어짐)가 그 판별자다:
        #       curve_px 크고 bend 작다  -> 곱선이 다가온다 -> 감속
        #       curve_px 크고 bend 크다  -> 이미 곱선 안   -> 해제
        strength = _lerp_factor(bend0, self.curve_preview_release_lo,
                                self.curve_preview_release_hi, 1.0, 0.0)
        f = 1.0 - (1.0 - f) * strength
        if f < 0.99:
            reasons.append(f"preview({pv:.0f}px)")
        v *= f

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

        # 3-b) 방해차량이 보임 — 기동 여부와 무관하게 걸린다.
        if vehicle_seen and self.vehicle_seen_factor < 1.0:
            v *= self.vehicle_seen_factor
            reasons.append("vehicle")

        # 4) 회피 기동 중 — 트랙 반쪽에 붙어 달리는 중이라 여유가 없다
        if overtake_active:
            v *= self.overtake_factor
            reasons.append("overtake")

        if v > 0.0:
            v = max(v, self.min_speed)

        self.last_reason = ", ".join(reasons) if reasons else "clear"
        return v
