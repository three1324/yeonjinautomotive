"""목표 -> 실제 명령 (조향각/속도) + 안전장치.

ROS 의존성 없음.

조향은 3항으로 만든다:

    steer = K_lat   * (offset_near - target_offset)     ① 횡오차 피드백
          + K_curve * (offset_far  - offset_near)       ② 곡률 선행보상
          + K_damp  * d(error)/dt                       ③ 진동 억제

Pure Pursuit 과의 관계:
    Pure Pursuit 은 delta = atan(2·L·sin(α)/Ld) 인데, Ld·sin(α) 가 곧 전방주시점의
    횡방향 편차다. 즉 우리 ②항과 대응한다. **구조는 같고 단위만 픽셀이다.**
    미터 단위로 가려면 호모그래피 캘리브레이션이 필요한데, 이 카메라는 배럴 왜곡이
    심하고 하단이 차체에 가려 캘리브레이션이 까다롭다. 게다가 xycar_motor 가 받는
    angle 이 이미 임의 단위(-50~50)라 어느 쪽이든 실차 게인 튜닝이 필요하다.
    그래서 픽셀 기반으로 가고, 게인 3개만 맞춘다.

    한계: 속도가 변하면 최적 게인이 변한다. speed_gain_ref 로 1차 보완한다.

안전장치 3가지는 전부 측정 결과에 근거한다:
    - rate limiter : 프레임간 100px 초과 급변이 2.6% 관측됨
    - LPF          : 서보 떨림·기계 부담 감소
    - 결측 대응    : lane valid=False 면 마지막 조향 유지 + 감속, 오래가면 정지
"""


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SteeringController:

    def __init__(
        self,
        k_lat, k_curve, k_damp,
        angle_limit,
        rate_limit_per_sec,
        lpf_alpha,
        speed_gain_ref=0.0,
        invert=False,
    ):
        self.k_lat = k_lat
        self.k_curve = k_curve
        self.k_damp = k_damp
        self.angle_limit = angle_limit              # xycar_motor angle 절대 상한
        self.rate_limit = rate_limit_per_sec        # 초당 허용 변화량
        self.lpf_alpha = lpf_alpha                  # 0~1. 클수록 응답 빠르고 떨림 많음
        # 속도 적응 게인: 빠를수록 조향을 덜 먹인다. 0이면 비활성.
        self.speed_gain_ref = speed_gain_ref
        # 조향 부호가 반대로 나오면 True (배선/장착에 따라 다름)
        self.invert = invert

        self._prev_err = None
        self._cmd = 0.0

    def reset(self):
        self._prev_err = None
        self._cmd = 0.0

    def update(self, dt, offset_near, offset_far, target_offset, speed):
        err = offset_near - target_offset

        derr = 0.0
        if self._prev_err is not None and dt > 1e-6:
            derr = (err - self._prev_err) / dt
        self._prev_err = err

        bend = offset_far - offset_near

        gain = 1.0
        if self.speed_gain_ref > 0.0 and speed > 0.0:
            # 고속에서 같은 게인을 쓰면 과조향이 된다
            gain = self.speed_gain_ref / max(speed, 1e-3)
            gain = _clamp(gain, 0.3, 1.5)

        raw = gain * (self.k_lat * err + self.k_curve * bend + self.k_damp * derr)
        if self.invert:
            raw = -raw

        # 저역통과
        filtered = self.lpf_alpha * raw + (1.0 - self.lpf_alpha) * self._cmd

        # 변화율 제한 — 인식이 튀어도 조향은 급변하지 않게
        max_step = self.rate_limit * max(dt, 1e-3)
        filtered = _clamp(filtered, self._cmd - max_step, self._cmd + max_step)

        self._cmd = _clamp(filtered, -self.angle_limit, self.angle_limit)
        return self._cmd

    def sync(self, angle):
        """다른 제어기가 조향을 잡고 있는 동안 내부 상태를 그 값에 맞춰둔다.

        라바콘 구간에서는 rubbercone_node 가 조향을 만든다. 그동안 이 제어기의
        `_cmd` 를 0 인 채로 두면, 구간을 빠져나와 제어권이 돌아오는 순간
        rate limit 이 0 에서부터 다시 올라가느라 실제 바퀴 각도와 어긋난다
        (예: 콘 구간을 +30도로 빠져나왔는데 0도부터 시작).

        미분항도 무효화한다 — 제어를 놓고 있던 동안의 오차 변화는 의미가 없다.
        """
        self._cmd = _clamp(float(angle), -self.angle_limit, self.angle_limit)
        self._prev_err = None

    def hold(self, dt):
        """차선을 못 봤을 때 — 마지막 조향을 유지하되 서서히 중립으로 되돌린다.

        완전히 유지하면 못 보는 동안 계속 같은 방향으로 돌아 코스를 이탈한다.
        """
        decay = _clamp(1.0 - 0.5 * dt, 0.0, 1.0)
        self._cmd *= decay
        return self._cmd


class SpeedLimiter:
    """가감속 제한. VESC 쪽에도 램핑이 있지만 명령 단계에서 한 겹 둔다."""

    def __init__(self, accel_per_sec, decel_per_sec, speed_limit):
        self.accel = accel_per_sec
        self.decel = decel_per_sec
        self.speed_limit = speed_limit
        self._cmd = 0.0

    def reset(self):
        self._cmd = 0.0

    def update(self, dt, target_speed):
        target = _clamp(target_speed, 0.0, self.speed_limit)
        if target > self._cmd:
            step = self.accel * max(dt, 1e-3)
            self._cmd = min(target, self._cmd + step)
        else:
            step = self.decel * max(dt, 1e-3)
            self._cmd = max(target, self._cmd - step)
        return self._cmd

    def stop_now(self):
        self._cmd = 0.0
        return self._cmd
