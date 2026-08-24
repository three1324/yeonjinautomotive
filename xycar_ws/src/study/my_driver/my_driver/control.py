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

    def follow_external(self, dt, target):
        """바깥에서 준 목표 조향각을 **변화율 제한만 걸어** 따라간다.

        [2026-08-22] race_control/control.py 에서 그대로 옮겼다. 좌회전
        하드코딩 주행(TimedLeftDrive)의 TURN 위상이 쓰던 것인데, 그 구현을
        2026-08-23 에 지우면서 **지금은 호출하는 곳이 없다.** 외부 조향을
        rate limit 만 걸어 따라가는 일반 유틸이라 남겨 둔다.

        update() 와 다른 점: 차선 오차를 안 본다. 목표각이 이미 정해져
        있으므로 P/D 계산 없이 rate limit 과 상한만 통과시킨다.

        sync() 와 다른 점: sync 는 내부값을 **즉시** 그 각도로 맞추지만
        이쪽은 dt 만큼씩 걸어간다. 급격한 조향 입력이 서보에 그대로 가는
        것을 막으려는 것이다 — 원본이 그렇게 돼 있다.

        미분항은 무효화한다. 외부 조향 중의 오차 변화는 의미가 없다.
        """
        target = _clamp(float(target), -self.angle_limit, self.angle_limit)
        step = self.rate_limit * max(dt, 1e-3)
        self._cmd = _clamp(target, self._cmd - step, self._cmd + step)
        self._prev_err = None
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
    """가감속 제한. VESC 쪽에도 램핑이 있지만 명령 단계에서 한 겹 둔다.

    ────────────────────────────────────────────────────────────────
    정지 -> 주행 순간의 kick (2026-08-21)

    센서리스 BLDC 는 저 ERPM 에서 정류 동기를 못 잡는다. 이 차량 상수
    (speed_to_erpm_gain 4614, speed_weight 0.08)로 환산하면

        speed 4.06 = 1500 ERPM   <- 이 아래가 데드밴드
        speed 7.00 = 2584 ERPM   <- speed.min 을 여기로 올린 이유(8/19)

    그런데 accel_per_sec=8 로 0 부터 램프하면 speed 가 0->4.06 을 지나는
    **첫 0.5초가 통째로 데드밴드**다. 모터가 물었다 놨다 하며 "깔짝깔짝"
    거린다. 그래서 **정지에서 출발하는 순간에만** kick 까지 건너뛰고,
    그 뒤로는 평소 램프를 탄다.

    ⚠️ kick 은 반드시 데드밴드 위여야 한다. kick=4.0 은 1476 ERPM 이라
       여전히 데드밴드 안이다 — 그 값으로는 증상이 그대로 남는다.
       기본값을 speed.min 과 같게 두는 이유가 이것이다.
    ────────────────────────────────────────────────────────────────
    """

    def __init__(self, accel_per_sec, decel_per_sec, speed_limit, kick=0.0,
                 accel_bounds=None, accel_rates=None,
                 startup_hold_sec=0.0, startup_hold_speed=0.0):
        self.accel = accel_per_sec
        self.decel = decel_per_sec
        self.speed_limit = speed_limit
        self.kick = kick
        # ── 출발 직후 저속 유지 (2026-08-24) ──────────────────
        # kick 으로 데드밴드를 넘은 직후 바로 램프를 타면, 역기전력이
        # 아직 작은 상태에서 가속 전류가 기동 전류 위에 곹친다.
        # 잠시 그 속도를 유지해 회전수가 붙은 뒤에 올리면 첨두가 나뉘어진다.
        # 0 이면 이 기능을 끔다(옆 동작).
        self.startup_hold_sec = startup_hold_sec
        # 유지할 속도. 0 이면 kick 값을 그대로 쓴다.
        self.startup_hold_speed = startup_hold_speed
        self._hold_t = 0.0
        self._holding = False
        self._cmd = 0.0

        # ── 구간별 가속 (2026-08-24) ───────────────────────────────────
        # accel_bounds = [11.0, 15.0], accel_rates = [4.0, 7.0, 10.0] 이면
        #     speed <  11        -> 4.0 /s
        #     11 <= speed < 15   -> 7.0 /s
        #     15 <= speed        -> 10.0 /s
        # rates 는 bounds 보다 **정확히 하나 많아야** 한다.
        #
        # 왜 나누나: VESC UNDER_VOLTAGE(fault 2)는 전류가 배터리 전압을
        # 끌어내려 생긴다. BLDC 전류는 대략 (인가전압 - 역기전력)/저항 인데,
        # **역기전력은 회전수에 비례**한다. 즉 같은 가속 요구라도
        # **저속일수록 전류가 크다.** 평탄한 accel 은 가장 위험한 저속
        # 구간에 고속 구간과 똑같은 기울기를 준다 — 그게 문제였다.
        #
        # 저속만 완만하게 하고 고속은 오히려 키우면, 총 도달 시간은 거의
        # 같으면서 전류 첨두만 깎인다(합성계산: 7->18 이 1.83s vs 1.87s).
        self.accel_bounds = list(accel_bounds or [])
        self.accel_rates = list(accel_rates or [])
        if len(self.accel_rates) != len(self.accel_bounds) + 1:
            # 짝이 안 맞으면 조용히 틀리게 도는 것보다 끄는 편이 낫다.
            self.accel_bounds, self.accel_rates = [], []

    def accel_at(self, speed):
        """현재 속도에서 쓸 가속률(/s). 구간이 없으면 평탄값."""
        if not self.accel_rates:
            return self.accel
        for bound, rate in zip(self.accel_bounds, self.accel_rates):
            if speed < bound:
                return rate
        return self.accel_rates[-1]

    def reset(self):
        self._cmd = 0.0
        self._hold_t = 0.0
        self._holding = False

    def update(self, dt, target_speed):
        target = _clamp(target_speed, 0.0, self.speed_limit)

        # 정지 상태에서 "가라"는 명령이 처음 들어온 순간: 데드밴드를 건너뛴다.
        # target 을 넘지는 않게 한다 — 목표가 kick 보다 낮으면 그건 저속으로
        # 가라는 뜻이므로 억지로 밀어올릴 이유가 없다.
        if self._cmd <= 0.0 and target > 0.0 and self.kick > 0.0:
            self._cmd = min(target, self.kick)
            self._hold_t = 0.0
            self._holding = self.startup_hold_sec > 0.0
            return self._cmd

        # 출발 직후 저속 유지 — 이 동안은 목표를 hold 속도로 눌러둔다.
        if self._holding:
            self._hold_t += dt
            if self._hold_t >= self.startup_hold_sec:
                self._holding = False
            else:
                cap = (self.startup_hold_speed
                       if self.startup_hold_speed > 0.0 else self.kick)
                target = min(target, cap)

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
