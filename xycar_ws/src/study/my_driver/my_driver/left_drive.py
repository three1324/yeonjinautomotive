"""좌회전 하드코딩 주행 — LEFT 신호 소실 뒤 직진·고정조향·합류.

ROS 의존성 없음.

[2026-08-23 이식] 팀원 구현(race_control/left_drive.py 의 TimedLeftDrive)을
**그대로** 옮겼다. 실차에서 동작이 확인된 코드다. 위상·기본값·전이 조건을
바꾸지 않았다 — 바꾸면 그 검증이 무효가 된다.

이전에 있던 자체 구현(SHORTCUT: SHIFT -> GO -> TURN_IN -> FOLLOW)은
지웠다. 두 개를 동시에 두면 어느 쪽이 도는지 알 수 없다.

────────────────────────────────────────────────────────────────────────
다섯 위상

    IDLE        아무것도 안 함.

    WAIT_CLEAR  LEFT 를 확정한 **직후**. 아직 안 꺾는다. 신호등 표지가
                화면에서 사라질 때까지 평소대로 차선을 따라가되, 속도만
                미리 좌회전 구간 속도(left.speed)로 맞춰 둔다.
                왜 기다리나: "표지가 사라졌다" 는 것이 곧 **신호등을
                지나쳤다**는 위치 사건이다. 시간이 아니라 위치로 기점을
                잡아야 매 랩 같은 지점에서 꺾는다.

    STRAIGHT    straight_seconds(1.30s) 동안 **평소 차선 추종으로 직진**.
                분기점 입구까지 가는 거리다. 신호등 바로 앞에서 꺾으면
                분기 전에 꺾는 셈이 된다. 인지는 그대로 믿는다.

    TURN        turn_seconds(1.40s) 동안 **고정 조향**(left.turn_steer_deg).
                조건 없이 무조건 돈다 — 인지를 전혀 안 본다. 분기 안쪽은
                차선 표시가 신뢰할 수 없기 때문이다.

    EXIT        exit_seconds(7.0s) 동안 **본선으로 합류**. 다시 차선을
                따라가되 두 가지가 다르다:
                  1) 목표선을 0 -> exit_offset_px(+20px) 로
                     exit_ramp_seconds(0.5s)에 걸쳐 옮긴다. 합류하면서
                     본선 안쪽으로 붙기 위한 것이다.
                  2) 인지 쪽에서 **흰 실선을 버리고 노란 점선만** 쓴다
                     (/left_exit 토픽 -> perception_node 의
                     horizontal_dashed_left_strip). 합류 지점에서 앞을
                     가로지르는 본선 노란선·흰선이 그대로 들어오면 중앙
                     추정이 무너지기 때문이다.

    DONE        끝. 평소 차선주행으로 돌아간다.
────────────────────────────────────────────────────────────────────────
"""


class TimedLeftDrive:
    """LEFT 소실 뒤 직진·고정조향하고 제한된 dashed 로 복귀한다."""

    IDLE, WAIT_CLEAR, STRAIGHT, TURN, EXIT, DONE = range(6)
    NAMES = ("IDLE", "WAIT_CLEAR", "STRAIGHT", "TURN", "EXIT", "DONE")

    def __init__(self, straight_seconds, turn_seconds, exit_seconds=7.0,
                 exit_offset_px=20.0, exit_ramp_seconds=0.5):
        self.straight_seconds = float(straight_seconds)
        self.turn_seconds = float(turn_seconds)
        self.exit_seconds = float(exit_seconds)
        self.exit_offset_px = float(exit_offset_px)
        self.exit_ramp_seconds = float(exit_ramp_seconds)
        self.phase = self.IDLE
        self.started = 0.0
        self.held_speed = 0.0

    @property
    def phase_name(self):
        return self.NAMES[self.phase]

    @property
    def active(self):
        """무장~합류까지. DONE/IDLE 이 아니면 참."""
        return self.phase in (self.WAIT_CLEAR, self.STRAIGHT,
                              self.TURN, self.EXIT)

    @property
    def driving(self):
        """실제로 좌회전 시퀀스를 수행 중 (WAIT_CLEAR 는 아직 평소 주행)."""
        return self.phase in (self.STRAIGHT, self.TURN, self.EXIT)

    @property
    def exiting(self):
        """합류 위상. perception 의 /left_exit 스위치를 켜는 조건이다."""
        return self.phase == self.EXIT

    def arm(self):
        """LEFT 확정 시 호출. WAIT_CLEAR 로 무장. 이미 진행 중이면 False."""
        if self.active:
            return False
        self.phase = self.WAIT_CLEAR
        return True

    def begin_after_signal(self, now, speed):
        """LEFT 표지가 사라진 것이 확정됐을 때 호출. STRAIGHT 시작."""
        if self.phase != self.WAIT_CLEAR:
            return False
        self.phase = self.STRAIGHT
        self.started = float(now)
        self.held_speed = max(0.0, float(speed))
        return True

    def start(self, now, speed):
        """WAIT_CLEAR 를 건너뛰고 바로 시작 (수동/시험용)."""
        if self.active:
            return False
        self.phase = self.STRAIGHT
        self.started = float(now)
        self.held_speed = max(0.0, float(speed))
        return True

    def update(self, now):
        """프레임당 1회. 시간만으로 위상을 넘긴다. 새 위상을 반환."""
        if (self.phase == self.STRAIGHT
                and now - self.started >= self.straight_seconds):
            self.phase = self.TURN
            self.started = float(now)
        elif (self.phase == self.TURN
              and now - self.started >= self.turn_seconds):
            self.phase, self.started = self.EXIT, float(now)
        elif (self.phase == self.EXIT
              and now - self.started >= self.exit_seconds):
            self.phase = self.DONE
        return self.phase

    def target_offset(self, now):
        """EXIT 시작 0.5초 동안 목표선을 0 에서 +20px 까지 옮긴다."""
        elapsed = max(0.0, float(now) - self.started)
        if self.phase == self.EXIT:
            return self.exit_offset_px * min(
                1.0, elapsed / max(0.05, self.exit_ramp_seconds))
        return 0.0

    def remain(self, now):
        """현재 위상의 남은 시간(초). 로그·시각화용."""
        limits = {self.STRAIGHT: self.straight_seconds,
                  self.TURN: self.turn_seconds,
                  self.EXIT: self.exit_seconds}
        if self.phase not in limits:
            return 0.0
        return max(0.0, limits[self.phase] - (float(now) - self.started))

    def cancel(self):
        if self.active:
            self.phase = self.DONE

    def reset(self):
        self.phase = self.IDLE
        self.started = 0.0
        self.held_speed = 0.0

    def shift_time(self, seconds):
        """정지(STOP) 등으로 멈춘 시간만큼 기점을 밀어 위상 시간을 보존한다."""
        if self.driving:
            self.started += float(seconds)
