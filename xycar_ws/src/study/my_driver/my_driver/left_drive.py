"""좌회전 하드코딩 주행 — LEFT 표지가 사라진 뒤 직진 + 고정조향.

ROS 의존성 없음. tools/ 에서 그대로 돌려볼 수 있다.

────────────────────────────────────────────────────────────────────────
출처 (2026-08-22)

race_control/left_drive.py 의 TimedLeftDrive 를 **그대로** 옮겼다.
상태기 본체(arm / begin_after_signal / update / cancel / shift_time)는
원본과 동일하다. 실차에서 검증된 코드라 손대지 않는 것이 목적이다.

원본에는 이 클래스 말고 LeftDrive(분기 차선 검출 추종)도 있었으나
가져오지 않았다 — 인지(branch detector)까지 딸려와야 하고, 이 저장소의
차선 추정에 개입해서 평소 주행을 바꾼다.

────────────────────────────────────────────────────────────────────────
기존 fsm.ShortcutPhase 와의 관계

같은 일을 하는 구현이 이미 fsm.py 에 있다(ARM_WAIT -> ARM_GO -> TURN_IN).
둘이 동시에 돌면 좌회전을 두 번 시도하므로 **하나만 켠다**:

    fsm.enable_shortcut: false   +   left.hardcoded_enabled: true    <- 지금
    fsm.enable_shortcut: true    +   left.hardcoded_enabled: false   <- 되돌리기

위상 대응은 1:1 이고 값만 다르다:

    ARM_WAIT (LEFT 소실 대기)  <->  WAIT_CLEAR
    ARM_GO   0.5s              <->  STRAIGHT   1.32s
    TURN_IN  1.0s @ 35도       <->  TURN       1.20s @ 20도

★ 꺾는 방식이 다르다. fsm 쪽 TURN_IN 은 조향을 rate limit 없이 그대로
  내지만(steering.sync), 이쪽 TURN 은 steering.follow_external() 로
  **변화율 제한을 통과시킨다.** 그래서 각도가 작아도(20도) 실제 궤적은
  비슷할 수 있다 — 어느 쪽이 맞는지는 실차에서 봐야 한다.
"""


class TimedLeftDrive:
    """LEFT 표지가 사라진 뒤 시작하는 직진-고정조향 상태기.

    IDLE       : 대기.
    WAIT_CLEAR : LEFT 를 확인하고 arm() 된 상태. 표지가 **사라질 때까지**
                 평소대로 차선을 따라간다.
    STRAIGHT   : 표지가 사라진 시점부터 straight_seconds 동안 차선 추종.
                 분기점까지 가는 거리다.
    TURN       : turn_seconds 동안 고정 조향으로 꺾는다.
    DONE       : 끝. 다시 쓰려면 새로 arm() 해야 한다.

    시간 기준은 호출부가 넘기는 now (초). ROS 시계든 time.monotonic 이든
    상관없지만 **한 종류로 일관**되어야 한다.
    """

    IDLE, WAIT_CLEAR, STRAIGHT, TURN, DONE = range(5)
    NAMES = ("IDLE", "WAIT_CLEAR", "STRAIGHT", "TURN", "DONE")

    def __init__(self, straight_seconds, turn_seconds):
        self.straight_seconds = float(straight_seconds)
        self.turn_seconds = float(turn_seconds)
        self.phase = self.IDLE
        self.started = 0.0
        self.held_speed = 0.0

    @property
    def active(self):
        return self.phase in (self.WAIT_CLEAR, self.STRAIGHT, self.TURN)

    @property
    def driving(self):
        return self.phase in (self.STRAIGHT, self.TURN)

    def arm(self):
        if self.active:
            return False
        self.phase = self.WAIT_CLEAR
        return True

    def begin_after_signal(self, now, speed):
        if self.phase != self.WAIT_CLEAR:
            return False
        self.phase = self.STRAIGHT
        self.started = float(now)
        self.held_speed = max(0.0, float(speed))
        return True

    def start(self, now, speed):
        if self.active:
            return False
        self.phase = self.STRAIGHT
        self.started = float(now)
        self.held_speed = max(0.0, float(speed))
        return True

    def update(self, now):
        if (self.phase == self.STRAIGHT
                and now - self.started >= self.straight_seconds):
            self.phase = self.TURN
            self.started = float(now)
        elif (self.phase == self.TURN
              and now - self.started >= self.turn_seconds):
            self.phase = self.DONE
        return self.phase

    def cancel(self):
        if self.active:
            self.phase = self.DONE

    def shift_time(self, seconds):
        if self.driving:
            self.started += float(seconds)
