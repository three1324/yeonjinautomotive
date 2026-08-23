"""주행 상태기계.

ROS 의존성 없음. 상태 전이만 담당하고 조향/속도는 계산하지 않는다.

상태를 최소한만 두는 이유 — 무엇을 상태로 빼고 무엇을 안 뺐나:
    - 라바콘: **상태로 빼지 않았다.** 콘 구간 진입/이탈은 cone_zone.py 가
      판정하고, driver_node 가 그때 제어권을 통째로 rubbercone_node 에
      넘긴다(mux). FSM 상태로 또 끊으면 판정자가 둘이 되어 어긋난다.
    - 추월  : LANE_DRIVE 안의 서브행동 (lateral.py). 별도 상태로 빼면 복귀가
      지저분해진다.
    - 좌회전: LANE_DRIVE 안의 서브위상이 아니라 **상태(LEFT_TURN)로 뒀다.**
      추월과 달리 구간 안에서 인지를 안 믿고 정해진 각도로 꺾기 때문에,
      "지금 인지를 무시 중"이라는 사실이 상태로 드러나야 한다.
      위상 자체는 이 파일이 아니라 left_drive.TimedLeftDrive 가 들고 있다.

────────────────────────────────────────────────────────────────────────
[2026-08-23] 좌회전을 팀원 구현으로 **통째로 교체**했다.

    이전(자체 SHORTCUT): SHIFT -> GO -> TURN_IN -> FOLLOW.
        LEFT 확정 즉시 목표를 왼쪽으로 90px 옮겨 붙고, 신호등을 지나면
        1초 직진 뒤 1.5초 고정조향, 그 다음 좌측밴드 추종.
        -> 전부 지웠다.

    이후(TimedLeftDrive): WAIT_CLEAR -> STRAIGHT -> TURN -> EXIT.
        실차에서 동작이 확인된 코드다. 이 파일은 **트리거만** 담당하고
        (LEFT 몇 프레임 / 소실 몇 프레임), 위상과 시간은 left_drive.py 가
        가진다. 위상 정의는 그 파일 머리말 참고.

    두 구현을 같이 두지 않았다 — 어느 쪽이 도는지 알 수 없게 된다.
────────────────────────────────────────────────────────────────────────

출발 로직 검증: tools/start_sim.py (실측 검출률 68% 기준 몬테카를로).
"""

from enum import Enum

from my_driver.left_drive import TimedLeftDrive

# /light 토픽의 정수값. my_perception/light_vote.py 의 상수와 **같은 값이어야
# 한다** — 그쪽이 발행하고 여기서 해석한다.
#   (light_vote.py: NONE=0 RED=1 YELLOW=2 GREEN=3 LEFT=4)
LIGHT_NONE = 0
LIGHT_RED = 1
LIGHT_YELLOW = 2
LIGHT_GREEN = 3
LIGHT_LEFT = 4

LIGHT_NAMES = {LIGHT_NONE: "NONE", LIGHT_RED: "RED", LIGHT_YELLOW: "YELLOW",
               LIGHT_GREEN: "GREEN", LIGHT_LEFT: "LEFT"}


class State(Enum):
    WAIT_LIGHT = "WAIT_LIGHT"   # 출발선 정지, 초록불 대기
    LANE_DRIVE = "LANE_DRIVE"   # 기본 주행
    LEFT_TURN = "LEFT_TURN"     # 좌회전 (WAIT_CLEAR -> STRAIGHT -> TURN -> EXIT)
    FINISH = "FINISH"           # 정지


class DriveFSM:
    """상태 전이만 담당한다.

    start_confirm_frames:
        초록불이 몇 프레임 연속 유지돼야 출발할지. LightVoter 가 이미 투표로
        확정한 값을 주지만, 출발은 되돌릴 수 없는 동작이라 한 겹 더 확인한다.

    left_confirm_frames:
        좌회전 화살표가 몇 프레임 연속 확정돼야 무장할지. 팀원 설정은 **1**
        이다 (mission.yaml: signal.left_confirm_frames). LightVoter 가 이미
        투표로 확정한 뒤이므로 한 프레임으로도 오발동이 잘 안 난다는 판단이다.
        무장(WAIT_CLEAR)은 아직 안 꺾고 속도만 맞추는 단계라 되돌릴 수 있다 —
        되돌릴 수 없는 출발(start_confirm_frames=5)과 문턱이 다른 이유다.

    left_clear_frames:
        LEFT 가 아닌 프레임이 몇 번 연속이어야 "표지를 지나쳤다"로 볼지.
        팀원 설정은 3. 이 순간이 STRAIGHT 의 기점이다.
    """

    def __init__(self, start_confirm_frames=5, enable_left_turn=False,
                 auto_start=False,
                 left_confirm_frames=1, left_clear_frames=3,
                 left_speed=12.0,
                 left_straight_sec=1.30, left_turn_sec=1.40,
                 left_exit_sec=7.0, left_exit_offset_px=20.0,
                 left_exit_ramp_sec=0.5,
                 start_on_green=True, start_on_left=False):
        self.start_confirm_frames = start_confirm_frames
        # ── 어떤 신호에서 출발할지 (2026-08-22) ──
        # 예전에는 GREEN 고정이었다. 좌회전만으로 출발시키고 싶은 경우가
        # 있어서(코스 진입을 좌회전 화살표로 시작) 신호별로 분리했다.
        # 둘 다 true 면 둘 중 아무거나로 출발한다. 둘 다 false 면 영영
        # 출발하지 않는다 — auto_start 로만 움직일 수 있다.
        self._start_lights = set()
        if start_on_green:
            self._start_lights.add(LIGHT_GREEN)
        if start_on_left:
            self._start_lights.add(LIGHT_LEFT)
        self.enable_left_turn = enable_left_turn
        # 신호등 없이 바로 주행 (실내 튜닝용). 실전에서는 반드시 False.
        self.auto_start = auto_start

        self.left_confirm_frames = left_confirm_frames
        self.left_clear_frames = left_clear_frames
        self.left_speed = left_speed

        # 위상과 시간은 전부 이쪽이 들고 있다. 이 파일은 트리거만 본다.
        self.left = TimedLeftDrive(
            straight_seconds=left_straight_sec,
            turn_seconds=left_turn_sec,
            exit_seconds=left_exit_sec,
            exit_offset_px=left_exit_offset_px,
            exit_ramp_seconds=left_exit_ramp_sec,
        )

        self.state = State.LANE_DRIVE if auto_start else State.WAIT_LIGHT
        self._green_count = 0
        self._left_count = 0
        self._left_gone_count = 0
        # TimedLeftDrive 는 절대시각을 쓴다. FSM 은 ROS 를 모르므로 dt 를
        # 누적해 자체 시계를 만든다 — 단조증가면 충분하다.
        self._t = 0.0
        self._reason = "init"

    @property
    def reason(self):
        """마지막 전이 사유. 로그·디버깅용."""
        return self._reason

    def reset(self):
        self.state = State.LANE_DRIVE if self.auto_start else State.WAIT_LIGHT
        self._green_count = 0
        self._left_count = 0
        self._left_gone_count = 0
        self._t = 0.0
        self.left.reset()
        self._reason = "reset"

    def update(self, light_state, lane_valid, dt=0.0):
        """프레임당 1회. 새 상태를 반환한다.

        dt: 직전 호출로부터의 경과 시간(초). 좌회전 위상 시간을 재는 데만 쓴다.

        lane_valid: **현재 상태 전이에 쓰지 않는다.** 의도적이다.
            차선을 놓쳤다고 상태를 바꾸면 안 된다 — 콘 구간에서는 차선이 안
            보여도 라이다 복도로 계속 가야 하기 때문이다. 차선/복도를 모두
            놓쳤을 때의 정지 판단은 driver_node 가 융합 결과(ref.valid)와
            stale_timeout 으로 처리한다.
            시그니처에 남겨둔 이유는 2단계에서 "차선 장기 소실 -> 복구 동작"
            전이를 여기에 붙일 자리이기 때문이다.
        """
        self._t += dt

        if self.state is State.WAIT_LIGHT:
            if light_state in self._start_lights:
                self._green_count += 1
                if self._green_count >= self.start_confirm_frames:
                    self.state = State.LANE_DRIVE
                    name = LIGHT_NAMES.get(light_state, "?")
                    self._reason = f"{name.lower()} x{self._green_count}"
            else:
                self._green_count = 0

        elif self.state is State.LANE_DRIVE:
            # 좌회전 무장. 여기서는 아직 안 꺾는다 — WAIT_CLEAR 로 들어가
            # 표지가 사라지기를 기다린다.
            if self.enable_left_turn and light_state == LIGHT_LEFT:
                self._left_count += 1
                if (self._left_count >= self.left_confirm_frames
                        and self.left.arm()):
                    self.state = State.LEFT_TURN
                    self._left_gone_count = 0
                    self._reason = (f"left arrow x{self._left_count}"
                                    f" -> wait clear")
            else:
                self._left_count = 0

        elif self.state is State.LEFT_TURN:
            if self.left.phase == TimedLeftDrive.WAIT_CLEAR:
                # 표지가 사라지기를 기다린다. 소실 = 신호등이 화면 위로
                # 벗어났다 = 지나쳤다는 위치 사건. 시간이 아니라 위치로
                # 기점을 잡아야 매 랩 같은 지점에서 꺾는다.
                if light_state == LIGHT_LEFT:
                    self._left_gone_count = 0
                else:
                    self._left_gone_count += 1
                    if self._left_gone_count >= self.left_clear_frames:
                        self.left.begin_after_signal(self._t, self.left_speed)
                        self._reason = (f"left lost x{self._left_gone_count}"
                                        f" -> straight")
            else:
                # STRAIGHT / TURN / EXIT — **신호가 사라져도 끝내지 않는다.
                # 시간으로만 넘긴다.** 꺾기 시작하면 신호등이 곧 시야를
                # 벗어나 LIGHT_LEFT 가 NONE 이 되는데, 신호 유지로 끊으면
                # 진입 도중에 차선주행으로 돌아가 분기를 놓친다.
                prev = self.left.phase
                phase = self.left.update(self._t)
                if phase != prev:
                    self._reason = (f"{TimedLeftDrive.NAMES[prev]} done"
                                    f" -> {TimedLeftDrive.NAMES[phase]}")
                if phase == TimedLeftDrive.DONE:
                    self.state = State.LANE_DRIVE
                    self.left.reset()
                    self._left_count = 0
                    self._left_gone_count = 0
                    self._reason = "left turn done"

        return self.state

    def force(self, state, reason="manual"):
        self.state = state
        if state is not State.LEFT_TURN:
            self.left.reset()
        self._reason = reason

    @property
    def now(self):
        """FSM 내부 시계(초). TimedLeftDrive 에 넘긴 것과 같은 값이다.

        driver_node 가 target_offset(now)/remain(now) 를 부를 때 쓴다 —
        ROS 시계와 섞으면 기점이 어긋난다.
        """
        return self._t

    @property
    def left_phase(self):
        """좌회전 위상 정수. 다른 상태면 IDLE."""
        if self.state is not State.LEFT_TURN:
            return TimedLeftDrive.IDLE
        return self.left.phase

    @property
    def left_phase_name(self):
        return TimedLeftDrive.NAMES[self.left_phase]

    @property
    def left_remain(self):
        """현재 좌회전 위상의 남은 시간(초). 로그·시각화용."""
        if self.state is not State.LEFT_TURN:
            return 0.0
        return self.left.remain(self._t)

    @property
    def should_drive(self):
        """이 상태에서 차를 움직여도 되는가."""
        return self.state in (State.LANE_DRIVE, State.LEFT_TURN)
