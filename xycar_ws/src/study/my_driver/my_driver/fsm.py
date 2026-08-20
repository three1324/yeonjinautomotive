"""주행 상태기계.

ROS 의존성 없음. 상태 전이만 담당하고 조향/속도는 계산하지 않는다.

상태를 최소한만 두는 이유 — 무엇을 상태로 빼고 무엇을 안 뺐나:
    - 라바콘: **상태로 빼지 않았다.** 콘 구간 진입/이탈은 cone_zone.py 가
      판정하고, driver_node 가 그때 제어권을 통째로 rubbercone_node 에
      넘긴다(mux). FSM 상태로 또 끊으면 판정자가 둘이 되어 어긋난다.
    - 추월  : LANE_DRIVE 안의 서브행동 (lateral.py). 별도 상태로 빼면 복귀가 지저분해진다
    - 지름길: 분기점 판단 근거가 아직 미정이라 1단계에서는 기본 비활성

> 이전 주석에 "라바콘은 차선을 가리지 않으므로 상태 전환 불필요"라고 적혀
> 있었는데 **그 전제는 틀렸다** (사진 판독 오류). 결론(상태로 빼지 않음)만
> 우연히 같았다. 근거는 위와 같이 바뀌었다.

출발 로직 검증: tools/start_sim.py (실측 검출률 68% 기준 몬테카를로).
"""

from enum import Enum


class State(Enum):
    WAIT_LIGHT = "WAIT_LIGHT"   # 출발선 정지, 초록불 대기
    LANE_DRIVE = "LANE_DRIVE"   # 기본 주행
    SHORTCUT = "SHORTCUT"       # 지름길 분기 판단 (2단계 이후)
    FINISH = "FINISH"           # 정지


# light_vote 의 상수와 맞춰야 한다
LIGHT_NONE, LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN, LIGHT_LEFT = 0, 1, 2, 3, 4


class DriveFSM:
    """상태 전이만 담당한다.

    start_confirm_frames:
        초록불이 몇 프레임 연속 유지돼야 출발할지. LightVoter 가 이미 투표로
        확정한 값을 주지만, 출발은 되돌릴 수 없는 동작이라 한 겹 더 확인한다.
    """

    def __init__(self, start_confirm_frames=5, enable_shortcut=False,
                 auto_start=False):
        self.start_confirm_frames = start_confirm_frames
        self.enable_shortcut = enable_shortcut
        # 신호등 없이 바로 주행 (실내 튜닝용). 실전에서는 반드시 False.
        self.auto_start = auto_start

        self.state = State.LANE_DRIVE if auto_start else State.WAIT_LIGHT
        self._green_count = 0
        self._reason = "init"

    @property
    def reason(self):
        """마지막 전이 사유. 로그·디버깅용."""
        return self._reason

    def reset(self):
        self.state = State.LANE_DRIVE if self.auto_start else State.WAIT_LIGHT
        self._green_count = 0
        self._reason = "reset"

    def update(self, light_state, lane_valid):
        """프레임당 1회. 새 상태를 반환한다.

        lane_valid: **현재 상태 전이에 쓰지 않는다.** 의도적이다.
            차선을 놓쳤다고 상태를 바꾸면 안 된다 — 콘 구간에서는 차선이 안
            보여도 라이다 복도로 계속 가야 하기 때문이다. 차선/복도를 모두
            놓쳤을 때의 정지 판단은 driver_node 가 융합 결과(ref.valid)와
            stale_timeout 으로 처리한다.
            시그니처에 남겨둔 이유는 2단계에서 "차선 장기 소실 -> 복구 동작"
            전이를 여기에 붙일 자리이기 때문이다.
        """
        if self.state is State.WAIT_LIGHT:
            if light_state == LIGHT_GREEN:
                self._green_count += 1
                if self._green_count >= self.start_confirm_frames:
                    self.state = State.LANE_DRIVE
                    self._reason = f"green x{self._green_count}"
            else:
                self._green_count = 0

        elif self.state is State.LANE_DRIVE:
            # 지름길 분기는 판단 근거(위치 or 신호등)가 확정되면 여기에 붙인다.
            # 근거 없이 넣으면 오전이·오작동으로 코스를 이탈할 수 있어 기본 비활성.
            if self.enable_shortcut and light_state == LIGHT_LEFT:
                self.state = State.SHORTCUT
                self._reason = "left arrow"

        elif self.state is State.SHORTCUT:
            if light_state != LIGHT_LEFT:
                self.state = State.LANE_DRIVE
                self._reason = "shortcut done"

        return self.state

    def force(self, state, reason="manual"):
        self.state = state
        self._reason = reason

    @property
    def should_drive(self):
        """이 상태에서 차를 움직여도 되는가."""
        return self.state in (State.LANE_DRIVE, State.SHORTCUT)
