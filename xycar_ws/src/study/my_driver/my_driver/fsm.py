"""주행 상태기계.

ROS 의존성 없음. 상태 전이만 담당하고 조향/속도는 계산하지 않는다.

상태를 최소한만 두는 이유 — 무엇을 상태로 빼고 무엇을 안 뺐나:
    - 라바콘: **상태로 빼지 않았다.** 콘 구간 진입/이탈은 cone_zone.py 가
      판정하고, driver_node 가 그때 제어권을 통째로 rubbercone_node 에
      넘긴다(mux). FSM 상태로 또 끊으면 판정자가 둘이 되어 어긋난다.
    - 추월  : LANE_DRIVE 안의 서브행동 (lateral.py). 별도 상태로 빼면 복귀가 지저분해진다
    - 지름길: LANE_DRIVE 안의 서브위상이 아니라 **상태(SHORTCUT)로 뒀다.**
      추월과 달리 구간 안에서 인지를 안 믿고 정해진 각도로 꺾기 때문에,
      "지금 인지를 무시 중"이라는 사실이 상태로 드러나야 한다.
      SHORTCUT 내부는 두 위상이다 — SHIFT(왼쪽으로 70px 붙기) ->
      FOLLOW(좌측밴드 노란선 추종). 총 시간으로만 끊는다.

> 이전 주석에 "라바콘은 차선을 가리지 않으므로 상태 전환 불필요"라고 적혀
> 있었는데 **그 전제는 틀렸다** (사진 판독 오류). 결론(상태로 빼지 않음)만
> 우연히 같았다. 근거는 위와 같이 바뀌었다.

출발 로직 검증: tools/start_sim.py (실측 검출률 68% 기준 몬테카를로).
"""

from enum import Enum

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
    SHORTCUT = "SHORTCUT"       # 지름길 좌회전 (SHIFT -> FOLLOW, 총 15초)
    FINISH = "FINISH"           # 정지


class ShortcutPhase(Enum):
    """SHORTCUT 안의 서브위상. 두 걸음뿐이다.

    SHIFT  : 진입 직후 shortcut_shift_sec 동안 **왼쪽으로 shortcut_shift_px**
             목표를 옮긴다(회피 기동과 같은 방식). 1차선으로 붙어서 좌측
             노란 픽셀이 화면에 잘 들어오게 하는 것이 목적이다.
    FOLLOW : 목표 오프셋을 0 으로 되돌리고, SHORTCUT 이 끝날 때까지
             좌측밴드 노란선을 계속 따라간다.

    **두 위상 모두 좌측밴드 노란선(offset_*_left)을 횡방향 기준으로 쓴다.**
    분기점에서는 dashed 마스크에 직진 가지와 좌회전 가지가 같이 들어오는데,
    전부 평균내면 그 사이를 겨냥해 어느 쪽도 못 탄다. 행별로 가장 왼쪽
    픽셀에서 lane.left_band_px 안쪽만 남기면 좌회전 가지가 선택되고,
    좌회전을 마친 뒤에는 합류 차선을 그대로 따라간다.

    ────────────────────────────────────────────────────────────────
    2026-08-22 개편 — 고정 조향에서 **인지 추종**으로

    예전에는 ARM_WAIT(LEFT 소실 대기) -> ARM_GO(직진) -> TURN_IN(고정 조향
    +고정 속도, 인지 안 봄) 세 위상이었다. TURN_IN 은 차선을 전혀 안 보고
    정해진 각도로 정해진 시간 꺾는 개루프라, 진입 위치·속도가 조금만 달라도
    회전 반경이 통째로 어긋났고 되돌릴 방법이 없었다.

    지금은 좌회전 차선 자체를 인지로 따라간다. 그래서:
      - LEFT 소실을 기다릴 필요가 없다 (위치 사건으로 쓰던 것) -> 확정 즉시 진입
      - turn_angle / turn_sec / arm_sec 같은 개루프 상수가 필요 없다
      - 끝은 **총 시간**으로만 끊는다 (shortcut_total_sec)

    ⚠️ 대신 좌회전 구간에서 노란선을 못 보면 갈 곳이 없다. 그때는
       driver_node 가 평소 차선 추정으로 폴백한다(lane_valid_left=False).
    ────────────────────────────────────────────────────────────────
    """

    SHIFT = "SHIFT"
    FOLLOW = "FOLLOW"


class DriveFSM:
    """상태 전이만 담당한다.

    start_confirm_frames:
        초록불이 몇 프레임 연속 유지돼야 출발할지. LightVoter 가 이미 투표로
        확정한 값을 주지만, 출발은 되돌릴 수 없는 동작이라 한 겹 더 확인한다.
    """

    def __init__(self, start_confirm_frames=5, enable_shortcut=False,
                 auto_start=False, shortcut_total_sec=15.0,
                 shortcut_shift_sec=1.5, shortcut_confirm_frames=3,
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
        self.enable_shortcut = enable_shortcut
        # 신호등 없이 바로 주행 (실내 튜닝용). 실전에서는 반드시 False.
        self.auto_start = auto_start
        # SHORTCUT 전체 유지 시간. **신호가 사라져도 이 시간으로만 끊는다.**
        # 꺾기 시작하면 신호등이 곧 시야를 벗어나 LIGHT_LEFT 가 NONE 이 되는데,
        # 신호 유지로 끊으면 진입 도중에 차선주행으로 돌아가 분기를 놓친다.
        self.shortcut_total_sec = shortcut_total_sec
        # 그중 앞부분 — 왼쪽으로 목표를 옮겨 1차선으로 붙는 시간.
        self.shortcut_shift_sec = shortcut_shift_sec
        # 좌회전 화살표가 몇 프레임 연속 확정돼야 진입할지.
        self.shortcut_confirm_frames = shortcut_confirm_frames

        self.state = State.LANE_DRIVE if auto_start else State.WAIT_LIGHT
        self._green_count = 0
        self._left_count = 0
        self._shortcut_t = 0.0
        self._phase = None
        self._reason = "init"

    @property
    def reason(self):
        """마지막 전이 사유. 로그·디버깅용."""
        return self._reason

    def reset(self):
        self.state = State.LANE_DRIVE if self.auto_start else State.WAIT_LIGHT
        self._green_count = 0
        self._left_count = 0
        self._shortcut_t = 0.0
        self._phase = None
        self._reason = "reset"

    def update(self, light_state, lane_valid, dt=0.0):
        """프레임당 1회. 새 상태를 반환한다.

        dt: 직전 호출로부터의 경과 시간(초). SHORTCUT 위상 시간을 재는 데만 쓴다.

        lane_valid: **현재 상태 전이에 쓰지 않는다.** 의도적이다.
            차선을 놓쳤다고 상태를 바꾸면 안 된다 — 콘 구간에서는 차선이 안
            보여도 라이다 복도로 계속 가야 하기 때문이다. 차선/복도를 모두
            놓쳤을 때의 정지 판단은 driver_node 가 융합 결과(ref.valid)와
            stale_timeout 으로 처리한다.
            시그니처에 남겨둔 이유는 2단계에서 "차선 장기 소실 -> 복구 동작"
            전이를 여기에 붙일 자리이기 때문이다.
        """
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
            # 좌회전(지름길) 진입. 초록불 출발과 같은 방식으로 연속 확정을 요구한다 —
            # 진입하면 트랙 왼쪽 끝으로 붙으므로 오검출 한 번에 들어가면 위험하다.
            if self.enable_shortcut and light_state == LIGHT_LEFT:
                self._left_count += 1
                if self._left_count >= self.shortcut_confirm_frames:
                    # LEFT 소실을 기다리지 않는다 — 확정 즉시 진입한다.
                    # 예전에는 "신호등을 지났다"는 위치 사건이 필요했지만,
                    # 지금은 좌회전 차선을 인지로 따라가므로 그 기준점이
                    # 필요 없다. 오히려 일찍 1차선으로 붙어야 좌측 노란
                    # 픽셀이 화면에 잘 들어온다.
                    self.state = State.SHORTCUT
                    self._phase = ShortcutPhase.SHIFT
                    self._shortcut_t = 0.0
                    self._reason = f"left arrow x{self._left_count}"
            else:
                self._left_count = 0

        elif self.state is State.SHORTCUT:
            # **신호가 사라져도 끝내지 않는다. 총 시간으로만 끊는다.**
            # 꺾기 시작하면 신호등이 곧 시야에서 벗어나(지나쳐 버리거나 각도가
            # 틀어져) LIGHT_LEFT 가 금방 NONE 이 된다. 신호 유지로 끊으면
            # 진입 도중에 차선주행으로 돌아가 분기를 놓친다.
            self._shortcut_t += dt
            if (self._phase is ShortcutPhase.SHIFT
                    and self._shortcut_t >= self.shortcut_shift_sec):
                self._phase = ShortcutPhase.FOLLOW
                self._reason = (f"shift done ({self.shortcut_shift_sec:.1f}s) "
                                f"-> follow left band")
            if self._shortcut_t >= self.shortcut_total_sec:
                self.state = State.LANE_DRIVE
                self._phase = None
                self._left_count = 0
                self._reason = f"shortcut done ({self.shortcut_total_sec:.0f}s)"

        return self.state

    def force(self, state, reason="manual"):
        self.state = state
        self._phase = ShortcutPhase.SHIFT if state is State.SHORTCUT else None
        self._shortcut_t = 0.0
        self._reason = reason

    @property
    def shortcut_phase(self):
        """SHORTCUT 서브위상. 다른 상태면 None.

        driver_node 가 이걸 보고 SHIFT 동안 목표를 왼쪽으로 옮긴다.
        """
        if self.state is not State.SHORTCUT:
            return None
        return self._phase

    @property
    def shortcut_remain(self):
        """현재 SHORTCUT 위상의 남은 시간(초). 다른 상태면 0. 로그·시각화용.

        위상과 무관하게 **SHORTCUT 전체가 끝날 때까지** 남은 시간이다.
        시계가 하나뿐이라(총 시간) 그것을 그대로 보여주는 편이 오해가 없다.
        """
        if self.state is not State.SHORTCUT:
            return 0.0
        return max(0.0, self.shortcut_total_sec - self._shortcut_t)

    @property
    def should_drive(self):
        """이 상태에서 차를 움직여도 되는가."""
        return self.state in (State.LANE_DRIVE, State.SHORTCUT)
