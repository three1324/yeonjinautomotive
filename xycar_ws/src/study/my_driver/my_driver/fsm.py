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
    SHORTCUT = "SHORTCUT"       # 지름길 (SHIFT -> GO -> TURN_IN -> FOLLOW)
    FINISH = "FINISH"           # 정지


class ShortcutPhase(Enum):
    """SHORTCUT 안의 서브위상. 네 걸음이다.

    SHIFT   : LEFT 확정 **즉시** 진입. 목표를 왼쪽으로 shortcut_shift_px
              옮겨(회피 기동과 같은 방식) 1차선 안쪽으로 붙는다. 횡방향
              기준은 평소 추정 그대로다. **LEFT 가 사라질 때까지 유지**한다
              — 그 순간이 곧 "신호등을 지나쳤다"는 위치 사건이다.
    GO      : 신호등을 지난 뒤 shortcut_straight_sec(1.0s) 동안 **직진**.
              안쪽으로 붙은 오프셋은 그대로 두고 평소 추정으로 달린다.
              분기점 입구까지 가는 거리다 — 신호등 바로 앞에서 꺾으면
              분기 전에 꺾는 셈이 된다.
    TURN_IN : shortcut_turn_sec(1.5s) 동안 **고정 조향으로 좌회전**.
              인지를 전혀 안 본다 — 정해진 각도(turn_angle)와 속도
              (turn_speed)만 낸다.
    FOLLOW  : 목표 오프셋을 0 으로 되돌리고, SHORTCUT 이 끝날 때까지
              좌측밴드 노란선을 계속 따라간다.

    ────────────────────────────────────────────────────────────────
    왜 이 순서인가 (2026-08-23, 사용자 결정)

    SHIFT 를 LEFT 확정 즉시 시작하는 이유: 좌회전 차선의 노란 픽셀이
    화면에 들어오려면 미리 안쪽으로 붙어 있어야 한다. 신호등을 지난 뒤
    붙기 시작하면 이미 늦다.

    GO -> TURN_IN 을 **조건 없이 시간으로** 도는 이유(2026-08-23 사용자
    결정): 좌측밴드 탐지를 조건으로 걸었더니 분기 초입에서 안 잡히는
    프레임이 있어 회전이 안 걸리거나 늦게 걸렸다. 분기 위치는 신호등
    기준으로 거의 고정이므로, 인지를 기다리지 말고 **신호등 소실 +
    1초 직진 + 고정 조향**의 개루프로 확정적으로 꺾는다.

    ⚠️ 대가는 명확하다 — **LEFT 오검출 한 번에 조건 없이 좌회전한다.**
       분기가 없는 곳이면 그대로 트랙을 벗어난다. 방어는
       shortcut_confirm_frames(3) 하나뿐이다. 오발동이 보이면 그 값과
       light.min_weight_left / min_ratio_left 를 먼저 올릴 것.

    ⚠️ 개루프이므로 진입 속도가 달라지면 회전량이 달라진다. 그래서
       TURN_IN 은 속도까지 고정한다(turn_speed).

    신호등 본체는 먼 거리에서도 안정적으로 잡힌다. 즉 **LEFT 확정 시점이
    신호등에서 몇 m 앞인지가 매번 다르다**(속도·조명·각도에 따라).
    그래서 꾫는 시점은 확정이 아니라 **LEFT 소실**로 끊는다 — 그것은
    신호등이 화면 위로 벗어났다는 뜻 = 차가 신호등에 거의 다다랐다는
    위치 사건이고 속도와 무관하다.
    (반면 SHIFT 는 미리 붙어야 하므로 확정 즉시 시작한다.)

    ⚠️ 소실 감지에는 구조적 지연이 있다:
           실제로 화면에서 벗어남
             + light.miss_tolerance      LightVoter 가 NONE 을 내기까지
             + shortcut_lost_frames      여기서 소실을 확정하기까지
       더 빨리 반응시키려면 **light.miss_tolerance 를 줄여야** 한다
       (그쪽이 지배적이다). 단 그 값은 빨간불 정지 등 다른 판정에도 영향을
       준다.

    ⚠️ LEFT 가 끝내 안 사라지는 경우(신호등 앞에 섬거나 화살표가 계속
       보이는 각도)를 위해 shortcut_arm_timeout_sec 안전장치를 둔다.
       여기 갇히면 좌회전을 영영 못 한다.
    ──────────────────────────────────────────────────────────

    **좌측밴드 노란선(offset_*_left)은 FOLLOW 에서만 본다.**
    SHIFT/GO 는 평소 추정 + 오프셋으로 달리고, TURN_IN 은 인지를 아예
    안 본다(고정 조향).
    분기점에서는 dashed 마스크에 직진 가지와 좌회전 가지가 같이 들어오는데,
    전부 평균내면 그 사이를 겨냥해 어느 쪽도 못 탄다. 행별로 가장 왼쪽
    픽셀에서 lane.left_band_px 안쪽만 남기면 좌회전 가지가 선택되고,
    좌회전을 마친 뒤에는 합류 차선을 그대로 따라간다.

    ⚠️ FOLLOW 에서 노란선을 못 보면 갈 곳이 없다. 그때는 driver_node 가
       평소 차선 추정으로 폴백한다(lane_valid_left=False).
    """

    SHIFT = "SHIFT"
    GO = "GO"
    TURN_IN = "TURN_IN"
    FOLLOW = "FOLLOW"


class DriveFSM:
    """상태 전이만 담당한다.

    start_confirm_frames:
        초록불이 몇 프레임 연속 유지돼야 출발할지. LightVoter 가 이미 투표로
        확정한 값을 주지만, 출발은 되돌릴 수 없는 동작이라 한 겹 더 확인한다.
    """

    def __init__(self, start_confirm_frames=5, enable_shortcut=False,
                 auto_start=False, shortcut_total_sec=15.0,
                 shortcut_straight_sec=1.0, shortcut_turn_sec=1.5,
                 shortcut_confirm_frames=3,
                 shortcut_lost_frames=3, shortcut_arm_timeout_sec=5.0,
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
        # 신호등을 지난 뒤 **직진**하는 시간. 분기점 입구까지 가는 거리다.
        self.shortcut_straight_sec = shortcut_straight_sec
        # 그 뒤 **고정 조향으로 좌회전**하는 시간. 조건 없이 무조건 돈다.
        self.shortcut_turn_sec = shortcut_turn_sec
        # 좌회전 화살표가 몇 프레임 연속 확정돼야 진입할지.
        self.shortcut_confirm_frames = shortcut_confirm_frames
        # LEFT 가 아닌 프레임이 몇 번 연속이어야 "사라졌다"로 볼지.
        # SHIFT -> TURN_IN 전이 조건이다 (= 신호등을 지나쳤다).
        self.shortcut_lost_frames = shortcut_lost_frames
        # LEFT 가 끝내 안 사라질 때의 안전장치. SHIFT 에서 이만큼 지나면
        # 사라진 것으로 치고 TURN_IN 으로 넘어간다 — 여기 갇히면 못 꺾는다.
        self.shortcut_arm_timeout_sec = shortcut_arm_timeout_sec

        self.state = State.LANE_DRIVE if auto_start else State.WAIT_LIGHT
        self._green_count = 0
        self._left_count = 0
        self._left_gone_count = 0
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
        self._left_gone_count = 0
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
                    # **확정 즉시 안쪽으로 붙기 시작한다.** 좌회전 차선의
                    # 노란 픽셀이 화면에 들어오려면 미리 붙어 있어야 한다.
                    # 꺾는 것은 신호등을 지난 뒤(TURN_IN)다.
                    self.state = State.SHORTCUT
                    self._phase = ShortcutPhase.SHIFT
                    self._left_gone_count = 0
                    self._shortcut_t = 0.0
                    self._reason = f"left arrow x{self._left_count} -> shift"
            else:
                self._left_count = 0

        elif self.state is State.SHORTCUT:
            self._shortcut_t += dt

            if self._phase is ShortcutPhase.SHIFT:
                # 안쪽으로 붙은 채로 LEFT 가 사라지기를 기다린다.
                # 소실 = 신호등이 화면 위로 벗어났다 = 지나쳤다는 위치 사건.
                if light_state == LIGHT_LEFT:
                    self._left_gone_count = 0
                else:
                    self._left_gone_count += 1
                if self._left_gone_count >= self.shortcut_lost_frames:
                    self._phase = ShortcutPhase.GO
                    self._shortcut_t = 0.0      # 총 시간은 여기서부터 센다
                    self._reason = (f"left lost x{self._left_gone_count}"
                                    f" -> go straight")
                elif self._shortcut_t >= self.shortcut_arm_timeout_sec:
                    # 안전장치. 신호등 앞에 섰거나 화살표가 계속 보이는
                    # 각도다. 사라진 것으로 치고 진행한다.
                    self._phase = ShortcutPhase.GO
                    self._shortcut_t = 0.0
                    self._reason = (f"arm timeout "
                                    f"({self.shortcut_arm_timeout_sec:.0f}s)"
                                    f" -> go straight")

            else:
                # GO / TURN_IN / FOLLOW — **신호가 사라져도 끝내지 않는다.
                # 총 시간으로만 끊는다.** 꺾기 시작하면 신호등이 곧 시야를
                # 벗어나 LIGHT_LEFT 가 NONE 이 되는데, 신호 유지로 끊으면
                # 진입 도중에 차선주행으로 돌아가 분기를 놓친다.
                #
                # 시각은 GO 시작(= 신호등 소실)부터 하나의 타이머로 잰다:
                #     0 ~ straight_sec              GO
                #     ~ straight_sec + turn_sec     TURN_IN
                #     ~ total_sec                   FOLLOW
                turn_end = self.shortcut_straight_sec + self.shortcut_turn_sec
                if (self._phase is ShortcutPhase.GO
                        and self._shortcut_t >= self.shortcut_straight_sec):
                    self._phase = ShortcutPhase.TURN_IN
                    self._reason = (
                        f"straight done ({self.shortcut_straight_sec:.1f}s) "
                        f"-> turn in")
                elif (self._phase is ShortcutPhase.TURN_IN
                        and self._shortcut_t >= turn_end):
                    self._phase = ShortcutPhase.FOLLOW
                    self._reason = (
                        f"turn in done ({self.shortcut_turn_sec:.1f}s) "
                        f"-> follow left band")
                if self._shortcut_t >= self.shortcut_total_sec:
                    self.state = State.LANE_DRIVE
                    self._phase = None
                    self._left_count = 0
                    self._left_gone_count = 0
                    self._reason = (f"shortcut done "
                                    f"({self.shortcut_total_sec:.0f}s)")

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
