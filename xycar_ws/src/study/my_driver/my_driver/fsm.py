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
      SHORTCUT 내부는 세 위상이다 — ARM_WAIT(LEFT 소실 대기) ->
      ARM_GO(조금 더 직진) -> TURN_IN(고정 조향). 앞의 둘은 평소 주행이다.

> 이전 주석에 "라바콘은 차선을 가리지 않으므로 상태 전환 불필요"라고 적혀
> 있었는데 **그 전제는 틀렸다** (사진 판독 오류). 결론(상태로 빼지 않음)만
> 우연히 같았다. 근거는 위와 같이 바뀌었다.

출발 로직 검증: tools/start_sim.py (실측 검출률 68% 기준 몬테카를로).
"""

from enum import Enum


class State(Enum):
    WAIT_LIGHT = "WAIT_LIGHT"   # 출발선 정지, 초록불 대기
    LANE_DRIVE = "LANE_DRIVE"   # 기본 주행
    SHORTCUT = "SHORTCUT"       # 지름길 좌회전 진입 (ARM -> TURN_IN)
    STOP_RED = "STOP_RED"       # 주행 중 빨간불 -> 정지
    FINISH = "FINISH"           # 정지


class ShortcutPhase(Enum):
    """SHORTCUT 안의 서브위상. ARM 이 두 걸음으로 나뉜다.

    ARM_WAIT : LEFT 신호가 **사라질 때까지** 대기. 평소대로 차선 주행한다.
    ARM_GO   : 거기서 shortcut_arm_sec 만큼 더 직진. 역시 평소 주행.
    TURN_IN  : 고정 조향각 + 고정 속도로 정해진 시간 꺾는다. 이 동안은
               인지를 전혀 안 본다 (차선/콘 모두). 분기 초입에서는 차선이
               끊기거나 엉뚱하게 잡혀서, 믿으면 오히려 못 꺾는다.

    ────────────────────────────────────────────────────────────────
    왜 ARM 을 "확정 후 고정 시간"이 아니라 "LEFT 소실"로 끊는가

    신호등 본체는 먼 거리에서도 conf 0.80~0.88 로 안정적으로 잡힌다. 즉
    **LEFT 확정 시점이 신호등에서 몇 m 떨어진 곳인지가 매번 다르다**
    (속도·조명·각도에 따라). 그 흔들리는 지점에서 고정 시간을 세면 진입
    위치가 재현되지 않는다.

    반면 LEFT 가 사라지는 순간은 신호등이 화면 위로 벗어났다는 뜻 =
    **차가 신호등에 거의 다다랐다는 위치 사건**이다. 속도와 무관하다.

    ⚠️ 소실 감지에는 구조적 지연이 있다:
           실제로 화면에서 벗어남
             + miss_tolerance(10프레임)  LightVoter 가 NONE 을 내기까지
             + shortcut_lost_frames(3)   여기서 소실을 확정하기까지
           = 약 13프레임 ≈ 0.6초 (22fps 기준)
       이 지연이 이미 "약간 직진"의 상당 부분을 먹는다. 그래서
       shortcut_arm_sec 는 1.5 가 아니라 0.5 다. 더 빨리 반응시키려면
       **light.miss_tolerance 를 줄여야지** shortcut_lost_frames 가 아니다
       (그쪽이 10프레임으로 지배적이다). 단 miss_tolerance 는 빨간불
       정지 등 다른 신호 판정에도 영향을 준다.
    """
    ARM_WAIT = "ARM_WAIT"
    ARM_GO = "ARM_GO"
    TURN_IN = "TURN_IN"


# light_vote 의 상수와 맞춰야 한다
LIGHT_NONE, LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN, LIGHT_LEFT = 0, 1, 2, 3, 4


class DriveFSM:
    """상태 전이만 담당한다.

    start_confirm_frames:
        초록불이 몇 프레임 연속 유지돼야 출발할지. LightVoter 가 이미 투표로
        확정한 값을 주지만, 출발은 되돌릴 수 없는 동작이라 한 겹 더 확인한다.
    """

    def __init__(self, start_confirm_frames=3, enable_shortcut=False,
                 auto_start=False, shortcut_arm_sec=0.5,
                 shortcut_turn_sec=1.0, shortcut_arm_timeout_sec=5.0,
                 shortcut_lost_frames=3, shortcut_confirm_frames=3,
                 enable_red_stop=True, red_confirm_frames=3,
                 red_release_sec=3.0, none_tolerance=1):
        self.start_confirm_frames = start_confirm_frames
        self.enable_shortcut = enable_shortcut
        # 신호등 없이 바로 주행 (실내 튜닝용). 실전에서는 반드시 False.
        self.auto_start = auto_start
        # LEFT 가 **사라진 뒤** 평소대로 더 달릴 시간. 분기점까지의 거리다.
        # 소실 감지 자체가 이미 ~0.6초 늦으므로 이 값은 작아야 한다
        # (ShortcutPhase docstring 의 지연 계산 참고).
        self.shortcut_arm_sec = shortcut_arm_sec
        # LEFT 가 끝내 안 사라지는 경우의 안전장치 — 신호등 앞에 섰거나
        # 다른 화살표가 계속 보이면 ARM_WAIT 에 영원히 갇힌다.
        self.shortcut_arm_timeout_sec = shortcut_arm_timeout_sec
        # LEFT 가 아닌 프레임이 몇 번 연속이어야 "사라졌다"로 볼지.
        # 진입(shortcut_confirm_frames)과 대칭이지만 값은 따로 둔다 —
        # 늦게 확정하면 분기를 지나치므로 진입보다 짧게 잡는다.
        self.shortcut_lost_frames = shortcut_lost_frames
        # 고정 조향으로 꺾는 시간. 이 둘 다 실차에서 맞춰야 하는 값이다.
        self.shortcut_turn_sec = shortcut_turn_sec
        # 좌회전 화살표가 몇 프레임 연속 확정돼야 진입할지. 출발과 같은 이유로
        # 한 겹 더 확인한다 (한 번 꺾으면 되돌릴 수 없다).
        self.shortcut_confirm_frames = shortcut_confirm_frames

        # ── 주행 중 빨간불 정지 (2026-08-21) ────────────────────────
        # 출발선의 WAIT_LIGHT 와 **다른 상태**로 뒀다. 둘의 탈출 조건이
        # 정반대이기 때문이다:
        #   WAIT_LIGHT : 초록불이 올 때까지 **무한정** 기다린다. 출발선에서
        #                신호를 못 봤다고 제멋대로 출발하면 실격이다.
        #   STOP_RED   : 신호등이 시야에서 사라지면 **스스로 풀린다**. 트랙
        #                한복판에서 오검출로 멈췄을 때 영영 못 움직이면
        #                미완주다 — 정지보다 나쁘다.
        # 하나로 합치면 둘 중 하나를 반드시 희생하게 된다.
        #
        # ★ WAIT_LIGHT 블록은 이 변경으로 **한 글자도 바뀌지 않는다.**
        #   초록불 출발이 안 되던 문제를 다시 만들지 않기 위한 원칙이다.
        self.enable_red_stop = enable_red_stop
        self.red_confirm_frames = red_confirm_frames
        self.red_release_sec = red_release_sec
        # 확정 카운트 도중 NONE(신호등 판독 실패)이 몇 프레임까지 끼어도
        # 무시할지. **모든 신호 확정에 공통 적용된다** (출발·빨간불·좌회전).
        #
        # NONE 은 "그 신호가 아니다"가 아니라 "못 읽었다"이다. 한 프레임
        # 놓쳤다고 카운트를 0 으로 되돌리면, 인식이 조금만 흔들려도 카운트가
        # 영영 안 차서 출발도 정지도 못 한다 — 실차에서 겪은 증상이 이것이다.
        # 반면 **다른 신호를 확실히 본 경우**(예: 빨간불 세는 중에 초록불)는
        # 관용 없이 즉시 리셋한다. 둘을 같게 다루면 안 된다.
        self.none_tolerance = none_tolerance

        self.state = State.LANE_DRIVE if auto_start else State.WAIT_LIGHT
        self._green_count = 0
        self._left_count = 0
        self._red_count = 0
        self._green_none = 0
        self._left_none = 0
        self._red_none = 0
        self._shortcut_t = 0.0
        self._phase = None
        self._left_gone_count = 0
        self._none_t = 0.0
        self._reason = "init"

    def _tick_count(self, hit, light_state, count, none_run):
        """확정 카운터 한 스텝. (새 count, 새 none_run) 을 돌려준다.

        hit          : 이번 프레임이 찾던 신호인가
        light_state  : 아니라면 무엇이었나 (NONE 인지 구분하려고 받는다)

        규칙은 세 갈래다:
            찾던 신호       -> +1, NONE 연속 기록 초기화
            NONE (판독실패) -> 관용 안에서는 **그대로 유지**, 넘으면 리셋
            다른 신호       -> 즉시 리셋

        네 군데(출발/빨간불/좌회전/재출발)가 같은 규칙을 써야 하므로
        복붙하지 않고 여기 하나로 모았다.
        """
        if hit:
            return count + 1, 0
        if light_state == LIGHT_NONE:
            none_run += 1
            return (0 if none_run > self.none_tolerance else count), none_run
        return 0, 0

    @property
    def reason(self):
        """마지막 전이 사유. 로그·디버깅용."""
        return self._reason

    def reset(self):
        self.state = State.LANE_DRIVE if self.auto_start else State.WAIT_LIGHT
        self._green_count = 0
        self._left_count = 0
        self._red_count = 0
        self._green_none = 0
        self._left_none = 0
        self._red_none = 0
        self._shortcut_t = 0.0
        self._phase = None
        self._left_gone_count = 0
        self._none_t = 0.0
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
            # 초록불 **또는 좌회전 화살표**면 출발한다 (2026-08-21 LEFT 추가).
            # 좌회전 신호도 "가도 된다"는 뜻이라 출발선에서 이것만 켜지는
            # 경우 초록을 기다리면 영영 못 나간다.
            # ★ 초록불 경로는 이 변경으로 **동작이 바뀌지 않는다** — 조건에
            #   LEFT 를 더했을 뿐 카운터·확정프레임·리셋은 그대로다.
            go = light_state in (LIGHT_GREEN, LIGHT_LEFT)
            self._green_count, self._green_none = self._tick_count(
                go, light_state, self._green_count, self._green_none)
            if go and self._green_count >= self.start_confirm_frames:
                self.state = State.LANE_DRIVE
                self._reason = (
                    f"{'green' if light_state == LIGHT_GREEN else 'left'} "
                    f"x{self._green_count}")

        elif self.state is State.LANE_DRIVE:
            # 주행 중 빨간불 -> 정지. 좌회전 진입보다 **먼저** 본다.
            # 투표는 한 번에 하나만 확정하므로 둘이 동시에 참일 수는 없지만,
            # 우선순위를 코드로 못박아 둔다 — 안전 정지가 미션보다 위다.
            is_red = self.enable_red_stop and light_state == LIGHT_RED
            self._red_count, self._red_none = self._tick_count(
                is_red, light_state, self._red_count, self._red_none)
            if is_red and self._red_count >= self.red_confirm_frames:
                self.state = State.STOP_RED
                self._red_count = self._red_none = 0
                self._green_count = self._green_none = 0
                self._left_count = self._left_none = 0
                self._none_t = 0.0
                self._reason = f"red x{self.red_confirm_frames}"
                return self.state

            # 좌회전(지름길) 진입. 초록불 출발과 같은 방식으로 연속 확정을 요구한다 —
            # 진입하면 트랙 왼쪽 끝으로 붙으므로 오검출 한 번에 들어가면 위험하다.
            is_left = self.enable_shortcut and light_state == LIGHT_LEFT
            self._left_count, self._left_none = self._tick_count(
                is_left, light_state, self._left_count, self._left_none)
            if is_left and self._left_count >= self.shortcut_confirm_frames:
                self.state = State.SHORTCUT
                self._phase = ShortcutPhase.ARM_WAIT
                self._left_gone_count = 0
                self._shortcut_t = 0.0
                self._reason = f"left arrow x{self._left_count}"

        elif self.state is State.STOP_RED:
            # 탈출은 두 가지. 어느 쪽도 한 프레임으로는 안 풀린다.
            go = light_state == LIGHT_GREEN
            self._green_count, self._green_none = self._tick_count(
                go, light_state, self._green_count, self._green_none)
            if go:
                self._none_t = 0.0
                if self._green_count >= self.start_confirm_frames:
                    self.state = State.LANE_DRIVE
                    self._green_count = self._green_none = 0
                    self._reason = f"green x{self.start_confirm_frames}"
            elif light_state == LIGHT_NONE:
                # 신호등이 **아예 안 보인다**. LightVoter 는 본체를
                # miss_tolerance 프레임 연속 놓쳐야 NONE 을 내므로, 이건
                # "시야에 신호등이 없다"는 뜻이지 단발 결측이 아니다.
                # 트랙 한복판에서 오검출로 멈춘 경우가 여기 해당한다.
                # (green 카운트는 위 _tick_count 가 관용 규칙대로 이미 처리했다)
                self._none_t += dt
                if self._none_t >= self.red_release_sec:
                    self.state = State.LANE_DRIVE
                    self._none_t = 0.0
                    self._reason = f"red released (no light {self.red_release_sec:.0f}s)"
            else:
                # RED/YELLOW/LEFT — 신호등은 보이는데 초록이 아니다. 계속 선다.
                self._none_t = 0.0

        elif self.state is State.SHORTCUT:
            # **신호가 사라져도 끝내지 않는다. 전부 시간으로 끊는다.**
            # 꺾기 시작하면 신호등이 곧 시야에서 벗어나(지나쳐 버리거나 각도가
            # 틀어져) LIGHT_LEFT 가 금방 NONE 이 된다. 신호 유지로 끊으면
            # 진입 도중에 차선주행으로 돌아가 분기를 놓친다.
            #
            # 빨간불 정지(STOP_RED)는 여기서 보지 않는다. 꺾는 도중에 멈추면
            # 분기 초입에 비스듬히 선 채로 남는다 — 그게 더 위험하다.
            # 어차피 2.5초면 끝나고, 끝나면 LANE_DRIVE 에서 다시 본다.
            self._shortcut_t += dt
            if self._phase is ShortcutPhase.ARM_WAIT:
                # LEFT 가 사라지기를 기다린다 = 신호등을 지났다는 위치 사건.
                if light_state == LIGHT_LEFT:
                    self._left_gone_count = 0
                else:
                    self._left_gone_count += 1
                if self._left_gone_count >= self.shortcut_lost_frames:
                    self._phase = ShortcutPhase.ARM_GO
                    self._shortcut_t = 0.0
                    self._reason = f"left lost x{self._left_gone_count} -> arm go"
                elif self._shortcut_t >= self.shortcut_arm_timeout_sec:
                    # 안전장치. 신호등 앞에 섰거나 화살표가 계속 보이는
                    # 경우다. 사라진 것으로 치고 진행한다 — 여기 갇히면
                    # 좌회전을 영영 못 한다.
                    self._phase = ShortcutPhase.ARM_GO
                    self._shortcut_t = 0.0
                    self._reason = (f"arm timeout "
                                    f"({self.shortcut_arm_timeout_sec:.0f}s) -> arm go")
            elif self._phase is ShortcutPhase.ARM_GO:
                if self._shortcut_t >= self.shortcut_arm_sec:
                    self._phase = ShortcutPhase.TURN_IN
                    self._shortcut_t = 0.0
                    self._reason = f"arm done ({self.shortcut_arm_sec:.1f}s) -> turn in"
            else:
                if self._shortcut_t >= self.shortcut_turn_sec:
                    self.state = State.LANE_DRIVE
                    self._phase = None
                    self._left_count = 0
                    self._reason = f"turn in done ({self.shortcut_turn_sec:.1f}s)"

        return self.state

    def force(self, state, reason="manual"):
        self.state = state
        self._phase = ShortcutPhase.ARM_WAIT if state is State.SHORTCUT else None
        self._left_gone_count = 0
        self._shortcut_t = 0.0
        self._reason = reason

    @property
    def shortcut_phase(self):
        """SHORTCUT 서브위상. 다른 상태면 None.

        driver_node 가 이걸 보고 TURN_IN 동안 인지를 끊고 고정 조향을 낸다.
        """
        if self.state is not State.SHORTCUT:
            return None
        return self._phase

    @property
    def shortcut_remain(self):
        """현재 SHORTCUT 위상의 남은 시간(초). 다른 상태면 0. 로그·시각화용.

        ARM_WAIT 에서는 **타임아웃까지 남은 시간**이다 — 그 위상은 시간이
        아니라 LEFT 소실로 끝나므로, 세고 있는 유일한 시계가 그것이다.
        """
        if self.state is not State.SHORTCUT:
            return 0.0
        total = {
            ShortcutPhase.ARM_WAIT: self.shortcut_arm_timeout_sec,
            ShortcutPhase.ARM_GO: self.shortcut_arm_sec,
        }.get(self._phase, self.shortcut_turn_sec)
        return max(0.0, total - self._shortcut_t)

    @property
    def should_drive(self):
        """이 상태에서 차를 움직여도 되는가."""
        return self.state in (State.LANE_DRIVE, State.SHORTCUT)
