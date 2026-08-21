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
    STOP_RED = "STOP_RED"       # 주행 중 빨간불 -> 정지
    FINISH = "FINISH"           # 정지


# light_vote 의 상수와 맞춰야 한다
LIGHT_NONE, LIGHT_RED, LIGHT_YELLOW, LIGHT_GREEN, LIGHT_LEFT = 0, 1, 2, 3, 4


class DriveFSM:
    """상태 전이만 담당한다.

    start_confirm_frames:
        초록불이 몇 프레임 연속 유지돼야 출발할지. LightVoter 가 이미 투표로
        확정한 값을 주지만, 출발은 되돌릴 수 없는 동작이라 한 겹 더 확인한다.
    """

    def __init__(self, start_confirm_frames=3, enable_shortcut=False,
                 auto_start=False, shortcut_sec=12.0,
                 shortcut_confirm_frames=3,
                 enable_red_stop=True, red_confirm_frames=3,
                 red_release_sec=3.0, none_tolerance=1):
        self.start_confirm_frames = start_confirm_frames
        self.enable_shortcut = enable_shortcut
        # 신호등 없이 바로 주행 (실내 튜닝용). 실전에서는 반드시 False.
        self.auto_start = auto_start
        # 좌회전(지름길) 구간을 몇 초 유지할지. **시간으로 끊는다** — 아래 참고.
        self.shortcut_sec = shortcut_sec
        # 좌회전 화살표가 몇 프레임 연속 확정돼야 진입할지. 출발과 같은 이유로
        # 한 겹 더 확인한다 (진입하면 트랙 왼쪽 끝으로 붙으므로 되돌리기 비싸다).
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
        self._none_t = 0.0
        self._reason = "reset"

    def update(self, light_state, lane_valid, dt=0.0):
        """프레임당 1회. 새 상태를 반환한다.

        dt: 직전 호출로부터의 경과 시간(초). SHORTCUT 유지시간을 재는 데만 쓴다.

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
            # **신호가 사라져도 끝내지 않는다. 시간으로 끊는다.**
            # 좌회전 구간에 들어가면 신호등이 곧 시야에서 벗어나(지나쳐 버리거나
            # 각도가 틀어져) LIGHT_LEFT 가 금방 NONE 이 된다. 신호 유지로 끊으면
            # 구간 초입에서 바로 차선주행으로 돌아가 지름길을 못 탄다.
            self._shortcut_t += dt
            if self._shortcut_t >= self.shortcut_sec:
                self.state = State.LANE_DRIVE
                self._left_count = 0
                self._reason = f"shortcut done ({self.shortcut_sec:.0f}s)"

        return self.state

    def force(self, state, reason="manual"):
        self.state = state
        self._reason = reason

    @property
    def shortcut_remain(self):
        """SHORTCUT 남은 시간(초). 다른 상태면 0. 로그·시각화용."""
        if self.state is not State.SHORTCUT:
            return 0.0
        return max(0.0, self.shortcut_sec - self._shortcut_t)

    @property
    def should_drive(self):
        """이 상태에서 차를 움직여도 되는가."""
        return self.state in (State.LANE_DRIVE, State.SHORTCUT)
