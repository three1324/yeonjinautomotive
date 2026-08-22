"""횡방향 목표 결정 — "트랙 중앙에서 얼마나 벗어난 곳을 목표로 삼을까".

ROS 의존성 없음.

평소에는 트랙 중앙(target_offset = 0)이 목표다 — 실측 근거가 있다
(perception_analysis.md 338표본: 차로 중앙 추종은 -151px/+206px 로 벌어졌고
트랙 중앙 기준은 ±40px 였다). 방해차량이 앞에 있을 때만 **옆 차선으로
옮겨 유지하다가** 차가 사라지면 되돌아온다.

────────────────────────────────────────────────────────────────────────
회피량 — 왜 고정 픽셀인가 (2026-08-22 사용자 결정)

    지금은 좌 shift_left_px / 우 shift_right_px **고정**이고, 좌우 값이
    다르다. 차량이 좌측으로 쏠리는 성향이 있어 오른쪽으로 피할 때 더
    크게 밀어야 실제로 같은 만큼 옮겨지기 때문이다(실차 관측).

    ※ 이전 방식은 학습된 트랙 반폭의 25%(half_near x 0.25)였다. 원근과
      트랙 폭이 자동 반영되고 목표가 **구조적으로 트랙 안쪽**이라는 장점이
      있었지만, 좌우를 다르게 줄 수 없었다. 고정값으로 오면서 그 보장이
      사라졌다 — **트랙이 좁은 구간에서는 실선을 넘을 수 있다.**
      값을 키울 때는 그 점을 염두에 둘 것.
      (LaneEstimator 는 여전히 행별 반폭을 EMA 로 학습한다(lane.py §5).
       되돌리고 싶으면 그 값을 다시 쓰면 된다.)
────────────────────────────────────────────────────────────────────────

3단계(레이싱 라인) 확장 지점:
    waypoint에서 얻은 목표 횡위치를 여기서 블렌딩한다.
    blend_waypoint() 자리를 미리 만들어뒀고, 다른 파일은 손대지 않아도 된다.
"""

from enum import Enum


class OvertakePhase(Enum):
    IDLE = "IDLE"
    HOLD = "HOLD"       # 옆 차선으로 옮겨 그대로 유지하는 중


class OvertakeBehavior:
    """방해차량 회피 서브행동. **카메라만 쓴다 (라이다 사용 안 함).**

    ────────────────────────────────────────────────────────────────
    2026-08-22 개편 — 램프 기동에서 **차선 변경**으로

    예전에는 5위상(SHIFT->PASS->RETURN->OVERSHOOT->쿨다운)으로 옆으로
    "벌렸다가" 되돌아왔다. 실차에서 그게 두 가지 문제를 만들었다:
      - 벌리는 데 0.8초가 걸려, 그 사이에도 차는 방해차량 쪽으로 계속
        전진한다. 목표가 서서히 옮겨가니 조향도 서서히 붙는다.
      - 되돌아오는 시점을 관측 셋(멀어짐/가장자리/미검출)으로 재느라
        판정이 복잡했고, 그 중 둘은 옆을 스쳐가는 차에서 안 걸렸다.

    지금은 **차선을 바꾸는 동작**으로 본다:
        트리거 충족 -> 피할 쪽 결정 -> **즉시** 그 오프셋으로 목표를 옮긴다
                    -> 그 차선을 유지한다 (HOLD)
                    -> 차가 안 보이는 상태가 lost_hold_sec 지속되면
                       목표를 0 으로 되돌린다 = 노란선(트랙중앙) 추종

    "즉시"인 이유: 목표만 계단으로 옮기는 것이고, 실제 차가 계단으로 움직이는
    것은 아니다. 조향에는 rate limit(180도/s)과 LPF 가 이미 걸려 있어
    거동은 그쪽이 정한다. 즉 램프를 여기서 또 만들 이유가 없었다.

    ────────────────────────────────────────────────────────────────
    왜 라이다를 뺐나 (2026-08-19 실차 결정)

    이전에는 라이다 전방거리로 트리거를 교차확인하고 좌/우 여유로 피할
    쪽을 골랐다. 실차에서 그게 두 가지 문제를 만들었다:

      1. 라이다가 콘·벽·기둥을 방해차량과 구분하지 못해, 차가 없는데도
         전방거리가 임계 아래로 떨어지거나 반대로 차가 있는데 섹터를
         비껴가 트리거가 안 걸렸다.
      2. 옆으로 벌리는 순간 장애물이 섹터에서 빠져 front_dist 가 커지고,
         그러면 탈출 조건이 곧바로 참이 돼 회피가 무의미해졌다.

    방해차량인지 아닌지는 **YOLO만 안다**(AvanteN/ionic5 클래스).
    피할 방향도 카메라로 정한다 — 차가 화면 왼쪽에 있으면 오른쪽으로.

    ────────────────────────────────────────────────────────────────
    회피량은 **좌/우가 다른 고정값**이다 (2026-08-22 사용자 결정)

    예전에는 학습된 트랙 반폭의 25%(half_near x 0.25)를 썼다. 지금은
    shift_left_px / shift_right_px 고정이다. 좌우가 다른 이유는 차량이
    좌측으로 쏠리는 성향이 있어서, 오른쪽으로 피할 때 더 크게 밀어야
    실제로 같은 만큼 옮겨지기 때문이다(사용자 실차 관측).

    ⚠️ 고정 픽셀이므로 **트랙 폭·원근이 반영되지 않는다.** 반폭 기반이었을
       때는 목표가 구조적으로 트랙 안쪽이었지만, 지금은 그 보장이 없다 —
       트랙이 좁은 구간에서는 실선을 넘을 수 있다. 값을 키울 때 특히 주의.
    """

    def __init__(self, trigger_height_px, shift_left_px, shift_right_px,
                 lost_hold_sec=2.0, cooldown_sec=1.0):
        # 차량 bbox 높이 임계 (클수록 가까워야 발동). 거리 대용값이다 —
        # 하단 y 는 카메라 피치·노면 기울기에 흔들리지만 높이는 거의 순수하게
        # 거리의 함수다 (cone_zone 의 진입 크기 조건과 같은 근거).
        self.trigger_height_px = trigger_height_px
        self.shift_left_px = shift_left_px      # 왼쪽으로 피할 때의 오프셋 크기
        self.shift_right_px = shift_right_px    # 오른쪽으로 피할 때
        # 차량이 **안 보이는 상태가 이만큼 이어져야** 트랙 중앙으로 되돌아간다.
        # 한 프레임만 놓쳐도 복귀하면, 옆으로 옮긴 순간 방해차량이 화면 밖으로
        # 잠깐 나가거나 YOLO 가 한 프레임 놓치는 것만으로 기동이 끝난다.
        # 그러면 아직 차 옆을 지나는 중인데 중앙으로 돌아와 부딪힌다.
        self.lost_hold_sec = lost_hold_sec
        self.cooldown_sec = cooldown_sec        # 복귀 후 재발동 금지 시간

        self.phase = OvertakePhase.IDLE
        self._dir = 0        # +1: 오른쪽으로 피함, -1: 왼쪽으로 피함
        # 기동을 시작할 때 **방해차량이 어느 쪽에 있다고 봤는지**.
        # -1 = 화면 왼쪽, +1 = 오른쪽, 0 = 기동 중 아님. 항상 _dir 의 반대다.
        # _dir 로부터 유도할 수는 있지만, 시각화에서 "피하는 방향"과 "차가
        # 있던 방향"을 나란히 보여줘야 부호를 오해하지 않는다.
        self._car_side = 0
        self._car_cx = 0.0   # 판단 근거가 된 x중심(픽셀). 진단·시각화용
        self._amount = 0.0   # 이번 기동의 회피량(픽셀)
        self._cooldown = 0.0
        self._lost_t = 0.0   # 차량을 연속으로 못 본 시간
        self.last_reason = ""   # 진단용: 왜 시작/종료했는지

    def reset(self):
        self.phase = OvertakePhase.IDLE
        self._dir = 0
        self._car_side = 0
        self._car_cx = 0.0
        self._amount = 0.0
        self._lost_t = 0.0

    @property
    def active(self):
        return self.phase is not OvertakePhase.IDLE

    @property
    def amount(self):
        """이번 기동의 회피량(픽셀). 진단·시각화용."""
        return self._amount

    @property
    def direction(self):
        """+1 오른쪽 / -1 왼쪽 / 0 없음. 진단·시각화용."""
        return self._dir

    @property
    def car_side(self):
        """기동 시작 시 방해차량이 있다고 판단한 쪽. -1 왼쪽 / +1 오른쪽 / 0 없음.

        진단·시각화 전용 — 제어에는 쓰지 않는다(_dir 이 이미 그 결과다).
        """
        return self._car_side

    @property
    def car_cx(self):
        """그 판단의 근거가 된 차량 x중심(픽셀). 시작 시점에 고정된다."""
        return self._car_cx

    def _pick_side(self, car_cx, image_width):
        """차량 반대쪽으로 피한다. 카메라의 차량 x중심만 본다."""
        car_on_left = car_cx < image_width / 2.0
        return 1 if car_on_left else -1         # 차가 왼쪽이면 오른쪽으로

    def _offset(self):
        """이번 기동의 목표 오프셋(픽셀). **부호가 _dir 과 반대다.**

        ────────────────────────────────────────────────────────────
        왜 뒤집나 (2026-08-21 버그 수정)

        offset 의 정의가 "트랙중앙 - 화면중심"(lane.py) 이고, 제어기는
        err = offset_near - target_offset 을 0 으로 몰아간다. 즉 정상상태에서
        offset_near 가 target_offset 이 된다.

            target_offset > 0  ->  트랙중앙이 화면 오른쪽  ->  차는 트랙 **왼쪽**
            target_offset < 0  ->                          차는 트랙 **오른쪽**

        그런데 _dir 은 +1 이 "오른쪽으로 피한다"는 뜻이다. 그대로 내보내면
        오른쪽으로 피하겠다면서 목표는 왼쪽이 된다 — 실차에서 로그에는
        start right 가 찍히는데 차는 방해차량 쪽으로 붙었다.

        _dir 자체는 뒤집지 않는다. +1=오른쪽 이라는 표기가 로그·시각화
        (ot_dir, viz 의 AVOID RIGHT/LEFT)에 이미 쓰이고 있어서, 여기서만
        좌표 규약으로 변환하는 편이 헷갈리지 않는다.
        ────────────────────────────────────────────────────────────
        """
        return -self._dir * self._amount

    def _finish(self, why):
        """기동 종료 + 쿨다운 시작. 목표는 즉시 0(트랙중앙 = 노란선)이 된다."""
        self.reset()
        # 복귀 직후 같은 차가 아직 앞에 있으면 즉시 재발동해 지그재그가 된다.
        self._cooldown = self.cooldown_sec
        self.last_reason = f"{why} -> lane center (cooldown {self.cooldown_sec:.1f}s)"

    def update(self, dt, car_present, car_cx, car_h, image_width):
        """회피로 인한 목표 오프셋 보정량(픽셀)을 반환한다. 평소 0.

        인자에 라이다 값이 없다 — 의도적이다. 클래스 주석 참고.
        """
        if self.phase is OvertakePhase.IDLE:
            if self._cooldown > 0.0:
                self._cooldown = max(0.0, self._cooldown - dt)
                return 0.0

            # (1) 거리 조건: bbox 높이가 임계 이상인가 (= 충분히 가까운가)
            if car_present and car_h >= self.trigger_height_px:
                # (2) 피할 쪽을 정하고 (3) **즉시** 그 오프셋으로 목표를 옮긴다.
                d = self._pick_side(car_cx, image_width)
                self._dir = d
                self._car_side = -d      # 피하는 쪽의 반대 = 차가 있던 쪽
                self._car_cx = float(car_cx)
                self._amount = (self.shift_right_px if d > 0
                                else self.shift_left_px)
                self.phase = OvertakePhase.HOLD
                self._lost_t = 0.0
                side = "left" if d > 0 else "right"
                move = "right" if d > 0 else "left"
                self.last_reason = (
                    f"start: car {side}(cx{car_cx:.0f} h{car_h:.0f}) -> "
                    f"move {move} {self._amount:.0f}px")
                return self._offset()
            return 0.0

        # HOLD — 옮긴 차선을 그대로 유지한다.
        # 끝내는 조건은 **하나뿐이다**: 차가 안 보이는 상태가 lost_hold_sec 지속.
        # 예전의 "bbox 가 작아짐 / 화면 가장자리로 밀려남 / 시간 상한" 셋은
        # 없앴다 (2026-08-22 사용자 결정). 옆을 스쳐 지나가는 차는 가까워지면서
        # 화면 밖으로 나가므로 앞의 둘은 원래도 잘 안 걸렸다.
        if car_present:
            self._lost_t = 0.0
        else:
            self._lost_t += dt
            if self._lost_t >= self.lost_hold_sec:
                self._finish(f"car gone({self._lost_t:.1f}s)")
                return 0.0
        return self._offset()


class LateralPlanner:
    """횡방향 목표 오프셋을 최종 결정한다."""

    def __init__(self, overtake: OvertakeBehavior, enable_overtake=True):
        self.overtake = overtake
        self.enable_overtake = enable_overtake

    def blend_waypoint(self, target_offset, waypoint_offset, weight):
        """3단계 확장 지점 — 레이싱 라인 반영.

        아직 쓰지 않는다. waypoint 기반 목표 횡위치가 생기면 여기서 섞는다.
        weight=0 이면 완전히 차선 기준(현재 동작).
        """
        if weight <= 0.0 or waypoint_offset is None:
            return target_offset
        return (1.0 - weight) * target_offset + weight * waypoint_offset

    def update(self, dt, obs, image_width, allow_overtake=True):
        """obs: driver_node 가 모아 넘기는 관측 묶음. 목표 오프셋(픽셀) 반환.

        allow_overtake: False 면 이번 tick 은 회피를 하지 않는다. 좌회전 직후
            차단 구간에서 driver_node 가 내린다 (그 파라미터 주석 참고).
            **차단 중에는 overtake 를 reset 한다** — 좌회전 TURN_IN 동안 이
            경로가 아예 안 불렸으므로, 그 전에 기동 중이던 상태가 그대로
            남아 있을 수 있다. 그걸 들고 복귀하면 엉뚱하게 벌린 채 달린다.

        좌회전(지름길)은 여기서 다루지 않는다. 진입 전(ARM)은 **평소 주행과
        완전히 같고**, 꺾는 동안(TURN_IN)은 driver_node 가 이 경로를 아예
        거치지 않고 고정 조향을 낸다.
        """
        target = 0.0   # 기본은 트랙 중앙

        if not allow_overtake:
            self.overtake.reset()
        elif self.enable_overtake:
            # 카메라 관측만 넘긴다. obs 에는 라이다 값(front_dist 등)도 들어 있지만
            # 회피는 그걸 쓰지 않는다 (OvertakeBehavior 주석 참고).
            target += self.overtake.update(
                dt,
                obs.car_present, obs.car_cx, obs.car_h,
                image_width,
            )

        return target
