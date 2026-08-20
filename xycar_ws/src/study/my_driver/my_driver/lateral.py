"""횡방향 목표 결정 — "트랙 중앙에서 얼마나 벗어난 곳을 목표로 삼을까".

ROS 의존성 없음.

평소에는 트랙 중앙(target_offset = 0)이 목표다 — 실측 근거가 있다
(perception_analysis.md 338표본: 차로 중앙 추종은 -151px/+206px 로 벌어졌고
트랙 중앙 기준은 ±40px 였다). 방해차량이 앞에 있을 때만 일시적으로
**트랙의 반쪽 중앙**으로 옮겼다가 되돌린다.

────────────────────────────────────────────────────────────────────────
회피 목표를 "반폭/2" 로 잡는 이유

    트랙 반쪽의 중앙 = 트랙중앙 ± 반폭/2
      왼쪽 반 중앙   = (왼쪽 흰실선 + 노란선) / 2
      오른쪽 반 중앙 = (노란선 + 오른쪽 흰실선) / 2

    이전에는 고정 픽셀(shift_px=120)로 밀었는데, 픽셀 거리는 원근과 트랙 폭에
    따라 의미가 달라진다. 같은 120px 이 가까운 행에서는 조금, 먼 행에서는 트랙
    밖까지를 뜻할 수 있다. 반면 LaneEstimator 는 좌우 흰선을 동시에 볼 때마다
    **행별 반폭을 EMA 로 이미 학습하고 있다**(lane.py §5). 그 값을 쓰면 원근·
    트랙폭이 자동으로 반영되고, 목표가 항상 트랙 안쪽이라 **실선을 넘는 상황이
    구조적으로 생기지 않는다.**

    반폭을 아직 학습하지 못했으면(half=0) 고정 shift_px 로 폴백한다.
────────────────────────────────────────────────────────────────────────

3단계(레이싱 라인) 확장 지점:
    waypoint에서 얻은 목표 횡위치를 여기서 블렌딩한다.
    blend_waypoint() 자리를 미리 만들어뒀고, 다른 파일은 손대지 않아도 된다.
"""

from enum import Enum


class OvertakePhase(Enum):
    IDLE = "IDLE"
    SHIFT = "SHIFT"     # 옆으로 벌리는 중
    PASS = "PASS"       # 벌린 상태로 통과 중
    RETURN = "RETURN"   # 트랙 중앙으로 복귀 중


class OvertakeBehavior:
    """방해차량 회피 서브행동. **카메라만 쓴다 (라이다 사용 안 함).**

    트리거 조건 (모두 만족해야 시작):
      - 카메라가 차량을 봤다 (라바콘이 아니라 차량이라는 건 YOLO만 안다)
      - 그 차량이 충분히 가깝다 (bbox 하단 y가 임계 이상)
      - 쿨다운 중이 아니다

    ────────────────────────────────────────────────────────────────
    왜 라이다를 뺐나 (2026-08-19 실차 결정)

    이전에는 라이다 전방거리로 트리거를 교차확인하고 좌/우 여유로 피할
    쪽을 골랐다. 실차에서 그게 두 가지 문제를 만들었다:

      1. 라이다가 콘·벽·기둥을 방해차량과 구분하지 못해, 차가 없는데도
         전방거리가 임계 아래로 떨어지거나 반대로 차가 있는데 섹터를
         비껴가 트리거가 안 걸렸다.
      2. 옆으로 벌리는 순간 장애물이 섹터에서 빠져 front_dist 가 커지고,
         그러면 PASS 탈출 조건이 곧바로 참이 돼 회피가 무의미해졌다.

    방해차량인지 아닌지는 **YOLO만 안다**(AvanteN/ionic5 클래스). 라이다는
    그 판단에 기여하는 정보가 없으면서 오작동 경로만 늘렸다. 그래서 회피는
    카메라 단독으로 하고, 라이다는 라바콘 구간 전용으로 분리했다
    (cone_zone.py / driver_node.py 참고).

    피할 방향도 카메라로 정한다 — 차가 화면 왼쪽에 있으면 오른쪽으로.
    회피량이 트랙 반폭의 절반이라 목표가 **구조적으로 트랙 안쪽**이므로,
    라이다 여유 확인 없이도 실선을 넘지 않는다.
    ────────────────────────────────────────────────────────────────

    회피량: 트랙 반폭의 절반 = 트랙 반쪽의 중앙. 반폭 미학습이면 shift_px 폴백.
    시작 시점에 한 번 계산해 **고정**한다 — 기동 도중 반폭 추정이 흔들려도
    목표가 따라 흔들리면 조향이 진동하기 때문이다.
    """

    def __init__(self, shift_px, trigger_bottom_y,
                 shift_sec, pass_sec, return_sec,
                 cooldown_sec=1.0, pass_exit_ratio=0.85,
                 pass_exit_cx_ratio=0.85, lost_hold_sec=1.0):
        self.shift_px = shift_px                    # 반폭 미학습 시 폴백값 (픽셀)
        self.trigger_bottom_y = trigger_bottom_y    # 차량 bbox 하단 y 임계 (클수록 가까움)
        self.shift_sec = shift_sec
        self.pass_sec = pass_sec                    # PASS 유지의 **상한** (안전장치)
        self.return_sec = return_sec
        self.cooldown_sec = cooldown_sec            # 복귀 후 재발동 금지 시간
        # PASS 탈출용 히스테리시스. bottom_y 가 trigger 의 이 배율 아래로 내려가면
        # "지나쳤다"고 본다. 1.0 으로 두면 임계 근처에서 진입/이탈이 떨린다.
        self.pass_exit_ratio = pass_exit_ratio
        # 차량이 화면 가장자리로 밀려나면 옆으로 지나친 것이다.
        # |cx - 중심| / (폭/2) 가 이 값을 넘으면 통과로 본다.
        # [실측 2026-08-19] 테스트영상 f2741~2811 구간에서 cx 가 440->612 로
        # 이동하는 동안 bottom_y 는 계속 커져(302->439) "멀어짐" 조건이 걸리지
        # 않았고, pass_sec 상한(1.5s)으로만 복귀했다. 옆을 스쳐 지나가는 차는
        # 가까워지면서 화면 밖으로 나가므로 bottom_y 만으로는 못 잡는다.
        self.pass_exit_cx_ratio = pass_exit_cx_ratio
        # 차량이 **안 보이는 상태가 이만큼 이어져야** 복귀한다 (2026-08-21).
        # 한 프레임만 놓쳐도 복귀하면, 옆으로 벌린 순간 방해차량이 화면 밖으로
        # 잠깐 나가거나 YOLO 가 한 프레임 놓치는 것만으로 기동이 중단된다.
        # 그러면 아직 차 옆을 지나는 중인데 트랙 중앙으로 돌아와 부딪힌다.
        self.lost_hold_sec = lost_hold_sec

        self.phase = OvertakePhase.IDLE
        self._t = 0.0
        self._dir = 0        # +1: 오른쪽으로 피함, -1: 왼쪽으로 피함
        self._amount = 0.0   # 이번 기동의 회피량(픽셀). 시작 시 고정된다
        self._cooldown = 0.0
        self._lost_t = 0.0      # 차량을 연속으로 못 본 시간 (PASS 중에만 의미)
        self.last_reason = ""   # 진단용: 왜 시작/종료했는지

    def reset(self):
        self.phase = OvertakePhase.IDLE
        self._t = 0.0
        self._dir = 0
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

    def _pick_side(self, car_cx, image_width):
        """차량 반대쪽으로 피한다. 카메라의 차량 x중심만 본다.

        라이다 좌/우 여유 확인을 하지 않는 이유는 클래스 주석 참고 —
        회피량이 트랙 반폭의 절반이라 목표가 항상 트랙 안쪽이다.
        """
        car_on_left = car_cx < image_width / 2.0
        return 1 if car_on_left else -1         # 차가 왼쪽이면 오른쪽으로

    def _offset(self, ratio=1.0):
        """이번 기동의 목표 오프셋(픽셀). **부호가 _dir 과 반대다.**

        ────────────────────────────────────────────────────────────
        왜 뒤집나 (2026-08-21 버그 수정)

        offset 의 정의가 `트랙중앙 - 화면중심`(lane.py) 이고, 제어기는
        `err = offset_near - target_offset` 을 0 으로 몰아간다. 즉 정상상태에서
        offset_near 가 target_offset 이 된다.

            target_offset > 0  ->  트랙중앙이 화면 오른쪽  ->  차는 트랙 **왼쪽**
            target_offset < 0  ->                          차는 트랙 **오른쪽**

        그런데 _dir 은 +1 이 "오른쪽으로 피한다"는 뜻이다. 그대로 내보내면
        오른쪽으로 피하겠다면서 목표는 왼쪽이 된다 — 실차에서 로그에는
        `start right` 가 찍히는데 차는 방해차량 쪽으로 붙었다.

        _dir 자체는 뒤집지 않는다. +1=오른쪽 이라는 표기가 로그·시각화
        (`ot_dir`, viz 의 AVOID RIGHT/LEFT)에 이미 쓰이고 있어서, 여기서만
        좌표 규약으로 변환하는 편이 헷갈리지 않는다.
        ────────────────────────────────────────────────────────────
        """
        return -self._dir * self._amount * ratio

    def _shift_amount(self, half_near):
        """이번 기동에서 옆으로 옮길 양(픽셀).

        트랙 반쪽의 중앙 = 트랙중앙에서 반폭/2 만큼. 반폭을 아직 학습하지
        못했으면(0) 고정값으로 폴백한다 — 근거는 약하지만 아예 못 피하는 것보다는
        낫다는 판단(사용자 결정). 대신 폴백인지 아닌지를 last_reason 에 남긴다.
        """
        if half_near > 0.0:
            return half_near / 2.0
        return self.shift_px

    def update(self, dt, car_present, car_cx, car_bottom_y,
               image_width, half_near=0.0):
        """회피로 인한 목표 오프셋 보정량(픽셀)을 반환한다. 평소 0.

        인자에 라이다 값이 없다 — 의도적이다. 클래스 주석 참고.
        """
        if self.phase is OvertakePhase.IDLE:
            if self._cooldown > 0.0:
                self._cooldown = max(0.0, self._cooldown - dt)
                return 0.0

            if car_present and car_bottom_y >= self.trigger_bottom_y:
                d = self._pick_side(car_cx, image_width)
                self._dir = d
                self._amount = self._shift_amount(half_near)
                self.phase = OvertakePhase.SHIFT
                self._t = 0.0
                src = "half/2" if half_near > 0.0 else "shift_px(fallback)"
                self.last_reason = (
                    f"start {'right' if d > 0 else 'left'} "
                    f"{self._amount:.0f}px[{src}]")
            return 0.0

        self._t += dt

        if self.phase is OvertakePhase.SHIFT:
            ratio = min(self._t / max(self.shift_sec, 1e-3), 1.0)
            if ratio >= 1.0:
                self.phase = OvertakePhase.PASS
                self._t = 0.0
                self._lost_t = 0.0
            return self._offset(ratio)

        if self.phase is OvertakePhase.PASS:
            # 시간이 아니라 **관측**으로 끝낸다. 통과에 걸리는 시간은 속도에 따라
            # 달라지므로 고정 시간은 빠르면 너무 일찍, 느리면 너무 늦게 복귀한다.
            # "안 보임"은 **유지시간을 채워야** 복귀 사유가 된다. 한 프레임
            # 결측으로 기동을 끊으면 차 옆을 지나는 중에 중앙으로 돌아온다.
            if car_present:
                self._lost_t = 0.0
            else:
                self._lost_t += dt

            reason = None
            if not car_present and self._lost_t >= self.lost_hold_sec:
                reason = f"car gone({self._lost_t:.1f}s)"
            elif not car_present:
                # 아직 유지시간 중 — 벌린 상태를 그대로 유지한다.
                return self._offset()
            elif car_bottom_y < self.trigger_bottom_y * self.pass_exit_ratio:
                reason = f"car receding(y{car_bottom_y:.0f})"
            elif (image_width > 0
                  and abs(car_cx - image_width / 2.0) / (image_width / 2.0)
                  > self.pass_exit_cx_ratio):
                reason = f"car at edge(cx{car_cx:.0f})"
            elif self._t >= self.pass_sec:
                # 위 셋 다 아닌데 시간이 다 됐다 = 관측이 계속 "앞에 있다"고 말하는
                # 상황이다. 무한정 벌린 채로 달릴 수는 없으니 상한으로 끊는다.
                reason = f"pass timeout({self.pass_sec:.1f}s)"

            if reason is not None:
                self.phase = OvertakePhase.RETURN
                self._t = 0.0
                self.last_reason = f"return: {reason}"
            return self._offset()

        if self.phase is OvertakePhase.RETURN:
            ratio = min(self._t / max(self.return_sec, 1e-3), 1.0)
            if ratio >= 1.0:
                self.reset()
                # 복귀 직후 같은 차가 아직 앞에 있으면 즉시 재발동해 지그재그가 된다.
                # 쿨다운 동안은 트리거를 막는다.
                self._cooldown = self.cooldown_sec
                self.last_reason = f"done (cooldown {self.cooldown_sec:.1f}s)"
                return 0.0
            return self._offset(1.0 - ratio)

        return 0.0


class LateralPlanner:
    """횡방향 목표 오프셋을 최종 결정한다."""

    def __init__(self, overtake: OvertakeBehavior, enable_overtake=True,
                 shortcut_half_car_px=45.0):
        self.overtake = overtake
        self.enable_overtake = enable_overtake
        # 좌회전(지름길) 구간에서 좌측 실선으로부터 안쪽으로 띄울 양(픽셀).
        # **반차폭**이다 — 차량 왼쪽면이 실선에 닿는 위치를 목표로 삼는다.
        self.shortcut_half_car_px = shortcut_half_car_px

    def shortcut_target(self, half_near):
        """좌회전 구간 목표 오프셋 — 가장 좌측 흰 실선을 따라간다.

        ────────────────────────────────────────────────────────────
        유도

        offset 규약은 `트랙중앙 - 차량중심`(lane.py) 이고, 제어기는 정상상태에서
        offset_near 를 target_offset 으로 만든다. 즉

            target = 트랙중앙 - 차량중심   ->   차량중심 = 트랙중앙 - target

        좌측 실선은 트랙중앙에서 반폭(half_near)만큼 왼쪽이다:

            좌측실선 = 트랙중앙 - half_near

        목표는 **차량 왼쪽면이 그 실선에 붙는 것**이므로 차량중심은 실선에서
        반차폭만큼 오른쪽이다:

            차량중심 = 좌측실선 + 반차폭 = 트랙중앙 - half_near + 반차폭

        두 식을 맞추면:

            target = half_near - 반차폭

        검산: 반차폭 0 이면 target = half_near -> 차량중심이 실선 위. 맞다.
        ────────────────────────────────────────────────────────────

        half_near 가 0 이면(좌우 흰선을 아직 동시에 본 적이 없어 반폭 미학습)
        실선 위치를 모른다. 그때는 **트랙 중앙을 유지**한다 — 근거 없이 왼쪽으로
        밀면 코스를 이탈한다. 로그에 남으니 자주 뜨면 인지 쪽을 봐야 한다.
        """
        if half_near <= 0.0:
            return 0.0
        return half_near - self.shortcut_half_car_px

    def blend_waypoint(self, target_offset, waypoint_offset, weight):
        """3단계 확장 지점 — 레이싱 라인 반영.

        아직 쓰지 않는다. waypoint 기반 목표 횡위치가 생기면 여기서 섞는다.
        weight=0 이면 완전히 차선 기준(현재 동작).
        """
        if weight <= 0.0 or waypoint_offset is None:
            return target_offset
        return (1.0 - weight) * target_offset + weight * waypoint_offset

    def update(self, dt, obs, image_width, shortcut=False):
        """obs: driver_node 가 모아 넘기는 관측 묶음. 목표 오프셋(픽셀) 반환.

        shortcut: 좌회전(지름길) 구간인가. True 면 트랙 중앙 대신 좌측 실선
                  안쪽을 기준으로 삼는다.
        """
        half_near = getattr(obs, "half_near", 0.0)

        if shortcut:
            # 좌회전 구간에서는 회피를 하지 않는다. 이미 트랙 왼쪽 끝에 붙어
            # 달리는 중이라 더 옆으로 벌릴 여유가 없고, 12초 안에 구간을
            # 빠져나가는 것이 우선이다.
            self.overtake.reset()
            return self.shortcut_target(half_near)

        target = 0.0   # 기본은 트랙 중앙

        if self.enable_overtake:
            # 카메라 관측만 넘긴다. obs 에는 라이다 값(front_dist 등)도 들어 있지만
            # 회피는 그걸 쓰지 않는다 (OvertakeBehavior 주석 참고).
            target += self.overtake.update(
                dt,
                obs.car_present, obs.car_cx, obs.car_bottom_y,
                image_width,
                half_near=half_near,
            )

        return target
