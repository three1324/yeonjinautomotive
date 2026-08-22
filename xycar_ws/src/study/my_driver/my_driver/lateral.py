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
    OVERSHOOT = "OVERSHOOT"  # 중앙을 지나 반대쪽으로 잠시 더


class OvertakeBehavior:
    """방해차량 회피 서브행동. **카메라만 쓴다 (라이다 사용 안 함).**

    트리거 조건 (모두 만족해야 시작):
      - 카메라가 차량을 봤다 (라바콘이 아니라 차량이라는 건 YOLO만 안다)
      - 그 차량이 충분히 가깝다 (**bbox 높이**가 임계 이상)
      - 쿨다운 중이 아니다

    ★ [2026-08-22] 거리 판단을 **bbox 하단 y -> bbox 높이**로 바꿨다.
      "보이자마자 피한다"가 문제였다. 하단 y 는 거리뿐 아니라 카메라 피치와
      노면 기울기에 흔들리고, 차가 화면 아래로 잘리면 오히려 **작아진다**.
      높이는 거의 순수하게 거리의 함수다 — 라바콘 구간 진입 트리거가
      cone_max_h 를 쓰는 것과 정확히 같은 이유다(cone_zone.py 참고).
      순서도 명확해졌다: **높이 임계 충족 -> 피할 쪽 결정 -> 기동 시작.**

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

    def __init__(self, shift_px, trigger_height_px,
                 shift_sec, pass_sec, return_sec,
                 cooldown_sec=1.0, pass_exit_ratio=0.85,
                 pass_exit_cx_ratio=0.85, lost_hold_sec=1.0, shift_scale=0.5, overshoot_sec=0.5,
                 overshoot_scale=1.0):
        self.shift_px = shift_px                    # 반폭 미학습 시 폴백값 (픽셀)
        self.trigger_height_px = trigger_height_px  # 차량 bbox 높이 임계 (클수록 가까움)
        # 추월 후 중앙을 지나 반대쪽으로 더 머물 시간과 그 크기 배율.
        # 0 으로 두면 예전처럼 중앙에서 바로 끝난다.
        self.overshoot_sec = overshoot_sec
        self.overshoot_scale = overshoot_scale
        # 회피량 배율. 반폭/2 로는 과하게 벌어져 30% 줄였다 (_shift_amount 참고).
        self.shift_scale = shift_scale
        self.shift_sec = shift_sec
        self.pass_sec = pass_sec                    # PASS 유지의 **상한** (안전장치)
        self.return_sec = return_sec
        self.cooldown_sec = cooldown_sec            # 복귀 후 재발동 금지 시간
        # PASS 탈출용 히스테리시스. **bbox 높이**가 trigger 의 이 배율 아래로 내려가면
        # "지나쳤다"고 본다. 1.0 으로 두면 임계 근처에서 진입/이탈이 떨린다.
        self.pass_exit_ratio = pass_exit_ratio
        # 차량이 화면 가장자리로 밀려나면 옆으로 지나친 것이다.
        # |cx - 중심| / (폭/2) 가 이 값을 넘으면 통과로 본다.
        # [실측 2026-08-19] 테스트영상 f2741~2811 구간에서 cx 가 440->612 로
        # 이동하는 동안 bottom_y 는 계속 커져(302->439) "멀어짐" 조건이 걸리지
        # 않았고, pass_sec 상한(1.5s)으로만 복귀했다. 옆을 스쳐 지나가는 차는
        # **가까워지면서** 화면 밖으로 나가므로 거리 조건 하나로는 못 잡는다.
        # 이 관찰은 트리거를 bbox 높이로 바꾼 지금도 그대로다 — 스쳐 지나가는
        # 동안 높이는 오히려 커지므로 receding 조건이 안 걸린다. 그래서 이
        # cx 조건이 여전히 필요하다.
        self.pass_exit_cx_ratio = pass_exit_cx_ratio
        # 차량이 **안 보이는 상태가 이만큼 이어져야** 복귀한다 (2026-08-21).
        # 한 프레임만 놓쳐도 복귀하면, 옆으로 벌린 순간 방해차량이 화면 밖으로
        # 잠깐 나가거나 YOLO 가 한 프레임 놓치는 것만으로 기동이 중단된다.
        # 그러면 아직 차 옆을 지나는 중인데 트랙 중앙으로 돌아와 부딪힌다.
        self.lost_hold_sec = lost_hold_sec

        self.phase = OvertakePhase.IDLE
        self._t = 0.0
        self._dir = 0        # +1: 오른쪽으로 피함, -1: 왼쪽으로 피함
        # 기동을 시작할 때 **방해차량이 어느 쪽에 있다고 봤는지**.
        # -1 = 화면 왼쪽, +1 = 오른쪽, 0 = 기동 중 아님. 항상 _dir 의 반대다.
        # _dir 로부터 유도할 수는 있지만, 시각화에서 "피하는 방향"과 "차가
        # 있던 방향"을 나란히 보여줘야 부호를 오해하지 않는다 — 실제로
        # 8/21 에 그 혼동으로 차가 방해차량 쪽으로 붙는 버그가 있었다.
        self._car_side = 0
        self._car_cx = 0.0   # 판단 근거가 된 x중심(픽셀). 진단·시각화용
        self._amount = 0.0   # 이번 기동의 회피량(픽셀). 시작 시 고정된다
        self._cooldown = 0.0
        self._lost_t = 0.0      # 차량을 연속으로 못 본 시간 (PASS 중에만 의미)
        self.last_reason = ""   # 진단용: 왜 시작/종료했는지

    def reset(self):
        self.phase = OvertakePhase.IDLE
        self._t = 0.0
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

    def _overshoot(self):
        """복귀 방향으로 중앙을 지나친 목표 오프셋(픽셀).

        부호가 _offset() 과 **반대**다. 회피했던 쪽의 반대쪽이므로
        방해차량이 서 있던 차선 쪽이다 — 이미 지나친 뒤에만 쓴다.
        크기는 회피량 x overshoot_scale.
        """
        return -self._dir * self._amount * self.overshoot_scale

    def _finish(self):
        """기동 종료 + 쿨다운 시작. RETURN/OVERSHOOT 둘 다 여기로 끝난다."""
        self.reset()
        # 복귀 직후 같은 차가 아직 앞에 있으면 즉시 재발동해 지그재그가 된다.
        # 쿨다운 동안은 트리거를 막는다.
        self._cooldown = self.cooldown_sec
        self.last_reason = f"done (cooldown {self.cooldown_sec:.1f}s)"

    def _shift_amount(self, half_near):
        """이번 기동에서 옆으로 옮길 양(픽셀).

        트랙 반쪽의 중앙 = 트랙중앙에서 반폭/2 만큼. 반폭을 아직 학습하지
        못했으면(0) 고정값으로 폴백한다 — 근거는 약하지만 아예 못 피하는 것보다는
        낫다는 판단(사용자 결정). 대신 폴백인지 아닌지를 last_reason 에 남긴다.

        마지막에 shift_scale 을 곱한다. 반폭/2 는 실차에서 너무 크게 벌어졌다
        — 8/21 에 0.7(30% 감소), 8/22 에 0.5(절반)까지 줄였다. **폴백 경로에도
        같이 곱해야** 두 경로의 크기 감각이 어긋나지 않는다.
        """
        base = half_near / 2.0 if half_near > 0.0 else self.shift_px
        return base * self.shift_scale

    def update(self, dt, car_present, car_cx, car_h,
               image_width, half_near=0.0):
        """회피로 인한 목표 오프셋 보정량(픽셀)을 반환한다. 평소 0.

        인자에 라이다 값이 없다 — 의도적이다. 클래스 주석 참고.
        """
        if self.phase is OvertakePhase.IDLE:
            if self._cooldown > 0.0:
                self._cooldown = max(0.0, self._cooldown - dt)
                return 0.0

            # ① 거리 조건: bbox 높이가 임계 이상인가 (= 충분히 가까운가)
            if car_present and car_h >= self.trigger_height_px:
                # ② 그 시점에 피할 쪽을 정하고 ③ 기동을 시작한다.
                d = self._pick_side(car_cx, image_width)
                self._dir = d
                self._car_side = -d      # 피하는 쪽의 반대 = 차가 있던 쪽
                self._car_cx = float(car_cx)
                self._amount = self._shift_amount(half_near)
                self.phase = OvertakePhase.SHIFT
                self._t = 0.0
                src = "half/2" if half_near > 0.0 else "shift_px(fallback)"
                self.last_reason = (
                    f"start: car {'left' if d > 0 else 'right'}"
                    f"(cx{car_cx:.0f} h{car_h:.0f}) -> "
                    f"avoid {'right' if d > 0 else 'left'} "
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
            elif car_h < self.trigger_height_px * self.pass_exit_ratio:
                reason = f"car receding(h{car_h:.0f})"
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
                if self.overshoot_sec > 0.0:
                    # 중앙에서 멈추지 않고 **반대쪽으로 더 넘어간다.**
                    self.phase = OvertakePhase.OVERSHOOT
                    self._t = 0.0
                    self.last_reason = (
                        f"overshoot {self._overshoot():+.0f}px "
                        f"{self.overshoot_sec:.1f}s")
                    return self._overshoot()
                self._finish()
                return 0.0
            return self._offset(1.0 - ratio)

        if self.phase is OvertakePhase.OVERSHOOT:
            # 복귀 방향으로 0.5초 더 꾸족 넘어간 뒤 차선 제어에 맡긴다.
            # 오프셋을 계단으로 떨구고 끝내는 것이 맞다 — 여기서 다시 0 으로
            # 램프를 내리면 반대쪽으로 갔다가 돌아오는 지그재그가 한 번 더 생긴다.
            if self._t >= self.overshoot_sec:
                self._finish()
                return 0.0
            return self._overshoot()

        return 0.0


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
        half_near = getattr(obs, "half_near", 0.0)
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
                half_near=half_near,
            )

        return target
