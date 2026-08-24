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
    """방해차량 회피. **카메라만 쓴다 (라이다 사용 안 함).**

    ════════════════════════════════════════════════════════════════
    [2026-08-24] 팀원 구현(race_control/overtake_drive.OvertakeDrive)을
    **그대로 이식**했다. 위상·문턱·회피량·탈출조건을 바꾸지 않았다.

    이전 자체 구현과 무엇이 달라졌나 (넷 다 실차 증상의 원인이었다):

      1. 회피 **방향**을 화면 중앙이 아니라 **중앙 dashed** 로 정한다.
         차량 bbox **하단**에서 dashed 를 평가해 그 좌/우로 차선을 가른다.
             lane 1 = 차가 dashed 왼쪽  -> 오른쪽으로 피한다
             lane 2 = 차가 dashed 오른쪽 -> 왼쪽으로 피한다
         화면 중앙 기준은 **우리 차가 이미 옆으로 치우쳐 있으면 틀린다** —
         회피 중에 두 번째 차를 만나면 특히 그렇다.
         dashed 를 못 본 프레임만 화면중앙 폴백(deadband 안이면
         center_fallback_direction).

      2. 진입 판정이 **2단 신뢰도**다 (staged_vehicle_entry).
             (conf >= early_conf  AND  h_ratio >= early_ratio)   멀리서: 엄격
          OR (conf >= normal_conf AND  h_ratio >= normal_ratio)  가까이: 완화
         거리 대용값도 **화면 높이 대비 비율**(height_ratio)이라 해상도에
         종속되지 않는다. 옛 구현은 픽셀 절대값(car_h >= 55px)이었다.

      3. 탈출 타이머가 **회피를 시작시킨 그 라벨만** 본다 (observe_target).
         옛 구현은 car_present(=화면에 아무 차나) 로 리셋해서, 앞에 두 번째
         차가 보이거나 오검출이 하나만 있어도 **영원히 안 풀렸다**.
         실차에서 "회피 뒤 바깥 차선으로 쭉" 이 정확히 그 증상이다.

      4. 탈출 시간이 **0.8초**다 (옛 2.5초). 라벨을 좁혔으니 짧아도 된다.

    ════════════════════════════════════════════════════════════════
    회피량은 **모델별 · 좌우별** 고정 픽셀이다
        AvanteN  left 65 / right 85
        ionic5   left 70 / right 90
    좌우가 다른 이유는 차량이 좌측으로 쏠리는 성향이 있어 오른쪽으로 피할
    때 더 크게 밀어야 실제로 같은 만큼 옮겨지기 때문이다(실차 관측).
    ⚠️ 고정 픽셀이라 트랙 폭·원근이 반영되지 않는다 — 좁은 구간에서는
       실선을 넘을 수 있다.
    """

    IDLE, PASS, RECOVER = range(3)
    NAMES = ("IDLE", "PASS", "RECOVER")

    def __init__(self, cfg):
        self.cfg = cfg
        if cfg["target_lost_seconds"] <= 0.0:
            raise ValueError("target_lost_seconds must be positive")
        self.phase = self.IDLE
        self.phase_started = 0.0
        self.direction = -1.0
        self.cooldown_until = 0.0
        self.target_last_seen_at = None
        self.target_label = None
        self.active_target_lost_seconds = cfg["target_lost_seconds"]
        self.active_pass_offset_px = 0.0
        self.active_pass_seconds = cfg.get("pass_seconds", 0.0)
        self.offset_source = "idle"
        self.last_reason = ""

    # ── 진단·시각화용 (driver_node / viz 가 읽는다) ──
    @property
    def active(self):
        return self.phase != self.IDLE

    @property
    def phase_name(self):
        return self.NAMES[self.phase]

    @property
    def amount(self):
        """이번 기동의 회피량(픽셀)."""
        return self.active_pass_offset_px

    @property
    def dir_sign(self):
        """+1 오른쪽으로 피함 / -1 왼쪽으로 피함."""
        return 1 if self.direction > 0.0 else -1

    def reset(self):
        self.phase = self.IDLE
        self.target_last_seen_at = None
        self.target_label = None
        self.active_target_lost_seconds = self.cfg["target_lost_seconds"]
        self.active_pass_offset_px = 0.0
        self.active_pass_seconds = self.cfg.get("pass_seconds", 0.0)
        self.offset_source = "idle"

    def trigger(self, now, center_x, image_width, target_label=None,
                direction_override=None):
        """회피 시작. 시작했으면 True.

        direction_override: +1 / -1 이면 그 방향으로 확정(중앙 dashed 기준).
                            None 이면 화면중앙 폴백을 쓴다.
        """
        if self.active or now < self.cooldown_until or image_width <= 0:
            return False
        reference_x = image_width / 2.0
        deadband_px = image_width * self.cfg["side_deadband_ratio"]
        if direction_override is not None:
            # obstacle_lane 1 -> 오른쪽 2차선, lane 2 -> 왼쪽 1차선.
            self.direction = 1.0 if float(direction_override) >= 0.0 else -1.0
        elif center_x < reference_x - deadband_px:
            self.direction = 1.0
        elif center_x > reference_x + deadband_px:
            self.direction = -1.0
        else:
            self.direction = self.cfg["center_fallback_direction"]
        self.phase, self.phase_started = self.PASS, now
        self.target_last_seen_at = now
        self.target_label = target_label
        per_label = self.cfg.get("target_lost_seconds_by_label", {})
        self.active_target_lost_seconds = float(
            per_label.get(target_label, self.cfg["target_lost_seconds"])
        )
        if self.active_target_lost_seconds <= 0.0:
            raise ValueError("active target_lost_seconds must be positive")
        label_offsets = self.cfg.get("offsets_by_label", {})
        offsets = label_offsets.get(target_label, self.cfg["default_offsets"])
        # direction +1=차량 오른쪽 이동, -1=차량 왼쪽 이동.
        self.active_pass_offset_px = float(
            offsets["right_px"] if self.direction > 0.0 else offsets["left_px"]
        )
        # ★ [사용자 2026-08-24] 유지시간도 **모델별 고정**이다.
        per_sec = self.cfg.get("pass_seconds_by_label", {})
        self.active_pass_seconds = float(
            per_sec.get(target_label, self.cfg.get("pass_seconds", 0.0)))
        source = "dashed_lane" if direction_override is not None else "screen_fallback"
        self.offset_source = source + ":" + str(target_label or "vehicle")
        move = "right" if self.direction > 0.0 else "left"
        self.last_reason = (
            f"start {self.offset_source} cx{center_x:.0f} -> "
            f"move {move} {self.active_pass_offset_px:.0f}px "
            f"for {self.active_pass_seconds:.1f}s")
        return True

    def observe_target(self, now, visible):
        """회피를 시작시킨 **동일 라벨**의 마지막 검출시각을 갱신한다.

        ★ 여기가 핵심이다. 다른 차종이 보여도 타이머를 갱신하지 않는다.
          갱신하면 앞의 두 번째 차 하나로 회피가 영원히 안 풀린다.
        """
        if self.active and visible:
            self.target_last_seen_at = now

    def target_missing_seconds(self, now):
        if self.target_last_seen_at is None:
            return 0.0
        return max(0.0, now - self.target_last_seen_at)

    def update(self, now):
        """프레임당 1회. 위상을 갱신하고 현재 위상을 반환한다.

        ★ [사용자 결정 2026-08-24] 탈출을 **고정 시간**으로 바꿨다.
          팀원 원본은 "회피를 시작시킨 라벨이 target_lost_seconds(0.8s)
          동안 안 보이면" 복귀였다. 지금은 **차가 보이든 말든**
          pass_seconds 가 지나면 무조건 복귀한다.

          왜: 탈출이 관측에 걸려 있으면 오검출·두 번째 차량 하나로
          복귀가 늦어지거나 아예 안 된다(실차 증상). 고정 시간은
          그 의존을 통째로 끊는다 — 코스가 정해져 있으므로 "얼마나
          비켜서 얼마나 가야 하는지"를 직접 정하는 편이 재현성이 높다.

          ⚠️ 대가: 그 시간 안에 실제로 못 지나가면 아직 차 옆인데
             중앙으로 돌아온다. pass_seconds 는 **실제 추월에 필요한
             시간보다 넉넉히** 잡아야 한다.
        """
        if (self.phase == self.PASS
                and now - self.phase_started >= self.active_pass_seconds):
            held = now - self.phase_started
            recover = float(self.cfg.get("recover_seconds", 0.0))
            if recover > 0.0:
                # 반대쪽을 짧게 겨냥해 자세를 세운다. 곧바로 0 으로 놓으면
                # 비스듬한 자세 그대로 흘러 중앙을 지나친다.
                self.phase, self.phase_started = self.RECOVER, now
                self.last_reason = (
                    f"label{self.target_label} pass done({held:.1f}s)"
                    f" -> recover {recover:.1f}s")
            else:
                self._to_idle(now, f"label{self.target_label} pass done({held:.1f}s)")
        elif (self.phase == self.RECOVER
              and now - self.phase_started
              >= float(self.cfg.get("recover_seconds", 0.0))):
            self._to_idle(now, "recover done")
        return self.phase

    def _to_idle(self, now, why):
        self.phase = self.IDLE
        self.cooldown_until = now + self.cfg["cooldown_seconds"]
        self.target_last_seen_at = None
        self.target_label = None
        self.active_target_lost_seconds = self.cfg["target_lost_seconds"]
        self.active_pass_offset_px = 0.0
        self.active_pass_seconds = self.cfg.get("pass_seconds", 0.0)
        self.offset_source = "idle"
        self.last_reason = why + " -> lane center"

    def elapsed(self, now):
        """현재 PASS 를 유지한 시간(초). 진단용."""
        return max(0.0, now - self.phase_started) if self.active else 0.0

    def offset(self, now):
        """목표 오프셋(px). **부호가 direction 과 반대다.**

        ────────────────────────────────────────────────────────────
        [사용자 2026-08-24 수정] 이식 직후 회피 방향이 반대였다.

        target_offset 의 규약은 이 저장소 기준으로
            target_offset > 0  ->  트랙중앙이 화면 오른쪽  ->  차는 트랙 **왼쪽**
        인데, direction 은 팀원 규약대로 **+1 = 차를 오른쪽으로**다.
        두 규약이 반대라 그대로 내보내면 피하겠다는 쪽의 정반대로 간다.
        여기서만 뒤집는다 — direction 자체는 로그·시각화(ot_dir)에
        이미 "+1=오른쪽"으로 쓰이고 있어서 그쪽을 건드리면 더 헷갈린다.

        RECOVER 는 그 반대 부호다. 회피를 끝내고 목표를 곧바로 0 으로
        놓으면 차가 비스듬한 자세 그대로 관성으로 흘러 중앙을 지나친다.
        짧게(recover_sec) 반대쪽을 겨냥해 자세를 세운 뒤 0 으로 간다.
        ────────────────────────────────────────────────────────────
        """
        if self.phase == self.PASS:
            return -self.direction * self.active_pass_offset_px
        if self.phase == self.RECOVER:
            return (self.direction * self.active_pass_offset_px
                    * self.cfg.get("recover_ratio", 1.0))
        return 0.0

    def shift_time(self, seconds):
        """정지(STOP) 등으로 멈춘 시간만큼 기점을 밀어 위상 시간을 보존한다."""
        if self.active:
            self.phase_started += seconds
        if self.target_last_seen_at is not None:
            self.target_last_seen_at += seconds


def staged_vehicle_entry(confidence, height_ratio,
                         early_confidence, early_height_ratio,
                         normal_confidence, normal_height_ratio):
    """멀리서는 높은 신뢰도, 가까이서는 완화된 신뢰도로 진입한다.

    [2026-08-24 이식] 팀원 race_control/overtake_drive.py 그대로.

    왜 2단인가: 문턱이 하나면 둘 중 하나를 포기해야 한다 —
      낮게 잡으면 먼 거리의 오검출에 회피가 걸리고,
      높게 잡으면 진짜 차를 코앞에서야 알아본다.
    멀리서는 conf 를 엄격히(0.90) 요구하는 대신 작은 크기도 받아주고,
    가까이서는 크기가 확실하니 conf 를 완화한다(0.80).
    """
    return (
        (float(confidence) >= float(early_confidence)
         and float(height_ratio) >= float(early_height_ratio))
        or
        (float(confidence) >= float(normal_confidence)
         and float(height_ratio) >= float(normal_height_ratio))
    )


class LateralPlanner:
    """횡방향 목표 오프셋을 최종 결정한다."""

    def __init__(self, overtake: OvertakeBehavior, enable_overtake=True,
                 rearm_sec=0.5):
        self.overtake = overtake
        self.enable_overtake = enable_overtake
        # ── 재무장 (2026-08-24 버그 수정) ────────────────────────────
        # 증상: 회피가 끝나도 중앙으로 안 오고 2차선으로 계속 달린다.
        # 원인: 탈출을 **고정시간**으로 바꾸면서, 시간이 다 됐을 때 그 차가
        #   아직 화면에 보이면 cooldown_sec=0 이라 **곧바로 재발동**한다.
        #   PASS -> RECOVER -> IDLE -> PASS ... 로 래치가 반복돼 결과적으로
        #   옆 차선에 눌러앉는다. (팀원 원본은 "그 라벨이 안 보이면" 탈출
        #   이라 탈출 시점에 이미 차가 없어 이 문제가 없었다.)
        # 해법: 방금 회피한 **그 라벨**은 rearm_sec 동안 안 보여야 다시
        #   발동할 수 있다. 다른 차종은 즉시 발동 가능 — 두 번째 차량을
        #   바로 피해야 하는 코스라 전면 쿨다운은 쓰지 않는다.
        self.rearm_sec = rearm_sec
        self._blocked_label = None
        self._gone_since = None

    def blend_waypoint(self, target_offset, waypoint_offset, weight):
        """3단계 확장 지점 — 레이싱 라인 반영.

        아직 쓰지 않는다. waypoint 기반 목표 횡위치가 생기면 여기서 섞는다.
        weight=0 이면 완전히 차선 기준(현재 동작).
        """
        if weight <= 0.0 or waypoint_offset is None:
            return target_offset
        return (1.0 - weight) * target_offset + weight * waypoint_offset

    def update(self, now, obs, image_width, allow_overtake=True,
               entry_cfg=None):
        """obs: driver_node 가 모아 넘기는 관측 묶음. 목표 오프셋(픽셀) 반환.

        [2026-08-24] 팀원 구현 이식으로 시그니처가 바뀌었다 — dt 가 아니라
        **절대시각 now** 를 받는다. OvertakeDrive 가 타임스탬프 기반이다.

        allow_overtake: False 면 이번 tick 은 회피를 하지 않는다. 좌회전 직후
            차단 구간에서 driver_node 가 내린다 (그 파라미터 주석 참고).
            **차단 중에는 overtake 를 reset 한다** — 좌회전 동안 이 경로가
            아예 안 불렸으므로 그 전에 기동 중이던 상태가 남아 있을 수 있다.

        entry_cfg: staged_vehicle_entry 문턱 묶음. None 이면 진입 판정을 하지
            않는다(이미 회피 중일 때의 유지·탈출만 돌린다).

        좌회전 EXIT 의 목표 오프셋은 여기가 아니라 driver_node 가 이 함수의
        결과에 더한다 — FSM 위상을 아는 쪽이 거기이기 때문이다.
        """
        if not allow_overtake:
            self.overtake.reset()
            return 0.0
        if not self.enable_overtake:
            return 0.0

        vehicles = getattr(obs, "vehicles", None) or []
        self._update_rearm(now, vehicles)

        if self.overtake.active:
            # ★ 회피를 시작시킨 **그 라벨**이 이번 프레임에 보이는가만 본다.
            #   다른 차종이 보여도 타이머를 갱신하지 않는다.
            same = any(int(v["cls"]) == int(self.overtake.target_label or 0)
                       for v in vehicles)
            self.overtake.observe_target(now, same)
        elif entry_cfg is not None:
            v = self._pick_entry(vehicles, entry_cfg, self._blocked_label)
            if v is not None:
                lane = int(v.get("lane", 0))
                # lane 1 -> 차가 dashed 왼쪽 -> 오른쪽(+1)으로 피한다
                # lane 2 -> 차가 dashed 오른쪽 -> 왼쪽(-1)으로 피한다
                # lane 0 -> dashed 를 못 봤다 -> 화면중앙 폴백(None)
                direction_override = (
                    1.0 if lane == 1 else -1.0 if lane == 2 else None
                )
                self.overtake.trigger(
                    now, float(v["cx"]), image_width,
                    target_label=int(v["cls"]),
                    direction_override=direction_override,
                )

        was = self.overtake.target_label
        if self.overtake.update(now) == self.overtake.IDLE and was is not None:
            # 방금 끝난 라벨을 재무장 대기로 넣는다.
            self._blocked_label = was
            self._gone_since = None
        return self.overtake.offset(now)

    def _update_rearm(self, now, vehicles):
        """차단된 라벨이 rearm_sec 동안 안 보이면 다시 발동 가능하게 푼다."""
        if self._blocked_label is None:
            return
        still = any(int(v["cls"]) == int(self._blocked_label) for v in vehicles)
        if still:
            self._gone_since = None
            return
        if self._gone_since is None:
            self._gone_since = now
        elif now - self._gone_since >= self.rearm_sec:
            self._blocked_label = None
            self._gone_since = None

    @staticmethod
    def _pick_entry(vehicles, cfg, blocked_label=None):
        """진입 조건을 만족하는 차 중 **height_ratio 가 가장 큰** 것.

        가장 가까운 차를 고르는 것이다 — conf 가 아니라 크기로 고른다.
        """
        eligible = [
            v for v in vehicles
            if int(v["cls"]) != int(blocked_label or -1)
            and staged_vehicle_entry(
                v["conf"], v["h_ratio"],
                cfg["early_conf"], cfg["early_ratio"].get(int(v["cls"]), 1.0),
                cfg["normal_conf"], cfg["normal_ratio"].get(int(v["cls"]), 1.0),
            )
        ]
        if not eligible:
            return None
        return max(eligible, key=lambda x: float(x["h_ratio"]))
