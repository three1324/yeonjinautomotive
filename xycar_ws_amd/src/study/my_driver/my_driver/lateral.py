"""횡방향 목표 결정 — "트랙 중앙에서 얼마나 벗어난 곳을 목표로 삼을까".

ROS 의존성 없음.

평소에는 트랙 중앙(target_offset = 0)이 목표다. 방해차량이 앞에 있을 때만
일시적으로 옆으로 밀었다가 되돌린다.

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
    """방해차량 추월 서브행동.

    트리거 조건 (모두 만족해야 시작):
      - 카메라가 차량을 봤다 (라바콘이 아니라 차량이라는 건 YOLO만 안다)
      - 그 차량이 충분히 가깝다 (bbox 하단 y가 임계 이상)
      - 라이다 전방 거리도 임계 이하 (카메라 오검출에 속지 않기 위한 교차확인)
      - 피할 쪽에 여유가 있다 (라이다 좌/우 여유)

    카메라와 라이다를 둘 다 요구하는 이유: 한쪽만 믿으면 오검출로 갑자기
    차선을 벗어나는 위험한 동작이 나온다.
    """

    def __init__(self, shift_px, trigger_bottom_y, trigger_front_dist,
                 side_clearance, shift_sec, pass_sec, return_sec):
        self.shift_px = shift_px                    # 얼마나 옆으로 밀지 (픽셀)
        self.trigger_bottom_y = trigger_bottom_y    # 차량 bbox 하단 y 임계 (클수록 가까움)
        self.trigger_front_dist = trigger_front_dist  # 라이다 전방 거리 임계 (m)
        self.side_clearance = side_clearance        # 피할 쪽 최소 여유 (m)
        self.shift_sec = shift_sec
        self.pass_sec = pass_sec
        self.return_sec = return_sec

        self.phase = OvertakePhase.IDLE
        self._t = 0.0
        self._dir = 0    # +1: 오른쪽으로 피함, -1: 왼쪽으로 피함

    def reset(self):
        self.phase = OvertakePhase.IDLE
        self._t = 0.0
        self._dir = 0

    @property
    def active(self):
        return self.phase is not OvertakePhase.IDLE

    def _pick_side(self, car_cx, image_width, left_free, right_free):
        """차량 반대쪽으로 피하되, 여유가 없으면 포기(0)."""
        car_on_left = car_cx < image_width / 2.0
        first = 1 if car_on_left else -1        # 차가 왼쪽이면 오른쪽으로
        second = -first

        for d in (first, second):
            free = right_free if d > 0 else left_free
            if free >= self.side_clearance:
                return d
        return 0

    def update(self, dt, car_present, car_cx, car_bottom_y,
               front_dist, left_free, right_free, image_width):
        """추월로 인한 목표 오프셋 보정량(픽셀)을 반환한다. 평소 0."""
        if self.phase is OvertakePhase.IDLE:
            triggered = (
                car_present
                and car_bottom_y >= self.trigger_bottom_y
                and front_dist <= self.trigger_front_dist
            )
            if triggered:
                d = self._pick_side(car_cx, image_width, left_free, right_free)
                if d != 0:
                    self._dir = d
                    self.phase = OvertakePhase.SHIFT
                    self._t = 0.0
            return 0.0

        self._t += dt

        if self.phase is OvertakePhase.SHIFT:
            ratio = min(self._t / max(self.shift_sec, 1e-3), 1.0)
            if ratio >= 1.0:
                self.phase = OvertakePhase.PASS
                self._t = 0.0
            return self._dir * self.shift_px * ratio

        if self.phase is OvertakePhase.PASS:
            # 앞이 비었으면 예정보다 일찍 복귀를 시작한다
            if self._t >= self.pass_sec or front_dist > self.trigger_front_dist * 1.5:
                self.phase = OvertakePhase.RETURN
                self._t = 0.0
            return self._dir * self.shift_px

        if self.phase is OvertakePhase.RETURN:
            ratio = min(self._t / max(self.return_sec, 1e-3), 1.0)
            if ratio >= 1.0:
                self.reset()
                return 0.0
            return self._dir * self.shift_px * (1.0 - ratio)

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

    def update(self, dt, obs, image_width):
        """obs: driver_node 가 모아 넘기는 관측 묶음. 목표 오프셋(픽셀) 반환."""
        target = 0.0   # 기본은 트랙 중앙

        if self.enable_overtake:
            target += self.overtake.update(
                dt,
                obs.car_present, obs.car_cx, obs.car_bottom_y,
                obs.front_dist, obs.left_free, obs.right_free,
                image_width,
            )

        return target
