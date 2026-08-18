"""신호등 램프 상태 시간누적 투표.

ROS/ultralytics 의존성 없음. 영상으로 오프라인 검증 가능.

왜 투표가 필요한가 (reference/perception_analysis.md):
- 켜진 LED가 흰색으로 포화되어 색으로는 빨강/초록 구분이 불가능하다.
  (실측: 초록 (234,246,244) / 빨강 (241,236,236) — RGB 차이 없음)
  구분 단서는 "몇 번째 램프가 켜졌나"뿐이고 그건 YOLO만 안다.
- 그런데 램프 판독 성공률이 거리에 크게 의존한다:
      신호등 박스 ~80px  -> 68%  (출발 대기 위치가 여기)
      80~110px          -> 88%
      110px+            -> 94%
  단일 프레임을 믿으면 출발 위치에서 3번 중 1번은 틀린다.
- 반면 traffic_light 본체는 전 거리에서 conf 0.80~0.88로 안정적이다.
  따라서 "신호등이 보이는가"와 "무슨 색인가"를 분리해서 다뤄야 한다.
"""

from collections import deque

NONE = 0
RED = 1
YELLOW = 2
GREEN = 3
LEFT = 4

NAME_TO_STATE = {"RED": RED, "YELLOW": YELLOW, "GREEN": GREEN, "LEFT": LEFT}
STATE_TO_NAME = {NONE: "NONE", RED: "RED", YELLOW: "YELLOW", GREEN: "GREEN", LEFT: "LEFT"}


class LightVoter:
    """최근 N프레임의 램프 관측을 가중 투표해 하나의 상태로 확정한다."""

    def __init__(self, window, min_weight, min_ratio, miss_tolerance=10):
        self.window = window          # 투표에 쓸 프레임 수 (30fps 기준 30 = 1초)
        self.min_weight = min_weight  # 이 가중치합에 못 미치면 확정하지 않음
        self.min_ratio = min_ratio    # 1위 상태가 전체의 이 비율은 넘어야 확정
        # 신호등이 몇 프레임 연속 안 보여야 "시야에서 벗어났다"고 볼지.
        # 1프레임만 놓쳐도 표를 버리면 검출이 잠깐 깜빡일 때마다 판정이 초기화된다.
        self.miss_tolerance = miss_tolerance
        self._obs = deque(maxlen=window)
        self._state = NONE
        self._miss = 0

    def reset(self):
        self._obs.clear()
        self._state = NONE
        self._miss = 0

    def update(self, lamp_name, lamp_conf, light_width):
        """프레임당 1회 호출.

        lamp_name: 이번 프레임에 검출된 램프 클래스명("RED"/"GREEN"/...) 또는 None
        lamp_conf: 그 신뢰도 (없으면 0)
        light_width: traffic_light 박스 폭(px). 0이면 신호등이 화면에 없음.

        반환: 확정된 상태 상수. 확정할 만큼 근거가 없으면 직전 확정값을 유지한다.
        """
        if light_width <= 0:
            # 신호등이 안 보인다. 다만 단발성 미검출과 실제 시야이탈을 구분해야 한다.
            # traffic_light 본체는 전 거리에서 conf 0.80~0.88로 안정적이므로
            # 연속으로 여러 프레임 놓쳤을 때만 시야이탈로 판단한다.
            self._miss += 1
            if self._miss >= self.miss_tolerance:
                self._obs.clear()
                self._state = NONE
            return self._state

        self._miss = 0

        if lamp_name in NAME_TO_STATE:
            # 멀수록 신뢰도가 낮으므로 박스 크기를 가중치에 반영한다.
            # 110px 이상이면 1.0, 80px 부근이면 0.7 수준.
            size_w = min(1.0, light_width / 110.0)
            self._obs.append((NAME_TO_STATE[lamp_name], lamp_conf * size_w))
        else:
            self._obs.append((NONE, 0.0))

        tally = {}
        for state, w in self._obs:
            if state != NONE:
                tally[state] = tally.get(state, 0.0) + w

        if not tally:
            return self._state

        total = sum(tally.values())
        best_state, best_w = max(tally.items(), key=lambda kv: kv[1])

        if best_w >= self.min_weight and best_w / total >= self.min_ratio:
            self._state = best_state
        return self._state

    @property
    def state(self):
        return self._state

    @property
    def state_name(self):
        return STATE_TO_NAME[self._state]
