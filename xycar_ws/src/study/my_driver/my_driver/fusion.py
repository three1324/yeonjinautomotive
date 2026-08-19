"""차선(카메라)과 콘 복도(라이다)를 섞어 최종 횡방향 기준을 만든다.

ROS 의존성 없음 (표준 라이브러리만). 오프라인 검증 가능.

────────────────────────────────────────────────────────────────────────
왜 섞나

평소에는 카메라 차선이 기준이다. 그런데 라바콘 구간에서는 **콘 벽이 실제
주행 가능 경계**이고, 우측 콘 벽이 흰 실선보다 안쪽에 있어서 페인트 차선
중심을 따라가면 콘을 친다. 그 구간만 라이다 복도를 따라야 한다.

────────────────────────────────────────────────────────────────────────
왜 스위치가 아니라 혼합인가

"콘이 보이면 복도, 아니면 차선" 식으로 딱 끊으면 전환 순간 목표가 수십~수백
픽셀 점프한다. 그 점프가 그대로 조향에 실려 차가 휘청인다.
그래서 가중치를 두고 **시간에 걸쳐 서서히** 옮긴다.

가중치 결정 — 구간에 따라 **센서가 통째로 바뀐다** (2026-08-19 실차 결정)

    라바콘 구간 밖  -> 가중치 0.  차선(카메라) 단독. 라이다 간섭 없음.
    라바콘 구간 안  -> 가중치 1.  복도(라이다) 단독. 차선을 섞지 않음.
    둘 다 invalid   -> 결과도 invalid (driver 가 hold/정지 판단)

목표 가중치는 0 아니면 1 이다. 0.5 같은 중간값은 **전환 중에만** 나타나며,
그것도 아래 rate limit 때문에 생기는 과도값이지 의도된 혼합이 아니다.

왜 혼합을 그만뒀나: 콘 구간에서는 콘 벽이 실제 주행 가능 경계이고 페인트
차선은 그것과 **어긋나 있다**(우측 콘 벽이 흰 실선보다 안쪽). 차선을 10%만
섞어도 목표가 콘 쪽으로 밀려 접촉 위험이 생긴다. 반대로 콘 구간 밖에서는
라이다가 트랙 밖 벽·기둥·관중을 잡아 엉뚱한 곳을 가리킨다. 두 센서는
각자의 구간에서만 옳으므로, 섞지 말고 갈아타는 것이 맞다.

가중치 자체도 초당 변화율을 제한해서 튀지 않게 한다.
"""

from dataclasses import dataclass


@dataclass
class LateralRef:
    """차선/복도/융합 결과가 공유하는 형식. LaneResult, CorridorResult 와 호환."""

    offset_near: float = 0.0
    offset_far: float = 0.0
    valid: bool = False
    quality: float = 0.0


@dataclass
class FusedResult(LateralRef):
    corridor_weight: float = 0.0   # 0=차선만, 1=복도만. 진단·로그용
    source: str = "none"           # "lane" | "corridor" | "blend" | "none"


class LateralFusion:

    def __init__(
        self,
        weight_rate_per_sec=1.5,
        # ↑ 가중치가 0->1 로 가는 데 최소 1/1.5 = 0.67초. 전환 점프를 막는다.
        #   목표 가중치는 0 아니면 1 이므로, 전환의 부드러움은 **전적으로**
        #   이 값이 결정한다.
        min_corridor_quality=0.4,
        # ↑ 복도 신뢰도가 이보다 낮으면 복도를 못 만든 것으로 본다
    ):
        self.weight_rate_per_sec = weight_rate_per_sec
        self.min_corridor_quality = min_corridor_quality

        self._w = 0.0     # 현재 복도 가중치 (서서히 변한다)
        # 직전 출력. 한쪽 입력이 끊겼을 때 여기서부터 서서히 옮겨간다.
        self._last = FusedResult()

    def reset(self):
        self._w = 0.0
        self._last = FusedResult()

    @property
    def corridor_weight(self):
        return self._w

    def _target_weight(self, lane, corridor, cone_zone):
        """이번 프레임이 원하는 복도 가중치 (아직 rate limit 적용 전).

        값은 사실상 0 또는 1 이다 — 구간에 따라 **어느 센서로 달릴지**가
        완전히 갈리기 때문이다. 중간값은 전환 중에만 rate limit 때문에 생긴다.
        """
        # 라바콘 구간 밖: 라이다 간섭 0. 차선만으로 달린다.
        if not cone_zone:
            return 0.0

        corridor_ok = corridor.valid and corridor.quality >= self.min_corridor_quality
        if not corridor_ok:
            # 콘 구간인데 복도를 못 만들었다 = 라이다 쪽 실패.
            # 차선으로 되돌아가는 것은 **설계 의도가 아니라 비상 폴백**이다.
            # 콘 구간 한가운데서 멈추는 것보다는 낫다는 판단이고, 실제로는
            # 복도가 한두 프레임 끊기는 정도라 rate limit 이 덮어준다.
            # 자주 뜨면 obstacle_node 의 복도 추정 파라미터를 봐야 한다.
            return 0.0

        # 라바콘 구간 = 복도 단독 주행. 차선을 섞지 않는다.
        return 1.0

    def update(self, dt, lane, corridor, cone_zone=False):
        """프레임당 1회. lane/corridor 는 LateralRef 호환 객체.

        cone_zone: 지금이 라바콘 구간인가 (cone_zone.ConeZoneDetector 판정).
                   True 면 복도 단독, False 면 차선 단독.
        """
        target = self._target_weight(lane, corridor, cone_zone)

        # 가중치 변화율 제한 — 전환 시 목표가 점프하지 않게
        step = self.weight_rate_per_sec * max(dt, 1e-3)
        if target > self._w:
            self._w = min(target, self._w + step)
        else:
            self._w = max(target, self._w - step)

        w = self._w

        if not lane.valid and not corridor.valid:
            self._last = FusedResult(self._last.offset_near, self._last.offset_far,
                                     False, 0.0, w, "none")
            return self._last

        # 한쪽만 유효하면 나머지 자리에는 **직전 출력**을 넣는다.
        # 그냥 유효한 쪽으로 즉시 갈아타면 두 기준의 차이만큼 한 프레임에 점프한다
        # (실측: 차선 소실 순간 120px 점프). 직전 값에서 가중치를 따라 옮겨가면
        # 그 점프가 weight_rate_per_sec 에 걸려 부드러워진다.
        if lane.valid:
            l_near, l_far, l_q = lane.offset_near, lane.offset_far, lane.quality
        else:
            l_near, l_far, l_q = self._last.offset_near, self._last.offset_far, 0.0

        if corridor.valid:
            c_near, c_far, c_q = corridor.offset_near, corridor.offset_far, corridor.quality
        else:
            c_near, c_far, c_q = self._last.offset_near, self._last.offset_far, 0.0

        if w <= 1e-3:
            source = "lane"
            # 가중치가 0인데 차선이 없으면 실제로 참고할 게 없다.
            # (차선 소실 + 복도 저신뢰) — 붙잡아둔 값을 valid 로 내보내면
            # driver 가 유효한 기준으로 오해한다.
            live = lane.valid
        elif w >= 1.0 - 1e-3:
            source = "corridor"
            live = corridor.valid
        else:
            source = "blend"
            live = lane.valid or corridor.valid

        self._last = FusedResult(
            offset_near=(1.0 - w) * l_near + w * c_near,
            offset_far=(1.0 - w) * l_far + w * c_far,
            valid=live,
            quality=(1.0 - w) * l_q + w * c_q,
            corridor_weight=w,
            source=source,
        )
        return self._last
