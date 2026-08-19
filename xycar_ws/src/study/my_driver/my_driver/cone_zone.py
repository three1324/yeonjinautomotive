"""라바콘 구간 판정 — "지금 라이다를 써도 되는 구간인가".

ROS 의존성 없음.

────────────────────────────────────────────────────────────────────────
왜 이 판정이 따로 필요한가

라이다를 **라바콘 구간에서만** 쓰기로 했다(2026-08-19 실차 결정).
그러려면 "지금이 그 구간인가"를 한 곳에서 정해야 한다. 각 모듈이 저마다
cone_n 임계를 들고 판단하면 서로 다른 시점에 켜지고 꺼져서, 예를 들어
fusion 은 복도를 쓰기 시작했는데 longitudinal 은 아직 아니라 속도만
평소대로 나가는 어긋남이 생긴다.

────────────────────────────────────────────────────────────────────────
판정 근거는 **카메라**다 (라이다가 아니다)

"라이다를 언제 쓸까"를 라이다로 정하면 순환이다. 콘이 있는지는 YOLO 가
traffic_cone 클래스로 직접 안다(cone_n). 라이다는 그 구간에서 **복도의
위치**를 알려주는 역할만 한다. 역할이 이렇게 갈려야 한 센서가 죽었을 때
무엇이 무너지는지가 분명해진다.

────────────────────────────────────────────────────────────────────────
히스테리시스와 유지시간이 필요한 이유

콘 개수는 프레임마다 흔들린다(가려짐·검출 실패). 단일 임계로 끊으면
경계에서 켜졌다 꺼졌다 하고, 그때마다 fusion 가중치가 방향을 바꿔
목표가 떨린다. 그래서

    enter_n 이상  -> 진입 (높은 문턱)
    exit_n  이하  -> 이탈 (낮은 문턱)
    이탈 조건을 만족해도 exit_hold_sec 동안은 유지

콘 구간을 빠져나오는 순간 마지막 콘 몇 개가 시야에서 사라지는데, 그때
차체는 아직 콘 사이에 있다. 유지시간은 그 구간을 덮기 위한 것이다.
"""


class ConeZoneDetector:
    """카메라 콘 개수로 라바콘 구간 진입/이탈을 판정한다.

    enter_n:       이 개수 이상이면 진입
    exit_n:        이 개수 이하로 떨어지면 이탈 후보 (enter_n 보다 낮게 둘 것)
    exit_hold_sec: 이탈 후보가 된 뒤에도 이 시간만큼은 구간을 유지한다
    """

    def __init__(self, enter_n=3, exit_n=1, exit_hold_sec=1.5):
        self.enter_n = enter_n
        self.exit_n = exit_n
        self.exit_hold_sec = exit_hold_sec

        self._active = False
        self._exit_t = 0.0      # 이탈 조건이 연속으로 유지된 시간
        self.last_reason = "init"

    def reset(self):
        self._active = False
        self._exit_t = 0.0
        self.last_reason = "reset"

    @property
    def active(self):
        return self._active

    def update(self, dt, cone_n):
        """프레임당 1회. 지금이 라바콘 구간인지 반환한다."""
        if not self._active:
            if cone_n >= self.enter_n:
                self._active = True
                self._exit_t = 0.0
                self.last_reason = f"enter(cone {cone_n})"
            return self._active

        # 구간 안에 있을 때
        if cone_n <= self.exit_n:
            self._exit_t += dt
            if self._exit_t >= self.exit_hold_sec:
                self._active = False
                self._exit_t = 0.0
                self.last_reason = f"exit(cone {cone_n}, {self.exit_hold_sec:.1f}s)"
        else:
            # 콘이 다시 보이면 이탈 타이머를 되돌린다
            self._exit_t = 0.0

        return self._active
