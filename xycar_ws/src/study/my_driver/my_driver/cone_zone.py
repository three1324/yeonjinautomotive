"""라바콘 구간 판정 — "지금 라이다를 써도 되는 구간인가".

ROS 의존성 없음.

────────────────────────────────────────────────────────────────────────
왜 이 판정이 따로 필요한가

라이다를 **라바콘 구간에서만** 쓰기로 했다(2026-08-19 실차 결정).
그 판정이 곧 "누가 차를 모는가"를 가른다 — 참이면 제어권이 통째로
rubbercone_node 로 넘어가고, 거짓이면 차선 단독이다(driver_node 참고).
판정이 떨리면 운전자가 매 프레임 바뀌는 셈이라 아래 히스테리시스가 필수다.

────────────────────────────────────────────────────────────────────────
판정 근거는 **카메라**다 (라이다가 아니다)

"라이다를 언제 쓸까"를 라이다로 정하면 순환이다. 콘이 있는지는 YOLO 가
traffic_cone 클래스로 직접 안다(cone_n). 라이다는 그 구간에서 **복도의
위치**를 알려주는 역할만 한다. 역할이 이렇게 갈려야 한 센서가 죽었을 때
무엇이 무너지는지가 분명해진다.

────────────────────────────────────────────────────────────────────────
왜 되살렸나 (2026-08-19 2차 실차)

한동안 이 판정을 rubbercone_node(라이다 부채꼴 점밀도)에 맡겼다. 실차에서
**콘 구간이 아닌 곳에서도 구간이라고 판정**해 제어권이 라이다로 넘어갔고,
차가 차선을 무시하고 엉뚱하게 달렸다. 원인은 그 판정에 콘 모양 필터가
없다는 것이다 — rubbercone_node._update_zone() 은 ROI/클러스터 필터를 거치지
않은 원본 스캔점을 세기 때문에, 전방 1.5m 안의 벽·기둥·사람이면 무엇이든
점 20개를 쉽게 넘긴다.

그래서 판정을 다시 카메라로 되돌린다. **콘 8개 이상 + 충분히 가까우면 진입,
4개 이하면 이탈**이다 (크기 조건은 바로 아래 절 참고). /cone_zone_active(라이다 판정)는 이제 제어에 쓰지 않고 진단
로그로만 남긴다.

────────────────────────────────────────────────────────────────────────
왜 개수만으로는 부족한가 — 크기 문턱 (2026-08-19)

콘 개수만 보면 **구간에 닿기 한참 전에** 트리거가 걸린다. 라바콘 구간은
콘이 줄지어 서 있어서 직선 끝에서도 8개, 10개가 한꺼번에 시야에 들어온다.
그 순간 제어권이 라이다로 넘어가면 아직 차선 구간인데 콘 복도를 따라가려
들고, 실제 콘까지는 몇 미터가 남아 있어 엉뚱하게 달린다.

그래서 진입에는 조건이 둘이다:

    콘 개수 >= enter_n          (구간이 맞는가)
    가장 큰 콘 bbox 높이 >= enter_min_size_px   (충분히 가까운가)

크기를 쓰는 이유: bbox 높이는 거의 순수하게 거리의 함수다. 하단 y 는 카메라
피치·노면 기울기에 흔들리고 콘이 화면 아래로 잘리면 오히려 작아진다.

**이탈에는 크기 조건을 걸지 않는다.** 구간을 빠져나갈 때 남은 콘은 멀어지며
작아지는데, 크기로도 끊으면 개수 조건보다 먼저 이탈해 아직 콘 사이인 차가
차선 주행으로 돌아간다. 나가는 판단은 개수 + 유지시간으로만 한다.
(2026-08-24 크기 조건을 잠깐 추가했다가 같은 날 되돌렸다 — 유지시간을
1.5s -> 0.8s 로 줄여 반응성을 확보하는 쪽을 택했다.)

────────────────────────────────────────────────────────────────────────
히스테리시스와 유지시간이 필요한 이유

콘 개수는 프레임마다 흔들린다(가려짐·검출 실패). 단일 임계로 끊으면
경계에서 켜졌다 꺼졌다 하고, 그때마다 제어권이 차선 <-> 라이다로
넘나들어 목표가 떨린다. 그래서

    enter_n 이상  -> 진입 (높은 문턱)
    exit_n  이하  -> 이탈 (낮은 문턱)
    이탈 조건을 만족해도 exit_hold_sec 동안은 유지

콘 구간을 빠져나오는 순간 마지막 콘 몇 개가 시야에서 사라지는데, 그때
차체는 아직 콘 사이에 있다. 유지시간은 그 구간을 덮기 위한 것이다.
"""


class ConeZoneDetector:
    """카메라 콘 개수로 라바콘 구간 진입/이탈을 판정한다.

    enter_n:            이 개수 이상이면 진입 (크기 조건도 함께 만족해야 함)
    enter_min_size_px:  가장 큰 콘의 bbox 높이가 이 값 이상이어야 진입.
                        0 이면 크기 조건을 끈다(개수만으로 판정).
    exit_n:             이 개수 이하로 떨어지면 이탈 후보 (enter_n 보다 낮게 둘 것)
    exit_hold_sec:      이탈 후보가 된 뒤에도 이 시간만큼은 구간을 유지한다
    """

    def __init__(self, enter_n=8, exit_n=4, exit_hold_sec=1.5,
                 enter_min_size_px=100.0):
        self.enter_n = enter_n
        self.exit_n = exit_n
        self.exit_hold_sec = exit_hold_sec
        self.enter_min_size_px = enter_min_size_px

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

    def update(self, dt, cone_n, cone_size_px=0.0):
        """프레임당 1회. 지금이 라바콘 구간인지 반환한다.

        cone_size_px: 가장 큰 콘의 bbox 높이(px). 거리 대용값이다.
        """
        if not self._active:
            near_enough = (self.enter_min_size_px <= 0.0
                           or cone_size_px >= self.enter_min_size_px)
            if cone_n >= self.enter_n and near_enough:
                self._active = True
                self._exit_t = 0.0
                self.last_reason = f"enter(cone {cone_n}, h{cone_size_px:.0f}px)"
            elif cone_n >= self.enter_n:
                # 개수는 찼는데 아직 멀다. 이 상태가 오래 이어지면 문턱이
                # 높은 것이니 enter_min_size_px 를 낮춰야 한다.
                self.last_reason = f"far(cone {cone_n}, h{cone_size_px:.0f}px)"
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
