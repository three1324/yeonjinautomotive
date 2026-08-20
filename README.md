# 국민대 자율주행 경진대회 2026 — xycar 자율주행 프로젝트

> **이 문서는 프로젝트를 처음 접하는 사람(또는 AI 에이전트)이 맥락 없이 읽어도
> 작업을 이어갈 수 있도록 쓰였다.** 설계 결정에는 근거를 함께 적어두었다.
> 근거 없이 바꾸면 이미 검증으로 걸러낸 오답으로 되돌아가기 쉽다.
>
> 최종 갱신: 2026-08-14

---

## 1. 프로젝트 개요

| 항목 | 내용 |
|---|---|
| 대회 | 국민대 자율주행 경진대회 (9회), **2026-08-25(화)** |
| 장소 | 국민대 자율주행 스튜디오 |
| 차량 | xycar (1/10 스케일 RC카 기반, Ackermann 조향) |
| 성적 | 주행시간 + 벌초. 오전/오후 2회 중 좋은 기록 |
| 별도 경기 | 자율주차 (주최측이 SLAM 지도 제공, 도착지 추첨) |

### 주행 미션 (3바퀴)

```
①신호등 인식 출발 → ②차선주행 → ③라바콘 구간 → ⑤방해차량 추월
    → ⑥신호등 보고 지름길 경로선택 → ⑦결승선 통과
```

- **보행자 회피 미션 없음** (규정 PDF에는 있으나 실제 트랙에서 제외됨)
- **언덕 구간 없음** (동일)
- ⚠️ **라바콘 구간 재검토 필요 (2026-08-17)**
  콘이 좌우로 촘촘히 늘어서 **복도(벽)** 를 이루고 그 복도가 S자로 굽어 있다.
  **우측 콘 벽이 흰 실선보다 안쪽**이라 페인트 차선 중심을 따라가면 콘을 친다.
  라이다 기반 복도 중앙 추종이 필요. → `reference/tuning_guide.md` §4-1

---

## 2. 실행 환경 — 3단계로 나뉜다

```
[1] 개발      Windows PC (지금 이 저장소)
                 └ 소스 편집 + 영상 기반 인식 검증만 가능. ROS2 없음.
                        ↓ git / 파일 복사
[2] 빌드·실행  reComputer Super J4012 = Jetson Orin NX 16GB
                 └ JetPack 6.x / Ubuntu 22.04 / Python 3.10 / ROS2 Humble
                 └ colcon build 는 여기서만 수행 (aarch64)
                        ↓ 장착
[3] 차량      xycar 본체 (VESC 모터 + YDLidar + 카메라 + IMU)
```

### ⚠️ Windows에서 절대 할 수 없는 것

`colcon build`, `ros2 run/launch`, 실제 노드 실행. **ROS2 툴체인이 없다.**
빌드 산출물(`build/`, `install/`, `log/`)은 아키텍처 종속이라 복사해도 무의미하다 —
**젯슨에서 소스로부터 새로 빌드해야 한다.**

### ✅ Windows에서 할 수 있는 것 (적극 활용할 것)

**인식 로직의 오프라인 검증**. `my_perception`의 핵심 모듈은 ROS 의존성이 전혀 없게
분리해두었고, 실제 주행영상으로 바로 돌려볼 수 있다. → [6. 개발 워크플로우](#6-개발-워크플로우)

---

## 3. 디렉토리 구조

```
E:\자율주행\auto\
├── README.md                 ← 이 문서
├── xycar_ws/src/             ★ 젯슨 실차 워크스페이스 — 여기서 colcon build 한다
│   ├── study/                ★ 우리가 작성한 패키지 (이 폴더만 우리 코드)
│   │   ├── my_perception/       카메라 인식 (YOLO11n-seg) + models/best5.pt(.engine)
│   │   │                        tools/offline_check.py (영상으로 검증)
│   │   ├── my_obstacle/         라이다. rubbercone_node 하나뿐 —
│   │   │                        ★ 라바콘 구간 주행 전담 (그 밖에선 안 쓴다)
│   │   ├── my_driver/           판단(FSM)·제어 + 라바콘 구간 mux
│   │   │                        tools/sim_check.py (폐루프 시뮬)
│   │   ├── my_slam/             매핑/측위 + waypoint 도구 (기본 off)
│   │   ├── my_debug/            시각화. pipeline_view_node(카메라 시점) +
│   │   │                        viz_node(RViz2 피더) + video_pub_node(영상 재생)
│   │   ├── my_bringup/       ★ 통합 launch + config/drive_params.yaml
│   │   │                        (모든 튜닝 파라미터가 여기 한 파일에)
│   │   ├── VEHICLE_TEST.md   ★ 실차 테스트 체크리스트 (차 꽂기 전에 열 것)
│   │   └── SESSION_LOG.md       실차에서 무엇이 왜 문제였나의 기록
│   ├── rf2o_laser_odometry/     라이다 오도메트리 (my_slam 전용, 외부 소스)
│   ├── xycar_device/            조직위 벤더 원본 (cam/lidar/imu/ultrasonic/msgs)
│   ├── yolo_ros/                조직위 벤더 원본
│   ├── track_drive/             조직위 벤더 원본
│   ├── xycar_application/       조직위 벤더 데모 app_* 10개
│   └── VENDORED.md              벤더 패키지 출처·수정 이력
├── JETSON_ROS1_DOCKER_MOTOR.md  ★ 모터(ROS1 도커) 구축 절차
├── motor_ros1_bundle.zip        그 도커에 넣을 ROS1 소스 (vesc + xycar_motor)
├── amd.zip                      원본 차량 홈디렉토리 백업 (복구용, 건드리지 말 것)
└── reference/                   문서·자료 (빌드 대상 아님)
    ├── perception_analysis.md   ★ 인식 성능 실측 분석 (설계 근거의 원천)
    ├── tuning_guide.md
    ├── docs/모터제어기_VESC_설정방법.pdf
    ├── vesc/                    VESC 펌웨어·설정 백업
    └── videos/                  차선인식 튜닝용 주행영상
```

---

## 4. 아키텍처

### 노드 구성

```
 /image_raw ──→ my_perception ──┬─→ /lane      차선 오프셋
   (카메라)      YOLO11n-seg    ├─→ /light     신호등 상태(투표 완료)
                  1회 추론       └─→ /objects   라바콘·차량

 /scan ───────→ my_obstacle ─────→ /obstacle   전방거리·좌우여유
   (라이다)

 /imu, /odom ─→ my_slam ─────────→ /map, TF    (slam_toolbox)
                                      │
                    ┌─────────────────┴──────────────┐
                    ▼                                 │
              my_driver  ← /lane /light /objects /obstacle
              FSM + 차선추종 + 회피보정
                    │
                    ▼
              xycar_motor 토픽 [angle, speed]
                    ▼
              xycar_motor 노드 → ackermann_cmd → vesc_ackermann → vesc_driver → VESC
```

### 왜 인식을 한 노드에 몰았나

**YOLO를 같은 이미지에 여러 번 돌릴 수 없기 때문**이다. `best5.pt` 한 번의 추론으로
차선·라바콘·신호등·차량이 전부 나온다. 인식을 미션별 노드로 쪼개면 추론이 3배가 된다.
대신 차선 마스크 → 오프셋 변환까지 `my_perception` 안에서 끝낸다
(마스크를 토픽으로 넘기는 것은 낭비).

### 토픽 계약 (표준 메시지 사용)

커스텀 메시지 패키지를 두지 않기로 했다 — 젯슨 첫 빌드에서 `rosidl` 생성 실패 리스크를
없애고, Python 노드 재빌드 부담을 피하기 위해서다. **대신 필드 의미를 여기 명시한다.**

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/lane` | `Float32MultiArray` | `[offset_near, offset_far, valid, quality, half_near, half_far]`<br>`half_*` 는 학습된 트랙 반폭(px). 회피 목표(§5-3-1)에 쓴다. 0이면 미학습 |
| `/light` | `Int32` | `0=NONE 1=RED 2=YELLOW 3=GREEN 4=LEFT` (투표 확정값) |
| `/objects` | `Float32MultiArray` | `[cone_n, car_present, car_cx, car_area]` |
| `/obstacle` | `Float32MultiArray` | `[front_dist, left_free, right_free]` |
| `xycar_motor` | `Float32MultiArray` | `[angle, speed]` — 기존 xycar 관례, 범위 -50~50 |

- `offset_*` 단위는 **픽셀**. `+`면 트랙 중앙이 화면 중심보다 오른쪽.
- `valid=0`이면 이번 프레임에 차선을 못 본 것(hold 중) → driver가 판단에 반영할 것.

---

## 5. 주행 알고리즘

### 5-1. 차선 추정 — `my_perception/lane.py`

**목표는 "차로 중심"이 아니라 "트랙 중앙"이다.** 이게 이 프로젝트에서 가장 중요한
발견이고, 직관과 반대라서 반드시 짚고 가야 한다.

| 기준 | 오프셋 중앙값 (주행영상 / 신호등영상) |
|---|---|
| 왼쪽 차로 중심 | −151px / −231px |
| 오른쪽 차로 중심 | +206px / +152px |
| **트랙 중앙** | **+29px / −36px** ✅ |

좌/우 차로를 목표로 두면 대칭으로 크게 벌어지고, 트랙 중앙으로 두면 0에 수렴한다.
즉 **노란 점선은 차로 구분선이 아니라 트랙 중앙 표시**다.
338표본 검증 결과 `(노란 점선) − (좌우 흰선 중점) = 중앙값 +3.0px`로 사실상 일치했다.

**추정 우선순위** (`LaneEstimator`):

1. `dashed` + 좌우 `solid` 모두 → 중앙선과 좌우중점의 평균 (quality 1.0)
2. `dashed`만 → 그 곡선이 곧 트랙 중앙 (quality 0.9) ← **가용률 97%로 주력**
3. 좌우 `solid`만 → 두 곡선의 중점 (quality 0.8)
4. 한쪽 `solid`만 → 학습된 행별 반폭으로 보완 (quality 0.4)
5. 아무것도 없음 → 직전 값 유지(hold), `valid=False`

**설계상 반드시 지켜야 할 세 가지** (전부 실측으로 밝혀진 함정):

- **행 단위 샘플링 금지.** `dashed`는 물리적으로 끊긴 점선이라 특정 행을 찍으면
  결측된다(프레임 단위 96% ↔ 행 단위 55%). 조각 픽셀을 전부 모아
  **2차 다항식 `x = f(y)` 피팅**을 하면 97%로 회복된다.
- **`solid_line`은 좌/우를 반드시 분리.** 합쳐서 중앙값을 내면 좌우 경계선이 섞여
  엉뚱한 값이 나온다. `dashed` 피팅 곡선을 기준으로 가른다.
- **bounding box 사용 금지.** 차선이 대각선이라 bbox가 화면 절반을 덮는다
  (`solid_line` bbox 255×145 관측). **반드시 segmentation mask를 쓸 것.**

**측정된 성능** (주행영상 428프레임): 차선 유효율 100%, quality 중앙값 1.00,
프레임간 변화량 중앙값 1px / 95%tile 36px, 100px 초과 급변 2.6%.
→ 추정은 안정적이며, 산포는 실제 주행 편차다. 급변 2.6%는 driver에서
rate limiter로 흡수할 것.

### 5-2. 신호등 판정 — `my_perception/light_vote.py`

**색으로는 판별이 불가능하다.** 카메라 자동노출이 LED를 완전히 날려버린다:

| 켜진 램프 | 실측 RGB |
|---|---|
| 초록 | (234, 246, 244) |
| 노랑 | (233, 235, 226) |
| 빨강 | (241, 236, 236) |
| 꺼짐 | (150, 160, 155) |

세 색의 RGB가 사실상 동일하다. **HSV 색상 기반 판별을 시도하지 말 것.**
구분 단서는 "몇 번째 램프가 켜졌는가"(위치)뿐이고, 그건 YOLO만 안다.
`best5.pt`에 `RED`/`YELLOW`/`GREEN`/`LEFT` 클래스가 따로 있는 이유가 이것이다.

**거리에 따라 신뢰도가 갈리므로 시간 누적 투표가 필수다:**

| `traffic_light` 박스 폭 | 램프 판독 성공률 |
|---|---|
| ~80px (출발 대기 위치) | **68%** |
| 80–110px | 88% |
| 110px+ | 94% |

`traffic_light` 본체 자체는 전 거리에서 conf 0.80~0.88로 안정적이다.
`LightVoter`가 최근 N프레임을 신뢰도 × 박스크기로 가중 투표해 확정한다.
**단일 프레임 판독을 믿으면 출발 위치에서 3번 중 1번 틀린다.**

추가 제약: **결승선에 근접하면 신호등이 화면 위로 벗어난다.** 멀리서 미리 읽어야 하고
근접 후 재확인 전략은 쓸 수 없다.

### 5-3. 주행 상태기계 — `my_driver/fsm.py`

라바콘·언덕이 상태에서 빠지면서 FSM은 최소한만 남았다:

```
WAIT_LIGHT ──GREEN──→ LANE_DRIVE ──분기점──→ SHORTCUT ──→ FINISH
                          ↑   │
                          └───┘ 추월은 서브행동 (상태 아님)
```

- **라바콘**: ⚠️ 재설계 필요. 콘이 복도(벽)를 이루고 흰선보다 안쪽이라
  트랙 중앙 유지만으로는 콘을 친다. 라이다 복도 추종 추가 예정
  (`reference/tuning_guide.md` §4-1). 현재는 감속만 함.
- **추월**: `LANE_DRIVE` 안의 일시적 서브행동. 끝나면 자동 복귀.
- **SLAM 위치는 보조(advisory)**, 차선추종이 주(primary).
  SLAM이 틀어져도 주행은 차선만으로 계속되어야 한다.

### 5-3-1. 방해차량 회피 — 트랙 반쪽의 중앙으로

FSM 을 늘리지 않고 `LANE_DRIVE` 안의 **서브행동**으로 둔다 (`my_driver/lateral.py`).
상태로 빼면 복귀가 지저분해지고, 회피는 "어디를 따라갈지"의 문제라 횡방향 목표
결정에 속하기 때문이다.

```
IDLE ──(트리거)──▶ SHIFT ──▶ PASS ──▶ RETURN ──▶ IDLE ──(쿨다운)──
                  벌리는 중   유지     복귀
```

**회피량 = 트랙 반폭 / 2 = 트랙 반쪽의 중앙**

```
왼쪽 반 중앙   = (왼쪽 흰실선 + 노란선) / 2 = 트랙중앙 − 반폭/2
오른쪽 반 중앙 = (노란선 + 오른쪽 흰실선) / 2 = 트랙중앙 + 반폭/2
```

`LaneEstimator` 가 좌우 흰선을 동시에 볼 때마다 **행별 반폭을 EMA 로 이미 학습**하고
있다(§5-1). 그 값을 `/lane` 에 실어 보내 회피량으로 쓴다. 고정 픽셀로 미는 것과 달리
**원근·트랙폭이 자동 반영되고, 목표가 항상 트랙 안쪽이라 실선을 넘는 상황이 구조적으로
생기지 않는다.** 반폭 미학습이면 `shift_px` 로 폴백한다.

**트리거** (모두 만족): 차량 검출 · `bottom_y ≥ trigger_bottom_y`(가까움) ·
라이다 `front_dist ≤ 임계`(오검출 교차확인) · 피할 쪽 여유 · 쿨다운 아님.

**PASS 종료는 시간이 아니라 관측**이다. 통과 시간은 속도에 따라 달라져 고정 시간으로는
못 맞춘다.

| 종료 조건 | 뜻 |
|---|---|
| `car gone` | 차량이 안 보임 |
| `car receding` | `bottom_y` 가 임계×0.85 아래 (멀어짐) |
| `car at edge` | `cx` 가 화면 가장자리 (옆으로 지나침) |
| `front clear` | 라이다 전방이 트임 (라이다 쓸 때만) |
| `pass timeout` | 위 어느 것도 안 걸릴 때의 안전 상한 |

**회피 중에는 감속한다** (`speed.overtake_factor: 0.7`). 옆으로 벌리면 전방 장애물이
라이다 섹터에서 빠져 `front_dist` 가 커지고, 그러면 장애물 상한이 풀려 **가장 위험한
순간에 오히려 가속**하기 때문이다.

**복귀 후 쿨다운** (`cooldown_sec: 1.0`) — 없으면 같은 차에 다시 걸려 지그재그한다.

#### 실측 검증 (2026-08-19, `테스트용(신호등미포함).mp4`)

영상 전체에서 방해차량은 643프레임 샘플 중 110프레임(약 17%)에 나온다. 그중 한 구간:

```
f2400  IDLE    target=  +0.0px  half=376
f2741  SHIFT   ← 트리거. cx=440(오른쪽) → 왼쪽으로,  183px = half(367)/2
f2766  PASS    target=-183.3px
f2798  RETURN  ← "car at edge(cx593)"
f2829  IDLE    ← 복귀 완료, 쿨다운 진입
```

`car at edge` 조건이 없었을 때는 `pass timeout(1.5s)` 으로 f2811 에 복귀했다 —
**옆을 스쳐 지나가는 차는 가까워지면서 화면 밖으로 나가므로**(cx 440→612 인데
bottom_y 는 302→439 로 증가) `bottom_y` 만으로는 통과를 못 잡는다. 이 조건 덕분에
약 0.4초 일찍 트랙 중앙으로 복귀한다.

ROS 통합 실행에서도 `ov=- → SHIFT → PASS → RETURN` 과 속도 12.0→8.4 감속을 확인했다:

```bash
ros2 launch my_bringup replay.launch.py video:=~/테스트용.mp4 \
    enable:=true start_frame:=2350 rate_scale:=0.5
```

> 회피는 **카메라 전용**이다(2026-08-19). 예전에 있던 라이다 교차확인
> (`require_lidar_confirm`)은 파라미터째 없앴다 — 영상 재생과 실차가 서로 다른
> 경로로 돌아 replay 로 검증한 동작이 실차에서 재현되지 않았기 때문이다.

### 5-3-2. 라바콘 S자 구간 — `rubbercone_node` 에 통째로 넘긴다

콘이 좌우 두 줄로 **복도(벽)** 를 이루고 그 복도가 S자로 굽어 있다. 우측 콘 벽이 흰
실선보다 안쪽이라 페인트 차선 중심을 따라가면 콘을 친다. **이 구간만** 라이다가 주
센서다.

**구간 판정은 카메라, 주행은 라이다.**

    perception(YOLO cone_n, cone_max_h) -> cone_zone.ConeZoneDetector -> mux 판단
    rubbercone_node --/cone_cmd--------------------------------------> 구간이면 그대로 통과

| 조건 | 동작 |
|---|---|
| 콘 구간 + `/cone_cmd` 신선 | `rubbercone_node` 의 조향·속도를 **그대로** 통과 |
| 콘 구간 + `/cone_cmd` 끊김 | **정지** (차선으로 되돌아가면 콘을 친다) |
| 콘 구간 아님 | 차선 단독. 라이다 값을 읽지도 않는다 |

`rubbercone_node` 는 팀원이 실차에서 성공시킨 구현을 그대로 가져온 것이다 —
콘 클러스터링 → 좌우 페어링 → Pure Pursuit, 실패 시 Follow-the-Gap. 우리가 중간에서
손대면 검증된 동작이 깨지므로 **조향은 건드리지 않는다.** 속도만 `SpeedLimiter` 를
통과시키는데, 알고리즘이 아니라 VESC 저전압 fault 방지(하드웨어 보호) 때문이다.

**구간 판정을 카메라가 하는 이유** (2026-08-19 2차 실차): `rubbercone_node` 의
`/cone_zone_active` 는 전방 부채꼴 안의 점 개수만 세고 콘 모양을 보지 않아서, 콘이
없는 곳의 벽·기둥에도 참이 됐다. 그러면 제어권이 통째로 라이다로 넘어가 차가 차선을
무시하고 달린다. 지금은 **콘 8개 이상 + 가장 큰 콘 bbox 높이가 충분**할 때 진입하고,
**2개 이하가 1.5초 지속**되면 이탈한다. `/cone_zone_active` 는 진단 로그로만 남는다.

#### 없앤 것 — 라이다 복도 추정 (`obstacle_node`, 2026-08-21 삭제)

예전에는 `obstacle_node` 가 `/scan` 에서 복도 중앙선(`corridor.py`)과 섹터 최근접
거리(`sectors.py`)를 뽑아, 그것을 차선과 **연속 혼합**(`fusion.py`)하고 전방 정지
상한으로도 썼다. 전부 지웠다. 근거:

- 그 경로는 **정상 주행에서 도달조차 못 하는 코드**였다. 콘 구간이면 위 표 첫 줄에서
  return 하고, 아니면 차선 단독이다. 오직 "콘 구간인데 rubbercone 이 죽은" 경우에만
  실행됐다.
- 그런데 그때가 가장 위험하다. `corridor.py` 는 **합성 S자 코스로만** 검증됐고
  `px_per_meter: 300.0` 은 실측 전 값이었다. 라이다가 죽었을 때 검증 안 된 추정치로
  콘 사이를 계속 달리는 구조였다. **콘 사이에서 멈추는 벌점이, 콘을 치거나 코스를
  이탈하는 것보다 낫다.**
- 남은 용도가 RViz 시각화뿐이었는데, 그것만을 위해 라이다 복도 추정을 상시 돌릴
  이유가 없다.

이제 `/scan` 을 구독하는 노드는 **`rubbercone_node` 하나**다(+ `viz_node` 의
포인트클라우드 변환). RViz 에서 "지금 무엇을 따르는가"는 상태 텍스트 **색**으로 본다 —
**노랑 = 카메라 차선 / 빨강 = 라바콘 구간.**

> ⚠️ `rubbercone_node` 의 값(클러스터 임계·페어링 간격·lookahead)은 팀원이 **다른
> 차·다른 코스**에서 맞춘 것이다. 우리 트랙에서는 아직 미검증이다. 콘 구간
> `/scan` 을 `ros2 bag record` 로 따 오면 차를 다시 굴리지 않고 재튜닝할 수 있다 —
> 절차는 `VEHICLE_TEST.md` §9.

### 5-4. 제어 전략 — 단계적 융합

**결론부터: 차선추종을 베이스라인으로 깔고, waypoint를 단계적으로 얹는다.**

#### 왜 waypoint 전면 도입이 아닌가 — 라이다가 트랙을 볼 수 없다

f1tenth 표준 트랙과 우리 트랙의 결정적 차이다.

```
f1tenth 트랙 : 벽으로 둘러싸인 통로   → 라이다가 트랙 경계를 직접 본다
우리 트랙    : 바닥에 붙인 테이프     → 라이다에 안 보인다 (평면)
```

2D 라이다는 수평 스캔이므로 바닥의 흰 실선·노란 점선을 감지하지 못한다.
따라서 **SLAM 지도는 "트랙의 지도"가 아니라 "주변 가구의 지도"** 다. 결과적으로:

- 측위 오차가 그대로 차선 이탈이 된다. 규정이 "차선을 벗어나지 않으며 주행"이고
  벌초 대상이라 직접적인 위험이다.
- 주변 환경(사람·의자·책상)이 계속 움직인다. 영상에서 확인됨. 주차 규정에도
  "당일 장애물 추가 또는 기존 물체 이동 가능"이 명시돼 있다.
- 오전/오후 2회 경기 사이에 배치가 바뀔 수 있다.

반면 **차선추종은 벌초 기준(차선 이탈)을 직접 관측**하고 측위가 필요 없다.
(예외: 라바콘 구간은 라이다에 보이므로 그 구간에선 실제 트랙 정보를 얻을 수 있다)

#### 그래서 역할을 나눈다 — 현업의 HD맵 + 카메라 차선 융합과 같은 방식

```
waypoint/지도 → "앞 코너가 얼마나 급한가"      → 속도 계획 (선행 감속)
                "지름길 분기점이 어디인가"      → 분기 판단
차선(카메라)  → "지금 트랙 중앙에서 얼마나 벗어났나" → 횡방향 조향
```

- **속도는 지도에서** — 아직 안 보이는 코너의 곡률을 미리 알 수 있으니 유리
- **횡위치는 차선에서** — 벌초를 직접 막으니 유리
- 측위가 흔들려도 조향은 멀쩡 → 완주 보장
- 측위가 좋으면 코너 진입 속도가 최적화 → 기록 단축

#### 3단계 진행 계획

| 단계 | 내용 | 완료 조건 |
|---|---|---|
| **1** | 차선추종만으로 완주 가능한 상태 | `my_driver` + 실차 게인 튜닝 |
| **2** | SLAM 지도 + waypoint를 **속도 계획에만** 사용 | 측위 안정화, 곡률 기반 선행 감속 |
| **3** | 레이싱 라인 비중 확대 (횡방향에도 waypoint 반영) | 측위 신뢰도 검증 후 |

**어느 단계에서 멈춰도 주행은 된다.** 이게 이 계획의 핵심이다. 반대로 처음부터
waypoint 전면 도입으로 가면 측위 튜닝이 안 끝났을 때 돌아갈 곳이 없다.

> 주차 경기는 별개다. 차선이 없으므로 **처음부터 지도 기반**이어야 하고,
> f1tenth의 `pure_pursuit` + `particle_filter` 조합이 그대로 원형이 된다.
> (`f1tenth_ws.zip` 참고 — `LOOKAHEAD_DISTANCE=3.6`, `WB=0.27`,
> Pure Pursuit 기하해 + PID 보정 하이브리드 구조)

#### 모듈을 이 계획에 맞춰 나눈다

단계가 올라갈 때 기존 코드를 뜯어고치지 않도록, `my_driver` 내부를 이렇게 분리한다:

```
my_driver/
├── driver_node.py    ROS 래퍼 (구독/발행만)
├── fsm.py            상태 관리 (WAIT_LIGHT / LANE_DRIVE / SHORTCUT / FINISH)
├── lateral.py        횡방향 목표 결정   → 2단계까진 차선만, 3단계에서 waypoint 블렌딩
├── longitudinal.py   종방향 목표 결정   → 2단계에서 waypoint 곡률 추가
└── control.py        목표 → 조향/속도 명령 + 안전장치 (rate limit, LPF, fallback)
```

2단계 확장 = `longitudinal.py`에 곡률 입력만 추가.
3단계 확장 = `lateral.py`에 블렌딩만 추가. **다른 파일은 건드리지 않는다.**

---

## 6. 개발 워크플로우

### 오프라인 인식 검증 (Windows에서, ROS 없이)

`my_perception/lane.py`와 `light_vote.py`는 **numpy만 의존**하도록 일부러 분리했다.
젯슨 없이 실제 주행영상으로 바로 튜닝할 수 있다.

```bash
cd xycar_ws/src/study/my_perception
python3 tools/offline_check.py <영상.mp4> --every 15
```

주요 옵션:

| 옵션 | 설명 |
|---|---|
| `--every N` | N프레임마다 1장 처리 (속도 조절) |
| `--limit N` | 최대 N프레임만 |
| `--bias X` | `center_bias_px`. 카메라 장착 편차 보정 |
| `--viz DIR` | 피팅 곡선 그린 프레임 저장 |
| `--out-video P` | 시각화 영상 저장 (CPU에서 느림, 주의) |

출력: 차선 유효율, offset 중앙값·분포, quality, 프레임간 변화량, 신호등 상태 분포.

사전 준비: `pip install ultralytics opencv-python`
테스트 영상: `reference/videos/` 또는 실제 주행영상.

### 파라미터 정합성 검사 (Windows에서, ROS 없이) ★ 젯슨에 올리기 전 필수

```bash
cd xycar_ws/src/study/my_bringup
python3 tools/check_params.py
```

`drive_params.yaml` 의 키와 노드의 `declare_parameters()` 를 대조한다.
**선언하지 않은 파라미터가 yaml 에 있으면 노드가 시작 시 예외로 죽기 때문에**,
파라미터를 추가·개명한 뒤에는 반드시 돌려볼 것. (실제로 `light.miss_tolerance`
누락을 이 검사가 잡아냈다.)

### 제어 폐루프 시뮬레이션 (Windows에서, ROS 없이)

`my_driver` 의 판단·제어 모듈도 ROS 의존성이 없어서 폐루프로 돌려볼 수 있다.

```bash
cd xycar_ws/src/study/my_driver
python3 tools/sim_check.py
python3 tools/sim_check.py --k-lat 0.15 --k-curve 0.30
```

시나리오: 직선 복귀 / 곡선 추종 / 차선 결측 / 추월. 각각 최종오차·진동폭·조향포화율을 낸다.

**확인 가능**: 발산 여부, 진동, 곡선 정상상태 오차, 조향 포화, 결측·추월 로직
**확인 불가**: **실제 게인 값**. 차량 모델의 픽셀↔운동 변환 상수가 임의값이라
절대적 튜닝값은 줄 수 없다. 게인 확정은 실차에서만 가능하다.

### 젯슨으로 옮기는 순서

1. `xycar_ws/` 를 보드로 복사 (`build/ install/ log/` 는 제외)
2. `cd xycar_ws && rosdep install --from-paths src --ignore-src -r -y`
   - `serial_driver`(transport_drivers), `ackermann_msgs`, `slam_toolbox`,
     `rf2o_laser_odometry`, `robot_localization` 등이 여기서 설치됨
3. `pip install ultralytics` (Jetson용 torch 휠 필요 — PyPI x86 휠 아님)
4. `colcon build --symlink-install`  ← **첫 빌드에서 에러가 나는 것이 정상**
5. `source install/setup.bash`
6. 아래 [7. 실차 캘리브레이션](#7-실차-캘리브레이션-미완료) 수행
7. 주행:
   ```bash
   ros2 launch my_bringup drive.launch.py
   # 차는 아직 안 움직인다 (require_enable). 출발시키려면:
   ros2 topic pub --once /drive_enable std_msgs/msg/Bool '{data: true}'
   ```

### 실행 명령 요약

| 목적 | 명령 |
|---|---|
| 전체 주행 | `ros2 launch my_bringup drive.launch.py` |
| **전체 주행 + 시각화** | `ros2 launch my_bringup drive.launch.py rviz:=true` |
| 시각화만 (주행 스택이 이미 떠 있을 때) | `ros2 launch my_debug viz.launch.py` |
| **영상 파일로 실시간 재생 테스트** | `ros2 launch my_bringup replay.launch.py video:=~/test.mp4` |
| **실차 사전 점검** ★ | `python3 my_bringup/tools/preflight.py` (연결 후 `--live`) |
| 인지만 (디버깅) | `ros2 launch my_perception perception.launch.py` |
| 제어만 재시작 | `ros2 launch my_driver driver.launch.py` |
| 지도 생성 | `ros2 launch my_slam mapping.launch.py` |
| 지도 저장 | `ros2 run nav2_map_server map_saver_cli -f ~/track_map` |
| 지도로 측위 | `ros2 launch my_slam localization.launch.py map:=~/track_map` |
| waypoint 기록 | `ros2 run my_slam record_waypoints --ros-args -p out_path:=~/wp.csv` |
| waypoint 평활화 | `python3 my_slam/tools/clean_waypoints.py ~/wp.csv --plot` |

파라미터를 바꿔 띄우려면: `ros2 launch my_bringup drive.launch.py params_file:=/경로/내파일.yaml`

**젯슨에서 주의**: `ultralytics`/`torch`는 PyPI x86 휠이 아니라 **NVIDIA Jetson용 휠**을
써야 한다. 추론 속도가 부족하면 `best5.pt`를 **TensorRT로 export**할 것.

### 영상 파일로 실시간 재생 테스트 (카메라·라이다·모터 없이)

```bash
ros2 launch my_bringup replay.launch.py video:='/home/e-on/테스트용(신호등미포함).mp4'
```

`my_debug/video_pub_node` 가 영상을 **원래 fps 로 실시간** 재생해 `/image_raw` 로 내보내고,
그 뒤로는 실차와 **완전히 같은 노드·같은 파라미터**가 돈다. 카메라·라이다 드라이버와
모터 스택은 아예 띄우지 않는다.

```
video_pub_node → /image_raw → perception_node → /lane /light /objects
                                              → driver_node → /xycar_motor
                                              → 시각화 2창 (OpenCV + RViz2)
```

**`offline_check.py` 와 뭐가 다른가:** 그쪽은 인지 모듈만 ROS 없이 최대 속도로 돌린다
(인지 *정확도* 검증). 이쪽은 실시간 재생이라 **보드가 30fps 를 따라가는지**가 드러난다 —
그건 정확도와 다른 문제고, 실차에서 "가다 멈추고"를 일으킨 바로 그 문제다.

주요 인자:

| 인자 | 기본 | 설명 |
|---|---|---|
| `video` | (필수) | 재생할 영상 파일 |
| `rate_scale` | `1.0` | 재생 배속. `0.5` 면 절반 속도로 천천히 |
| `loop` | `true` | 끝나면 처음부터 다시 |
| `width`/`height` | `640`/`480` | 발행 해상도. ⚠️ `driver_node.image_width` 와 맞춰야 조향 중심이 안 틀어진다 (테스트 영상이 632px 여도 640 으로 늘려 내보내는 이유) |
| `rviz` | `true` | RViz2 도 띄울지 |
| `auto_start` | `true` | 신호등 없이 바로 `LANE_DRIVE` 로 시작. ⚠️ 아래 표 |
| `enable` | `false` | `/drive_enable` 자동 켜기. ⚠️ 아래 표 |

#### ⚠️ `angle=0 speed=0` 만 나올 때 — 원인은 둘 중 하나다

| 화면 표시 | 원인 | 해결 |
|---|---|---|
| `WAIT_LIGHT  ref[none]  why: wait` | FSM 이 **초록불을 기다리는 중.** `fsm.auto_start` 기본값이 false 라 신호등 없는 영상에서는 영원히 출발하지 않는다 | `replay.launch.py` 는 이 값을 **true 로 뒤집어** 놓았다(실차 launch 는 그대로). 신호등 로직 자체를 시험하려면 `auto_start:=false` |
| `LANE_DRIVE ... why: disabled` | `/drive_enable` 이 아직 false | `enable:=true` 를 주거나 다른 터미널에서 직접 발행 |

시각화 텍스트의 **`why:` 줄이 그 답을 그대로 보여준다** (`disabled` / `wait` /
`no_lane_yet` / `stale(..)` / `ref lost` / 정상 주행 시엔 감속 사유).

모터 명령까지 확인하려면:

```bash
ros2 launch my_bringup replay.launch.py video:=~/test.mp4 enable:=true
```

`enable:=false`(기본)에서도 인지 계산은 전부 돌고 화면에 나오지만, FSM 이 `disabled`
경로로 빠져 조향·속도는 0 으로 고정된다. `enable:=true` 는 **모터 스택이 없는 벤치에서만**
쓸 것 — 모터 드라이버 ROS1 도커가 **상시 떠 있으므로**(§3) 이 launch 가
모터를 안 띄워도 `/xycar_motor` 발행만으로 차가 실제로 달린다.

**라이다가 없으므로** `/scan` `/cone_cmd` 는 오지 않는다. 콘 구간 판정은 카메라가
하므로 영상에 콘이 나오면 구간으로는 들어가지만, `/cone_cmd` 가 없어 정지한다 —
차선추종만 검증된다는 뜻이다.

영상은 파일로 재생하면서 **라이다만 실물 실시간**으로 쓰려면 `lidar:=true` 를 준다
(라이다 드라이버 + `rubbercone_node` 를 같이 띄운다):

```bash
ros2 launch my_bringup replay.launch.py \
    video:='/home/e-on/테스트용(신호등미포함).mp4' lidar:=true
```

화면(영상 속 코스)과 라이다(실제 방)는 서로 다른 장면이라는 점만 기억할 것 —
구간 진입은 영상 쪽 콘 개수로 나고, 조향은 눈앞의 실물 콘에서 나온다.

#### 실측 (2026-08-18, 젯슨 Orin + `테스트용(신호등미포함).mp4` 632x480 29.97fps)

**GPU 가속은 실제로 걸려 있다.** torch 2.8.0 / `cuda_available=True` / device `Orin`,
모델 파라미터·검출 결과 모두 `cuda:0`. `perception_node` 는 `model.predict()` 에 device 를
지정하지 않는데, ultralytics 가 CUDA 를 자동 선택하므로 별도 설정이 필요 없다.

| 측정 | 값 |
|---|---|
| `video_pub_node` 발행 | 30.0 fps (영상 원래 속도) |
| **파이프라인 실측 (`perception_node`)** | **11~13 fps** — 영상 속도의 약 1/3 |
| YOLO `predict()` 단독 | 22 fps (45 ms/프레임) |
| `predict` + `extract`+`lane` | 16 fps (62 ms/프레임) |
| `publish_debug_image` on/off 차이 | **거의 없음** (둘 다 11~13 fps) |

즉 병목은 **YOLO 추론 45ms** 이고, 그 위에 마스크 후처리(`detect.extract` + 차선 폴리핏)가
**16ms 를 더 얹는다**(전체의 약 26%, 순수 CPU). 시각화 창은 병목이 아니다.
`stale_timeout_sec` 은 프레임 주기 ~0.09s 대비 0.5s 라 젯슨에서는 여유가 있지만,
인지가 느려지면(전력모드/`.pt` 폴백) 이 값을 늘려야 하는 이유가
여기서 재확인된다.

**TensorRT(`best5.engine`)는 실제로 빠르다 — 단 `task='segment'` 를 반드시 줘야 한다.**

| 로드 방식 | 순수 추론 | 실영상 전체 파이프라인 | 검출 결과 |
|---|---|---|---|
| `.pt` | 38.5 ms | 63.2 ms (15.8 fps) | dashed 71 / solid 85 / cone 33 |
| `.engine` (task 미지정) | 38.6 ms | — | — |
| `.engine` `task='segment'` | **12.9 ms** | **44.8 ms (22.3 fps)** | dashed 70 / solid 85 / cone 33 |

`task` 를 안 주면 ultralytics 가 `detect` 로 오인식해 **가속이 전혀 안 걸린다**(`.pt` 와
같은 38.6ms). 이 함정 때문에 처음엔 "이득 없음"으로 잘못 측정했다가 정정했다. 제대로
주면 순수 추론 **3배**, 전체 파이프라인 **1.4배**이고 검출 결과는 사실상 동일하다.

launch 기본값이 `.pt` 인 것은 **이식성** 때문이다 — `.engine` 은 빌드한 보드의
TensorRT/CUDA 버전에 종속돼 다른 장비로 복사할 수 없다(`.gitignore` 에도 들어 있다).
쓰려면 그 보드에서 직접 export 하고 `model_path:=.../best5.engine` 로 지정한다.

⚠️ 이 영상에는 신호등이 없는데도 로그에 `light=RED` 가 한 번 떴다. 투표 로직이 걸러야 할
오검출이 실제로 존재한다는 뜻이므로, 신호등 튜닝 시 이 영상으로 오검출률을 먼저 확인할 것.

### 주행 시각화 — 명령 한 줄로 두 창

```bash
# 이 한 줄이면 주행 스택 + 시각화 창 2개가 모두 뜬다
ros2 launch my_bringup drive.launch.py     rviz:=true      # 젯슨 차량
```

뜨는 창은 **두 개**다. 보는 축이 달라서 하나로 합칠 수 없다.

| 창 | 무엇을 보나 | 띄우는 노드 |
|---|---|---|
| `xycar pipeline` (OpenCV) | **카메라 시점.** 좌: YOLO 검출 박스 / 우: 차선 추정(변환 결과)·전방주시점, 하단 바에 판단(FSM)·계획·제어 텍스트 | `my_debug/pipeline_view_node` |
| RViz2 | **공간 시점(top-down).** 라이다 포인트클라우드, 차량 오도메트리, 지나온 궤적·예측 경로·기준 경로 | `my_debug/viz_node` + `rviz2 -d drive.rviz` |

`rviz:=true` 는 `perception_node` 의 `publish_debug_image` 도 자동으로 켠다
(그래야 좌/우 2분할 영상이 `/debug_image` 로 나온다). 카메라 창만 필요하면
`debug:=true` 만 줘도 된다.

**RViz2 설정 파일:** `my_debug/config/drive.rviz` — 아래 패널이 이미 등록돼 있다.

| 패널 | 토픽 | 내용 |
|---|---|---|
| Image | `/debug_image` | 좌 YOLO / 우 차선 2분할 (기본 켬) |
| Image | `/image_raw` | 원본 카메라 (기본 끔 — 필요할 때 체크) |
| LaserScan | `/scan` | 라이다 원시 스캔 |
| PointCloud2 | `/viz/scan_cloud` | `viz_node` 가 변환한 포인트클라우드 |
| Odometry | `/viz/odom` | 차량 위치·자세 (빨강 화살표) |
| Path | `/viz/driven_path` | 지나온 궤적 (흰색) |
| Path | `/viz/plan_path` | 현재 조향/속도 명령의 예측 경로 (초록) |
| Path | `/viz/ref_path` | 라바콘 복도 중앙선 = 추종 기준 (노랑) |
| MarkerArray | `/viz/markers` | FSM 상태 텍스트 / 전방 최근접 거리 / 차체 |

**오도메트리가 없어도 경로가 보이는 이유:** 이 차량은 기본 구성에 오도메트리가 없다
(SLAM 을 켜야 `/odom_rf2o` 가 생긴다). `viz_node` 는 `/odom` 이 1초간 안 오면
**발행한 조향/속도 명령을 자전거 모델로 적분해** `/viz/odom` 과 `odom→base_link` TF 를
직접 낸다. 추측항법이라 시간이 지나면 반드시 어긋난다 — 경로 *모양*을 보기 위한
것이지 측위가 아니다. 진짜 측위가 필요하면 SLAM 을 켜고, `drive_params.yaml` 의
`viz_node.odom_topic` 을 `/odom_rf2o` 로 바꾼다 (그러면 추측항법이 자동으로 꺼진다):

```bash
ros2 launch my_bringup drive.launch.py rviz:=true slam:=true
```

**튜닝값**은 전부 `my_bringup/config/drive_params.yaml` 의 `viz_node:` 섹션에 있다.
명령값(임의 단위) → 미터 환산(`speed_to_mps`, `max_steer_deg`)이 틀리면 예측 경로
길이만 어긋난다(주행에는 영향 없음). 예측 경로가 좌우 **반대**로 휘면 `angle_sign`
부호를 뒤집을 것.

⚠️ **시각화는 CPU 를 쓴다.** YOLO 추론이 이미 병목이므로,
기록 주행·실전에서는 `rviz`/`debug` 를 **둘 다 끄고**(기본값) 돌릴 것.

### 실차 연결 테스트 — `preflight.py` 부터

차량에 전원을 넣기 전에 **반드시** 이것부터 돌린다.

```bash
cd ~/xycar_ws && source install/setup.bash
python3 src/study/my_bringup/tools/preflight.py            # 정적 점검 (ROS 실행 불필요)
python3 src/study/my_bringup/tools/preflight.py --live     # 센서 launch 후 토픽 실측
```

한 번에 판정하는 것: ROS 환경 · 워크스페이스 빌드 여부와 **최신성** · CUDA/ultralytics ·
YOLO 가중치 · 카메라/라이다/VESC 장치 · udev 심볼릭 링크 · `vesc.yaml` 의 `port` 가 실제로
존재하는 경로인지 · `drive_params.yaml` 정합성. `--live` 는 여기에 `/image_raw` `/scan`
실측 Hz 와 **카메라 해상도 vs `driver_node.image_width` 일치 여부**를 더한다
(어긋나면 조향이 계통적으로 한쪽으로 치우치는데, 눈으로는 못 잡는 종류의 버그다).

실패 항목마다 `[조치]` 줄에 고치는 방법이 그대로 나온다. 실패가 하나라도 있으면
종료 코드 1 이고, **그 상태로 `/drive_enable` 을 켜지 않는다.**

전체 순서(연결 → 빌드 → 센서만 → 들어올리고 조향 → 저속 주행)와 증상별 대처는
**`study/VEHICLE_TEST.md`** 에 체크박스 형태로 정리돼 있다.

### 센서 udev 규칙 — 젯슨 보드마다 별도로 설정해야 함

**이건 git으로 안 옮겨진다.** `/etc/udev/rules.d/`는 리눅스 시스템 설정이라 저장소 밖이다.
코드(`vesc.yaml`의 `port: /dev/ttyMOTOR` 등)는 `git pull`로 그대로 받아지지만, 그 경로가
실제로 존재하려면 **이 젯슨 본체에 udev 규칙이 있어야 한다.** 새 젯슨으로 옮기거나
재설치(reflash)하면 아래를 다시 실행해야 한다. (반대로 코드는 안 고쳐도 된다 —
규칙이 `idVendor`/`idProduct`로 장치 종류를 식별하지, 이 보드 개체를 식별하지 않는다)

```bash
# VESC (ChibiOS 펌웨어, 0483:5740) -> /dev/ttyMOTOR
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyMOTOR"' | sudo tee /etc/udev/rules.d/99-vesc.rules

# YDLidar (CP2102, 10c4:ea60) -> /dev/ttyLIDAR
# serial 값은 라이다 개체마다 다를 수 있다. udevadm info -q property -n /dev/ttyUSB0 로 확인 후 맞출 것.
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyLIDAR"' | sudo tee /etc/udev/rules.d/99-ydlidar.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
```

적용 확인: `ls -l /dev/ttyMOTOR /dev/ttyLIDAR`

새 장치의 vendor:product ID를 모를 때:
```bash
lsusb                                              # 어떤 칩들이 꽂혀있는지
udevadm info -q property -n /dev/ttyACM0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_MODEL="
```

---

## 7. 실차 캘리브레이션 (미완료)

**전부 실물 차량이 있어야 하는 작업이고, 안 하면 동작이 틀어진다.**

| 항목 | 위치 | 왜 필요한가 |
|---|---|---|
| ~~wheelbase~~ | `xycar_motor/config/vesc.yaml` | ✅ **완료 (2026-08-16): 0.333 m** (실측 33.3cm) |
| ~~라이다 장착 위치~~ | `my_slam/launch/*.launch.py` | ✅ **완료 (2026-08-16): x=0.418, y=0.0, z=0.10**<br>(축거 33.3cm + 앞바퀴축 앞 8.5cm = 41.8cm, 차량 중앙, 지면에서 10cm) |
| **VESC 펌웨어 전압 컷오프** | 실기기 (`old_vesc_tool`) | `reference/docs/모터제어기_VESC_설정방법.pdf` 절차대로. 현재 백업(`2026_0617_vesc_Motor_cfg.xml`)은 PDF 기준과 다름 (`l_max_vin` 15V ↔ 30V 등). ROS 파라미터가 아니라 펌웨어에 굽는 값이라 코드로 못 고친다. |
| **VESC 시리얼 포트** | `xycar_motor/config/vesc.yaml` | UART 로 교체했다면 `/dev/ttyTHS1` 등으로 바꿔야 한다. 아래 별도 절 참고. |
| **`center_bias_px`** | `my_perception` 파라미터 | 카메라가 차량 중심선에서 벗어나 장착됐을 때의 보정. 트랙 중앙에 정지시켜 놓고 offset을 읽어 그 값을 넣으면 0이 목표가 된다. |
| **조향 캘리브레이션** | `xycar_motor/config/vesc.yaml` | `angle_offset`, `angle_bias_*`, `speed_bias_*`. ROS1에서 전부 0이었으므로 0에서 시작해 실주행으로 튜닝. |
| **카메라 노출** | `v4l2-ctl` | 기존값 `exposure_time_absolute=100`. **낮추면 신호등 LED 포화가 줄어 색이 살아날 가능성**이 있다. 신호등 신뢰도를 올릴 가장 값싼 개선안이므로 꼭 실험해볼 것. |

### VESC 를 UART 로 연결할 때 (2026-08-15 하드웨어 변경)

기존 5핀 연결이 고장나 **UART 직결로 교체**했다. 이 변경 때문에 코드를 한 군데 고쳤다.

**★ 코드 수정: 흐름제어 HARDWARE → NONE**

`vesc/vesc_driver/src/vesc_interface.cpp` 의 `connect()` 에서
`FlowControl::HARDWARE` 를 `NONE` 으로 바꿨다.

> VESC 의 COMM 포트와 젯슨 40핀 헤더 UART 는 **둘 다 TX/RX/GND 만 있고 RTS/CTS 핀이 없다.**
> 하드웨어 흐름제어를 켜두면 드라이버가 CTS 신호를 기다리다 송신이 막혀
> **모터가 전혀 반응하지 않는다.** USB(CDC-ACM)에서는 흐름제어가 무시되므로
> NONE 으로 둬도 USB 경로가 깨지지 않는다 — 두 경우 모두에서 안전하다.
>
> f1tenth/vesc 원본 대비 수정한 부분이므로 **재클론하면 다시 적용해야 한다.**

**포트 경로**

| 연결 방식 | 경로 |
|---|---|
| USB | `/dev/ttyACM0` (udev 로 `/dev/ttyMOTOR` 심볼릭 링크) |
| USB-UART 어댑터 | `/dev/ttyUSB0` |
| 젯슨 40핀 UART 직결 | `/dev/ttyTHS1` (JetPack 버전에 따라 `ttyTHS0`) |

**젯슨 UART 최대 함정 — nvgetty**

젯슨은 시리얼 콘솔 서비스(`nvgetty`)가 `/dev/ttyTHS*` 를 **기본으로 점유**한다.
끄지 않으면 포트가 열리지 않는다.

```bash
sudo systemctl stop nvgetty && sudo systemctl disable nvgetty
sudo usermod -aG dialout $USER      # 재로그인 필요
sudo reboot
```

**점검 스크립트**

```bash
bash (구) xycar_ws/src/xycar_motor/scripts/check_vesc_port.sh
```

포트 탐색 / nvgetty / 권한 / 점유 프로세스 / 실제 열기까지 확인하고,
문제가 있으면 해결 명령을 알려준다. 아무것도 바꾸지 않는다.

통신 확인:
```bash
ros2 launch xycar_motor xycar_motor.launch.py
ros2 topic echo /sensors/core      # 전압/온도가 올라오면 성공
```

---

## 8. 진행 상황

### ✅ 완료 — 1단계 코드 전체

- **모터 ROS2 이식** — `xycar_motor` 노드 신규 작성(ROS1 `xycar_motor.py`의 캘리브레이션·
  속도램핑 로직 이식), `f1tenth/vesc` ros2 브랜치 도입, ERPM/전압을 조직위 PDF 기준
  (`-1000~10000`)으로 반영
- **인식 성능 분석** — `reference/perception_analysis.md`
- **아키텍처 확정** — 노드 구성, 토픽 계약, FSM 범위, 단계적 융합 전략
- **패키지 5개** — `my_perception` / `my_obstacle` / `my_driver` / `my_slam` / `my_bringup`
- **인식 코어** — `lane.py`, `light_vote.py`, `detect.py` + 영상 검증 완료
- **판단·제어** — `fsm.py`, `lateral.py`, `longitudinal.py`, `control.py` + 폐루프 시뮬 검증
- **SLAM 설정** — 매핑/측위 launch + config 4종 (기존 `shortcut_slam` 정리·확장)
- **2단계 준비물** — `record_waypoints`(기록), `clean_waypoints.py`(평활화+곡률 계산)
- **통합 파라미터** — `my_bringup/config/drive_params.yaml` 한 파일로 전부 튜닝
- **검증 도구** — `offline_check.py`(인식), `sim_check.py`(제어)

전체 46개 Python 파일 문법 검사 + 5개 YAML 파싱 검사 통과.

### 🔧 하드웨어 현황 (2026-08-15)

| 항목 | 상태 |
|---|---|
| 젯슨 (reComputer Super J4012) | 확보 |
| 카메라 / 라이다 / IMU | **팀원이 젯슨에서 작동 확인 완료** |
| VESC | 5핀 연결 고장 → UART 로 교체. **케이블 주문, 8/16 작업 예정** |

센서 3종이 확인됐으므로 **모터를 기다리지 않고 인지 노드부터 붙여볼 수 있다:**

```bash
ros2 launch my_bringup drive.launch.py motor:=false
# 또는 인지만
ros2 launch my_perception perception.launch.py
ros2 topic echo /lane
```

여기서 확인할 것 두 가지:
1. **추론 FPS** — perception 로그에 찍힌다. 30fps 미달이면 TensorRT 변환 검토
2. **`image_width`** — 632 로 잡아뒀다. 젯슨 카메라 설정이 다르면 반드시 맞출 것.
   차선 오프셋이 화면 중심 기준이라 폭이 틀리면 조향이 계통적으로 치우친다.

### 🐛 첫 빌드 전에 미리 잡아둔 문제들 (2026-08-15)

젯슨에서 시간을 낭비하지 않도록 코드를 다시 훑어 네 가지를 선제 수정했다.

| 문제 | 증상이었을 것 | 조치 |
|---|---|---|
| **VESC 흐름제어 HARDWARE** | UART 직결 시 **모터 무반응** (CTS 대기로 송신 차단) | `vesc_interface.cpp` → `FlowControl::NONE` |
| **`rf2o_laser_odometry` 가 워크스페이스 밖** | `my_slam` 빌드 실패 (apt 에 없는 패키지) | `Desktop/` → `xycar_ws/src/` 로 이동 |
| **rf2o package.xml 이 format 1** | `rosdep install` 이 `cmake_modules` 해석 실패 | format 3 으로 이관, ROS1 전용 키 제거 |
| **`light.miss_tolerance` 미선언** | **perception 노드가 시작 시 예외로 사망** | 노드에 선언 추가 + 검사 도구 상시화 |

추가로 `drive_params.yaml` 을 ROS2 정식 **중첩 형식**으로 바꾸고
(평면 점표기는 파서 해석이 불확실), `python3-opencv` 를 의존성에서 뺐다
(JetPack 의 CUDA OpenCV 와 충돌 위험).

### ⚠️ 다만 — "코드 완성"이지 "동작 검증"이 아니다

- **빌드 미검증.** Windows에 ROS2가 없어 `colcon build` 를 못 돌렸다.
  import 누락·의존성 문제는 젯슨 첫 빌드에서 드러난다. 에러가 몇 개 나오는 건 정상이다.
- **게인 미확정.** `k_lat`, `k_curve`, `k_damp` 는 시뮬레이션에서 구조적 안정성만
  확인했다. 절대값은 실차에서 잡아야 한다 (절차는 `drive_params.yaml` 주석 참고).
- **실측값 2개가 여전히 블로커** — wheelbase, 라이다 장착위치.

### 🔄 미착수

| 작업 | 상태 |
|---|---|
| 젯슨 빌드 + 에러 수정 | 보드 준비 후 |
| 실차 캘리브레이션 (7장) | 보드 + 차량 |
| 2단계: 지도 생성 → waypoint → 속도계획 연동 | 1단계 주행 안정화 후 |
| 지름길 분기 판단 (`fsm.enable_shortcut`) | 판단 근거 확정 후 |
| 주차 경기 (pure pursuit + 측위) | 별도 |

---

## 9. 이 프로젝트에서 하지 말 것 (검증으로 걸러진 오답)

작업을 이어받는 사람이 다시 시도하기 쉬운 것들:

1. **HSV 색상으로 신호등 판별** — LED가 흰색 포화라 불가능. 실측으로 확인됨.
2. **차선 bbox 사용** — 대각선이라 무의미. mask를 쓸 것.
3. **차선을 특정 행에서 샘플링** — 점선이 끊겨 결측된다. 곡선 피팅할 것.
4. **좌/우 차로 중심을 목표로 삼기** — 목표는 트랙 중앙이다.
5. ~~**라바콘 슬라럼 회피 로직** — 라바콘은 차선을 안 가린다~~
   ⚠️ **이 판단은 2026-08-17 철회됨.** 사진 판독 오류였다. 실제로는 콘이 좌우로
   촘촘히 늘어서 **복도(벽)** 를 이루고, **우측 콘 벽이 흰 실선보다 안쪽**에 있다.
   페인트 차선 중심을 따라가면 콘을 친다. 라이다 기반 복도 중앙 추종이 필요하다.
   → `reference/tuning_guide.md` §4-1 참고
6. **신호등 단일 프레임 판독** — 출발 위치에서 68%. 투표 필수.
7. **`build/install/log` 를 젯슨으로 복사** — 아키텍처가 달라 무의미. 소스 빌드할 것.

---

## 10. 참고 자료

| 자료 | 위치 |
|---|---|
| **파라미터 튜닝 가이드 / 수정 예정 사항** | **`reference/tuning_guide.md`** |
| 인식 성능 실측 분석 | `reference/perception_analysis.md` |
| VESC 설정 절차 (조직위) | `reference/docs/모터제어기_VESC_설정방법.pdf` |
| VESC 펌웨어·설정 백업 | `reference/vesc/` |
| 차선 튜닝용 주행영상 | `reference/videos/` |
| ROS1 원본 (모터 도커용) | `motor_ros1_bundle.zip` (또는 `amd.zip` 안 `noetic_ws/src/`) |
| 모터 패키지 상세 | `(구) xycar_ws/src/xycar_motor/README.md` |
| f1tenth VESC 드라이버 | <https://github.com/f1tenth/vesc/tree/ros2> |
