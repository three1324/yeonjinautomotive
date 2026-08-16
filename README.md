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
- 라바콘은 차선을 가리지 않고 트랙 경계 바깥/근처에 배치된다 →
  **별도 회피 기동이 아니라 "트랙 중앙 유지"가 이 구간의 과제**

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
├── amd.zip                   원본 차량 홈디렉토리 백업 (복구용, 건드리지 말 것)
├── amd/                      amd.zip에서 필요한 것만 추린 작업 트리
│   ├── xycar_ws/src/         ★ 메인 colcon 워크스페이스
│   │   ├── study/            ★ 우리가 새로 작성하는 패키지들
│   │   │   ├── my_perception/   카메라 인식 (YOLO) + models/best5.pt
│   │   │   │                    tools/offline_check.py (영상으로 검증)
│   │   │   ├── my_obstacle/     라이다 섹터 거리
│   │   │   ├── my_driver/       판단(FSM)·제어
│   │   │   │                    tools/sim_check.py (폐루프 시뮬)
│   │   │   ├── my_slam/         매핑/측위 + waypoint 도구
│   │   │   └── my_bringup/      ★ 통합 launch + config/drive_params.yaml
│   │   │                        (모든 튜닝 파라미터가 여기 한 파일에)
│   │   ├── xycar_motor/      ★ VESC 모터 노드 (ROS1→ROS2 이식 완료)
│   │   ├── vesc/             f1tenth/vesc (ros2 브랜치) — 그대로 사용
│   │   ├── xycar_device/     센서 드라이버 (cam/lidar/imu/ultrasonic/msgs)
│   │   └── yolo_ros/         YOLO ROS 래퍼 (기존)
│   ├── noetic_ws/src/        ROS1 원본 (이식 참고용, 빌드 대상 아님)
│   │   └── vesc, xycar_motor, my_motor
│   └── Desktop/              센서·시뮬 참고 소스
│       ├── sllidar_ros2, rf2o_laser_odometry, shortcut_slam, xycar_simulator
└── reference/                문서·자료 (빌드 대상 아님)
    ├── perception_analysis.md   ★ 인식 성능 실측 분석 (설계 근거의 원천)
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
| `/lane` | `Float32MultiArray` | `[offset_near, offset_far, valid, quality]` |
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

- **라바콘**: 상태 전환 없음. 트랙 중앙 유지 + 안전마진만.
- **추월**: `LANE_DRIVE` 안의 일시적 서브행동. 끝나면 자동 복귀.
- **SLAM 위치는 보조(advisory)**, 차선추종이 주(primary).
  SLAM이 틀어져도 주행은 차선만으로 계속되어야 한다.

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
cd amd/xycar_ws/src/study/my_perception
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
cd amd/xycar_ws/src/study/my_bringup
python3 tools/check_params.py
```

`drive_params.yaml` 의 키와 노드의 `declare_parameters()` 를 대조한다.
**선언하지 않은 파라미터가 yaml 에 있으면 노드가 시작 시 예외로 죽기 때문에**,
파라미터를 추가·개명한 뒤에는 반드시 돌려볼 것. (실제로 `light.miss_tolerance`
누락을 이 검사가 잡아냈다.)

### 제어 폐루프 시뮬레이션 (Windows에서, ROS 없이)

`my_driver` 의 판단·제어 모듈도 ROS 의존성이 없어서 폐루프로 돌려볼 수 있다.

```bash
cd amd/xycar_ws/src/study/my_driver
python3 tools/sim_check.py
python3 tools/sim_check.py --k-lat 0.15 --k-curve 0.30
```

시나리오: 직선 복귀 / 곡선 추종 / 차선 결측 / 추월. 각각 최종오차·진동폭·조향포화율을 낸다.

**확인 가능**: 발산 여부, 진동, 곡선 정상상태 오차, 조향 포화, 결측·추월 로직
**확인 불가**: **실제 게인 값**. 차량 모델의 픽셀↔운동 변환 상수가 임의값이라
절대적 튜닝값은 줄 수 없다. 게인 확정은 실차에서만 가능하다.

### 젯슨으로 옮기는 순서

1. `amd/xycar_ws/` 를 보드로 복사 (`build/ install/ log/` 는 제외)
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
   ros2 topic pub --once /drive_enable std_msgs/Bool '{data: true}'
   ```

### 실행 명령 요약

| 목적 | 명령 |
|---|---|
| 전체 주행 | `ros2 launch my_bringup drive.launch.py` |
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
bash amd/xycar_ws/src/xycar_motor/scripts/check_vesc_port.sh
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
5. **라바콘 슬라럼 회피 로직** — 라바콘은 차선을 안 가린다. 트랙 중앙 유지면 충분.
6. **신호등 단일 프레임 판독** — 출발 위치에서 68%. 투표 필수.
7. **`build/install/log` 를 젯슨으로 복사** — 아키텍처가 달라 무의미. 소스 빌드할 것.

---

## 10. 참고 자료

| 자료 | 위치 |
|---|---|
| 인식 성능 실측 분석 | `reference/perception_analysis.md` |
| VESC 설정 절차 (조직위) | `reference/docs/모터제어기_VESC_설정방법.pdf` |
| VESC 펌웨어·설정 백업 | `reference/vesc/` |
| 차선 튜닝용 주행영상 | `reference/videos/` |
| ROS1 원본 (이식 대조용) | `amd/noetic_ws/src/` |
| 모터 패키지 상세 | `amd/xycar_ws/src/xycar_motor/README.md` |
| f1tenth VESC 드라이버 | <https://github.com/f1tenth/vesc/tree/ros2> |
