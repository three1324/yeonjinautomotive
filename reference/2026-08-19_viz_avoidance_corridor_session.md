# 2026-08-19 작업 로그 — 주행 시각화 · 영상 재생 테스트 · 방해차량 회피 · 콘 복도 추정

전날(`2026-08-18_gpu_pipeline_session.md`)에 이어짐. 이 문서만 읽으면 다음 세션을
바로 이어갈 수 있도록 **무엇을 왜 바꿨고, 무엇이 아직 검증 안 됐는지**를 남긴다.

---

## 0. 워크스페이스 이름 변경

`xycar_ws_amd/` → `xycar_ws/` (`git mv`, 히스토리 유지). `README.md`, `VENDORED.md`
의 참조도 함께 수정. 메인 워크스페이스는 여전히 `amd/xycar_ws/` 이고, 두 트리의
`study/` 는 **동일하게 유지**한다(한쪽만 고치면 갈라진다).

---

## 1. 주행 시각화 — 명령 한 줄로 두 창

```bash
ros2 launch my_bringup drive_amd.launch.py rviz:=true    # AMD
ros2 launch my_bringup drive.launch.py     rviz:=true    # 젯슨
```

보는 축이 달라 창이 두 개다.

| 창 | 시점 | 담당 |
|---|---|---|
| `xycar pipeline` (OpenCV) | 카메라 | `my_debug/pipeline_view_node` (기존) |
| RViz2 | 공간(top-down) | `my_debug/viz_node` (신규) + `config/drive.rviz` |

**신규 `viz_node`** 가 내는 것: `/viz/scan_cloud`(PointCloud2), `/viz/plan_path`
(현재 명령의 자전거모델 예측 경로), `/viz/driven_path`, `/viz/ref_path`,
`/viz/odom`, `/viz/markers`.

**오도메트리가 없어도 경로가 보이는 이유**: `/odom` 이 1초간 없으면 발행한 조향/속도
명령을 자전거 모델로 적분해 `/viz/odom` 과 `odom→base_link` TF 를 **직접 낸다**.
추측항법이라 누적 오차가 있다 — 경로 *모양*을 보기 위한 것이지 측위가 아니다.
SLAM 을 켜면 `viz_node.odom_topic` 을 `/odom_rf2o` 로 바꾸면 자동으로 꺼진다.

`rviz:=true` 는 `OrSubstitution` 으로 `publish_debug_image` 도 같이 켠다.

---

## 2. 영상 파일 재생 테스트 (`replay.launch.py`)

```bash
ros2 launch my_bringup replay.launch.py video:=~/테스트영상.mp4 \
    enable:=true start_frame:=2350 rate_scale:=0.5
```

`my_debug/video_pub_node` 가 영상을 **원래 fps 로 실시간** 재생해 `/image_raw` 로
내보낸다. 그 뒤로는 실차와 같은 노드·같은 파라미터가 돈다. 카메라·라이다·모터
드라이버는 띄우지 않는다.

`offline_check.py` 와의 차이: 그쪽은 인지 모듈만 최대 속도로 돌린다(정확도 검증).
이쪽은 실시간이라 **보드가 30fps 를 따라가는지**가 드러난다 — AMD 차량에서 "가다
멈추고"를 일으킨 바로 그 문제.

**replay 전용으로 뒤집은 기본값** (실차 launch 는 안 건드림):
- `auto_start:=true` — 테스트 영상엔 보통 신호등이 없어 `WAIT_LIGHT` 에서 멈춘다
- `lidar_confirm:=false` — 라이다가 없어 `front_dist=99.0` 이라 회피가 발동 못 함

### 실측 (젯슨 Orin, `테스트용(신호등미포함).mp4` 632x480 29.97fps)

| 측정 | 값 |
|---|---|
| 영상 발행 | 30.0 fps |
| 파이프라인(`perception_node`) | **11~13 fps** (영상의 1/3) |
| YOLO `predict()` 단독 | 22 fps (45 ms) |
| `predict` + `extract`+`lane` | 16 fps (62 ms) |
| `publish_debug_image` on/off | **차이 거의 없음** |

병목은 YOLO 추론이고, 마스크 후처리가 26% 를 더 얹는다. **시각화 창은 병목이 아니다**
(예상과 달랐음).

---

## 3. ⚠️ 전날 결론 정정 — TensorRT 는 실제로 빠르다

**전날 로그(2배 가속)가 맞았고, 오늘 처음 잰 "이득 없음"이 틀렸다.**
원인은 `YOLO(...)` 로드 시 `task='segment'` 를 안 준 것.

| 로드 방식 | 순수 추론 | 실영상 전체 파이프라인 | 검출 결과 |
|---|---|---|---|
| `.pt` | 38.5 ms | 63.2 ms (15.8 fps) | dashed 71 / solid 85 / cone 33 |
| `.engine` (task 미지정) | 38.6 ms | — | — |
| `.engine` `task='segment'` | **12.9 ms** | **44.8 ms (22.3 fps)** | dashed 70 / solid 85 / cone 33 |

task 를 안 주면 ultralytics 가 `detect` 로 오인식해 **가속이 전혀 안 걸린다**(`.pt` 와
동일). 제대로 주면 순수 추론 3배, 전체 1.4배이고 검출 결과는 동일하다.
README 와 `preflight.py` 의 잘못된 기술을 정정했다.

launch 기본값이 `.pt` 인 이유는 성능이 아니라 **이식성**이다(`.engine` 은 빌드 보드 종속).

---

## 4. 실차 사전 점검 도구 (`preflight.py` + `VEHICLE_TEST.md`)

```bash
python3 src/study/my_bringup/tools/preflight.py          # 정적 점검
python3 src/study/my_bringup/tools/preflight.py --live   # 토픽 실측
```

판정 항목: ROS 환경 · 워크스페이스 빌드 여부와 **최신성** · CUDA/ultralytics ·
YOLO 가중치 · 카메라/라이다/VESC 장치 · udev 심볼릭 링크 · **`ydlidar.yaml`/`vesc.yaml`
의 port 가 실재하는지** · `drive_params.yaml` 정합성. `--live` 는 `/image_raw` `/scan`
실측 Hz + **카메라 해상도 vs `driver_node.image_width` 일치**까지.

실패마다 `[조치]` 줄에 고치는 명령이 나오고, 실패가 있으면 종료 코드 1.

`VEHICLE_TEST.md` 는 연결 → 빌드 → 센서만 → **라이다(§3-1)** → 들어올리고 조향 →
저속 주행 순의 체크박스 문서.

---

## 5. 라이다 문제 원인 규명 (로그 근거)

`~/.ros/log/xycar_lidar_node_*.log` 를 전수 조사한 결과:

| 시각 | 결과 |
|---|---|
| 08-15 15:19 | **정상** (`/scan` 구독자까지 붙음) |
| 08-16 16:30 | `Unknown error` |
| 08-16 16:33 | 정상 (3분 뒤) |
| 08-18 16:29 / 16:49 / 16:50 | `Unknown error` (실차 세션, 3회 모두) |

`Unknown error` = SDK `CYdLidar::initialize()` → `checkConnect()` →
`connect(port, baudrate)` 실패, 즉 **시리얼 포트를 못 연 것**. 그러면
`xycar_lidar_node.cpp` 의 `while (ret && rclcpp::ok())` 스캔 루프에 **아예 들어가지
않고** 프로세스만 살아 있다 → "노드는 떠 있는데 `/scan` 이 없다"는 조용한 실패.

`ydlidar.yaml` 의 `port: /dev/ttyLIDAR` 는 **udev 규칙이 없으면 경로 자체가 안 생긴다.**
08-15 에 정상이었으므로 **코드 문제가 아니라 연결/포트 문제**다.

**별개의 함정**: `/scan` 은 BEST_EFFORT 로 발행되는데 `ros2 topic echo` 기본값은
RELIABLE 이라 **한 줄도 안 받는다.** 08-15 로그에 그 QoS 경고가 실제로 남아 있다.
→ `ros2 topic echo /scan --qos-reliability best_effort`

**참고(미수정)**: `xycar_lidar.launch.py` 는 `LifecycleNode` 액션으로 띄우지만 실제
노드는 평범한 `rclcpp::Node` 다. 전이를 emit 하지 않아 지금은 무해하나 `ros2 lifecycle`
로는 제어 안 된다. 벤더 패키지라 손대지 않았다.

---

## 6. 방해차량 회피 (P1~P4)

### 핵심 변경 — 회피량을 "트랙 반쪽의 중앙"으로

```
왼쪽 반 중앙   = (왼쪽 흰실선 + 노란선) / 2 = 트랙중앙 − 반폭/2
오른쪽 반 중앙 = (노란선 + 오른쪽 흰실선) / 2 = 트랙중앙 + 반폭/2
```

`LaneEstimator` 가 좌우 흰선을 동시에 볼 때마다 **행별 반폭을 EMA 로 이미 학습**하고
있었다. 그 값을 `/lane` 에 실어 보내 회피량으로 쓴다(4개 → 6개 필드로 확장,
`len>=6` 로 하위호환). 고정 픽셀과 달리 원근·트랙폭이 자동 반영되고, 목표가 항상 트랙
안쪽이라 **실선을 넘는 상황이 구조적으로 생기지 않는다.** 미학습이면 `shift_px` 폴백
(사용자 결정).

### P2 — PASS 종료를 시간이 아니라 관측으로

`car gone` / `car receding` / `car at edge` / `front clear`(라이다 쓸 때만) /
`pass timeout`(안전 상한).

### P3 — 회피 중 감속 `speed.overtake_factor: 0.7`

옆으로 벌리면 전방 장애물이 라이다 섹터에서 빠져 `front_dist` 가 커지고, 장애물 상한이
풀려 **가장 위험한 순간에 오히려 가속**한다. 버그에 가까웠다.

### P4 — `cooldown_sec: 1.0` (복귀 직후 재발동 → 지그재그 방지)

### 구현 중 실측으로 잡은 것 두 가지

1. **`front clear` 조기복귀 버그** — 라이다 없이 돌리면 `front_dist=99.0` 이라 이 조건이
   기동 첫 tick 에 즉시 참이 되어 벌리자마자 복귀했다. `require_lidar_confirm` 일 때만
   보도록 수정.
2. **`car at edge` 조건 추가** — 옆을 스쳐가는 차는 cx 440→612 로 밀려나면서도 bottom_y
   는 302→439 로 **커진다**. `car receding` 이 안 걸려 timeout 까지 벌린 채 달렸다.

### 검증 (실영상 `테스트용(신호등미포함).mp4`)

방해차량은 643프레임 샘플 중 110프레임(약 17%)에 나온다.

```
f2400  IDLE    target=  +0.0px  half=376
f2741  SHIFT   ← 트리거. cx=440(오른쪽) → 왼쪽으로, 183px = half(367)/2
f2766  PASS    target=-183.3px
f2798  RETURN  ← "car at edge(cx593)"     ※ 이 조건 없으면 f2811 timeout
f2829  IDLE    ← 복귀 완료, 쿨다운
```

ROS 통합에서도 `ov=- → SHIFT → PASS → RETURN`, 속도 12.0 → 8.4 확인.

---

## 7. 라바콘 S자 구간 — 중앙선 추정 사다리

### 사용자 결정

- 전환은 **연속 혼합 유지**(하드 스위치 아님). 근거: 딱 끊으면 목표가 120px 점프(실측).
- 평상시 주행 기준은 **트랙 중앙 유지**.

### `corridor.py` 사다리 (`lane.py` 와 대칭)

| 순서 | 방식 | quality |
|---|---|---|
| 1 | **walls** — 좌/우 벽을 각각 폴리핏 후 두 곡선의 중점 | 1.0 |
| 2 | **refine** — 1차 추정 곡선에 **수직**으로 좌우 벽 재측정 | 0.8 |
| 3 | **bins** — 구간별 중점을 바로 피팅 (종전) | 0.4~ |

**왜**: 종전은 x축 수직 슬라이스로 벽을 찾는데, 복도가 굽으면 슬라이스가 복도를 비스듬히
자른다 → 중점이 곡선 안쪽으로 편향. S자가 정확히 그 구간. `SteeringController` 가
`offset_far − offset_near` 를 곡률 선행보상으로 쓰므로 far 과소평가는 곧 선행보상 부족.

### 실측 (합성 S자 코스를 따라 전진, 15지점)

| 코스 | 종전 far 오차 | 사다리 후 |
|---|---|---|
| 완만 S (0.25·sin1.6x) | 91.7px | **31.2px** |
| 중간 S (0.30·sin2.0x) | 148.7px | **97.4px** |

**측정으로 폐기한 후보** (기록해두지 않으면 또 시도하게 된다):

| 후보 | 결과 |
|---|---|
| 3차 다항식 | 2차 77.0px vs 3차 85.9px — 오히려 나쁨 |
| 관심영역 2.2m → 3.5m | 226 vs 228px — 효과 없음 |
| `wall_min_bins` 2~3 | 4가 최선. 3 이하는 점3개=계수3개라 보간이 되어 노이즈를 탐 |
| refine 밴드 1.5~2배 | 97 → 169 → 192px — 반대편 벽 점이 빨려들어와 크게 악화 |

### 시각화 (B·C·D)

`obstacle_node` 가 `publish_viz: true` 일 때:
- `/corridor_path` (`nav_msgs/Path`) — 복도 중앙선
- `/cone_walls` (MarkerArray) — **벽으로 채택된 콘 점**(좌 초록/우 빨강) + 중앙선

`viz_node` 는 `/viz/markers` 에 **지금 따르는 기준을 색으로**: 노랑=lane,
주황=blend, 빨강=corridor. 로그에는 `[walls]`/`[refine]`/`[bins]` 가 찍힌다.

**제어 계약은 그대로 `/corridor` 픽셀 오프셋**이다. 경로 추종으로 바꾸면 차선 추종과
제어기가 갈라지는데, 지금 구조의 핵심 장점이 "제어기 하나를 공유한다"는 것이라서.

---

## 8. ⚠️ 아직 검증 안 된 것 — 다음 세션에서 할 일

1. **`drive.launch.py` / `drive_amd.launch.py` 는 끝까지 실행된 적이 없다.**
   센서·모터가 없어 인자 파싱까지만 확인했다.
2. **라이다 복도 튜닝값은 전부 합성 데이터 기반이다.** 실제 콘 간격·복도 폭·반사
   특성은 모른다. → **콘 구간 rosbag 을 반드시 딸 것. 절차는 `VEHICLE_TEST.md` §9**
   (녹화 명령·주행 방법·저장 포맷 주의·녹화 직후 확인·`corridor_sim.py --bag` 재튜닝).
   특히 `corridor.px_per_meter`(현재 300.0, 가정값)를 먼저 확정해야 한다 — 틀리면
   라이다 복도와 카메라 차선의 단위가 안 맞아 융합이 통째로 어긋난다.
3. **회피 기동을 실차에서 `require_lidar_confirm: true` 로 돌려본 적이 없다.**
   영상 검증은 그 조건을 끈 상태였다.
4. **라이다 포트 문제**(§5) — udev 규칙 등록부터.
5. `vesc.yaml` 의 `port: /dev/ttyUSB0` 가 실제 연결 방식과 맞는지 확인.

순서와 체크박스는 `study/VEHICLE_TEST.md` 에 있다.

---

## 9. 추가/변경된 파일

**신규**
```
my_debug/my_debug/viz_node.py          RViz 피더
my_debug/my_debug/video_pub_node.py    영상 -> /image_raw 실시간 재생
my_debug/config/drive.rviz             RViz 설정 (모든 패널 등록됨)
my_debug/launch/viz.launch.py          rviz2 + viz_node + 뷰어 + 정적 TF
my_bringup/launch/replay.launch.py     영상 재생 통합 launch
my_bringup/tools/preflight.py          실차 사전 점검
study/VEHICLE_TEST.md                  실차 연결 테스트 체크리스트
```

**수정**
```
my_perception/lane.py              LaneResult 에 half_near/half_far
my_perception/perception_node.py   /lane 4개 -> 6개
my_driver/lateral.py               회피량 = 반폭/2, PASS 관측종료, 쿨다운
my_driver/longitudinal.py          overtake_factor
my_driver/driver_node.py           반폭 수신, 회피 진단 필드, 파라미터
my_obstacle/corridor.py            중앙선 추정 사다리 + 경로/벽점 출력
my_obstacle/obstacle_node.py       /corridor_path, /cone_walls
my_debug/pipeline_view_node.py     target_off + 회피 표시
my_bringup/config/drive_params.yaml  viz_node 섹션, 회피/복도 파라미터
my_bringup/tools/check_params.py   viz_node 등록
README.md                          §5-3-1 회피, §5-3-2 콘 복도, 시각화/재생/사전점검
```

두 워크스페이스(`amd/xycar_ws`, `xycar_ws`)의 `study/` 에 동일하게 반영.
