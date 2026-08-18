# AMD 보드 xycar(서울대 대회 차량)에 배포하기

이 `study/` 폴더를 **통째로 복사**하면 그대로 돌아간다. 코드는 젯슨(국민대 대회)용과
**완전히 동일한 파일**이고, 차량별 차이는 launch 하나 + 오버라이드 yaml 하나로만 흡수했다.

> 왜 이렇게 했나: 두 대회가 같은 주에 붙어 있어 코드를 복사해 두 벌로 갈라놓으면
> 한쪽 버그 수정이 다른 쪽에 반영되지 않는다. 그래서 **소스는 한 벌**로 두고
> 진입점만 나눴다.

---

## 1. 이 차량이 젯슨 차량과 다른 점 — 딱 두 가지

| | 젯슨 차량 (국민대) | **AMD 차량 (서울대)** |
|---|---|---|
| ROS | ROS2 Humble | ROS2 Humble (동일) |
| 카메라 토픽 | `/image_raw` | `/image_raw` (동일) |
| 라이다 토픽 | `/scan` | `/scan` (동일) |
| 해상도 | 640×480 | 640×480 (동일) |
| 모터 토픽 | `xycar_motor` `[angle, speed]`, angle ±50 | **동일** |
| **모터 드라이버** | 우리 `xycar_motor` 패키지 + vesc 를 직접 띄움 | **별도 ROS1 도커의 벤더 노드가 상시 구독 중** → 우리는 발행만 |
| **speed 실효 배율** | VESC ERPM 매핑 (base 12.0) | 벤더 변환식 (base 4.8 로 하향) |
| GPU | CUDA (Jetson) | **없음** (Radeon) → YOLO 는 CPU 추론 |

토픽 계약이 전부 같은 것은 우연이 아니다. 서울대 차량에서 돌던 `race_manager` 도
똑같이 `xycar_motor` 에 `Float32MultiArray [angle, speed]` 를 발행한다.
그래서 **주행 코드 자체는 한 줄도 고치지 않았다.**

---

## 2. 복사

```bash
# PC 에서 (경로는 각자 환경에 맞게)
scp -r study/ <차량계정>@<차량IP>:~/xycar_ws/src/
```

USB 로 옮긴다면 `study` 폴더를 `~/xycar_ws/src/study` 위치에 그대로 두면 된다.
(서울대 대회 때 `race_*` 를 넣던 바로 그 자리다.)

포함된 패키지 6개:

| 패키지 | 역할 | 이 차량에서 |
|---|---|---|
| `my_perception` | 카메라 인식 (YOLO11n-seg 1회 추론 → 차선/신호등/객체) | ✅ 사용 |
| `my_obstacle` | 라이다 섹터 거리 + 라바콘 복도 추정 | ✅ 사용 |
| **`my_driver`** | **주행 판단(FSM) + 횡·종방향 제어** | ✅ 사용 |
| `my_bringup` | 통합 launch + 파라미터 | ✅ 사용 |
| `my_slam` | SLAM 매핑/측위 | ⚠️ **동봉만. 기본 비활성** (§5 참고) |
| `my_debug` | 파이프라인 시각화 뷰어 (`debug:=true` 일 때만) | ⚠️ 디버깅할 때만 켤 것 (§9 참고) |

---

## 3. 사전 준비 (도커 안에서 1회)

```bash
python3 -c "import ultralytics, cv_bridge; print('ok')"
```

`ultralytics` 가 없으면:

```bash
pip install ultralytics
```

이 차량은 **CUDA 가 없으므로** PyPI 의 일반 CPU 휠이면 된다 (젯슨처럼 전용 휠이
필요하지 않다). `my_perception` 은 device 를 지정하지 않아 ultralytics 가 알아서
CPU 를 잡는다 — 코드 수정 불필요.

---

## 4. 빌드

```bash
cd ~/xycar_ws
colcon build --symlink-install --packages-select \
    my_perception my_obstacle my_driver my_bringup my_slam my_debug
source install/setup.bash
```

---

## 5. 실행

```bash
ros2 launch my_bringup drive_amd.launch.py
```

**차는 아직 안 움직인다** (`require_enable: true` 안전장치). 출발시키려면:

```bash
ros2 topic pub --once /drive_enable std_msgs/msg/Bool '{data: true}'
```

### ⚠️ 반드시 지킬 것 두 가지

1. **`race_*` 런치와 절대 동시에 띄우지 말 것.**
   `race_manager` 도 `xycar_motor` 를 발행한다. 둘이 같이 뜨면 서로 다른 명령이
   같은 토픽에 섞여 차가 요동친다. 한 번에 하나만.

2. **`drive.launch.py` (젯슨용) 를 쓰지 말 것.**
   그쪽은 `xycar_motor` 패키지를 include 하는데 이 차량엔 그 패키지가 없어서
   `FindPackageShare` 단계에서 launch 가 죽는다. 반드시 `drive_amd.launch.py`.

### 자주 쓰는 인자

| 목적 | 명령 |
|---|---|
| 센서 없이 (다른 데서 이미 띄웠을 때) | `ros2 launch my_bringup drive_amd.launch.py sensors:=false` |
| 파라미터 바꿔서 | `... drive_amd.launch.py params_file:=/경로/내파일.yaml` |
| 인지만 (디버깅) | `ros2 launch my_perception perception.launch.py` |

`my_slam` 은 기본 off 다. `slam:=true` 는 도커 안에 `slam_toolbox`,
`rf2o_laser_odometry`, `robot_localization` 이 **실제로 있을 때만** 켤 것.
없는 상태로 켜면 launch 가 죽는다. 확인:

```bash
ros2 pkg list | grep -E "slam_toolbox|rf2o|robot_localization"
```

---

## 6. 파라미터를 어디서 고치나 — 이 규칙을 지킬 것

```
config/drive_params.yaml   ← 알고리즘 튜닝값 전부. 두 차량이 공유한다.
config/amd_overrides.yaml  ← 이 차량이라서 달라지는 값만. drive_params 위에 덮인다.
```

- 차선 피팅, 신호등 투표, 조향 게인처럼 **차량과 무관한 값** → `drive_params.yaml`
- 속도 스케일처럼 **이 차량이라서 다른 값** → `amd_overrides.yaml`

`amd_overrides.yaml` 에 튜닝값을 통째로 복사해오면 그 순간부터 두 파일이 갈라져
관리가 안 된다. **덮어야 할 것만 덮을 것.**

파라미터를 추가하거나 이름을 바꾼 뒤에는 반드시:

```bash
python3 my_bringup/tools/check_params.py
```

선언하지 않은 파라미터가 yaml 에 있으면 **노드가 시작하자마자 예외로 죽는다.**

---

## 7. 실차에서 확인할 것 (순서대로)

| # | 항목 | 방법 | 안 하면 |
|---|---|---|---|
| 1 | 센서가 뜨는가 | `ros2 topic hz /image_raw` / `/scan` | 아무것도 안 됨 |
| 2 | 추론 FPS → `stale_timeout_sec` | perception 로그의 `X.Xfps` | CPU 추론이라 젯슨보다 느리고 들쭉날쭉하다. fps 주기가 `driver_node` 의 `stale_timeout_sec`(기본 0.5s)보다 길면 **매 프레임 "인지 끊김"으로 halt** — "가다 멈추고 가다 멈추고" 로 나타난다. 실측 fps 의 역수 × 2배 정도로 `amd_overrides.yaml` 의 `stale_timeout_sec` 를 맞출 것 (§9 시각화로 어느 프레임에서 걸리는지 바로 보임) |
| 3 | `image_width` | `ros2 topic echo /image_raw --field width --once` | 640 이 아니면 조향이 한쪽으로 계통적으로 치우친다 |
| 4 | `center_bias_px` | 트랙 중앙에 세워두고 perception 로그의 offset 을 읽어 그 값을 넣는다 | 목표 지점이 중앙이 아니게 됨 |
| 5 | 조향 방향 | 손으로 밀며 `steer.invert` 확인 | [실측 2026-08-18] 이 차량은 반대로 나와 `amd_overrides.yaml` 에 이미 `invert: true` 로 넣어뒀다. 그래도 실차에서 다시 한번 확인할 것 — 원인(카메라 offset 부호 / 서보 링키지 / ROS1 벤더 xycar_motor.py 의 `-steer_val`)이 바뀌면 값도 바뀔 수 있다 |
| 6 | 속도 체감 | `speed.base` 4.8 부터. 느리면 조금씩 올린다 | 12.0 은 이 차량에서 너무 빠르다 (§1) |
| 7 | 조향 게인 | `drive_params.yaml` 의 §steer 주석에 절차가 있다 (k_lat → k_curve → k_damp 순) | 진동하거나 코너에서 밀린다 |

> 3~7 은 젯슨 차량에서도 똑같이 해야 하는 항목이다. 두 차량은 카메라 장착 위치와
> 조향 링키지가 다르므로 **한쪽에서 잡은 값을 다른 쪽에 그대로 옮기지 말 것.**

---

## 8. 안 되는 경우

| 증상 | 원인 | 조치 |
|---|---|---|
| `package 'xycar_motor' not found` | 젯슨용 `drive.launch.py` 를 띄웠다 | `drive_amd.launch.py` 를 쓸 것 |
| `package 'slam_toolbox' not found` | `slam:=true` 인데 도커에 없다 | 기본값(false)으로 둘 것 |
| 노드가 시작 직후 죽음 | yaml 에 선언 안 된 파라미터가 있다 | `tools/check_params.py` |
| 차가 안 움직임 | `require_enable` 안전장치 | `/drive_enable` 발행 (§5) |
| 차가 요동침 | `race_manager` 가 같이 떠 있다 | `ros2 node list` 로 확인 후 하나만 남길 것 |
| `ModuleNotFoundError: ultralytics` | 도커에 미설치 | `pip install ultralytics` |
| **차가 가다 멈추고 가다 멈추고** | `driver_node` 가 `stale_timeout_sec` 마다 "인지 끊김" 판정으로 halt 반복 (CPU 추론 FPS 가 느려서) | §7 #2. `amd_overrides.yaml` 의 `stale_timeout_sec` 를 실측 fps 에 맞춰 늘릴 것 |
| `ros2 param set` 으로 `fsm.auto_start` 를 바꿨는데 그대로임 | `DriveFSM` 은 노드 시작 시 **한 번만** 만들어진다 — 런타임 파라미터 변경이 재생성 로직 없이는 반영 안 됨 | 그 파라미터를 **주면서 노드를 재시작**할 것: `ros2 run my_driver driver_node --ros-args --params-file ... -p fsm.auto_start:=true` |
| 시각화 창(§9)에 `waiting for /debug_image` 만 뜸 | `perception_node` 를 `debug:=true` 없이 띄웠다 | `ros2 launch my_bringup drive_amd.launch.py debug:=true` 로 다시 띄울 것 |

---

## 9. 파이프라인 시각화 (디버깅용)

```bash
ros2 launch my_bringup drive_amd.launch.py debug:=true
```

한 창에 5단계를 같이 보여준다: **1) YOLO 원시 검출**(박스+클래스+conf) /
**2) 차선 추정**(오프셋·품질·신호등·콘 개수) / **3) 판단(FSM) 상태** /
**4) 횡방향 계획**(목표 오프셋, 기준 소스, OK/HOLD) / **5) 제어**(각도·속도·정지 사유).

⚠️ **평소엔 반드시 `debug:=false`(기본값)로 둘 것.** `perception_node` 가 매
프레임 두 장짜리 합성 이미지를 그려서 발행하는데, YOLO CPU 추론이 이미 이
차량의 병목이다(§7 #2) — 디버그 이미지 그리기 비용까지 얹으면 FPS 가 더
떨어져서 "가다 멈추고" 증상이 오히려 악화된다. **문제를 재현해서 관찰할 때만
켜고, 원인을 찾으면 바로 끌 것.**

내부적으로는 두 노드가 각자 이미 계산한 걸 보여주기만 한다 — 새 판단 로직이
끼어들지 않는다:

```
perception_node --debug--> /debug_image (Image, publish_debug_image:=true 일 때만)
driver_node     --------->  /debug_state (String, JSON, 항상 발행 — 비용이 거의 없다)
                                   │
                                   ▼
                    my_debug/pipeline_view_node  (cv2.imshow 로 화면에 띄움)
```

`/debug_state` 는 driver_node 가 켜져 있으면 항상 나오므로, `debug:=true` 로
띄운 뒤 `perception_node` 만 따로 재시작해도(디버그 이미지 없이) 텍스트
패널만은 계속 갱신된다.
