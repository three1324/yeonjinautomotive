# 실차 연결 테스트 체크리스트

> **"지금 차량 연결해서 테스트해보려고"** 하면 이 문서부터 연다.
> 순서대로 따라가면 되고, 각 단계는 **실패하면 다음으로 넘어가지 않는다.**
>
> 관련 문서: 배포 절차는 `DEPLOY_AMD.md`, 파라미터 근거는
> `my_bringup/config/drive_params.yaml`, 시각화·재생 테스트는 프로젝트 README §6.

---

## 0. 먼저 이것부터 (30초)

```bash
cd ~/xycar_ws && source install/setup.bash
python3 src/study/my_bringup/tools/preflight.py
```

환경·GPU·모델·장치·udev·**라이다/모터 포트 실재 여부**·파라미터를 한 번에 판정한다. **실패(FAIL) 항목이 남아 있으면
차에 전원을 넣지 않는다.** 각 항목마다 `[조치]` 줄에 고치는 방법이 그대로 적혀 있다.

센서 launch 를 띄운 뒤에는 실측까지:

```bash
python3 src/study/my_bringup/tools/preflight.py --live
```

`/image_raw` `/scan` 이 실제로 몇 Hz 로 오는지, **카메라 해상도가
`driver_node.image_width` 와 일치하는지**까지 본다(어긋나면 조향이 계통적으로 한쪽으로
치우친다 — 눈으로는 못 잡는 종류의 버그다).

---

## 1. 하드웨어 연결

| 장치 | 확인 | 기대 |
|---|---|---|
| 카메라 | `ls /dev/video*` | `/dev/video0` 등 |
| 라이다 | `lsusb \| grep 10c4:ea60` | CP2102 보임 |
| VESC (젯슨만) | `lsusb \| grep 0483:5740` | ChibiOS 보임 |

AMD 차량은 모터가 **별도 ROS1 도커**라 VESC 가 안 보여도 정상이다.

### udev 규칙 — 보드마다 한 번씩

`/etc/udev/rules.d/` 는 git 으로 안 옮겨진다. 새 보드거나 재설치했으면 README
'센서 udev 규칙' 절의 명령을 다시 실행해야 `/dev/ttyLIDAR` `/dev/ttyMOTOR` 가 생긴다.

```bash
ls -l /dev/ttyLIDAR /dev/ttyMOTOR    # 적용 확인
```

직접 경로(`/dev/ttyUSB0`)를 쓸 거면 규칙 없이 가도 되지만, **그 경로는 꽂는 순서에 따라
바뀐다.** 대회장에서 라이다와 VESC 를 다른 순서로 꽂으면 그대로 오작동한다.

---

## 2. 빌드 최신성

```bash
cd ~/xycar_ws && colcon build --symlink-install
```

`--symlink-install` 이라도 **entry_point / launch / config 를 추가했으면 재빌드가
필요하다.** preflight 의 "빌드 최신성" 항목이 이걸 잡아준다.

---

## 3. 센서만 먼저 (모터 없이)

```bash
ros2 launch my_bringup drive.launch.py motor:=false rviz:=true      # 젯슨
ros2 launch my_bringup drive_amd.launch.py rviz:=true               # AMD
```

RViz2 와 OpenCV 창이 뜬다. 여기서 확인할 것:

- [ ] `/debug_image` 패널에 **좌 YOLO 박스 / 우 차선** 2분할이 나온다
- [ ] LaserScan / PointCloud2 가 차 주변에 그려진다
- [ ] `perception_node` 로그의 **실측 fps** — AMD 차량은 이 값의 역수 2배로
      `amd_overrides.yaml` 의 `stale_timeout_sec` 을 맞춘다 (안 맞으면 "가다 멈추고" 반복)
- [ ] 시각화 텍스트의 `why:` 줄이 `disabled` 인지 확인 (정상. 아직 안 켰으니까)

**차선 오프셋 부호 확인:** 차를 차선 왼쪽에 놓으면 `off` 가 한쪽 부호로, 오른쪽에
놓으면 반대 부호로 나와야 한다. 안 그러면 카메라가 뒤집혔거나 `image_width` 가 틀렸다.

### 3-1. 라이다 확인 — ★ 실차에서 실제로 막혔던 지점 (2026-08-18)

RViz 에 스캔이 안 보이면 **여기서 멈추고** 아래 3개를 순서대로 확인한다.
넘어가면 장애물 정지·라바콘 복도·회피가 전부 동작하지 않는다.

- [ ] **① 노드 로그부터** — 라이다 노드는 실패해도 죽지 않는다. 조용히 살아 있다

      ```bash
      ls -t ~/.ros/log/xycar_lidar_node_*.log | head -1 | xargs cat
      ```

      `Unknown error` 가 보이면 **시리얼 포트를 못 연 것**이다 → ②로.
      아무 에러도 없으면 발행은 되고 있다는 뜻이다 → ③으로.

- [ ] **② 포트** — `ydlidar.yaml` 의 `port` 는 `/dev/ttyLIDAR` 다.
      **udev 규칙이 없으면 이 경로 자체가 안 생긴다**(§1).

      ```bash
      lsusb | grep 10c4:ea60          # 라이다가 USB 로 보이는가
      ls -l /dev/ttyUSB* /dev/ttyLIDAR
      ```

      규칙을 걸거나 yaml 의 port 를 실제 경로로 바꾼다.
      (`preflight.py` 의 "ydlidar.yaml port" 항목이 이걸 자동으로 잡는다)

- [ ] **③ QoS** — 발행 중인데도 안 보이는 경우가 있다. 라이다는 **BEST_EFFORT** 인데
      `ros2 topic echo` 기본값은 RELIABLE 이라 **한 줄도 안 받는다.**

      ```bash
      ros2 topic echo /scan --qos-reliability best_effort
      ros2 topic hz   /scan --qos-reliability best_effort
      ```

      RViz 는 Display 의 Reliability Policy 를 Best Effort 로 (`drive.rviz` 는 이미 그렇다).

- [ ] `/scan` 이 살아난 뒤 `/obstacle` `/corridor` 도 오는지 확인
      (`obstacle_node` 가 이 둘을 낸다. 라이다가 죽어 있으면 둘 다 안 나온다)

- [ ] **★ `xycar_motor` 발행자가 하나뿐인지 확인** — 가장 위험한 실수

      ```bash
      ros2 topic info /xycar_motor
      ```

      **Publisher count 는 반드시 1** 이어야 한다(`driver_node`).
      `rubbercone_node` 의 `drive_topic` 기본값이 `xycar_motor` 라서,
      params 파일 없이 수동으로 띄우면(`ros2 run my_obstacle rubbercone_node`)
      발행자가 둘이 되어 서로 다른 명령이 섞이고 **차가 요동친다.**
      launch 로 띄우면 yaml 이 `cone_cmd` 로 돌려주므로 안전하다.

- [ ] **라바콘 구간 전환 확인** (2026-08-19 mux 구조)

      구간 판정과 주행은 `rubbercone_node` 가 전담하고, `driver_node` 는
      구간일 때 그 명령을 그대로 통과시킨다.

      ```bash
      ros2 topic echo /cone_zone_active     # 콘 앞에서 true 로 바뀌는지
      ros2 topic hz   /cone_cmd             # 약 10Hz (스캔 주기) 로 나오는지
      ```

      `driver_node` 로그에서 전환을 확인:

      ```
      [INFO] CONE_ZONE — rubbercone_node 로 전환
      [LANE_DRIVE] ... | cone_zone(rubbercone)     <- 라이다가 몰고 있음
      [INFO] CONE_ZONE 이탈 — 차선 주행으로 복귀
      ```

      ⚠️ `CONE_ZONE 인데 /cone_cmd 가 끊겼다` 경고가 뜨면 라이다나 그 노드가
      죽은 것이다. 차선 주행으로 비상 대체되지만 **콘을 칠 수 있으니**
      즉시 §3-1 로 라이다를 점검할 것.

- [ ] **콘 구간 밖에서 라이다 간섭이 없는지 확인**

      `cone_zone(rubbercone)` 이 아닌 상태에서 `reason` 에 `STOP(...)` 이나
      `obstacle(...)` 이 뜨면 **버그다** — 콘 구간 밖에서는 라이다 전방거리를
      안 쓰기로 했다(`speed.obstacle_cap_in_cone_only: true`).

- [ ] **★ 콘 구간 rosbag 을 딴다** — 복도 추정 튜닝값이 전부 **합성 데이터 기반**이라
      실제 콘 간격·복도 폭으로 재검증해야 한다. **절차는 §9 참고.**

자세한 근거와 로그 원문은 §8.

---

## 4. 조향만 확인 — **차를 들어올린 상태에서**

```bash
ros2 topic pub --once /drive_enable std_msgs/msg/Bool '{data: true}'
ros2 topic echo /xycar_motor                    # [angle, speed]
```

- [ ] 차를 좌우로 기울여 차선을 옮겨보며 **바퀴가 옳은 방향으로** 도는지 확인
- [ ] 반대로 돌면 → `drive_params.yaml` 의 `steer.invert` (AMD 는 `amd_overrides.yaml`)
      를 뒤집는다. **게인을 건드려 맞추려 하지 말 것**
- [ ] `speed` 가 예상 범위인지 (젯슨 base 12.0 / AMD base 10.0)

⚠️ AMD 차량은 모터 ROS1 도커가 **상시 떠 있으므로** `/xycar_motor` 발행만으로 바퀴가
실제로 돈다. 반드시 들어올린 상태에서 할 것.

⚠️ AMD 차량에 서울대 `race_*` 패키지가 같이 설치돼 있으면 `race_manager` 도
`/xycar_motor` 를 발행한다. **동시에 띄우면 차가 요동친다** — 하나만 실행할 것.

---

## 5. 저속 주행

- [ ] 직선에서 먼저. 중앙 복귀가 느리면 `steer.k_lat` ↑, 좌우로 진동하면 ↓
- [ ] 그다음 일정 곡률 코너: 바깥으로 밀리면 `steer.k_curve` ↑, 안쪽으로 파고들면 ↓
- [ ] 남은 떨림은 `steer.k_damp`

순서를 지킬 것 (근거는 `drive_params.yaml` 의 steer 절 주석).

---

## 6. 문제가 났을 때 — 증상별 첫 확인

| 증상 | 먼저 볼 것 |
|---|---|
| 차가 안 움직임 | 시각화의 **`why:`** 줄. `disabled`(=`/drive_enable` 안 켬) / `wait`(=`WAIT_LIGHT`, 초록불 대기) / `no_lane_yet` / `stale(..)` / `ref lost` |
| 가다 멈추기 반복 | `perception_node` 실측 fps vs `stale_timeout_sec`. AMD 는 CPU 추론이라 느리다 |
| 조향이 반대 | `steer.invert` |
| 한쪽으로 계속 치우침 | `driver_node.image_width` vs 실제 카메라 폭 (`preflight.py --live` 가 잡아준다) |
| 라이다 안 보임 | **§3-1 의 ①②③** (로그 → 포트 → QoS 순서). 근거는 §8 |
| 모터 안 돎 (젯슨) | `vesc.yaml` 의 `port` 가 실제 존재하는 경로인지 |
| RViz 에 아무것도 안 나옴 | Fixed Frame(`odom`) TF 가 오는지. SLAM 없이는 `viz_node` 가 추측항법으로 낸다 |

---

## 7. 하드웨어 없이 미리 해볼 수 있는 것

```bash
# 영상 파일로 파이프라인 전체를 실시간 재생 (카메라·라이다·모터 불필요)
ros2 launch my_bringup replay.launch.py video:=~/테스트영상.mp4 enable:=true
```

인지 정확도·fps·조향 계산까지 전부 검증된다. 라이다가 없어 장애물 로직만 빠진다.
자세한 건 README '영상 파일로 실시간 재생 테스트' 절.


---

## 8. 라이다가 안 나올 때 — 실측 기록 (2026-08-18)

`/scan` 이 안 오면 **먼저 노드 로그부터 본다.** 라이다 노드는 실패해도 죽지 않는다.

```bash
ls -t ~/.ros/log/xycar_lidar_node_*.log | head -1 | xargs cat
```

### 증상 1 — `Unknown error` (실차에서 실제로 난 것)

```
[INFO]  [YDLIDAR INFO] Current ROS Driver Version: 1.0.1
[ERROR] Unknown error
[INFO]  [YDLIDAR INFO] Now YDLIDAR is stopping .......
```

**뜻: 시리얼 포트를 못 열었다.** SDK 의 `CYdLidar::initialize()` → `checkConnect()` →
`connect(port, baudrate)` 가 실패한 것이다. 그러면 `xycar_lidar_node.cpp` 의
`while (ret && rclcpp::ok())` 스캔 루프에 **아예 들어가지 않고** 프로세스만 살아 있다.
→ "노드는 떠 있는데 `/scan` 이 없다"는 조용한 실패.

확인·조치 순서:

```bash
lsusb | grep 10c4:ea60        # 1) 라이다가 USB 로 보이는가
ls -l /dev/ttyUSB* /dev/ttyLIDAR   # 2) 실제 경로가 있는가
```

`params/ydlidar.yaml` 의 `port` 는 `/dev/ttyLIDAR` 다. **udev 규칙이 없으면 이 경로가
아예 생기지 않는다** (README '센서 udev 규칙'). 규칙을 걸거나 yaml 의 port 를 실제
경로(`/dev/ttyUSB0` 등)로 바꾼다. `preflight.py` 가 이 항목을 자동으로 잡는다.

> 참고: 08-15 15:19 로그에는 이 에러가 없었다 — **그때는 정상 동작했다.** 즉 코드
> 문제가 아니라 연결/포트 문제다. 08-16 16:30 실패 → 16:33 성공(3분 뒤)한 기록도
> 있어, 꽂는 순서·재인식 타이밍에 따라 갈리는 것으로 보인다.

### 증상 2 — 발행은 되는데 `ros2 topic echo` 에 아무것도 안 나온다

08-15 로그에 실제로 남아 있는 경고:

```
[WARN] New subscription discovered on topic '/scan', requesting incompatible QoS.
       No messages will be sent to it. Last incompatible policy: RELIABILITY
```

라이다는 **BEST_EFFORT** 로 발행하는데(`rclcpp::SensorDataQoS()`), `ros2 topic echo` 는
기본이 RELIABLE 이라 **한 줄도 안 받는다.** 라이다가 멀쩡한데 죽은 것처럼 보인다.

```bash
ros2 topic echo /scan --qos-reliability best_effort    # 이렇게 볼 것
ros2 topic hz   /scan --qos-reliability best_effort
```

`obstacle_node` 와 `viz_node` 는 BEST_EFFORT 로 구독하므로 정상 동작한다. RViz 는
Display 의 Reliability Policy 를 Best Effort 로 두면 된다(`drive.rviz` 는 이미 그렇다).

### 증상 3 — `ros2 topic list` 에 `/scan` 이 없는데 실제로는 발행 중 (2026-08-19)

**가장 헷갈리는 실패다. 라이다는 멀쩡하다.** 실측:

```
$ ros2 topic list | grep scan          # ← 아무것도 안 나옴
$ ros2 topic hz /scan
average rate: 9.656                    # ← 정상 발행 중
```

`ros2 topic list` / `ros2 node list` 는 **ros2 daemon** 의 캐시를 읽는다. 데몬이
옛 세션의 참가자 정보를 붙들고 있으면 지금 떠 있는 토픽이 목록에서 빠진다.
이때 콘솔에 이 줄이 도배된다:

```
sequence size exceeds remaining buffer
```

이건 라이다 에러가 **아니다** — rmw_fastrtps 가 깨진 discovery 데이터를 읽는 것이다.
조치:

```bash
ros2 daemon stop && ros2 daemon start
```

> 판정은 항상 `ros2 topic hz` 로 한다. `topic list` 에 없다고 죽었다고 보면 안 된다.

### launch 파일의 LifecycleNode 불일치 — 고침 (2026-08-19)

`xycar_lidar.launch.py` 는 노드를 `LifecycleNode` 액션으로 띄웠지만, 실제
`xycar_lidar_node.cpp` 는 평범한 `rclcpp::Node` 다(라이프사이클 서비스가 없다).
전이가 영영 안 오니 `ros2 lifecycle get` 이 실패하고, 그게 "라이다가 안 켜졌다"는
오해로 이어졌다. 일반 `Node` 액션으로 바꿨다.

동작 확인 (2026-08-19, 실차):

```
Lidar successfully connected [/dev/ttyLIDAR:512000]   Model: G2B
Now lidar is scanning...                              (기동 ~2.5초 소요)
/scan  9.66 Hz    ->  /obstacle  front 0.70m / left 0.29m / right 0.35m
```

> 기동에 2.5초가 걸린다. launch 직후 바로 `hz` 를 재면 안 나온다 — 5초는 기다릴 것.


---

## 9. 콘 구간 rosbag 뜨는 법 ★

**왜 필요한가:** `corridor.py` 의 중앙선 추정 개선(far 오차 91.7 → 31.2px)은 전부
**제가 만든 합성 S자 코스** 기준이다. 실제 콘 간격·복도 폭·라이다 반사 특성은 아직
모른다. bag 하나만 있으면 `corridor_sim.py --bag` 으로 PC 에서 실제 값으로 재튜닝할 수
있다 — 차를 다시 굴리지 않고도 파라미터를 몇 번이고 바꿔 볼 수 있다는 뜻이다.

### 9-1. 녹화

라이다가 살아 있는지(§3-1) 먼저 확인한 뒤, **주행 스택을 띄운 상태에서** 녹화한다.

```bash
# 터미널 1 — 주행 스택 (센서 + 인지/판단. 모터는 안 띄운다)
ros2 launch my_bringup drive.launch.py motor:=false rviz:=true      # 젯슨
ros2 launch my_bringup drive_amd.launch.py rviz:=true               # AMD

# 터미널 2 — 녹화 (콘 코스 앞에서 시작)
cd ~ && ros2 bag record -o cone_$(date +%m%d_%H%M) \
    /scan /image_raw /corridor /obstacle /lane /objects /debug_state
```

| 토픽 | 왜 넣나 |
|---|---|
| `/scan` | **필수.** 복도 추정을 다시 돌릴 원본 |
| `/image_raw` | 그때 눈으로 뭐가 보였는지 대조용 (용량이 크다. 아래 참고) |
| `/corridor` | 그때 추정이 뭐라고 했는지 — 개선 전후 비교 기준 |
| `/obstacle` `/lane` `/objects` `/debug_state` | 융합·판단이 어떻게 반응했는지 |

**차를 어떻게 움직이나:** 자율주행일 필요 없다. **손으로 밀거나 조종기로 천천히**
콘 코스를 통과시키면 된다. 중요한 건 콘 사이를 실제로 지나가는 것이다.

- [ ] 콘 구간 **전체를 최소 2~3회** 통과 (직선 진입 → S자 → 빠져나감)
- [ ] 가능하면 **복도 중앙 / 왼쪽 치우침 / 오른쪽 치우침** 을 각각 한 번씩
      (한 자세만 있으면 부호·편향 검증이 안 된다)
- [ ] 속도는 느리게. 프레임이 촘촘할수록 좋다

`Ctrl-C` 로 종료하면 `cone_MMDD_HHMM/` 디렉터리가 생긴다.

### 9-2. ⚠️ 저장 포맷 — 기본(sqlite3) 로 딸 것

```bash
ros2 bag record --storage mcap ...     # ← 이렇게 하면 안 된다
```

`rosbag_reader.py` 는 **ROS 설치 없이** 읽으려고 sqlite3+CDR 를 직접 파싱한다.
mcap 으로 녹화하면 못 읽는다. 이미 mcap 으로 땄다면 젯슨에서:

```bash
ros2 bag convert -i <mcap_bag> -o converted --storage sqlite3
```

### 9-3. 딴 직후 — 차에서 바로 확인

빈 bag 을 들고 오는 것이 최악이다. **차를 치우기 전에** 확인한다.

```bash
ros2 bag info cone_MMDD_HHMM
```

- [ ] `/scan` 의 메시지 수가 0 이 아닌가 (10Hz × 녹화 초 정도면 정상)
- [ ] `Duration` 이 의도한 길이인가
- [ ] `Storage id: sqlite3` 인가

### 9-4. 실측 재튜닝 (PC / 젯슨, ROS 불필요)

```bash
cd ~/xycar_ws/src/study/my_obstacle
python3 tools/corridor_sim.py --bag ~/cone_MMDD_HHMM
python3 tools/corridor_sim.py --bag ~/cone_MMDD_HHMM --plot    # 그림으로
```

출력에 **"스캔 N개 중 복도 유효 M개 (xx.x%)"** 가 나온다. 이 비율이 낮으면 그 아래
안내대로 `--x-max` / `--bin-size` / `--half-width` 를 바꿔가며 다시 돌린다:

```bash
python3 tools/corridor_sim.py --bag ~/cone_MMDD_HHMM --x-max 2.6 --bin-size 0.12
```

찾은 값을 `my_bringup/config/drive_params.yaml` 의 `obstacle_node.corridor` 에 반영한다.

### 9-5. 이 bag 으로 확정해야 할 값

지금 전부 **가정값**이라 실측이 필요한 것들:

| 파라미터 | 지금 값 | 무엇으로 정하나 |
|---|---|---|
| `corridor.px_per_meter` | 300.0 | **가장 중요.** `(lane 픽셀 반폭 × 2) / 실측 트랙폭(m)`. 줄자 + perception 로그면 된다 |
| `corridor.nominal_half_width_m` | 0.35 | 실측 콘 복도 폭의 절반 |
| `corridor.x_max` | 2.2 | 라이다가 콘을 실제로 잡는 거리 |
| `corridor.bin_size` | 0.15 | 실제 콘 간격에 맞춰 |
| `fusion.cone_n_lo/hi` | 2 / 6 | 콘 구간에서 YOLO 가 실제로 몇 개를 세는지 |

`px_per_meter` 가 틀리면 **라이다 복도와 카메라 차선의 단위가 안 맞아** 융합이 통째로
어긋난다. 다른 어떤 튜닝보다 먼저 이것부터 맞출 것.
