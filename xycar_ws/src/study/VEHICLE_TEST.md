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

- [ ] **★ 콘 구간 rosbag 을 딴다** — 복도 추정 튜닝값이 전부 **합성 데이터 기반**이라
      실제 콘 간격·복도 폭으로 재검증해야 한다. 콘 코스를 한 바퀴 굴리며:

      ```bash
      ros2 bag record /scan /image_raw /corridor /lane /objects /debug_state
      ```

      `my_obstacle/tools/rosbag_reader.py` 가 ROS 없이 읽으므로 PC 에서 재튜닝 가능.
      ⚠️ `--storage mcap` 으로 녹화하면 그 리더가 못 읽는다. 기본(sqlite3) 로 딸 것.

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

### 참고 — launch 파일의 불일치 (지금은 무해)

`xycar_lidar.launch.py` 는 노드를 `LifecycleNode` 액션으로 띄우지만, 실제
`xycar_lidar_node.cpp` 는 평범한 `rclcpp::Node` 다(라이프사이클 서비스가 없다).
전이를 아무것도 emit 하지 않으므로 프로세스는 그냥 일반 노드로 뜨고 동작에는
문제가 없다. 다만 나중에 `ros2 lifecycle` 로 제어하려 하면 안 먹는다.
**벤더 패키지라 지금은 건드리지 않았다.**
