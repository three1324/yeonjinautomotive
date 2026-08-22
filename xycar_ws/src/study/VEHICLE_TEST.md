# 실차 연결 테스트 체크리스트

> **"지금 차량 연결해서 테스트해보려고"** 하면 이 문서부터 연다.
> 순서대로 따라가면 되고, 각 단계는 **실패하면 다음으로 넘어가지 않는다.**
>
> 관련 문서: 파라미터 근거는
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

모터는 **별도 ROS1 도커**가 잡으므로 VESC 가 안 보여도 정상이다.

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

## 2-1. 젯슨 성능 설정 — **전원을 넣을 때마다** (2026-08-19 측정)

```bash
sudo nvpmodel -m 0     # MAXN_SUPER. 재부팅해도 유지된다
sudo jetson_clocks     # 클럭 고정. ★ 재부팅하면 풀린다 — 매번 다시 할 것
nvpmodel -q            # "NV Power Mode: MAXN_SUPER" 확인
```

perception_node 처리율 실측(시각화 없음, 같은 영상):

| 조건 | fps |
|---|---|
| `.pt` + 40W | 12.4 |
| `.engine` + 40W | 16.5 |
| **`.engine` + MAXN** | **약 29** |

**전력 모드가 TensorRT 보다 효과가 컸다** (+76% vs +33%). 40W 제한이 진짜
병목이었다. 이걸 안 하면 인지가 12fps 로 떨어져 카메라 30fps 를 한참 못 따라가고,
프레임이 실행마다 다르게 버려져 **같은 입력인데 결과가 달라진다.**

⚠️ MAXN 은 전력 상한이 없다. 장시간 주행 시 `tegrastats` 로 스로틀링을 볼 것.
   걸리면 25W/40W 로 내려도 `.pt` 보다는 빠르다.

### TensorRT 엔진

`my_perception/models/best5.engine` 이 있으면 launch 가 **자동으로** 그걸 쓴다
(없으면 `.pt` 로 폴백). 엔진은 **이 보드에서 만든 것만 유효**하다 — git 에 없으므로
보드를 바꾸거나 JetPack 을 올리면 다시 만든다:

```bash
cd ~/xycar_ws/src/study/my_perception
python3 -c "from ultralytics import YOLO; \
    YOLO('models/best5.pt').export(format='engine', half=True, imgsz=640, device=0)"
cd ~/xycar_ws && colcon build --symlink-install --packages-select my_perception
```

`perception_node` 로그의 `YOLO 모델 로드: .../best5.engine` 로 확인한다.

---

## 3. 센서만 먼저 (모터 없이)

```bash
ros2 launch my_bringup drive.launch.py rviz:=true
```

RViz2 와 OpenCV 창이 뜬다. 여기서 확인할 것:

- [ ] `/debug_image` 패널에 **좌 YOLO 박스 / 우 차선** 2분할이 나온다
- [ ] LaserScan / PointCloud2 가 차 주변에 그려진다
- [ ] `perception_node` 로그의 **실측 fps** — 이 값의 역수보다 `stale_timeout_sec`
      이 짧으면 매 프레임 "인지 끊김"으로 정지한다 ("가다 멈추고" 반복)
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

- [ ] `/scan` 이 살아난 뒤 `/cone_cmd` 가 오는지 확인
      (`rubbercone_node` 가 낸다. 라이다가 죽어 있으면 안 나온다)
      `/scan` 을 구독하는 노드는 이제 `rubbercone_node` 하나뿐이다
      (`obstacle_node` 는 2026-08-21 삭제).

- [ ] **★ `xycar_motor` 발행자가 하나뿐인지 확인** — 가장 위험한 실수

      ```bash
      ros2 topic info /xycar_motor
      ```

      **Publisher count 는 반드시 1** 이어야 한다(`driver_node`).
      `rubbercone_node` 의 `drive_topic` 기본값이 `xycar_motor` 라서,
      params 파일 없이 수동으로 띄우면(`ros2 run my_obstacle rubbercone_node`)
      발행자가 둘이 되어 서로 다른 명령이 섞이고 **차가 요동친다.**
      launch 로 띄우면 yaml 이 `cone_cmd` 로 돌려주므로 안전하다.

- [ ] **라바콘 구간 전환 확인** (2026-08-19 2차 — 판정은 카메라, 주행은 라이다)

      **구간 판정은 `driver_node` 가 카메라로 한다** — 콘 8개 이상이면서 가장 큰
      콘의 bbox 높이가 `enter_min_size_px` 이상이어야 진입하고, 2개 이하가
      1.5초 지속되면 이탈한다. 크기 조건이 있는 이유는 콘이 줄지어 서 있어
      **직선 끝에서도 8~13개가 한꺼번에 보이기 때문**이다 — 개수만 보면 구간에
      닿기 전에 전환된다. 구간 안에서의 조향·속도만
      `rubbercone_node` 명령을 그대로 통과시킨다.

      1차 실차에서는 판정도 `rubbercone_node`(/cone_zone_active)가 했는데,
      콘이 없는 곳의 벽·기둥에도 true 가 떠서 제어권이 라이다로 넘어가
      **차가 차선을 무시하고 이상하게 달렸다.** 그 판정은 전방 부채꼴 안
      원본 스캔점 개수만 세고 콘 모양을 보지 않는다.

      ```bash
      ros2 topic echo /debug_state          # cone_zone(카메라 판정) 과
                                            # zone_lidar(라이다 판정) 를 같이 본다
      ros2 topic echo /cone_zone_active     # = zone_lidar. 이제 제어에 안 쓴다
      ros2 topic hz   /cone_cmd             # 약 10Hz (스캔 주기) 로 나오는지
      ```

      ★ **콘이 없는 직선/곡선에서 `zone_lidar: true` 인데 `cone_zone: false`**
      라면 정상이다 — 라이다 오판정을 카메라가 막고 있다는 뜻이다. 반대로
      콘 구간인데 `cone_zone` 이 안 켜지면 YOLO 콘 검출부터 볼 것
      (`/objects` 의 cone_n, `debug_state` 의 `cone_n`).

      `driver_node` 로그에서 전환을 확인:

      ```
      [INFO] CONE_ZONE — rubbercone_node 로 전환 (enter(cone 3))
      [LANE_DRIVE] ... | cone_zone(rubbercone)     <- 라이다가 몰고 있음
      [INFO] CONE_ZONE 이탈 — 차선 주행으로 복귀 (exit(cone 0, 1.5s))
      ```

      ⚠️ `CONE_ZONE 인데 /cone_cmd 가 끊겼다` 에러가 뜨면 라이다나 그 노드가
      죽은 것이다. **차가 정지한다**(2026-08-21 변경 — 차선으로 되돌아가면
      콘을 치기 때문). 즉시 §3-1 로 라이다를 점검할 것.

- [ ] **콘 구간 밖에서 라이다 간섭이 없는지 확인**

      `cone_zone` 이 false 인 동안 driver_node 는 **라이다 토픽을 구독조차
      하지 않는다**(2026-08-21). 그러니 이때 나오는 조향·속도는 전부 카메라
      결과여야 한다. `reason` 에 거리(m) 기반 사유가 뜨면 코드가 되돌아간
      것이다.

      ```bash
      ros2 topic list | grep -E '/obstacle|/corridor'   # 아무것도 안 나와야 정상
      ros2 topic info /scan --verbose | grep -c 'Node name'  # 구독자는 rubbercone 뿐
      ```

- [ ] **콘 구간에서 라이다가 죽으면 정지하는지** (2026-08-21 변경)

      `/cone_cmd` 가 끊기면 차선으로 되돌아가지 않고 **정지**한다. 콘 구간에서
      차선 중심을 따라가는 것 자체가 콘으로 들어가는 길이기 때문이다.
      콘 구간 진입 후 rubbercone_node 를 죽여(`pkill -f rubbercone`) 확인:

      ```
      [ERROR] CONE_ZONE 인데 /cone_cmd 가 끊겼다 — 정지 (라이다/rubbercone_node 점검)
      ```

자세한 근거와 로그 원문은 §8.

---

## 4-0. ★ 조향 부호 — **확정됨 (2026-08-21 실측)**

> **결론: 명령 +값 = 좌회전. `steer.invert` = `true`.**
> 더는 추론하지 말 것. 아래는 재확인이 필요할 때의 절차다.

**주행 노드를 전부 끄고**(driver_node 가 같은 토픽에 쓰면 안 된다) 차를 들어올린
상태에서 **직접 명령을 넣고 바퀴를 눈으로 본다:**

```bash
pkill -f driver_node          # 다른 발행자 제거
ros2 topic pub -r 10 /xycar_motor std_msgs/msg/Float32MultiArray '{data: [20.0, 0.0]}'
```

| 앞바퀴가 향하는 쪽 | 뜻 | `steer.invert` |
|---|---|---|
| **왼쪽** | 양수 명령 = 좌회전 | **true** ← ✅ 2026-08-21 측정 결과 |
| 오른쪽 | 양수 명령 = 우회전 | false |

`-20.0` 으로도 한 번 해서 반대로 가는지 확인하면 끝이다.

### 이 측정으로 정리된 것

| 값 | 결과 |
|---|---|
| `driver_node.steer.invert` | `false` -> **`true`** |
| `viz_node.angle_sign` | `-1.0` -> **`1.0`** (REP-103 과 규약이 같아짐) |
| `rubbercone_node.invert_steer` | **`false` 그대로 — 건드리지 않는다** (아래) |

### ⚠️ `rubbercone_node.invert_steer` 는 같이 뒤집지 않는다

예전 이 문서에는 "두 값은 하나의 사실을 공유하니 같이 뒤집으라"고 적혀
있었는데 **그건 틀렸다.** 두 경로는 코드상 완전히 독립이다:

```
차선 주행 : driver_node -> SteeringController(steer.invert 적용) -> /xycar_motor
콘  구간 : rubbercone_node -> 자체 invert_steer 적용 -> /cone_cmd
           -> driver_node 가 조향을 **건드리지 않고 그대로** 통과
```

`rubbercone_node.py:332` 가 자기 `invert_steer` 를 적용해 이미 완성된 각도를
내보낸다. `steer.invert` 는 그 값에 닿지 않는다. 게다가 그쪽 `false` 는 팀원이
**실차 콘 주행으로 검증한 값**이다. 그대로 둔다.

### 이 이탈이 부호 문제가 아니었다면, 원인은 무엇이었나 ★

`invert: true` 는 8/19 낮부터 쓰던 값이고, **그 상태에서 S자 우측 이탈이
관측됐다.** 부호가 맞다면 그 이탈의 원인은 따로 있다. 그 뒤에 고쳐진 후보 셋:

| 후보 | 상태 |
|---|---|
| 라이다가 콘 구간 밖에서 제어권을 뺏고 있었음 | 해소 — 이제 구간 밖에서는 라이다를 구독조차 안 한다 |
| `lateral._offset()` 회피 목표 부호 뒤집힘 | 8/21 수정 |
| `angle_limit: 50` 포화 와인드업 (실제 포화는 35도) | 8/21 35 로 수정 |

셋 다 고쳐진 상태이므로 **재주행으로 확인해야 한다.** 재주행에서도 우측으로
이탈하면 위 세 가지가 아닌 네 번째 원인이 있다는 뜻이다.

---

## 4. 조향 반응 확인 — **차를 들어올린 상태에서**

```bash
ros2 topic pub --once /drive_enable std_msgs/msg/Bool '{data: true}'
ros2 topic echo /xycar_motor                    # [angle, speed]
```

- [ ] 차를 좌우로 기울여 차선을 옮겨보며 **바퀴가 옳은 방향으로** 도는지 확인
- [ ] 반대로 돌면 → **§4-0 을 먼저 하라.** 게인을 건드려 맞추려 하지 말 것
- [ ] **회피 방향** — 차량 모형을 화면 왼쪽에 두면 차가 **오른쪽**으로 붙어야 한다.
      로그의 `ot_dir`(+1=오른쪽)과 실제 이동 방향이 **일치**해야 한다.
      (2026-08-21 이전에는 여기가 반대였다 — `lateral._offset()` 참고)
- [ ] `speed` 가 예상 범위인지 (base 12.0)

⚠️ 모터 ROS1 도커가 **상시 떠 있으므로** `/xycar_motor` 발행만으로 바퀴가
실제로 돈다. 반드시 들어올린 상태에서 할 것.

---

## 4-2. 좌회전(지름길) 미션 — 2026-08-21 신규

좌회전 화살표를 보면 **가장 좌측 흰 실선을 따라 12초** 달리고 차선주행으로 복귀한다.
차량 왼쪽면이 실선에 붙는 위치가 목표다(중심 = 실선 + 반차폭).

**기본은 꺼져 있다.** 켜려면:

```bash
ros2 param set /driver_node fsm.enable_shortcut true    # 또는 yaml
```

- [ ] **먼저 반차폭 픽셀값을 맞춘다** — `lateral.shortcut_half_car_px` (기본 45.0 은 예시값)

      ```bash
      ros2 topic echo /debug_state --once        # half_near 를 읽는다
      ```

      줄자로 트랙폭 W(m) 을 재고: `shortcut_half_car_px = half_near × (0.15 / (W/2))`
      예) W=1.2m, half_near=180px → 45px

      ⚠️ 이 값이 0 이면 차량 중심이 실선 위에 와서 **실선을 밟는다.**

- [ ] **진입 확인** — 좌회전 화살표를 보여주고 로그를 본다

      ```
      [INFO] STATE LANE_DRIVE -> SHORTCUT (left arrow x5)
      [SHORTCUT] ... light=LEFT LEFT12s ...      <- 남은 시간이 줄어든다
      [INFO] STATE SHORTCUT -> LANE_DRIVE (shortcut done (12s))
      ```

      5프레임 연속 확정돼야 진입한다(오검출 한 번으로는 안 들어간다).

- [ ] **신호가 사라져도 12초를 채우는지** — 구간에 들어가면 신호등이 곧 시야를
      벗어난다. 그때 바로 복귀해 버리면 지름길을 못 탄다. 화살표를 치우고도
      `LEFT..s` 카운트다운이 계속 도는지 확인.

- [ ] **반폭 미학습 시 안전동작** — `half_near` 가 0 이면(좌우 흰선을 동시에 본
      적 없음) 실선 위치를 모르므로 **트랙 중앙을 유지**한다. 왼쪽으로 밀지 않는다.

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
| 가다 멈추기 반복 | `perception_node` 실측 fps vs `stale_timeout_sec` (§2-1 전력모드도 확인) |
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

`rubbercone_node` 와 `viz_node` 는 BEST_EFFORT 로 구독하므로 정상 동작한다. RViz 는
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

**왜 필요한가:** `rubbercone_node` 의 값(클러스터링 임계·페어링 간격·lookahead)은
팀원이 **다른 차·다른 코스**에서 맞춘 것이다. 실제 콘 간격·복도 폭·라이다 반사
특성은 아직 우리 트랙에서 확인되지 않았다. bag 하나면 차를 다시 굴리지 않고
`ros2 bag play` 로 몇 번이고 다시 돌려볼 수 있다.

### 9-1. 녹화

라이다가 살아 있는지(§3-1) 먼저 확인한 뒤, **주행 스택을 띄운 상태에서** 녹화한다.

```bash
# 터미널 1 — 주행 스택 (센서 + 인지/판단. 모터는 안 띄운다)
ros2 launch my_bringup drive.launch.py rviz:=true

# 터미널 2 — 녹화 (콘 코스 앞에서 시작)
cd ~ && ros2 bag record -o cone_$(date +%m%d_%H%M) \
    /scan /image_raw /cone_cmd /lane /objects /debug_state
```

| 토픽 | 왜 넣나 |
|---|---|
| `/scan` | **필수.** 복도 추정을 다시 돌릴 원본 |
| `/image_raw` | 그때 눈으로 뭐가 보였는지 대조용 (용량이 크다. 아래 참고) |
| `/cone_cmd` | 그때 rubbercone 이 뭐라고 했는지 — 튜닝 전후 비교 기준 |
| `/lane` `/objects` `/debug_state` | 판단이 어떻게 반응했는지 (구간 진입/이탈 시점 포함) |

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

### 9-4. 재튜닝 (bag 재생 + 파라미터 변경)

`corridor_sim.py` 는 없앴다(`obstacle_node` 와 함께 2026-08-21 삭제). 이제는
bag 을 틀어 놓고 `rubbercone_node` 만 띄운 뒤, 파라미터를 바꿔가며 `/cone_cmd`
와 `/rubbercone/debug_image` 를 본다.

```bash
# 터미널 1
ros2 bag play ~/cone_MMDD_HHMM --loop
# 터미널 2
ros2 run my_obstacle rubbercone_node --ros-args \
    --params-file ~/xycar_ws/install/my_bringup/share/my_bringup/config/drive_params.yaml
# 터미널 3 — 재시작 없이 값 바꾸기
ros2 param set /rubbercone_node cluster_max_span_m 0.35
```

찾은 값을 `my_bringup/config/drive_params.yaml` 의 `rubbercone_node` 에 반영한다.

### 9-5. 이 bag 으로 확정해야 할 값

지금 전부 **가정값**이라 실측이 필요한 것들:

**[2026-08-22 갱신]** 아래 rubbercone 값들은 **실측 치수로 확정**됐다
(콘 지름 10cm / 콘 간격 43.4cm / 복도 폭 80cm / 라이다-뒤축 41cm).
근거와 검산은 `my_obstacle/my_obstacle/geometry.py` docstring 참고.
실차 bag 이 없어 합성 검증으로 대신했다: `python3 my_obstacle/tools/synth_check.py`

| 파라미터 | 지금 값 | 무엇으로 정하나 |
|---|---|---|
| `rubbercone.cluster_max_span_m` | 0.20 | ✅ 콘 하나 span 0.10m 의 2배로 확정 |
| `rubbercone.min_gap_m` / `max_gap_m` | 0.60 / 1.00 | ✅ 복도 폭 0.80m ±25% 로 확정 |
| `rubbercone.range_max_m` | 1.40 | ⚠️ 라이다가 콘을 **실제로** 잡는 거리로 재확인 필요 |
| `rubbercone.lookahead_dist_m` | 0.85 | ⚠️ 코너를 깎으면 낮출 것(0.75), 조향이 떨리면 올릴 것 |
| `rubbercone.lidar_to_rear_axle_m` | 0.41 | ✅ 팀원 실측 base_link 기준값 |
| `cone_zone.enter_n / exit_n` | 8 / 2 | 콘 구간에서 YOLO 가 실제로 몇 개를 세는지 |
| `cone_zone.enter_min_size_px` | 90.0 | 구간에 **닿았을 때** 가장 큰 콘 bbox 높이. 화각·해상도 종속이라 실차 카메라로 재확인 필요 |

⚠️ 라이다 복도 추정(`corridor.*`)과 그것을 내던 `obstacle_node` 는 **통째로
삭제됐다**(2026-08-21). 라이다는 이제 `rubbercone_node` 한 노드에서만 쓰이고,
그 노드는 콘 구간에서만 제어권을 갖는다.

---

## 10. 팀원 실차 실측값 (2026-08-19) — 반영 상태

| 항목 | 값 | 반영 |
|---|---|---|
| 축거(wheelbase) | 0.33m (33cm) | ✅ `rubbercone_node.wheelbase_m` = 0.333 (프로젝트 공용값과 일치) |
| 조향 명령 단위 | **degree 그대로** (라디안×120 아님) | ✅ `rubbercone_node.steer_gain` = 57.29578 (rad→deg 변환상수) |
| 조향 부호 | 반전 불필요 | ✅ `rubbercone_node.invert_steer` = false (기존값 유지) |
| 실제 최대 조향각 | ±35° (기계적 한계, 40 이상 명령해도 안 꺾임) | ✅ `rubbercone_node.angle_limit` = 35.0, `viz_node.max_steer_deg` = 35.0 |

### 원본 실측 표 (조향 명령값 ↔ 실측 조향각)

| 명령값 | 실측(도) |
|---|---|
| 0.0 | 0 |
| 10.0 | 10 |
| 20.0 | 20 |
| 30.0 | 30 |
| 40.0 | 35 |
| 50.0 | 35 |
| -10.0 | -10 |
| -20.0 | -20 |
| -30.0 | -30 |
| -40.0 | -35 |
| -50.0 | -35 |
| 0.0 | 0 |

파생값: `min_turning_radius = wheelbase / tan(35°) = 0.33 / tan(35°) ≈ 0.471m`

### 차체 정보 (base_link 기준, 참고용 — §"사용처 없어 기록만 함" 참고)

- 앞 범퍼까지: +0.46m / 뒤 범퍼까지: 0.15m / 좌우 폭 절반: ±0.15m
- 라이다: `frame_id=laser_frame`, x=+0.41m, y=0, z=0(§ 라이다 z 오프셋 참고),
  방향은 차량 정면과 동일(회전 오프셋 0)

### ⚠️ 발견된 파급효과 — 재검토 필요 (미반영)

기존 `viz_node.max_steer_deg = 19.5` 는 **2026-08-16 최초 커밋**(VESC 연결 전)에
`[실측]`이라고만 표시된 채 들어간 값으로, 실제 실측 없이 잡힌 추정치였던 것으로
보인다. 팀원의 12개 지점 실측 테이블(0→0, 10→10, ..., 30→30, 40/50→35 포화)로
틀렸음이 확인됐다.

**`driver_node.steer` 의 게인(`k_lat=0.10`, `k_curve=0.15`, `k_damp=0.004`)이 이
잘못된 가정 위에서 시뮬레이션(`tools/sim_check.py`)됐을 가능성이 있다.** 실제
조향이 가정보다 거의 2배 민감하므로(명령 30 = 실제 30° vs 가정했던 명령
50=19.5°→명령 30≈11.7°), 같은 오프셋 오차에도 실차가 시뮬보다 훨씬 크게 꺾일 수
있다. 진동/과조향이 보이면 `k_lat`을 낮추는 것부터 시도할 것 — §9-1의 실차
튜닝 절차(직선→코너→댐핑 순서)를 따를 것.

이 항목은 `driver_node`/차선주행 경로에 영향을 주므로 **바로 고치지 않고
여기 기록만 해뒀다.** 실차에서 k_lat 진동 여부를 먼저 확인한 뒤 반영할 것.

### 라이다 z 오프셋 — 확인 완료, 변경 없음

`my_slam/launch/{mapping,localization}.launch.py` 의 `base_to_laser_tf` 기본값
(x=0.418, y=0.0, z=0.10)과 팀원 실측(x=0.41, y=0.0, z=0.0)이 z 만 달랐다.
**z(라이다 장착 높이)는 이 프로젝트에서 안 쓰기로 확정** — `laser_z` 는
2026-08-16 기록값(0.10) 그대로 둔다. x/y 는 반올림 오차 수준이라 애초에
문제없었다.

### 참고용 — 사용처 없어 기록만 함

차체 footprint (base_link 기준: 앞범퍼 +0.46m, 뒤범퍼 -0.15m, 좌우 폭 ±0.15m)는
현재 코드베이스에 이 값을 직접 쓰는 곳이 없다(costmap/footprint 개념 미사용).
나중에 필요해지면(URDF, 충돌 반경 등) 여기 값을 쓸 것.
