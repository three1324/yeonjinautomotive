# 작업지시서 — 젯슨에 ROS1 도커 모터 시스템 복원

> **이 문서를 읽는 Claude에게**: 당신은 이 작업의 사전 대화 맥락이 없다.
> 이 문서 하나로 작업이 가능하도록 필요한 사실을 전부 적어두었다.
> 추측하지 말고 여기 적힌 값과 실제 기기 확인 결과만 근거로 삼을 것.
> 확인이 필요한 지점은 본문에 **[확인]** 으로 표시해두었다.

---

## 0. 한 줄 요약

**AMD PC에서 검증된 "ROS1 도커 안의 VESC 드라이버 + ros1_bridge" 모터 제어 구조를
젯슨(arm64)에서 재구성한다.** 원본을 그대로 복사할 수는 없다(§3). 소스는 전부
`amd.zip` 안에 있으므로 arm64로 다시 빌드하면 된다.

---

## 1. 배경 — 왜 이 작업을 하는가

국민대 자율주행 경진대회(**2026-08-25**) 차량이다. 인지/판단/제어는 ROS2 Humble로
새로 작성했고 이미 젯슨에서 빌드·동작한다. **문제는 모터뿐이다.**

- 조직위가 준 **원본 모터 제어 스택은 ROS1(noetic)** 이다. AMD PC 차량에서
  도커 컨테이너로 돌려 **실제로 잘 동작하던 검증된 구성**이다.
- 우리는 이걸 ROS2로 포팅했으나(`amd/xycar_ws/src/xycar_motor/`,
  `amd/xycar_ws/src/vesc/`), 실기기 테스트에서 `/sensors/core` 값이 이상했다
  (§8 참고). 대회가 임박해 **검증된 원본 구성으로 되돌리는 쪽이 안전**하다는
  판단이다.

**따라서 목표는 "새로 잘 만들기"가 아니라 "검증된 것을 그대로 되살리기"다.**
벤더 소스를 개선하거나 리팩터링하지 말 것. 원본 그대로 빌드해서 돌리는 게 목적이다.

---

## 2. 현재 젯슨 상태 (이미 되어 있는 것)

| 항목 | 상태 |
|---|---|
| 보드 | Seeed reComputer Super J4012 / **Jetson Orin NX 16GB** |
| OS | JetPack 6.2.1, **Ubuntu 22.04 (jammy)**, Python 3.10, **arm64** |
| ROS | **ROS2 Humble** 설치 완료 |
| 계정 | `e-on` (호스트명 `eon-desktop`), sudo 가능 |
| `/dev/ttyMOTOR` | ✅ **udev 규칙 설정 완료** — VESC(`0483:5740`, STM ChibiOS) |
| `/dev/ttyLIDAR` | ✅ 설정 완료 — YDLidar G2B (`10c4:ea60`, CP2102) |
| `/dev/ttyIMU` | ✅ 동작 확인됨 (`1b4f:9d0f`) |
| 센서 | 카메라(640×480) / 라이다 / IMU 모두 개별 동작 확인 완료 |
| VESC 연결 | **USB 직결** (UART 아님). 커넥터 정상, 통신 확인됨 |

### 이미 설정된 udev 규칙 (참고 — 다시 만들 필요 없음)

```
/etc/udev/rules.d/99-vesc.rules
KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyMOTOR"
```

**작업 시작 전 반드시 확인:**
```bash
uname -m                 # aarch64 여야 한다
groups                   # dialout 포함 확인
docker --version         # JetPack 기본 포함. 없으면 먼저 설치
ls -l /dev/ttyMOTOR      # 심볼릭 링크가 살아 있어야 한다 (→ ttyACM0)
```

> 📌 **참고**: ROS2 포팅본 설정 파일
> `amd/xycar_ws/src/xycar_motor/config/vesc.yaml` 에 "USB 커넥터 물리 손상으로
> USB-UART 어댑터(CH340, `/dev/ttyUSB0`) 우회 중"이라는 주석과 `port:
> /dev/ttyUSB0` 설정이 남아 있다. **이 정보는 철회됐다 — 커넥터는 정상이고 USB
> 직결로 잘 동작한다.** 그 파일은 이번 ROS1 도커 작업과 무관하므로 신경 쓰지 말 것.
> 다만 나중에 ROS2 네이티브 경로로 되돌아간다면 `port` 를 `/dev/ttyMOTOR` 로
> 되돌려야 한다.

#### 흐름제어 — ROS1 원본은 수정 불필요 (확인 완료)

**ROS1 벤더 드라이버는 `serial::flowcontrol_none` 이라 그대로 쓰면 된다**
(`noetic_ws/src/vesc/vesc_driver/src/vesc_interface.cpp:25`, 115200 8N1).
우리 ROS2 포팅본만 `HARDWARE` 로 돼 있어 고쳐야 했던 것이다. **ROS1 원본은
손대지 말 것.**

---

## 3. 왜 "그대로 복사"가 안 되는가 — 반드시 이해할 것

`amd.zip`은 원본 AMD PC의 **홈디렉토리 백업**이다. 다음 두 가지 때문에 그대로는 못 쓴다.

### (1) 도커 이미지가 zip 안에 없다

원본이 쓰던 이미지는 `osrf/ros:noetic-xycar` 다. 이건 **조직위가 로컬에서 커스텀
빌드한 이미지**이고 Docker Hub에 없는 태그다. 도커 이미지는 `/var/lib/docker`에
저장되므로 홈디렉토리 백업인 zip에는 안 들어 있다. Dockerfile도 `docker save` tar도
zip 안에 없다. **확인 완료 — 찾지 말 것. 새로 만들어야 한다.**

### (2) 아키텍처가 다르다

원본 AMD PC는 **x86_64**, 젯슨은 **arm64(aarch64)** 다. 따라서 zip 안의 다음
디렉토리들은 **전부 x86_64 바이너리라 젯슨에서 실행 불가**하다:

- `ros-humble-ros1-bridge/install/` (89MB `libros1_bridge.so` 포함) → **재빌드 필요**
- `noetic_ws/devel/`, `noetic_ws/build/` → **재빌드 필요**
- `xycar_ws/build/`, `xycar_ws/install/` → 이번 작업과 무관

**하지만 소스는 전부 있다.** 아래 §4 참고.

---

## 4. 소스 준비 — `motor_ros1_bundle.zip`

**amd.zip(890MB)을 젯슨에 옮길 필요 없다.** 필요한 파일 48개만 추려
**`motor_ros1_bundle.zip` (44KB)** 로 만들어 이 저장소에 커밋해두었다.
`git pull` 하면 저장소 루트에 있다.

```bash
cd ~/자율주행/auto     # 저장소 위치는 실제 clone 경로에 맞출 것
git pull
unzip -o motor_ros1_bundle.zip -d ~
```

압축을 풀면 홈에 다음이 생긴다:

| 경로 | 내용 | 용도 |
|---|---|---|
| `~/noetic_ws/src/vesc/` | `vesc_driver`, `vesc_ackermann`, `vesc_msgs` | ROS1 VESC 드라이버 **원본** |
| `~/noetic_ws/src/xycar_motor/` | `xycar_motor.py` + launch + CMakeLists | 조직위 **원본** 모터 노드 |
| `~/xycar_ws/etc/motor_vesc/motor` | **기동 스크립트 원본** | §6 Step 4 에서 수정해 사용 |

### ⚠️ catkin 워크스페이스 초기화가 필요하다

원본 `noetic_ws/src/CMakeLists.txt` 는 **심볼릭 링크**
(`→ /opt/ros/noetic/share/catkin/cmake/toplevel.cmake`)라 Windows에서 압축할 때
빠졌다. 컨테이너 안에서 한 번 만들어주면 된다:

```bash
docker run -it --rm -v ~/noetic_ws:/root/noetic_ws osrf/ros:noetic-xycar \
  bash -c "source /opt/ros/noetic/setup.bash && cd /root/noetic_ws/src && catkin_init_workspace"
```

(Step 1에서 이미지를 만든 뒤에 실행할 것. Step 2 빌드 직전에 하면 된다.)

### 번들에 일부러 넣지 않은 것

| 제외한 것 | 이유 |
|---|---|
| `noetic_ws/devel/`, `build/` | **x86_64 빌드 산출물** — 젯슨에서 못 쓴다. Step 2에서 새로 만든다 |
| `ros-humble-ros1-bridge/install/` | 같은 이유(x86_64). Step 3에서 새로 빌드 |
| `noetic_ws/src/my_motor/` | 학습용 예제, 이 작업과 무관 |
| `old_vesc_tool`, `vesc_tool_6.05` | **x86_64 AppImage** — arm64 젯슨에서 실행 불가. VESC 펌웨어 설정은 별도 PC에서 할 것 |
| `VESC_2.18.bin`, `VESC_5.03.bin` | 펌웨어 이미지. 위 도구가 있어야 쓸 수 있음 |

> 원본 전체가 필요해지면 `amd.zip` 이 PC(Windows)에 있다. 저장소에는
> `.gitignore` 로 제외돼 있으므로 필요 시 `scp` 로 별도 전송할 것.

### ⚠️ 벤더 소스는 손대지 말 것

`noetic_ws/src/vesc/`, `noetic_ws/src/xycar_motor/` 는 **검증된 조직위 원본**이다.
"개선"하지 말고 그대로 빌드할 것. 특히 다음 값들이 우리 ROS2 포팅본과 다르지만
**원본 값이 맞다** — 고치지 말 것:

```yaml
# noetic_ws/src/vesc/vesc_driver/yaml/vesc.yaml (원본)
vesc_driver:
  port: /dev/ttyMOTOR
  speed_min: -20000        # ← 우리 ROS2 포팅본은 -1000 으로 바꿨었다. 원본은 이 값.
  speed_max: 40000         # ← 우리 ROS2 포팅본은 10000. 원본은 이 값.
speed_to_erpm_gain: 4614
steering_angle_to_servo_gain: -1.2135
steering_angle_to_servo_offset: 0.5004
```

---

## 5. 원본 시스템이 하던 일 (복원 대상)

`xycar_ws/etc/motor_vesc/motor` 스크립트 전문을 분석한 결과다.

```
[0] /dev/ttyMOTOR 존재 확인 (없으면 에러 후 종료)
[1] 기존 ros1_container 정리 (docker stop / rm)
[2] docker run -d \
      --privileged --name ros1_container --network host \
      -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v /home/xytron/Downloads:/root/Downloads \
      -v /home/xytron/noetic_ws:/root/noetic_ws \
      -v /dev:/dev --group-add dialout \
      -e ROS_MASTER_URI=http://localhost:11311 \
      osrf/ros:noetic-xycar \
      bash -ci "source /opt/ros/noetic/setup.bash; \
                source /root/noetic_ws/devel/setup.bash; motor"
[3] 호스트에서:
      source /opt/ros/humble/setup.bash
      source ~/ros-humble-ros1-bridge/install/local_setup.bash
      export ROS_MASTER_URI=http://localhost:11311
      ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &
[4] docker exec -it ros1_container bash  (쉘 유지)
[5] 종료 시 브릿지 kill + 컨테이너 stop
```

### 데이터 흐름

```
ROS2 (호스트, 네이티브)                      ROS1 (컨테이너 안)
─────────────────────                      ──────────────────
우리 주행 노드
   │
   │ xycar_motor  (Float32MultiArray [angle, speed])
   │ angle ±50, speed ±50 범위
   ▼
[ros1_bridge dynamic_bridge] ─────────────► xycar_motor.py
                                               │ angle*0.0068, speed*0.08
                                               ▼
                                            ackermann_cmd (AckermannDriveStamped)
                                               ▼
                                            ackermann_to_vesc → vesc_driver
                                               ▼
                                            /dev/ttyMOTOR (실제 VESC)
                                               │
                                            /sensors/core (VescStateStamped)
```

**핵심**: VESC 드라이버 자체가 컨테이너 안에 있다. ROS2 쪽은 토픽 하나만 던진다.

### 컨테이너 안의 `motor` — 실행 파일이 아니라 **alias** 다 (Step 0 에서 확정)

```bash
# 원본 컨테이너의 /root/.bashrc 107번째 줄
alias motor='roslaunch xycar_motor xycar_motor.launch'
```

`/usr/local/bin/motor` 같은 실행 파일은 **존재하지 않는다.** 그래서 원본
`motor` 스크립트가 컨테이너를 `bash -ci`(**대화형**)로 띄우는 것이다 —
alias 는 대화형 셸에서만 로드된다. `-i` 를 빼면 `motor: command not found` 가 난다.

`xycar_motor.launch` 는 `motor_type:=0`(VESC) 기본값으로
`vesc_drive_xycar_motor.launch`(vesc_driver + ackermann_to_vesc)와
`xycar_motor.py` 를 함께 띄운다. `roslaunch` 가 roscore 를 자동 기동한다.

---

## 6. 작업 단계

### ⭐ Step 0 — 원본 도커 이미지 확보 (가능하면 반드시)

> **사용자 요구사항: "ROS1 시스템과 도커 시스템이 아예 똑같아야 한다."**
>
> §3(1)에서 설명했듯 원본 이미지 `osrf/ros:noetic-xycar` 가 zip에 없다.
> Step 1의 Dockerfile은 **필요할 것으로 추정되는 패키지**를 넣은 것이라
> 원본과 다를 수 있다. 조직위가 그 이미지에 무엇을 더 넣었는지 모른다.
>
> **AMD 차량(원본 PC)에 접근할 수 있다면 이 단계를 먼저 할 것.**
> "추정 재현"이 "정확 재현"으로 바뀐다.
>
> ### ✅ 이 단계는 2026-08-18 에 이미 완료됐다.
> 분석 결과가 **§6 Step 1** 에 전부 반영돼 있다. 아래 절차는 재확인이
> 필요할 때만 쓰면 된다. **바로 Step 1 로 갈 것.**

**AMD 차량에서:**
```bash
docker save osrf/ros:noetic-xycar | gzip > ~/noetic-xycar.tar.gz
ls -lh ~/noetic-xycar.tar.gz        # 수백 MB ~ 2GB 정도
```

**젯슨으로 전송 후 (실행은 안 되지만 내용 분석은 된다):**
```bash
docker load < noetic-xycar.tar.gz

# (a) 빌드 이력 - 어떤 명령으로 만들어졌는지
docker history osrf/ros:noetic-xycar --no-trunc

# (b) 베이스 이미지 / 환경변수 / CMD
docker inspect osrf/ros:noetic-xycar | head -80

# (c) 설치된 패키지 목록 (QEMU 에뮬레이션 필요, 느리지만 됨)
sudo apt-get install -y qemu-user-static binfmt-support
docker run --rm --platform linux/amd64 osrf/ros:noetic-xycar dpkg -l > orig_pkgs.txt

# (d) /usr/local/bin/motor 원본 내용 확인 (§5 의 추정이 맞는지)
docker run --rm --platform linux/amd64 osrf/ros:noetic-xycar cat /usr/local/bin/motor
```

**얻은 정보로 Step 1의 Dockerfile을 원본과 일치시킬 것.** 특히:
- `orig_pkgs.txt` 에 있는 `ros-noetic-*` 패키지를 전부 반영
- `docker history` 에 보이는 `ENV` / `RUN` 을 그대로 재현
- (d)로 `motor` 스크립트 실제 내용 확인 — §5의 추정과 다르면 실제 것을 쓸 것

**결과를 §11 작업 기록에 남길 것.**

#### Step 0 이 불가능하면 (AMD 차량 접근 불가)

Step 1의 Dockerfile로 진행하되, **이건 추정 재현이라는 점을 사용자에게 명시할 것.**
동작이 원본과 다르면 누락된 패키지를 의심할 것. 다만 §7 검증을 통과하면
모터 제어 경로 자체는 동일하게 동작한다 — 소스와 설정값이 원본 그대로이기 때문이다.

#### 무엇이 동일하고 무엇이 동일할 수 없는가 (정직한 정리)

| 요소 | 원본과 동일? |
|---|---|
| ROS1 소스코드 (vesc / xycar_motor) | ✅ **바이트 단위 동일** (zip 원본) |
| `vesc.yaml` 설정값 (ERPM 게인·서보 매핑·speed 한계) | ✅ 동일 |
| launch 파일, 토픽 계약 | ✅ 동일 |
| `motor` 스크립트의 docker run 옵션 | ✅ 동일 (홈 경로만 수정) |
| **CPU 아키텍처** | ❌ x86_64 → arm64. **물리적으로 불가능** |
| 컴파일된 바이너리 | ❌ 위 때문에 재컴파일 필수 |
| 도커 이미지 내용물 | ⚠️ Step 0 하면 정확 재현 / 안 하면 추정 재현 |

**모터의 동작을 결정하는 것은 소스코드와 설정값이고 그 둘은 원본 그대로다.**
아키텍처 차이는 같은 소스를 다른 CPU용으로 컴파일하는 것일 뿐 로직을 바꾸지 않는다.

---

### Step 1 — ROS1 noetic arm64 이미지 만들기

> ✅ **Step 0 완료 (2026-08-18).** 아래 내용은 원본 이미지를 실제로 분석해서
> 확정한 것이다. 추정이 아니다.

#### 원본 이미지 분석 결과 (확정)

**베이스 체인** (`docker history` 아래→위):
```
ubuntu:20.04 (focal)
 → tzdata, ca-certificates/curl/dirmngr/gnupg2
 → apt 저장소: snapshots.ros.org/noetic/final/ubuntu focal main   ← packages.ros.org 아님
 → ros-noetic-ros-core=1.5.0-1*
 → ros_entrypoint.sh + ENTRYPOINT + CMD ["bash"]
 → build-essential, python3-rosdep, python3-rosinstall, python3-vcstools
 → rosdep init && rosdep update
 → ros-noetic-ros-base → ros-noetic-robot → ros-noetic-desktop
 → ros-noetic-desktop-full=1.5.0-1*        ← 여기까지가 공식 osrf/ros:noetic-desktop-full
 → [bash 레이어 14개, 약 172MB]            ← 조직위 커스터마이징
```

⚠️ **커스터마이징은 `docker commit` 으로 됐다.** 상위 14개 레이어의 `CREATED BY`
가 전부 `bash` 라서 **어떤 명령을 실행했는지 이력으로는 볼 수 없다.** 대신
`dpkg -l` / `pip3 list` / 파일 확인으로 역추적한 결과가 아래다.

**커스터마이징 내용 (역추적 확정):**

| 항목 | 확인 방법 | 값 |
|---|---|---|
| `ros-noetic-serial` | `dpkg -l` | `1.2.1-1focal.20250426.010535` |
| `ros-noetic-ackermann-msgs` | `dpkg -l` | `1.0.2-1focal.20250426.013235` |
| `pyserial` | `pip3 list` | **3.5** (focal apt 는 3.4 → pip 설치본) |
| `/usr/local/bin/` | `ls` | `pyserial-miniterm`, `pyserial-ports` (pyserial 부산물뿐) |
| **`motor` 의 정체** | `/root/.bashrc:107` | **`alias motor='roslaunch xycar_motor xycar_motor.launch'`** |
| `cm` alias | `/root/.bashrc:103` | `alias cm='cd /root/noetic_ws && catkin_make'` |
| `ENV DISPLAY` | `docker inspect` | `:0` |
| `/root` 하위 | `ls -la` | `noetic_ws/`(마운트 지점), `Downloads/`(마운트 지점), `tmp_ws/`(잔재) |

> 📌 **`motor` 는 실행 파일이 아니라 alias 다.** 그래서 원본 `motor` 스크립트가
> `bash -ci`(**대화형**)로 컨테이너를 띄운다 — alias 는 대화형 셸에서만 읽힌다.
> `bash -c`(비대화형)로 바꾸면 `motor: command not found` 가 난다. **절대 바꾸지 말 것.**
>
> zip 안의 `noetic_ws/src/.bashrc`(3328B)에는 이 alias 가 **없다**. 컨테이너
> 것(3451B)이 더 최신이라 아래 Dockerfile 에서 직접 추가한다.

#### 벤더 패키지가 실제로 요구하는 의존성 (package.xml 전수 조사)

```
nodelet  pluginlib  roscpp  rospy  serial  std_msgs
vesc_msgs  ackermann_msgs  geometry_msgs  nav_msgs  tf  message_generation/runtime
```

**rviz · gazebo · rqt · perception 은 하나도 안 쓴다.** `serial` 과
`ackermann_msgs` 를 뺀 나머지는 전부 `ros-noetic-robot` 안에 있다.

#### ⭐ 베이스는 A안(desktop-full)으로 고정한다 — 더 이상 선택지 아님

> **사용자 결정 (2026-08-18): "프로세서 차이로 생기는 것만 다르고 나머지는
> 전부 원본과 똑같아야 한다."** 즉 **CPU 아키텍처 때문에 어쩔 수 없이 바뀌는
> 것(바이너리가 arm64로 컴파일되는 것) 외에는 전부 원본과 동일해야 한다.**
> B안(최소구성)은 패키지 구성이 원본과 달라지므로 **이제부터 쓰지 않는다.**
> 아래 Dockerfile 한 가지로만 진행할 것 — 선택지가 아니다.

**[확인] `osrf/ros:noetic-desktop-full` 은 arm64 멀티아키가 아니다(amd64 전용).**
그래서 `arm64v8/ros:noetic-ros-core` 위에 `ros-noetic-desktop-full` 을 apt 로
직접 설치해서 **패키지 구성을 원본(236개, §6 Step 1 분석 결과)과 맞춘다.**
크기는 약 4.7GB, 빌드는 젯슨에서 30~60분 걸린다. `run_in_background` 로 걸고
기다릴 것 — 시간이 오래 걸린다고 축소된 구성으로 바꾸지 말 것.

#### ⚠️ `motor` 는 반드시 alias 로만 만들 것 — 스크립트 파일 금지

Step 0(§5)에서 확정한 사실: 원본은 `/usr/local/bin/motor` 같은 **실행 파일이
없다.** `/root/.bashrc:107` 의 **alias 하나**가 전부다:
```
alias motor='roslaunch xycar_motor xycar_motor.launch'
```
`/usr/local/bin/` 에는 pyserial 부산물(`pyserial-miniterm`, `pyserial-ports`)
**뿐**이다. **`/usr/local/bin/motor` 라는 파일을 만들지 말 것.** 스크립트로
만들면 동작은 비슷해도 원본과 다른 구현이 된다. 반드시 `.bashrc` 에 alias 로만
추가하고, 원본처럼 `bash -ci`(대화형)로만 로드되게 할 것.

**빌드 후 검증 (반드시 이 형태로 나와야 한다):**
```bash
docker run --rm osrf/ros:noetic-xycar bash -lc "type -a motor"
# 기대 출력: motor is aliased to `roslaunch xycar_motor xycar_motor.launch'
# 이렇게 나오면 잘못된 것: motor is /usr/local/bin/motor
```

#### Dockerfile

`~/noetic-xycar/Dockerfile`:

```dockerfile
FROM arm64v8/ros:noetic-ros-core

# 원본과 동일한 패키지 구성 (desktop-full). 시간이 걸려도 이대로 둘 것.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-desktop-full \
 && rm -rf /var/lib/apt/lists/*

# 조직위 커스터마이징 재현 (§6 Step 1 분석 결과 그대로)
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      python3-pip \
      python3-rosdep python3-rosinstall python3-vcstools \
      ros-noetic-serial \
      ros-noetic-ackermann-msgs \
 && rm -rf /var/lib/apt/lists/*

# 원본은 pyserial 3.5 (focal apt 는 3.4). pip 설치본이라 그대로 맞춘다.
RUN pip3 install --no-cache-dir 'pyserial==3.5'

# 원본 /root/.bashrc 의 alias 두 개 그대로 (line 103, 107).
# motor 는 alias 다 - /usr/local/bin/motor 같은 실행 파일을 만들지 말 것.
RUN printf "\nalias cm='cd /root/noetic_ws && catkin_make'\nalias motor='roslaunch xycar_motor xycar_motor.launch'\n" \
      >> /root/.bashrc

# 런타임 마운트 지점
RUN mkdir -p /root/noetic_ws /root/Downloads

ENV DISPLAY=:0
```

```bash
mkdir -p ~/noetic-xycar && cd ~/noetic-xycar
# (위 Dockerfile 작성 후)
docker build . -t osrf/ros:noetic-xycar --network host
```

**태그를 원본과 똑같이 `osrf/ros:noetic-xycar` 로 둘 것** — `motor` 스크립트가
이 이름을 참조하므로 스크립트 수정이 줄어든다.

**[확인] `rosdep init` 은 이미 베이스 이미지에 되어 있다** (history 참고).
다시 실행하면 에러가 나므로 넣지 말 것.

**[확인]** `ros-noetic-serial` / `ros-noetic-ackermann-msgs` 가 arm64 에 없으면
소스 빌드로 대체:
```bash
git clone https://github.com/wjwwood/serial.git      ~/noetic_ws/src/serial
git clone https://github.com/ros-drivers/ackermann_msgs.git ~/noetic_ws/src/ackermann_msgs
```

#### 재빌드 전 기존 이미지/빌드 산출물 정리

이전 시도(86개 패키지, `/usr/local/bin/motor` 스크립트 방식)가 이미 있다면
지우고 새로 만들 것:

```bash
docker rmi osrf/ros:noetic-xycar
rm -rf ~/noetic-xycar
rm -rf ~/noetic_ws/devel ~/noetic_ws/build     # 새 이미지로 다시 catkin_make 해야 함
```

**패키지 개수로 검증 — 236개에 근접해야 한다:**
```bash
docker run --rm osrf/ros:noetic-xycar bash -lc "dpkg -l | grep -c '^ii  ros-noetic'"
# 기대: 236 근처 (desktop-full 전체). 86 처럼 적게 나오면 잘못된 것.
```

### Step 2 — 컨테이너 안에서 ROS1 워크스페이스 빌드

```bash
docker run -it --rm \
  -v ~/noetic_ws:/root/noetic_ws \
  osrf/ros:noetic-xycar \
  bash -c "source /opt/ros/noetic/setup.bash && cd /root/noetic_ws && catkin_make"
```

성공하면 `~/noetic_ws/devel/` 이 **arm64로** 새로 생긴다.

**[확인]** zip에서 나온 기존 `devel/`, `build/` 는 x86_64 잔재다. 빌드 전에 지울 것:
```bash
rm -rf ~/noetic_ws/devel ~/noetic_ws/build
```

빌드 대상 확인 — 다음 4개가 나와야 한다: `vesc_msgs`, `vesc_driver`,
`vesc_ackermann`, `xycar_motor`.

### Step 3 — ros1_bridge arm64 빌드 (가장 오래 걸림)

Noetic은 Ubuntu 22.04용 공식 패키지가 없어서 직접 빌드가 까다롭다. 원본도
[TommyChangUMD/ros-humble-ros1-bridge-builder](https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder)
를 쓴 것으로 보인다(zip의 폴더명이 `ros-humble-ros1-bridge`). **이 빌더는 arm64 /
Jetson Orin 을 명시적으로 지원한다.**

```bash
cd ~
git clone https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder.git
cd ros-humble-ros1-bridge-builder
docker build . -t ros-humble-ros1-bridge-builder --network host
cd ~
docker run --rm ros-humble-ros1-bridge-builder | tar xvzf -
# → ~/ros-humble-ros1-bridge/ 생성
```

주의사항:
- **오래 걸린다** (젯슨에서 1~3시간). `run_in_background` 로 돌리고 다른 작업을 병행할 것.
- **메모리를 많이 쓴다.** Orin NX 16GB면 대체로 되지만 OOM으로 죽으면 스왑을 늘릴 것:
  ```bash
  sudo fallocate -l 16G /swapfile && sudo chmod 600 /swapfile
  sudo mkswap /swapfile && sudo swapon /swapfile
  ```
- 빌더 README에 커스텀 메시지 추가 절차가 있다. **`vesc_msgs` 가 브릿지에
  필요한지 [확인]할 것** — `/sensors/core`(VescStateStamped)를 ROS2로 넘겨야
  한다면 커스텀 메시지 등록이 필요하다. 단, 모터 구동 자체는
  `xycar_motor`(Float32MultiArray, 표준 타입) 한 방향만 되면 되므로
  **1차 목표에서는 필수가 아니다.**

### Step 4 — motor 스크립트 경로 수정

`~/xycar_ws/etc/motor_vesc/motor` 에서 원본 계정 경로를 젯슨 계정으로 바꾼다.

| 원본 | 젯슨 |
|---|---|
| `HOST_WS_PATH="/home/xytron/noetic_ws"` | `HOST_WS_PATH="/home/e-on/noetic_ws"` |
| `-v /home/xytron/Downloads:/root/Downloads` | `-v /home/e-on/Downloads:/root/Downloads` |
| `source ~/ros-humble-ros1-bridge/install/local_setup.bash` | 그대로 (경로 동일) |

`$HOME` 을 쓰도록 바꾸면 더 안전하다. 그 외에는 **건드리지 말 것.**

### Step 5 — 통합 실행

```bash
chmod +x ~/xycar_ws/etc/motor_vesc/motor
~/xycar_ws/etc/motor_vesc/motor
```

---

## 7. 검증 체크리스트 (순서대로)

| # | 확인 | 방법 | 기대 결과 |
|---|---|---|---|
| 1 | 컨테이너가 떴나 | `docker ps` | `ros1_container` 가 Up |
| 2 | ROS1 노드가 떴나 | `docker exec ros1_container bash -ci "source /opt/ros/noetic/setup.bash; source /root/noetic_ws/devel/setup.bash; rosnode list"` | `/xycar_motor`, `/vesc_driver`, `/ackermann_to_vesc` |
| 3 | **VESC 통신** | 위와 같은 방식으로 `rostopic echo /sensors/core -n1` | `voltage_input` 이 **실제 배터리 전압** |
| 4 | 브릿지가 떴나 | `ros2 node list` (호스트) | `/ros_bridge` 계열 노드 |
| 5 | **토픽 이름 대조** | 호스트 `ros2 topic list` ↔ 컨테이너 `rostopic list` | §9 (1) 참고 — **반드시 확인** |
| 6 | 조향 테스트 | §7 아래 명령 | 바퀴가 좌우로 움직임 |
| 7 | 구동 테스트 | **차를 들어올린 상태에서** speed 소량 | 바퀴 회전 |

### 조향/구동 테스트 명령

```bash
# 조향만 (speed=0) — 안전
ros2 topic pub -r 10 /xycar_motor std_msgs/msg/Float32MultiArray '{data: [30.0, 0.0]}'
ros2 topic pub -r 10 /xycar_motor std_msgs/msg/Float32MultiArray '{data: [-30.0, 0.0]}'

# 정지
ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray '{data: [0.0, 0.0]}'
```

> ⚠️ **구동 테스트는 반드시 차를 들어올려 바퀴가 뜬 상태에서** 할 것.
> `angle` 범위 ±50, `speed` 도 처음에는 5 이하로 시작할 것.

---

## 8. 알려진 함정

### (1) ROS_NAMESPACE 불일치 — 가장 가능성 높은 함정

원본 `x27.sh`/`x28.sh` 는 `export ROS_NAMESPACE=xycar` 를 설정한다. 그런데
ROS2 주행 노드들은 `'xycar_motor'` 를 **상대경로**로 발행한다. 네임스페이스가
붙으면 `/xycar/xycar_motor` 가 되는데, 컨테이너 안 ROS1 노드는 네임스페이스 없이
`/xycar_motor` 를 구독한다.

**반드시 양쪽에서 실제 토픽 이름을 대조할 것:**
```bash
ros2 topic list | grep motor                                     # 호스트
docker exec ros1_container bash -ci "source /opt/ros/noetic/setup.bash; rostopic list | grep motor"
```
어긋나면 `ROS_NAMESPACE` 를 빼거나, 브릿지에 remap 을 주거나, 발행 토픽명을
절대경로(`/xycar_motor`)로 맞춘다. **셋 중 무엇을 택했는지 반드시 기록할 것.**

### (2) ROS_DOMAIN_ID

원본 스크립트가 파일마다 다른 값을 쓴다 (`x27.sh`=21, `x29.sh`=7). 브릿지와
우리 ROS2 주행 노드가 **같은 DOMAIN_ID** 여야 서로 보인다. 하나로 통일하고
기록할 것.

### (3) 우리 ROS2 모터 패키지와 동시 실행 금지

젯슨에는 우리가 포팅한 ROS2판 `xycar_motor` / `vesc` 패키지도 빌드돼 있다
(`~/xycar_ws/src/xycar_motor/`). **이 도커 구성과 동시에 띄우면 `/dev/ttyMOTOR`
를 두 프로세스가 잡으려 해서 충돌한다.** 반드시 하나만.

```bash
ros2 node list | grep -E "vesc|xycar_motor"   # 중복 확인
```

### (4) `voltage_input: 0.0` 은 대개 배터리 미연결

ROS2 포팅본 테스트 때 관측된 값이다:
```
voltage_input: 0.0        current_motor: 181537.49
speed: 65536.0 (=2^16)    distance_traveled: 131072 (=2^17)
```
`xycar_motor` 노드가 "battery voltage is lower than 8V" 경고를 반복했다.
**이건 코드 버그가 아니라 배터리 전원이 VESC에 안 들어간 상태의 전형적 증상**일
가능성이 높다(USB만 꽂으면 통신 MCU는 살지만 전력단 ADC가 붕 뜬다).

**Step 5 검증 #3 에서 여전히 0.0 이면 소프트웨어를 의심하기 전에 배터리 물리
연결부터 확인할 것.** 그 경우 이 도커 작업 자체가 불필요했을 수도 있다 —
사용자에게 그 사실을 알릴 것.

### (5) `--privileged` 와 `--network host`

원본이 쓰는 옵션이다. 시리얼 접근과 roscore 통신에 필요하므로 그대로 둘 것.
젯슨에서도 동일하게 동작한다.

---

## 9. 실패 시 폴백

Step 3(ros1_bridge)에서 막히면 브릿지 없이도 갈 수 있는 우회로가 있다.
zip 안의 `Desktop/study/tools/race_tools/vesc_relay.py` 가 참고가 된다 —
`docker exec` 로 컨테이너 안에서 `rosrun topic_tools transform` 을 돌려
특정 토픽만 넘기는 방식이다.

같은 발상으로 **ROS2 → ROS1 단방향 릴레이 노드**를 만들 수 있다:
ROS2 `xycar_motor` 를 구독 → `docker exec ... rostopic pub` 으로 전달.
주기가 낮으면(10Hz 정도) 실용 가능하다. **브릿지 빌드가 3시간 넘게 걸리거나
실패하면 이 경로를 사용자에게 제안할 것.**

---

## 10. 작업 원칙

1. **벤더 소스(`noetic_ws/src/vesc/`, `xycar_motor/`)를 수정하지 말 것.**
   검증된 원본을 되살리는 게 목적이다. 값이 우리 ROS2 포팅본과 달라도 원본이 맞다.
2. **각 단계마다 검증하고 넘어갈 것.** 한 번에 다 만들고 끝에서 디버깅하면
   어디가 원인인지 못 찾는다.
3. **모터를 실제로 돌리기 전에 반드시 차를 들어올릴 것.**
4. **막히면 추측으로 진행하지 말고 사용자에게 물을 것.** 대회가 2026-08-25 이라
   시간이 없다. 잘못된 방향으로 오래 파는 게 가장 큰 손실이다.
5. 진행 상황과 실제로 택한 선택(§8의 네임스페이스/DOMAIN_ID 등)을 **이 문서 하단에
   기록**할 것. 다음 세션이나 다른 팀원이 읽는다.

---

## 11. 작업 기록 (진행하며 채울 것)

| 날짜 | 단계 | 결과 | 비고 |
|---|---|---|---|
| 2026-08-18 | **Step 0 원본 이미지 확보** | ✅ **완료** | 베이스=desktop-full · 커스텀=serial/ackermann_msgs/pyserial3.5/bashrc alias · motor는 alias (§6 Step 1 참고) |
| | Step 1 이미지 빌드 | | |
| | Step 2 catkin_make | | |
| | Step 3 ros1_bridge | | |
| | Step 4 스크립트 수정 | | |
| | Step 5 통합 실행 | | |
| | 검증 #3 VESC 전압 | | 실제 전압: ___ V |
| | 검증 #5 토픽 이름 | | 택한 방식: ___ |
| | ROS_DOMAIN_ID | | 확정값: ___ |
