# 외부에서 가져온 패키지 (vendored)

이 워크스페이스에는 외부 저장소에서 가져온 패키지가 섞여 있다.
git 저장소로 관리하기 위해 각 패키지 안의 `.git` 디렉토리는 **제거**했다.

> **왜 제거했나**
> `.git` 이 들어있는 디렉토리를 상위 저장소에서 `git add` 하면, git 이 이를
> 서브모듈(gitlink)로 취급해 **내용을 추적하지 않는다.** 그대로 올리면 팀원은
> 빈 폴더를 받게 되고, 아래 "수정한 부분"이 전달되지 않는다.
> 서브모듈로 두는 방법도 있지만, 우리가 원본을 수정했기 때문에 적절하지 않다.

가져온 시점의 출처는 아래와 같다. 나중에 원본과 비교하거나 업데이트해야 하면
이 정보로 같은 커밋을 다시 받아 diff 를 뜨면 된다.

| 패키지 | 출처 | 브랜치 @ 커밋 |
|---|---|---|
| `vesc/` | https://github.com/f1tenth/vesc.git | `ros2` |
| `rf2o_laser_odometry/` | https://github.com/MAPIRlab/rf2o_laser_odometry.git | `ros2` @ `b38c68e` |
| `xycar_device/xycar_lidar/YDLidar-SDK/` | https://github.com/YDLIDAR/YDLidar-SDK.git | `master` @ `16ca1f4` |
| `yolo_ros/` | https://github.com/mgonzs13/yolo_ros.git | `main` @ `a99a101` |
| (참고) `amd/Desktop/sllidar_ros2/` | https://github.com/Slamtec/sllidar_ros2.git | `main` @ `3430009` |

---

## ★ 원본에서 수정한 부분 — 재클론하면 다시 적용할 것

### 1. `vesc/vesc_driver/src/vesc_interface.cpp` — 흐름제어

```diff
- auto fc = drivers::serial_driver::FlowControl::HARDWARE;
+ auto fc = drivers::serial_driver::FlowControl::NONE;
```

VESC 의 COMM 포트와 젯슨 40핀 헤더 UART 는 **둘 다 TX/RX/GND 만 있고 RTS/CTS 핀이
없다.** 하드웨어 흐름제어를 켜두면 드라이버가 CTS 신호를 기다리다 송신이 막혀
**모터가 전혀 반응하지 않는다.** USB(CDC-ACM) 연결에서는 흐름제어 설정 자체가
무시되므로, NONE 은 UART/USB 두 경우 모두에서 안전하다.

### 2. `rf2o_laser_odometry/package.xml` — 매니페스트 포맷

원본은 package format 1 (ROS1 스타일) 이었다. ROS2 에서 두 가지가 문제였다:

- `<run_depend>` : format 1 전용 태그 → format 3 의 `<exec_depend>` / `<depend>` 로 이관
- `<build_depend>cmake_modules</build_depend>` : ROS1 전용 패키지라 ROS2 rosdep 이
  해석하지 못해 **`rosdep install` 이 실패**한다. `eigen3_cmake_module` 이 이미
  buildtool_depend 에 있으므로 제거해도 무방하다.

---

## 워크스페이스에 없던 것을 옮긴 이력

`rf2o_laser_odometry` 는 원래 `amd/Desktop/` 에만 있어서 **워크스페이스에서 빌드되지
않았다.** apt 에 없는 커뮤니티 패키지라 소스 빌드가 필수인데, `my_slam` 이 이것에
의존하므로 `xycar_ws/src/` 로 옮겼다.
