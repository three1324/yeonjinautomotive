# xycar_motor (ROS2)

`noetic_ws/src/xycar_motor`(ROS1) + `vesc_ackermann`(ROS1)을 ROS2로 이식한 모터 구동 스택.
`vesc_driver`/`vesc_ackermann`/`vesc_msgs`는 [f1tenth/vesc](https://github.com/f1tenth/vesc/tree/ros2)를
그대로 가져다 썼고(`../vesc`), 이 패키지는 xycar 프로젝트 고유의 `xycar_motor` 토픽 인터페이스만 담당한다.

## 데이터 흐름

```
(다른 ROS2 패키지: track_drive 등)
   │ xycar_motor 토픽, Float32MultiArray [angle, speed]  (-50~100 범위)
   ▼
xycar_motor_node ── 캘리브레이션 적용, 속도 램핑 ── AckermannDriveStamped
   │ ackermann_cmd
   ▼
ackermann_to_vesc_node (vesc_ackermann) ── ERPM/서보 위치로 변환
   │ commands/motor/speed, commands/servo/position
   ▼
vesc_driver_node ── /dev/ttyMOTOR 시리얼로 VESC와 통신
```

`/sensors/core`(VescStateStamped)는 반대 방향으로 흘러 배터리 전압·고장 코드를 감시한다.

## 실행

```bash
ros2 launch xycar_motor xycar_motor.launch.py
```

## 빌드 전 확인할 것 (젯슨 오리진에서)

- [ ] `rosdep install --from-paths src --ignore-src -r -y` 로 `serial_driver`(`transport_drivers`), `ackermann_msgs` 등 외부 의존성 설치
- [ ] `/dev/ttyMOTOR` udev 규칙이 ROS1 때와 동일하게 잡혀 있는지 확인 (안 잡혀있으면 `robot-bringup` 스킬 참고)
- [ ] `config/vesc.yaml`의 `wheelbase: 0.26`은 자리표시값 — 실차 축간거리 실측 후 갱신
- [ ] `angle_offset`, `angle_bias_1/2`, `speed_bias_1/2`는 ROS1에서 항상 0이었으므로 그대로 0으로 시작 — 실주행 테스트하며 조향 중심 튜닝
- [ ] 저속에서 실제 배선/방향이 ROS1 때와 같은지 확인 (`steering_angle_to_servo_gain`이 음수인 이유가 배선 반전 보정이라 반대로 꽂으면 반대로 꺾인다)

## motor_type=1 (아두이노 직결) 경로는 이식하지 않음

ROS1 원본은 `motor_type`으로 VESC/아두이노 두 경로를 선택할 수 있었지만 `rospy.get_param`이
주석 처리되어 항상 `motor_type=0`(VESC)으로 하드코딩되어 있었다. 실제로 쓰이지 않던 죽은 코드라
ROS2 이식 시 VESC 경로만 옮겼다. 아두이노 직결이 다시 필요해지면 원본
`noetic_ws/src/xycar_motor/src/xycar_motor.py`의 `set_arduino`/`auto_drive_arduino`를 참고할 것.
