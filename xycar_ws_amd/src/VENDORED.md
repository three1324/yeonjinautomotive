# 이 워크스페이스에 대하여

`amd.zip`(AMD 차량 홈디렉토리 백업)의 `xycar_ws/src/`를 **손대지 않고 그대로**
꺼낸 것이다. `track_drive`, `xycar_application`, `xycar_device`, `yolo_ros`
네 개는 AMD 차량이 실제로 쓰던 조직위 벤더 패키지 원본이고, 여기에 우리
`study/` 패키지 6개(`my_perception`/`my_obstacle`/`my_driver`/`my_bringup`/
`my_slam`/`my_debug`)만 추가했다.

## 왜 `amd/xycar_ws/`와 별도로 뒀나

`amd/xycar_ws/`는 이 프로젝트 진행 중에 조립한 워크스페이스라 `vesc`/
`xycar_motor`(우리가 만든 ROS2 네이티브 VESC 포팅) 등 AMD 원본에는 없던
패키지가 섞여 있고, `track_drive`/`xycar_application`은 아예 없다.

이 워크스페이스(`xycar_ws_amd/`)는 그런 조립 이력 없이 **AMD가 실제로
검증한 벤더 코드 그대로**를 기준으로 삼기 위한 것이다. 인지/판단/제어
버그(조향 반전, "가다 멈추고" 등)를 잡을 때 "혹시 우리가 손댄 벤더 패키지
때문 아닌가"라는 변수를 없애는 게 목적이다.

## 모터

이 워크스페이스에는 `vesc`/`xycar_motor` 패키지가 **없다**. AMD 원본과
동일하게 모터는 별도 ROS1 도커(`ros1_container`) 안의 벤더 노드가 담당하고,
`study/my_bringup`은 `xycar_motor` 토픽에 `Float32MultiArray [angle, speed]`를
**발행만** 한다. 실행은 반드시 `drive_amd.launch.py`를 쓸 것 —
`drive.launch.py`(젯슨 네이티브 VESC용)를 쓰면 `xycar_motor` 패키지를 못 찾아
launch가 죽는다. 도커/브릿지 설정은 `JETSON_ROS1_DOCKER_MOTOR.md` 참고.

## 제외한 것

| 제외 | 이유 |
|---|---|
| `.git/` (yolo_ros 등에 딸려 있던 것) | 상위 저장소가 gitlink로 잘못 인식해 내용을 못 가져간다 (아래 이유는 기존 `amd/xycar_ws/src/VENDORED.md`와 동일) |
| `xycar_device/xycar_lidar/YDLidar-SDK/build/` | 빌드 산출물, 아키텍처 종속 |
| `yolo_ros/yolov8m*.pt` (seg/pose/detect, 각 50MB+) | 사전학습 범용 모델. 우리는 `study/my_perception/models/best5.pt`만 쓴다 |
| `__pycache__/` | 재생성됨 |

추출 시점: 2026-08-18, `amd.zip` 기준.
