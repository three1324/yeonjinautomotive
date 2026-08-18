# 2026-08-18 작업 로그 — YOLO GPU/TensorRT 가속 + 파이프라인 단계별 시각화

## 1. TensorRT / PyTorch GPU 가속 (my_perception)

**출발점**: Jetson Orin NX(JetPack 6.2.1)에 CUDA 12.6 / cuDNN 9.3 / TensorRT
10.3.0은 이미 설치돼 있었고 PyTorch 2.8.0도 `torch.cuda.is_available()=True`
였지만, `ultralytics` 패키지가 없어서 `perception_node.py`가 임포트 단계에서
죽는 상태였음(GPU 가속 여부를 논하기 전에 실행 자체가 안 됨).

**한 일**:
- `pip install --no-deps ultralytics` — 기존 Jetson 전용 torch/torchvision
  빌드를 pip이 덮어쓰지 않도록 `--no-deps`로 설치.
- 부족한 하위 의존성(`tqdm`, `ultralytics-thop`, `onnx==1.16.1`,
  `onnxslim==0.1.96`, `ml_dtypes==0.3.2`, `protobuf 4.25.9`)을 시스템
  numpy 1.21.5 / cv2(ROS `cv_bridge`가 씀)와 호환되는 **구버전**으로 골라서
  설치 — 최신 onnx/ml_dtypes는 numpy>=2.0을 요구해서 그대로 깔면 ROS
  이미지 파이프라인이 깨질 뻔했음.
- 설치 전후 `torch`/`torchvision`/`numpy`/`cv2` 버전 불변 확인 완료.
- `models/best5.pt` → `models/best5.engine` (TensorRT FP16, imgsz=[480,640])
  export 성공 (엔진 빌드 약 470초 소요).

**벤치마크** (640×480, 30프레임 평균):

| 경로 | 평균 지연 | FPS |
|---|---|---|
| PyTorch (.pt, cuda:0) | 42.7 ms | 23.4 |
| TensorRT (.engine, FP16) | 21.5 ms | **46.5** |

→ 약 2배 가속. `.engine`/`.onnx`는 빌드 장비 종속이라 `.gitignore`에 추가,
git에는 안 올라감.

**남은 참고사항**:
- `.engine`으로 `YOLO(...)` 로드할 때 `task="segment"`를 명시해야 함 —
  안 그러면 task를 detect로 잘못 추측해서 클래스 인덱스가 깨짐
  (`KeyError` 발생 확인).
- `launch/perception.launch.py`의 `model_path` 인자 설명에 `.engine` 사용법
  주석 추가 (기본값은 이식성 위해 `.pt` 유지).

---

## 2. 파이프라인 단계별 오프라인 시각화 도구

**새 파일**: `amd/xycar_ws/src/study/my_driver/tools/pipeline_sim.py`

영상 파일 하나로 **YOLO 검출 → 인지 → 판단(FSM) → planning(차선/복도 융합,
추월) → 제어(조향·속도)** 5단계를 전부 재현하는 ROS-프리 오프라인 도구.
`driver_node.py`가 실제로 쓰는 것과 **같은 모듈**
(`my_driver.fsm/fusion/lateral/longitudinal/control`)을 그대로 가져다 써서
로직이 실제 노드와 갈라지지 않게 함. 파라미터(임계값·게인)도
`drive_params.yaml`을 그대로 읽어서 실차와 같은 값으로 계산.

**출력**:
1. `<name>_pipeline.mp4` — 좌: YOLO 원시 검출, 우: 인지 오버레이 +
   판단/planning/제어 HUD 텍스트를 합친 시각화 영상.
2. `<name>_trajectory.png` — angle/speed 제어 명령을 축간거리 0.333m
   (2026-08-16 실측, `xycar_motor/config/vesc.yaml`), speed_weight=0.08,
   angle_limit 50→약19.5°(자전거모델)로 적분한 **개루프 예상 경로**.
   ⚠️ 카메라가 실제 지나간 경로가 아니라 "이 판단 로직이 계산한 명령대로
   움직였다면"의 시뮬레이션 — 정성적 확인용(방향이 맞는지)이지 절대
   위치가 아님.
3. 콘솔 로그 — `driver_node._log()`와 동일 형식
   (`[state] angle=.. speed=.. | off=.. q=.. | light=.. cone=.. | reason`).

**이 도구가 실제 노드와 다른 점**:
- 라이다 `/corridor`가 없어 항상 invalid 취급 → 카메라 차선만으로 판단
  (라바콘 구간은 실제와 다를 수 있음).
- `/drive_enable`, 신호등 대기는 기본적으로 skip(`auto_start=True`).
  `--wait-light`로 실전처럼 초록불 대기부터 시작 가능.

**사용법**:
```bash
cd amd/xycar_ws/src/study/my_driver
python3 tools/pipeline_sim.py <영상경로> \
  --model ../my_perception/models/best5.engine \
  --out-video 저장경로.mp4
```

---

## 3. 테스트 실행 결과

세 개 영상으로 실행:

| 영상 | 길이 | 결과 |
|---|---|---|
| `reference/videos/xycar_track1.mp4` | 27.5s | 커브에서 감속(quality 저하 시 speed↓), offset 튀는 프레임 1건 발견(+380px, drive_params.yaml에 이미 기록된 "2차 피팅 발산" 케이스와 일치) |
| `/home/e-on/테스트용(신호등만).mp4` | 213s | traffic_light 검출됨. 경로 적분 결과가 **거의 폐루프**로 돌아옴(출발점 근처로 복귀) — 판단 로직이 대체로 올바른 방향으로 도는 정황 |
| `/home/e-on/테스트용(신호등미포함).mp4` | 214s | 라바콘 구간(9~14개 검출, t=41~60s)에서 회피 조향 반영, 경로가 S자로 구불구불 — 콘 회피 로직이 실제로 작동함을 확인 |

**출력 파일 위치**: `reference/videos/pipeline_out/`
(`.gitignore`에 추가됨 — 각 파일 35~420MB라 git에는 안 올라감)
```
xycar_track1_pipeline.mp4
signal_only_pipeline.mp4 / signal_only_pipeline_trajectory.png
no_signal_pipeline.mp4 / no_signal_pipeline_trajectory.png
```

---

## 4. 변경된/추가된 파일 목록

- `amd/xycar_ws/src/study/my_perception/models/best5.engine` (신규, gitignore)
- `amd/xycar_ws/src/study/my_perception/models/best5.onnx` (신규, gitignore)
- `amd/xycar_ws/src/study/my_perception/launch/perception.launch.py` (수정 — model_path 설명에 engine 사용법 추가)
- `amd/xycar_ws/src/study/my_driver/tools/pipeline_sim.py` (신규)
- `.gitignore` (수정 — `*.engine`, `*.onnx`, `reference/videos/pipeline_out/` 추가)
- `reference/videos/pipeline_out/*` (신규, gitignore — 생성된 시각화 영상/경로 이미지)

## 5. 다음에 이어서 할 만한 것

- 실제 카메라 해상도(640×480, 젯슨 실기기 확인값)와 다른 632×480 테스트
  영상 두 개를 썼음 — 두 값이 다르다는 drive_params.yaml 주석(2026-08-16)
  참고, 실차 캘리브레이션 시 유의.
- offset +380px 급변 케이스처럼 lane.py의 이상치 방어 파라미터를 더
  조일지 검토할 근거 영상이 쌓임 (`xycar_track1.mp4` t=21.1s 부근).
- TensorRT `.engine`은 이 젯슨(Orin NX, TensorRT 10.3, JetPack 6.2.1)
  전용이므로 다른 기기/재설치 시 `pipeline_sim.py`나 launch에서
  `.pt`로 재-export 필요.
