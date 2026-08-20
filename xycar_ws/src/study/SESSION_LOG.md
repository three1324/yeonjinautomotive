# 작업 기록 (세션 로그)

> 커밋 메시지에는 "무엇을 왜 바꿨나"가 들어 있다. 이 파일은 그보다 위,
> **"그날 무엇이 문제였고 어떻게 좁혀 들어갔나"**를 남긴다. 같은 증상을 다시
> 만났을 때 처음부터 다시 추적하지 않기 위한 것이다.
>
> 최신 항목을 맨 위에 쌓는다.

---

## 2026-08-21 — 조향 부호를 **측정으로** 확정하다 (+angle = 좌회전)

### 무엇을 했나

차를 들어올리고 주행 노드를 모두 끈 뒤, `/xycar_motor` 에 `[20.0, 0.0]` 을
직접 발행했다. **앞바퀴가 왼쪽으로 돌았다.**

    명령 +값 = 좌회전
    좌커브 -> offset_near < 0 -> err < 0 -> raw < 0
    좌커브에서 좌회전하려면 angle > 0  ⇒  angle = -raw  ⇒  invert = true

    driver_node.steer.invert   false -> true
    viz_node.angle_sign        -1.0  -> 1.0   (REP-103 과 규약이 같아짐)
    rubbercone_node.invert_steer  false 유지   (독립 경로 — 아래)

### 8/21 아침의 역산이 틀렸다

어제 "S자 좌커브에서 우측 이탈" 증상으로 부호를 역산해 `invert` 를 false 로
되돌렸다. 그 역산은 **"인지->제어 사이에 다른 반전이 없다"를 전제**했는데,
그 전제를 검증하지 못한 채 부호만 뒤집은 것이다. 같은 8/19 실측표
("양수 명령 = 양수 실측, 왼쪽 기준")와 오늘 측정이 독립적으로 일치하므로
이제 근거는 2:0 이다.

**교훈: 증상에서 부호를 역산하지 말 것. 30초면 직접 잴 수 있다.**
같은 실수를 이틀 연속 다른 방향으로 했다 — 8/19 에는 라이다에 오염된 관찰로,
8/21 에는 검증 안 된 전제 위의 계산으로.

### 그러면 그 우측 이탈은 왜 났나 ★ 미해결

`invert: true` 는 8/19 낮부터 쓰던 값이고 그 상태에서 이탈이 났다. 부호가
맞다면 원인은 따로 있다. 그 뒤에 고쳐진 후보가 셋이다:

- 라이다가 콘 구간 밖에서 제어권을 뺏고 있었음 (지금은 구독조차 안 함)
- `lateral._offset()` 회피 목표 부호 뒤집힘 (8/21 수정)
- `angle_limit: 50` 포화 와인드업 — 실제 기계 포화는 35도 (8/21 35 로 수정)

셋 다 고쳐졌으니 **재주행에서 이탈이 재현되는지**가 다음 확인 항목이다.
재현되면 네 번째 원인이 있다는 뜻이다.

### rubbercone 은 왜 안 뒤집나

코드상 완전히 독립된 경로다. `rubbercone_node.py:332` 가 자기 `invert_steer` 를
적용해 완성된 각도를 `/cone_cmd` 로 내보내고, `driver_node` 는 그것을
`SteeringController` 를 거치지 않고 그대로 통과시킨다. `steer.invert` 는 그
값에 닿지 않는다. 그쪽 `false` 는 팀원 실차 검증값이라 그대로 둔다.
VEHICLE_TEST 에 "같이 뒤집으라"고 적혀 있던 것은 잘못이라 오늘 고쳤다.

## 2026-08-19 (밤) — 라이다 간섭, 콘 구간 판정, TensorRT

### 발단

> "라바콘 구간이 아닌데 라이다가 경로에 간섭하고 있는 거 같은데 맞아?"

맞았다. 다만 간섭 경로가 예상과 달랐다.

### 1. 라이다가 간섭한 진짜 경로

설계상 콘 구간 밖에서 라이다는 이미 차단돼 있었다:

- `fusion.py` — 구간 밖이면 복도 가중치 0 (차선 단독)
- `longitudinal.py` — `obstacle_cap_in_cone_only: true` 라 전방거리 상한도 건너뜀
- `lateral.py` — 회피는 카메라 전용

**그런데 "지금이 콘 구간인가"를 라이다가 판정하고 있었다.** 그게 새는 곳이었다.

`rubbercone_node._update_zone()` 은 전방 부채꼴(0.2~1.5m, ±55°) 안의 점을
20개 이상이면 구간으로 본다. 문제는 그 점이 **ROI/클러스터 필터를 안 거친
원본 스캔점(`all_points`)** 이라는 것이다. 같은 파일 안에 콘 판별 필터
(`cluster_max_span_m: 0.4` — 벽/사람 제외)가 있는데 zone 판정은 그걸 안 탄다.
YDLidar 해상도면 1.2m 앞 벽 하나가 수백 점이다.

구간으로 오판정되면 `driver_node` 가 제어권을 통째로 넘겨서
(`if in_cone_zone and cone_cmd_fresh: return`), 차가 차선을 무시하고
라이다 Pure Pursuit 로 달린다. 이것이 "이상하게 주행"의 정체다.

### 2. 조치 — 판정은 카메라, 주행은 라이다

커밋 9e580fc 에서 지웠던 `cone_zone.ConeZoneDetector` 를 되살렸다.
`/cone_zone_active` 는 구독은 유지하되 **제어에 안 쓰고** `debug_state` 의
`zone_lidar` 로만 내보낸다(두 판정이 어긋나는 빈도를 현장에서 보려고).

구간 안에서의 조향·속도는 여전히 `rubbercone_node` 가 만든다 — 팀원이 실차에서
검증한 로직이라 건드리지 않았다.

### 3. 개수만으로는 부족했다 — 크기 조건

처음엔 콘 3개 진입 / 1개 이탈로 했는데, 사용자가 실제 영상을 보고 지적했다:
**"멀리서 바뀌면 안 되니까 콘 크기 기준으로 트리거하자."**

영상으로 확인해보니 정확한 지적이었다. 라바콘은 줄지어 서 있어서
**직선 끝에서도 8~13개가 한꺼번에 보인다.**

| 문턱값 | 진입 시점 | |
|---|---|---|
| 크기조건 없음 | 30.5s (콘 8개, h=24px) | 한참 멀다 |
| 90px | **43.2s** | 채택 |
| 100px | 43.4s | |
| 150px | 45.6s | 이미 콘 옆 |

개수 조건(8개)은 30.5s 에 이미 찬다. **크기 조건이 13초 앞당겨 켜지던 것을 막는다.**

크기 지표는 `cone_near_y`(하단 y)가 아니라 **bbox 높이**를 새로 만들었다
(`detect.cone_max_h` → `/objects` 6번째 필드). 하단 y 는 카메라 피치·노면
기울기에 흔들리고 콘이 화면 아래로 잘리면 오히려 작아진다.

이탈에는 크기 조건을 안 건다 — 나갈 때 남은 콘은 멀어지며 작아지는데 크기로도
끊으면 개수보다 먼저 이탈해 아직 콘 사이인 차가 차선으로 돌아간다.

### 4. 헤맨 것 — "zone 이 안 뜬다"

영상 테스트에서 `cone_zone` 이 계속 false 였다. `/debug_state` 를 직접 찍어보니
`reason: disabled`, `zone_why: init` 이었다.

`cone_zone.update()` 가 `on_tick()` 의 조기 return(`disabled`/`stale`/신호대기)
**뒤에** 있어서, `/drive_enable` 을 켜기 전에는 판정이 아예 안 돌았다.
벤치 테스트에서는 enable 을 안 켜는 게 정상이라 영영 안 보였던 것이다.
판정은 순수한 관측이므로 `on_tick()` 맨 앞으로 옮겼다.

### 5. "같은 영상인데 뜰 때가 있고 안 뜰 때가 있다"

당시엔 코드 버전이 세션 중에 세 번 바뀐 탓이 컸다(라이다 판정 → 3개/1개 →
8개+크기). 다만 진짜 비결정 요소도 하나 있었다: 인지가 영상 fps 를 못 따라가
프레임이 버려지는데 **어느 프레임이 버려지는지가 실행마다 달랐다.**
(`/image_raw` 가 BEST_EFFORT depth=1)

이건 아래 성능 작업으로 대부분 사라졌다.

### 6. TensorRT — 그리고 전력 모드가 더 컸다

사용자 요청으로 FP16 엔진을 만들었다(8분). 두 가지를 배웠다.

**함정:** `YOLO(path)` 만 쓰면 ultralytics 가 엔진을 **detect 로 로드**해서
출력 `(1,46,8400)` 의 32개 마스크 계수를 클래스 점수로 오독한다. 첫 측정에서
"클래스 24가 300개"가 나오고 콘이 하나도 안 잡혔다. `task="segment"` 필수.
`.pt` 는 파일에 태스크가 들어 있어 티가 안 난다.

**성능 (perception_node 실측, 시각화 없음):**

| 조건 | fps |
|---|---|
| `.pt` + 40W | 12.4 |
| `.engine` + 40W | 16.5 (+33%) |
| **`.engine` + MAXN** | **약 29 (+76%)** |

**전력 모드가 TensorRT 보다 효과가 컸다.** 젯슨이 40W 로 묶여 있던 게 진짜
병목이었다. 절차는 `VEHICLE_TEST.md §2-1`.

FP16 이 검출에 주는 영향은 확인했다 — 콘 구간 18지점 중 2곳만 콘 1~2개 차이,
진입 시점 43.3s 로 `.pt` 와 동일. 문턱값은 엔진/`.pt` 공용이다.

### 아직 검증 안 된 것 ★

**전부 영상 재생으로만 확인했다. 실차 주행은 아직이다.**

replay 에는 `/scan` 이 없어서 **이번에 고친 버그 자체를 재현할 수 없다.**
확인된 것은 "카메라 판정이 제대로 켜지고 꺼지는가"까지다.

실차에서 볼 것:

1. **`zone_lidar: true` 인데 `cone_zone: false`** — 이 조합이 뜨면 라이다
   오판정을 카메라가 막고 있다는 뜻이고, 이번 수정이 실제로 동작한 것이다.
2. `enter_min_size_px: 90` 이 실차 카메라에서도 맞는가. **이 값은 화각·해상도
   종속이다**(영상은 632x480). 콘 앞에서 `debug_state` 의 `cone_h` 를 읽어 확인.
   너무 크면 로그에 `far(cone N, h..px)` 가 계속 뜨고 구간에 못 들어간다.
3. 콘 구간에서 `reason: cone_zone(rubbercone)` 이 뜨는가 (라이다가 몰고 있다는 표시).
4. `jetson_clocks` 는 **재부팅하면 풀린다.** 전원 넣을 때마다 다시.

---

## 후속 (2026-08-21) — 비상 폴백 두 개를 제거

위 §1 에 "설계상 이미 차단돼 있었다"고 적은 두 경로(`fusion.py` 의 복도 융합,
`longitudinal.py` 의 전방거리 상한)를 **아예 들어냈다.**

### 왜

둘 다 `_drive_cone_zone()` 뒤의 `return` 아래에 있어서, **정상 주행에서는
도달조차 못 하는 코드**였다. 실행되는 경우가 딱 하나 — "콘 구간인데
rubbercone_node 가 죽은" 상황뿐이었다.

그런데 그 폴백이 안전하지도 않았다. `/corridor` 는 합성 데이터로만 검증됐고
`px_per_meter(300.0)` 는 실측 전 값이라, 라이다가 죽었을 때 **검증 안 된
추정치로 콘 사이를 계속 달리는** 구조였다.

### 어떻게 바꿨나

    콘 구간 + rubbercone 살아있음  ->  라이다 주행 (그대로)
    콘 구간 + rubbercone 죽음      ->  **정지** (기존: 복도 추정으로 계속 주행)
    콘 구간 아님                    ->  차선 단독. 라이다 토픽을 구독조차 안 함

콘 사이에서 멈추는 벌점이, 콘을 치거나 코스를 이탈하는 것보다 낫다는 판단.

### 제거된 것

    my_driver/my_driver/fusion.py          (삭제)
    my_driver/tools/fusion_sim.py          (삭제 — 위 모듈 전용 시뮬)
    driver_node  /corridor, /obstacle 구독  (제거)
    longitudinal stop_dist/slow_dist/obstacle_cap_in_cone_only (제거)
    debug_state  source, corridor_weight, front_dist 필드 (제거)

`obstacle_node` 는 계속 돌지만 이제 주행에 쓰이지 않는다 — RViz 시각화용
`/corridor_path` 만 `viz_node` 가 쓴다.

---

## 2026-08-21 · 왜 라이다로 달렸나 — launch 의 params_file 누수

콘 구간이 아닌데(cone=0), 차선도 신호등도 안 잡힌 상태에서 모터 명령이
들락거렸다. 원인은 코드 로직이 아니라 **launch 였다.**

    $ ros2 topic info /xycar_motor --verbose
    Publisher count: 2
      rubbercone_node      ← 있으면 안 되는 것
      driver_node

    $ ros2 param get /rubbercone_node drive_topic
    String value is: xycar_motor      ← drive_params.yaml 의 cone_cmd 가 아니라 기본값

    $ ps -ef | grep driver_node
    ... --params-file .../xycar_lidar/share/xycar_lidar/params/ydlidar.yaml

우리 노드 4개(perception/obstacle/rubbercone/driver)가 전부 **ydlidar.yaml**
을 받고 있었다. `drive_params.yaml` 의 값이 하나도 안 들어간 채, 전부
코드 기본값으로 떠 있었던 것이다.

### 왜

`IncludeLaunchDescription(..., launch_arguments={'params_file': ydlidar.yaml})`
의 launch_arguments 는 **현재 스코프에** `SetLaunchConfiguration` 을 깐다.
라이다 include 가 우리 노드들보다 앞에 있으므로, 그 뒤에 오는 모든
`LaunchConfiguration('params_file')` 이 ydlidar.yaml 로 바뀌었다.

라이다 yaml 을 못박은 것 자체는 맞는 조치였다(그게 없으면 라이다가
시리얼 포트를 못 열고 죽는다). 빠진 건 **스코프**였다.

### 고친 것

1. `drive.launch.py` — 라이다 include 를 `GroupAction(scoped=True)` 로 감쌌다.
   못박은 값이 그 그룹 밖으로 새지 않는다.
2. `rubbercone_node.py` — `drive_topic` 기본값을 `xycar_motor` -> `cone_cmd`,
   `zone_topic` 기본값을 `/cone_zone_active` 로. params 가 한 번이라도 안
   실려도 이 노드가 모터에 직접 쏘는 일은 다시는 없다.
3. `preflight.py` — `/xycar_motor` 발행자가 2개 이상이면 FAIL.

### 재발 방지 점검 한 줄

    ros2 topic info /xycar_motor --verbose | grep -c "Node name"   # 1 이어야 한다

### 이어서 — obstacle_node 통째로 삭제

위 누수를 고치고 나서도 RViz 에는 라이다에서 나온 파란 복도선이 계속 그려졌다.
주행에는 안 쓰이지만(구독자가 viz_node 뿐), **시각화만을 위해 라이다 복도
추정을 상시 돌리는 것 자체가 남겨둘 이유가 없다.** "라바콘 주행만 남기고
라이다는 없앤다"는 결정에 어긋나기도 한다.

삭제:

    my_obstacle/my_obstacle/obstacle_node.py   (노드)
    my_obstacle/my_obstacle/corridor.py        (복도 중앙선 추정)
    my_obstacle/my_obstacle/sectors.py         (섹터 최근접 거리)
    my_obstacle/tools/corridor_sim.py          (위 모듈 전용 시뮬)
    my_obstacle/launch/obstacle.launch.py
    setup.py entry_point  obstacle_node
    drive.launch.py       obstacle Node
    drive_params.yaml     obstacle_node 블록 · viz_node.corridor.*
    viz_node              /corridor /obstacle /corridor_path 구독, /viz/ref_path 발행,
                          전방거리 마커, 복도 색선 마커
    drive.rviz            ref path · corridor path · cone walls 디스플레이

이제 `/scan` 을 구독하는 노드는 **rubbercone_node 하나**다(+viz_node 의
포인트클라우드 변환). 그 노드는 콘 구간에서만 제어권을 갖는다.

RViz 에서 "지금 뭘 따르고 있나"는 상태 텍스트 **색**으로 본다 —
노랑=카메라 차선 / 빨강=라바콘 구간.

    $ ros2 topic list | grep -E '/corridor|/obstacle'   # 아무것도 안 나와야 한다
