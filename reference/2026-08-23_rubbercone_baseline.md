# 2026-08-23 — 라바콘 구간 확정 기준선

사용자 확인: **"지금 라이다 괜찮다, 일단 이걸로 돌리자."**

라바콘 구간 주행이 실차에서 납득할 만하게 도는 것을 확인한 시점의 설정을
그대로 기록해 둔다. **이 값들부터 다시 시작한다** — 이후 튜닝이 나빠지면
여기로 되돌아온다.

- 기준 커밋: `0663e1a`
- 대상: `rubbercone_node` (라이다 전담), `driver_node.cone_zone` (구간 판정)
- 검증: `python3 my_obstacle/tools/synth_check.py` 전부 통과

---

## 1. 이 상태가 되기까지 — 이번 라운드에 바뀐 것

증상은 하나였다: **조향이 약하게 들어가 커브에서 못 돌고 콘을 쳤다.**
네 가지를 순서대로 손봤고, 마지막 것이 가장 크게 들었다.

### (a) ROI 확장 — `range_max_m` 1.40 → 2.50, 폭 제한 무효화

깊은 굽이(R≈0.50m)에서 **바깥쪽 벽이 거리 상한 밖으로 밀려나 한쪽 벽이
통째로 안 보였다.** 반파장 1.30m / 횡변위 0.70m 코스를 작도해 확인:

```
s=1.2  좌4/우0      s=2.4  좌0/우4      s=3.6  좌4/우0
```

세 지점 모두 한쪽이 0개 — 양벽 중심선이 한 번도 안 만들어지고 계속 편측
폴백으로 갔다. 코스 전구간에서 "각 벽 3개"에 필요한 거리를 구하면
2.50m 에서 100% 다 (1.40 → 23%, 1.80 → 56%, 2.20 → 84%).

`side_half_m` 은 `range_max_m` 과 같은 값(2.50)으로 두어 무효화했다.
|y| 조건이 살아 있으면 달성 가능 지점 자체가 154개 중 27개로 줄어든다 —
폭 제한이 콘을 먼저 잘라내기 때문이다.

> ⚠️ 옆 벽을 이제 거리로는 못 자른다. **콘 모양 필터
> (`cluster_max_span_m: 0.20`) 하나가 벽을 막고 있다.** 끊긴 벽 조각이
> 콘으로 통과할 수 있으니 `debug_show_scan_points:=true` 로 확인할 것.

### (b) 목표점 스무딩 제거

`target_smoothing_alpha` (EMA) + `max_target_step_m` (프레임당 clamp) 을
**없앴다.** 목표점을 다듬지 않고 그대로 Pure Pursuit 에 준다.

이 둘이 조향 지연의 직접 원인이었다. 대가는 콘 검출이 프레임마다 튀면
조향도 그만큼 튄다는 것 — 지금까지는 문제되지 않았다.

### (c) Pure Pursuit 기준점 이동 — 0.41 → 0.24 → **0.14**

가장 크게 들은 변경이다. 기준점을 뒤축에서 라이다 쪽으로 당겼다.

```
라이다 0.0   ·   0.14 (현재)   ·   차량 중앙 0.24   ·   뒤축 0.41
```

`synth_check` 의 "참 곡률 대비 조향" (음수 = 코너 컷):

| 기준점 | 조향 |
|--------|------|
| 0.41 (뒤축) | −4.7도 |
| 0.24 (차량 중앙) | −1.2도 |
| **0.14 (현재)** | **+1.0도** |

> ⚠️ **이것은 교과서 Pure Pursuit 이 아니다.**
> `delta = atan(2 L sin(alpha) / ld)` 의 유도는 기준점이 **뒤축**일 때만
> 성립한다. 앞으로 당기는 것은 실질적으로 게인을 올린 것과 같다 —
> "더 정확해진" 것이 아니라 "더 세게 꺾는" 것이다.
>
> 0.14 는 옛 코드의 0.0(라이다 원점, 약 2배 과조향이 나던 값)에 꽤 가깝다.
> **지그재그가 나오면 여기부터 의심하고 0.24 → 0.41 순으로 되돌릴 것.**

### (d) 구간 진입/이탈 — `enter_n` 8→9, `exit_n` 4→**3**

진입을 보수적으로(오검출 방어), 이탈은 어렵게(구간 중간 이탈 방지).
히스테리시스 폭이 6 으로 넓어졌다.

한때 `exit_n: 6` 으로 올렸다가 폭이 3 밖에 안 남아 되돌렸다 — 라바콘
구간 **한가운데서 제어권이 차선주행으로 넘어가는 것**이 최악이다.

### 되돌린 것 — 폴백 두 개는 지웠다가 복구했다

편측 폴백(`offset_from_single_wall`)과 FTG 폴백을 한 번 지웠으나
(`6ffe883`), 없으면 안 된다는 판단으로 복구했다. 지웠을 때 합성검증에서
"경로없음" 비율이 폭증했다 — 결측 20%에서 38프레임 중 29, 급커브에서
39프레임 중 31 이 직전 조향각 유지로 빠졌다.

**목표점 스무딩 제거만 유지**하고 폴백은 살려 둔 것이 지금 상태다.

---

## 2. 확정 파라미터

### rubbercone_node

```yaml
# ROI
forward_min: 0.15
side_half_m: 2.50          # = range_max_m (사실상 무효)
range_max_m: 2.50          # (a) 참고

# 클러스터링 / 콘 판별
cluster_gap_m: 0.15
cluster_min_points: 2
cluster_max_span_m: 0.20   # ★ 벽을 막는 유일한 필터
cone_merge_dist_m: 0.20

# 좌/우 사슬
cone_chain_max_dist_m: 0.60
chain_extend_m: 0.55
chain_reassign_dist_m: 0.30
chain_min_cones: 2

# 중심선
min_gap_m: 0.70
max_gap_m: 0.82            # ★ P*2(0.85) 보다 작아야 한다
centerline_max_tangent_cos: 0.5
single_side_offset_m: 0.40

# Pure Pursuit
wheelbase_m: 0.333         # [실측] 축거 33.3cm
lidar_to_rear_axle_m: 0.41 # [실측] 디버그 표시용 (기준점 아님)
pursuit_ref_offset_m: 0.14 # ★ (c) 실제 기준점
lookahead_dist_m: 0.85
lookahead_min_m: 0.3
steer_gain: 57.29578       # rad->deg 변환 상수 그 자체
angle_limit: 35.0          # 기계적 포화점 (40/50 명령도 35 에서 포화)
invert_steer: false

# 속도
base_speed: 6.0
min_speed_ratio: 0.4
lost_speed_ratio: 0.7
min_speed: 5.0             # ★ 모터 데드밴드(1500 ERPM = 4.06) 위

pairing_mode: chain        # 롤백: nearest_pair
```

### driver_node.cone_zone (구간 판정 — **카메라 YOLO 콘 개수**)

```yaml
enter_n: 9
enter_min_size_px: 100.0
exit_n: 3
exit_hold_sec: 1.5
```

---

## 3. 다시 문제가 생기면 — 증상별 첫 수

| 증상 | 먼저 볼 것 |
|------|-----------|
| 지그재그 / 과조향 | `pursuit_ref_offset_m` 0.14 → 0.24 → 0.41 |
| 커브에서 못 돌고 콘 침 | `lookahead_dist_m` 낮추기 (컷 = ld²/8R, 여유 0.20m) |
| 벽 조각을 콘으로 오인 | `cluster_max_span_m` 0.20 → 0.15, 안 되면 `side_half_m` 1.6 |
| 구간 중간에 이탈 | `exit_hold_sec` 1.5 → 2.5 (`exit_n` 보다 먼저) |
| 멈춘 뒤 다시 안 나감 | `min_speed` 5.0 → 7.0 (+ `base_speed` 도 그 이상) |
| VESC fault 2 (UNDER_VOLTAGE) | `base_speed` 낮추기 → `angle_limit` 32~33 → 배터리 |
| 조향이 통째로 이상 | `pairing_mode: nearest_pair` 로 롤백 |

---

## 4. 아직 정리 안 된 것

- **`ros2 param set` 이 이 노드에 안 먹는다.** 파라미터를 `__init__` 에서
  한 번만 읽어 인스턴스 변수에 복사한다. 실시간으로 반영되는 것은
  `pairing_mode` 하나뿐이다. 값을 바꾸려면 yaml 수정 + 재시작.
  `drive_params.yaml` 의 다른 주석들("재시작 없이 조정 가능")도 같은
  이유로 대부분 사실이 아닐 가능성이 높다 — **확인 필요**.
- **`lidar_to_rear_axle_m` 불일치**: 코드는 0.41, TF(`viz.launch.py`)는
  0.418. 8mm 라 영향은 작지만 같은 값이어야 한다.
- **차체 외형 미실측**: `viz_node` 의 0.50 × 0.28 은 주석에 "대략" 이라고
  적혀 있다. 실측된 것은 축거(0.333)와 라이다 위치뿐이다.
- **실차 rosbag 이 여전히 없다.** 모든 검증이 `synth_check.py` 합성이다.
