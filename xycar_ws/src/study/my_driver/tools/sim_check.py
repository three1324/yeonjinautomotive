#!/usr/bin/env python3
"""제어 로직 폐루프 시뮬레이션. ROS 없이 PC 에서 돌린다.

    python3 tools/sim_check.py
    python3 tools/sim_check.py --k-lat 0.15 --k-curve 0.10

무엇을 확인할 수 있고 무엇은 확인할 수 없는가 (중요):
    확인 가능 — 제어 구조가 발산하는지, 진동이 남는지, 곡선에서 정상상태 오차가
                생기는지, 조향이 포화되는지, 차선 결측/추월 로직이 의도대로 도는지
    확인 불가 — **실제 게인 값이 맞는지**. 아래 차량 모델은 픽셀↔실제운동 변환
                상수를 임의로 잡은 것이라 절대적 튜닝값을 줄 수 없다.
                게인의 최종 확정은 실차에서만 가능하다.

차량 모델 (트랙 상대 좌표계):
    heading_err += (K_STEER * angle - curvature) * speed_scale * dt
    offset      += -heading_err * speed_scale * dt
조향이 차량 헤딩을 돌리고 트랙 접선도 곡률만큼 돌아간다. 둘의 차이가 헤딩 오차로
쌓이고, 헤딩 오차가 횡위치를 바꾼다. 이 구조는 실제 차량과 같고 상수만 다르다.

부호 규약 (lane.py 와 일치):
    offset > 0  = 트랙 중앙이 화면 중심보다 오른쪽 = 차가 트랙 왼쪽에 있음
    따라서 offset > 0 이면 오른쪽으로 조향해야 한다.
    실차에서 조향 방향이 반대로 나오면 drive_params.yaml 의 steer.invert 를 true 로.
"""

import argparse
import os
import sys

# Windows 기본 콘솔은 cp949 라 한글/기호 출력에서 죽는다. 젯슨(UTF-8)에서는 무해.
if getattr(sys.stdout, "encoding", "") and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001 - 출력 인코딩 때문에 도구가 죽으면 안 된다
        pass

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from my_driver.control import SpeedLimiter, SteeringController  # noqa: E402
from my_driver.lateral import LateralPlanner, OvertakeBehavior  # noqa: E402
from my_driver.longitudinal import LongitudinalPlanner  # noqa: E402

K_STEER = 3.0        # 조향 1단위가 만드는 헤딩 변화율. 임의값
LOOKAHEAD = 0.9      # 헤딩 오차가 offset_far 에 실리는 비율
CURVE_LOOK = 1.0     # 트랙 곡률이 offset_far 에 실리는 비율
V_REF = 12.0         # 이 속도를 기준으로 운동 속도를 비례시킨다

# 이 모델에서 정상 주행으로 감당 가능한 최대 곡률.
# 평형에서 K_STEER * angle = curvature 이므로 angle_limit 을 넘으면 못 따라간다.
MAX_TRACKABLE_CURVATURE = K_STEER * 35.0


class Obs:
    """driver_node 의 Obs 중 lateral 이 요구하는 필드만."""

    def __init__(self):
        self.car_present = False
        self.car_cx = 0.0
        self.car_bottom_y = 0.0
        # 라이다 필드는 없다 — 회피도 종방향도 카메라만 쓴다 (2026-08-21).


def make_controllers(a):
    steering = SteeringController(
        k_lat=a.k_lat, k_curve=a.k_curve, k_damp=a.k_damp,
        angle_limit=35.0, rate_limit_per_sec=180.0, lpf_alpha=0.45,
        speed_gain_ref=0.0, invert=False,
    )
    longitudinal = LongitudinalPlanner(
        base_speed=12.0, min_speed=4.0,
        curve_px_lo=30.0, curve_px_hi=150.0, curve_factor_min=0.45,
        quality_lo=0.4, quality_factor_min=0.5,
        cone_n_lo=2.0, cone_n_hi=8.0, cone_factor_min=0.6,
    )
    lateral = LateralPlanner(
        OvertakeBehavior(shift_px=120.0, trigger_bottom_y=300.0,
                         shift_sec=0.8, pass_sec=1.5, return_sec=1.0),
        enable_overtake=True,
    )
    speed = SpeedLimiter(accel_per_sec=20.0, decel_per_sec=60.0,
                         speed_limit=27.0, kick=7.0)
    return steering, longitudinal, lateral, speed


def run(a, offset0, curvature, duration, lane_lost=(), car_at=None, label=""):
    steering, longitudinal, lateral, speed_lim = make_controllers(a)
    obs = Obs()

    dt = 1.0 / 30.0
    n = int(duration / dt)
    offset = float(offset0)
    heading_err = 0.0

    hist = []
    saturated = 0

    for i in range(n):
        t = i * dt
        lane_valid = not any(lo <= t < hi for lo, hi in lane_lost)

        if car_at is not None and car_at[0] <= t < car_at[1]:
            obs.car_present = True
            obs.car_cx = 200.0       # 화면 왼쪽에 차량 -> 오른쪽으로 피해야 함
            obs.car_bottom_y = 350.0
        else:
            obs.car_present = False

        offset_far = offset - heading_err * LOOKAHEAD + curvature * CURVE_LOOK

        target_offset = lateral.update(dt, obs, image_width=632)
        target_speed = longitudinal.update(
            lane_valid, offset, offset_far, 1.0 if lane_valid else 0.0,
            0, overtake_active=lateral.overtake.active)
        v = speed_lim.update(dt, target_speed)

        if lane_valid:
            angle = steering.update(dt, offset, offset_far, target_offset, v)
        else:
            angle = steering.hold(dt)

        if abs(angle) >= 34.5:
            saturated += 1

        # 속도가 낮으면 모든 운동이 느려진다 (기하는 그대로, 시간만 늘어남)
        scale = max(v, 0.1) / V_REF
        heading_err += (K_STEER * angle - curvature) * scale * dt
        offset += -heading_err * scale * dt

        hist.append((t, offset, angle, v, target_offset))

    # --- 지표 ---
    tail = [h[1] - h[4] for h in hist[-int(1.5 / dt):]]   # 마지막 1.5초의 추종 오차
    final = sum(tail) / len(tail)
    ripple = (max(tail) - min(tail)) / 2.0                # 남은 진동 진폭
    peak = max(abs(h[1] - h[4]) for h in hist)
    sat_pct = saturated / n * 100.0

    # 두 실패 모드는 **처방이 반대**라 절대 섞으면 안 된다.
    #   정상상태 오차 : 중앙에서 벗어난 채 안정 (진동 없음) -> k_curve 를 **올린다**
    #   진동         : 중앙 근처를 계속 넘나든다            -> k_lat 을 내리거나 k_damp 를 올린다
    # 하나로 묶어 "발산"이라 부르면 k_curve 부족을 k_lat 문제로 오진한다.
    biased = abs(final) > 300.0
    oscillating = ripple > 150.0

    if oscillating:
        mark = "진동"
    elif biased:
        mark = "정상상태오차"
    elif sat_pct > 20.0:
        mark = "포화"
    else:
        mark = "OK"

    print(f"  {label:22} 최종오차 {final:+7.1f}px  진동폭 {ripple:6.1f}px  "
          f"최대 {peak:6.1f}px  조향포화 {sat_pct:4.1f}%   {mark}")
    return biased, oscillating


def main():
    ap = argparse.ArgumentParser()
    # 기본값은 **drive_params.yaml 의 steer 값과 일치시킬 것.**
    # 다르면 아무도 쓰지 않는 게인을 검증하게 된다 (실제로 k_curve 가
    # 0.06 으로 어긋나 있어서 곡선 발산이 잘못 보고됐다).
    ap.add_argument('--k-lat', type=float, default=0.10)
    ap.add_argument('--k-curve', type=float, default=0.15)
    ap.add_argument('--k-damp', type=float, default=0.004)
    a = ap.parse_args()

    print(f"게인  k_lat={a.k_lat}  k_curve={a.k_curve}  k_damp={a.k_damp}")
    print(f"모델  K_STEER={K_STEER}  추종가능 최대곡률={MAX_TRACKABLE_CURVATURE:.0f}"
          f"  (임의값 - 절대 튜닝 근거 아님)\n")

    n_bias = n_osc = 0

    def tally(r):
        nonlocal n_bias, n_osc
        b, o = r
        n_bias += 1 if b else 0
        n_osc += 1 if o else 0

    print("[직선 복귀] 초기 오프셋에서 트랙 중앙으로 돌아오는가")
    for off0 in (50, 150, 300):
        tally(run(a, off0, 0.0, 12.0, label=f"초기 {off0:+}px"))

    # 곡률 시나리오는 **모델의 추종 한계에 연동**한다. 절대값으로 박아두면
    # angle_limit 을 바꿀 때마다(2026-08-21: 50 -> 35) 한계를 넘는 곡률을
    # 테스트하게 되고, 그러면 "포화"가 뜨는데 그건 코드 문제가 아니라
    # 애초에 못 도는 코너를 넣은 것이다.
    print("\n[곡선 추종] 일정 곡률에서 정상상태 오차가 남는가")
    for frac in (0.4, 0.75, 0.95):
        curv = MAX_TRACKABLE_CURVATURE * frac
        tally(run(a, 0.0, curv, 12.0,
                  label=f"곡률 {curv:.0f} (한계의 {frac*100:.0f}%)"))

    print("\n[차선 결측] 2~3.5초 구간에서 차선을 놓쳤을 때")
    tally(run(a, 100.0, 0.0, 12.0, lane_lost=((2.0, 3.5),), label="결측 1.5s"))

    print("\n[추월] 2~4초 구간에 앞차 등장 (왼쪽 -> 오른쪽으로 회피)")
    tally(run(a, 0.0, 0.0, 12.0, car_at=(2.0, 4.0), label="앞차 회피"))

    print()
    if n_osc:
        print(f"진동 {n_osc}개 - k_lat 을 줄이거나 k_damp 를 키울 것")
    if n_bias:
        print(f"정상상태오차 {n_bias}개 - **k_curve 를 올릴 것** "
              f"(k_lat 을 건드리면 안 된다. 이론값은 1/K_STEER = {1.0/K_STEER:.3f})")
    if n_osc or n_bias:
        return 1
    print("모든 시나리오 안정. (실제 게인 값은 실차에서 확정할 것)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
