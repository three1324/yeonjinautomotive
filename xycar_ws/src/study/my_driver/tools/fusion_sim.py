#!/usr/bin/env python3
"""차선 <-> 콘 복도 융합 전환을 검증한다. ROS 없이 PC 에서 실행.

    python3 tools/fusion_sim.py

무엇을 확인하나:
    - 콘 구간 진입/이탈 시 목표가 **점프하지 않고** 서서히 옮겨가는가
    - 차선을 놓쳤을 때 복도로 넘어가는가 (그 반대도)
    - 둘 다 없을 때 invalid 로 떨어지는가
    - 복도 신뢰도가 낮으면 안 섞는가

핵심 관심사는 **전환 중 프레임간 목표 변화량**이다. 이게 크면 조향이 튄다.
"""

import os
import sys

if getattr(sys.stdout, "encoding", "") and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from my_driver.fusion import LateralFusion, LateralRef  # noqa: E402

DT = 1.0 / 30.0


def run(name, steps, **kw):
    """steps: [(설명, 프레임수, lane, corridor, cone_n), ...]"""
    fus = LateralFusion(**kw)
    prev = None
    max_jump = 0.0
    jump_at = ""
    rows = []

    for label, n, lane, cor, cone in steps:
        for i in range(n):
            r = fus.update(DT, lane, cor, cone)
            if prev is not None and r.valid and prev is not None:
                j = abs(r.offset_near - prev)
                if j > max_jump:
                    max_jump, jump_at = j, label
            prev = r.offset_near if r.valid else prev
            if i == n - 1:
                rows.append((label, r))

    print(f"\n=== {name} ===")
    print(f"  {'구간':<22}{'출처':>9}{'가중치':>8}{'offset':>9}{'quality':>9}{'valid':>7}")
    for label, r in rows:
        print(f"  {label:<22}{r.source:>9}{r.corridor_weight:>8.2f}"
              f"{r.offset_near:>9.0f}{r.quality:>9.2f}{str(r.valid):>7}")
    verdict = "OK" if max_jump < 30 else "점프 큼"
    print(f"  프레임간 최대 변화: {max_jump:.1f} px  ({jump_at})   {verdict}")
    return max_jump < 30


LANE_OK = LateralRef(+40, +60, True, 1.0)
COR_OK = LateralRef(-80, -120, True, 1.0)      # 복도는 반대쪽을 가리킴 (차이 120px)
COR_LOWQ = LateralRef(-80, -120, True, 0.3)
NONE = LateralRef(0, 0, False, 0.0)

ok = []

ok.append(run(
    "콘 구간 진입 -> 통과 -> 이탈",
    [("콘 없음 (차선)", 30, LANE_OK, COR_OK, 0),
     ("콘 진입 (2->6개)", 60, LANE_OK, COR_OK, 6),
     ("콘 구간 중", 30, LANE_OK, COR_OK, 8),
     ("콘 이탈", 60, LANE_OK, COR_OK, 0)],
))

ok.append(run(
    "차선 소실 -> 복도로 전환 -> 차선 복귀",
    [("차선 정상", 30, LANE_OK, COR_OK, 0),
     ("차선 소실", 60, NONE, COR_OK, 0),
     ("차선 복귀", 60, LANE_OK, COR_OK, 0)],
))

ok.append(run(
    "복도 신뢰도 낮음 — 섞이면 안 됨",
    [("콘 많지만 q낮음", 60, LANE_OK, COR_LOWQ, 8)],
))

ok.append(run(
    "둘 다 소실",
    [("정상", 30, LANE_OK, COR_OK, 0),
     ("둘 다 없음", 30, NONE, NONE, 0)],
))

ok.append(run(
    "복도만 있고 차선 없음 (콘 구간에서 차선 가림)",
    [("차선 없음, 콘 많음", 60, NONE, COR_OK, 8)],
))

# 차선도 없고 복도도 못 믿을 때 — invalid 로 떨어져야 driver 가 hold/정지한다.
# 붙잡아둔 값을 valid 로 내보내면 driver 가 유효한 기준으로 오해한다.
r = run(
    "차선 소실 + 복도 저신뢰 — invalid 여야 함",
    [("정상", 30, LANE_OK, COR_OK, 0),
     ("차선 소실, 복도 q낮음", 60, NONE, COR_LOWQ, 8)],
)
fus = LateralFusion()
for _ in range(30):
    fus.update(DT, LANE_OK, COR_OK, 0)
for _ in range(60):
    last = fus.update(DT, NONE, COR_LOWQ, 8)
print(f"  -> valid={last.valid} (False 여야 함)   "
      f"{'OK' if not last.valid else '틀림'}")
ok.append(r and not last.valid)

print()
print("전환 속도(weight_rate_per_sec)에 따른 최대 점프")
for rate in (0.5, 1.5, 5.0, 30.0):
    fus = LateralFusion(weight_rate_per_sec=rate)
    prev, mx = None, 0.0
    for i in range(120):
        cone = 0 if i < 30 else 8
        r = fus.update(DT, LANE_OK, COR_OK, cone)
        if prev is not None:
            mx = max(mx, abs(r.offset_near - prev))
        prev = r.offset_near
    print(f"  rate={rate:>5.1f}/s   최대 변화 {mx:>6.1f} px"
          f"   {'OK' if mx < 30 else '점프 큼'}")

print()
n = sum(ok)
print(f"{n}/{len(ok)} 시나리오 통과")
raise SystemExit(0 if n == len(ok) else 1)
