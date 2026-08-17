#!/usr/bin/env python3
"""출발 시퀀스 검증 — 신호등 투표 -> FSM -> 출발. ROS 없이 PC 에서 실행.

    python3 tools/start_sim.py
    python3 tools/start_sim.py --window 30 --min-ratio 0.5 --confirm 5

────────────────────────────────────────────────────────────────────────
왜 이걸 재나

출발은 **되돌릴 수 없다**. 두 방향으로 다 틀릴 수 있다:

    늦은 출발 : 초록불인데 못 알아봐서 서 있다  -> 기록 손해, 최악은 미완주
    조기 출발 : 빨간불인데 초록으로 오인        -> 실격

그런데 출발 대기 위치가 **가장 불리한 조건**이다. reference/perception_analysis.md
실측:

    신호등 박스 ~80px  -> 램프 판독 68%, conf 0.54   <- 출발 대기 위치가 여기
    80~110px          -> 88%, conf 0.71
    110px+            -> 94%, conf 0.70

즉 **3번에 1번은 램프를 틀리게 읽는다.** 단일 프레임을 믿으면 안 되고,
그래서 LightVoter 가 있다. 문제는 그 투표 설정(window/min_weight/min_ratio)과
FSM 의 start_confirm_frames 가 실제로 얼마나 걸리고 얼마나 안전한지를
**아무도 재본 적이 없다는 것**이다. 이 스크립트가 그걸 잰다.

────────────────────────────────────────────────────────────────────────
어떻게 재나

실측 검출률로 관측 스트림을 합성한다:

    빨간불 구간 : 68% 확률로 RED, 32% 는 오검출(다른 색) 또는 미검출
    t=0 에 초록으로 바뀜
    초록불 구간 : 68% 확률로 GREEN, 32% 는 오검출/미검출

이걸 LightVoter -> DriveFSM 에 흘려서 잰다:

    출발 지연  : 초록으로 바뀐 뒤 몇 초 만에 should_drive 가 True 가 되나
    조기 출발  : 초록 전에 출발해버린 횟수 (0 이어야 한다)
    미출발    : 제한 시간 안에 출발 못 한 횟수

오검출이 무엇으로 튀는지가 중요하다. 빨간불에서 오검출이 GREEN 으로 튀면
조기 출발 위험이고, 초록불에서 RED 로 튀면 출발이 늦어진다. 실측 데이터에
오검출 분포까지는 없어서 **균등 분포로 가정**한다 (보수적 — 실제 YOLO 는
RED/GREEN 을 그렇게 자주 헷갈리지 않는다).
"""

import argparse
import os
import random
import sys

if getattr(sys.stdout, "encoding", "") and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "my_perception"))

from my_driver.fsm import DriveFSM  # noqa: E402
from my_perception.light_vote import LightVoter  # noqa: E402

FPS = 30.0

# reference/perception_analysis.md 실측 — 출발 대기 위치(~80px)
HIT_RATE = 0.68
CONF_OK = 0.54
LIGHT_WIDTH = 80.0

# 오검출이 무엇으로 나올지. 실측에 분포가 없어 균등 가정 (보수적).
OTHERS = {"RED": ["GREEN", "YELLOW", "LEFT", None],
          "GREEN": ["RED", "YELLOW", "LEFT", None]}


def observe(truth, rng, hit_rate=HIT_RATE, adversarial=False):
    """한 프레임의 램프 관측을 합성한다. (lamp_name, conf, width)

    adversarial: 오검출을 **가장 위험한 방향**으로 몰아준다. 빨간불일 때
        오검출의 전부가 GREEN 으로 나온다고 가정 — 조기 출발(실격) 여유를
        보수적으로 재기 위한 최악 시나리오다. 실제 YOLO 는 이 정도로
        RED/GREEN 을 헷갈리지 않는다.
    """
    if rng.random() < hit_rate:
        return truth, CONF_OK, LIGHT_WIDTH
    if adversarial and truth == "RED":
        return "GREEN", CONF_OK * 0.6, LIGHT_WIDTH
    wrong = rng.choice(OTHERS[truth])
    if wrong is None:
        # 램프는 못 읽었지만 신호등 본체는 보인다 (실측: 본체는 전 거리 안정)
        return None, 0.0, LIGHT_WIDTH
    return wrong, CONF_OK * 0.6, LIGHT_WIDTH   # 오검출은 신뢰도도 낮은 편


def one_run(rng, red_frames, limit_frames, hit_rate, adversarial=False, **kw):
    """빨간불 red_frames 후 초록. 반환: (출발지연프레임 or None, 조기출발여부)"""
    voter = LightVoter(window=kw["window"], min_weight=kw["min_weight"],
                       min_ratio=kw["min_ratio"], miss_tolerance=kw["miss_tolerance"])
    fsm = DriveFSM(start_confirm_frames=kw["confirm"], auto_start=False)

    for i in range(red_frames):
        name, conf, w = observe("RED", rng, hit_rate, adversarial)
        fsm.update(voter.update(name, conf, w), True)
        if fsm.should_drive:
            return None, True          # 조기 출발 — 실격

    for i in range(limit_frames):
        name, conf, w = observe("GREEN", rng, hit_rate, adversarial)
        fsm.update(voter.update(name, conf, w), True)
        if fsm.should_drive:
            return i, False
    return None, False                 # 미출발


def sweep(label, runs, rng_seed, hit_rate, adversarial=False, red_frames=90, **kw):
    rng = random.Random(rng_seed)
    delays, early, never = [], 0, 0
    for _ in range(runs):
        d, e = one_run(rng, red_frames=red_frames, limit_frames=int(10 * FPS),
                       hit_rate=hit_rate, adversarial=adversarial, **kw)
        if e:
            early += 1
        elif d is None:
            never += 1
        else:
            delays.append(d / FPS)

    delays.sort()
    if delays:
        mean = sum(delays) / len(delays)
        p95 = delays[min(len(delays) - 1, int(0.95 * len(delays)))]
        worst = delays[-1]
    else:
        mean = p95 = worst = float("nan")

    ok = early == 0 and never == 0 and p95 <= 1.5
    print(f"  {label:<26}{mean:>7.2f}s{p95:>8.2f}s{worst:>8.2f}s"
          f"{early:>7}{never:>7}   {'OK' if ok else '주의'}")
    return ok, mean, p95, early, never


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=2000)
    ap.add_argument("--window", type=int, default=30)
    ap.add_argument("--min-weight", type=float, default=3.0)
    ap.add_argument("--min-ratio", type=float, default=0.5)
    ap.add_argument("--miss-tolerance", type=int, default=10)
    ap.add_argument("--confirm", type=int, default=5)
    a = ap.parse_args()

    base = dict(window=a.window, min_weight=a.min_weight, min_ratio=a.min_ratio,
                miss_tolerance=a.miss_tolerance, confirm=a.confirm)

    print("출발 시퀀스 몬테카를로 "
          f"({a.runs}회, 30fps, 실측 검출률 {HIT_RATE:.0%} @80px)")
    print("  기준: 조기출발 0, 미출발 0, p95 지연 <= 1.5s")
    print()
    print(f"  {'조건':<26}{'평균':>8}{'p95':>8}{'최악':>8}"
          f"{'조기':>7}{'미출발':>7}")

    print("\n[현재 설정]")
    ok_now, _, p95_now, _, _ = sweep(
        f"yaml 기본값", a.runs, 1, HIT_RATE, **base)

    print("\n[검출률 민감도] — 조명·거리가 나쁠 때")
    for hr in (0.50, 0.68, 0.88, 0.94):
        sweep(f"검출률 {hr:.0%}", a.runs, 2, hr, **base)

    print("\n[window] — 투표 창")
    for w in (10, 20, 30, 45, 60):
        kw = dict(base, window=w)
        sweep(f"window={w} ({w/FPS:.1f}s)", a.runs, 3, HIT_RATE, **kw)

    print("\n[min_ratio] — 1위가 차지해야 할 비율")
    for r in (0.4, 0.5, 0.6, 0.7):
        kw = dict(base, min_ratio=r)
        sweep(f"min_ratio={r}", a.runs, 4, HIT_RATE, **kw)

    print("\n[min_weight] — 확정에 필요한 가중치 합")
    for mw in (1.0, 3.0, 6.0, 10.0):
        kw = dict(base, min_weight=mw)
        sweep(f"min_weight={mw}", a.runs, 5, HIT_RATE, **kw)

    print("\n[start_confirm_frames] — FSM 추가 확인")
    for c in (1, 3, 5, 10):
        kw = dict(base, confirm=c)
        sweep(f"confirm={c}", a.runs, 6, HIT_RATE, **kw)

    # ── 최악 시나리오 ──────────────────────────────────────────────
    # 빨간불에서 오검출이 **전부 GREEN 으로** 튄다고 가정. 조기 출발은 실격이라
    # 여유가 얼마나 있는지 보수적으로 봐야 한다. 대기 시간도 길게 잡는다
    # (출발선에서 오래 기다릴수록 오검출이 누적될 기회가 많다).
    print(f"\n[최악] 빨간불 오검출이 전부 GREEN, 대기 10초 — 조기출발 0 이어야 함")
    for hr in (0.68, 0.60, 0.50):
        sweep(f"검출률 {hr:.0%} (적대적)", a.runs, 7, hr,
              adversarial=True, red_frames=int(10 * FPS), **base)
    for r in (0.5, 0.6, 0.7):
        kw = dict(base, min_ratio=r)
        sweep(f"min_ratio={r} (적대적)", a.runs, 8, HIT_RATE,
              adversarial=True, red_frames=int(10 * FPS), **kw)
    for w in (20, 30, 45):
        kw = dict(base, window=w)
        sweep(f"window={w} (적대적)", a.runs, 9, HIT_RATE,
              adversarial=True, red_frames=int(10 * FPS), **kw)

    print()
    print("해석")
    print("  조기출발 > 0 이면 실격 위험 — min_ratio/min_weight/confirm 을 올린다")
    print("  미출발 > 0  이면 경기 못 함 — window 를 늘리거나 min_weight 를 낮춘다")
    print("  p95 지연은 그대로 기록 손해다 (2초 = 출발선에서 2초 서 있음)")
    return 0 if ok_now else 1


if __name__ == "__main__":
    raise SystemExit(main())
