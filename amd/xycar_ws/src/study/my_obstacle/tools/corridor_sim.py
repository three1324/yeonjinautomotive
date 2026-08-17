#!/usr/bin/env python3
"""합성 라바콘 복도로 CorridorEstimator 를 검증한다. ROS 없이 PC 에서 실행.

    python3 tools/corridor_sim.py
    python3 tools/corridor_sim.py --plot        # 그림으로 확인 (matplotlib 필요)

무엇을 확인할 수 있고 무엇은 확인할 수 없는가:
    확인 가능 — 좌표 변환/부호가 맞는지, 직선·곡선·S자에서 중앙선을 제대로 잡는지,
                한쪽 벽만 보일 때 동작, 노이즈 내성, 급변 방어
    확인 불가 — **실제 콘 간격·복도 폭·라이다 반사 특성**. 실차 rosbag 이 있어야 한다.
                여기 쓰는 콘 배치는 가정값이다.

부호 규약 (corridor.py 와 동일):
    라이다  x=전방(+), y=좌측(+)
    출력    offset > 0  =  복도 중앙이 화면 중심보다 오른쪽  =  오른쪽으로 조향
    복도가 왼쪽에 있으면(y>0) offset 은 음수가 나와야 한다.
"""

import argparse
import os
import sys

if getattr(sys.stdout, "encoding", "") and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import numpy as np  # noqa: E402

from my_obstacle.corridor import CorridorEstimator  # noqa: E402

# 라이다 스펙 (YDLidar G2B 가정)
N_BEAMS = 500
ANGLE_MIN = -np.pi
ANGLE_INC = 2 * np.pi / N_BEAMS
RANGE_MAX = 10.0

PX_PER_M = 300.0    # 임시값. 실차에서 (픽셀 반폭 x2) / 실측 트랙폭 으로 구할 것


def make_cones(center_fn, half_width, x_from=0.2, x_to=3.5, spacing=0.30,
               jitter=0.0, drop_left=(), drop_right=(), rng=None):
    """콘 배치를 만든다.

    center_fn : x -> 복도 중앙의 y (m). 이게 정답(ground truth)이다.
    half_width: 복도 반폭 (m)
    spacing   : 콘 간격 (m)
    drop_*    : (x_start, x_end) 구간의 콘을 뺀다 (벽이 끊긴 상황 재현)
    """
    rng = rng or np.random.default_rng(0)
    xs = np.arange(x_from, x_to, spacing)
    pts = []
    for x in xs:
        c = center_fn(x)
        for side, drops in ((+1, drop_left), (-1, drop_right)):
            if any(a <= x <= b for a, b in drops):
                continue
            y = c + side * half_width
            if jitter:
                x_j = x + rng.normal(0, jitter)
                y_j = y + rng.normal(0, jitter)
            else:
                x_j, y_j = x, y
            pts.append((x_j, y_j))
    return np.array(pts)


def points_to_scan(pts, noise=0.0, rng=None):
    """(x, y) 점군 -> LaserScan ranges 배열.

    각 빔 방향에서 가장 가까운 점만 남긴다 (실제 라이다처럼).
    """
    rng = rng or np.random.default_rng(1)
    ranges = np.full(N_BEAMS, np.inf)
    if pts.size == 0:
        return ranges
    r = np.hypot(pts[:, 0], pts[:, 1])
    a = np.arctan2(pts[:, 1], pts[:, 0])
    idx = np.clip(((a - ANGLE_MIN) / ANGLE_INC).astype(int), 0, N_BEAMS - 1)
    for i, rr in zip(idx, r):
        if noise:
            rr = rr + rng.normal(0, noise)
        if rr < ranges[i]:
            ranges[i] = rr
    return ranges


def expected_offset_px(center_fn, x_eval):
    """정답 offset (px). 부호 규약: 복도가 왼쪽(y>0)이면 음수."""
    return -center_fn(x_eval) * PX_PER_M


def run_case(name, center_fn, half_width=0.35, **kw):
    est = CorridorEstimator(px_per_meter=PX_PER_M,
                            nominal_half_width_m=half_width)
    pts = make_cones(center_fn, half_width, **kw)
    ranges = points_to_scan(pts)
    res = est.update(ranges, ANGLE_MIN, ANGLE_INC, 0.05, RANGE_MAX)

    exp_near = expected_offset_px(center_fn, est.eval_near_m)
    exp_far = expected_offset_px(center_fn, est.eval_far_m)
    err_near = res.offset_near - exp_near
    err_far = res.offset_far - exp_far

    ok = res.valid and abs(err_near) < 40 and abs(err_far) < 60
    mark = "OK" if ok else "확인필요"
    print(f"  {name:<22}"
          f"near {res.offset_near:>+7.0f} (정답 {exp_near:>+6.0f}, 오차 {err_near:>+5.0f})  "
          f"far {res.offset_far:>+7.0f} (오차 {err_far:>+5.0f})  "
          f"q{res.quality:.2f} 폭{res.width_m:.2f}m 구간{res.n_bins:>2}  {mark}")
    return ok, pts, res, est


def run_bag(path, topic, stride, limit, plot, **est_kw):
    """실측 rosbag 으로 복도 추정을 돌린다.

    합성과 달리 **정답이 없다.** 대신 실측으로만 알 수 있는 것을 뽑는다:

        width_m      -> corridor.nominal_half_width_m 을 확정하는 근거.
                        px_per_meter 와 무관하게 미터로 나오므로 신뢰할 수 있다.
        valid 비율    -> 콘 구간에서 복도를 얼마나 자주 찾는가
        n_bins       -> bin_size / x_max 가 적절한가
        프레임간 변화 -> 이 값이 크면 조향이 튄다 (max_jump_px 근거)
    """
    import statistics

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from rosbag_reader import find_db3, list_topics, read_scans

    db3 = find_db3(path)
    print(f"bag: {db3}")
    for name, typ, cnt in list_topics(db3):
        mark = " <-" if name == topic else ""
        print(f"  {name:<28}{typ:<34}{cnt:>7}개{mark}")
    print()

    est = CorridorEstimator(**est_kw)
    widths, bins_n, quals, offs = [], [], [], []
    n_all = n_valid = 0
    jumps = []
    prev = None

    for m in read_scans(db3, topic, limit=limit, stride=stride):
        r = est.update(m.ranges, m.angle_min, m.angle_increment,
                       max(0.05, m.range_min), m.range_max)
        n_all += 1
        if r.valid:
            n_valid += 1
            widths.append(r.width_m)
            bins_n.append(r.n_bins)
            quals.append(r.quality)
            offs.append(r.offset_near)
            if prev is not None:
                jumps.append(abs(r.offset_near - prev))
            prev = r.offset_near
        else:
            prev = None

    if n_all == 0:
        print("스캔이 하나도 없다. --topic 을 확인할 것.")
        return False

    print(f"스캔 {n_all}개 중 복도 유효 {n_valid}개 ({100.0*n_valid/n_all:.1f}%)")
    if not widths:
        print("\n복도를 한 번도 못 찾았다. 확인할 것:")
        print("  - 이 bag 구간에 실제로 콘이 있나 (콘 구간을 녹화했나)")
        print("  - corridor.x_max / max_lateral 안에 콘이 들어오나")
        print("  - corridor.min_bins / min_span_m 이 너무 빡빡하지 않나")
        return False

    def stat(name, xs, unit=""):
        xs = sorted(xs)
        p5 = xs[int(0.05 * (len(xs) - 1))]
        p95 = xs[int(0.95 * (len(xs) - 1))]
        med = statistics.median(xs)
        print(f"  {name:<16}중앙 {med:>7.2f}{unit}   "
              f"5~95% {p5:>6.2f} ~ {p95:>6.2f}{unit}")

    print()
    stat("복도 폭", widths, "m")
    stat("구간 수", bins_n)
    stat("quality", quals)
    stat("offset_near", offs, "px")
    if jumps:
        stat("프레임간 변화", jumps, "px")

    half = statistics.median(widths) / 2.0
    print()
    print("반영할 값")
    print(f"  corridor.nominal_half_width_m: {half:.3f}"
          f"   (실측 복도폭 중앙값 {statistics.median(widths):.3f}m 의 절반)")
    if jumps:
        j95 = sorted(jumps)[int(0.95 * (len(jumps) - 1))]
        print(f"  corridor.max_jump_px: {max(60.0, j95 * 2):.0f} 이상"
              f"   (실측 프레임간 변화 95% = {j95:.0f}px)")
    print()
    print("⚠️ offset(px) 값은 px_per_meter 가 확정돼야 의미가 있다.")
    print("   폭(m) 과 유효율은 px_per_meter 와 무관하므로 지금 믿어도 된다.")

    if plot:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(13, 8))
        axes[0][0].plot(offs); axes[0][0].set_title("offset_near (px)")
        axes[0][0].axhline(0, color="gray", lw=0.5)
        axes[0][1].plot(widths); axes[0][1].set_title("복도 폭 (m)")
        axes[1][0].plot(bins_n); axes[1][0].set_title("구간 수")
        axes[1][1].hist(jumps or [0], bins=30); axes[1][1].set_title("프레임간 변화 (px)")
        plt.tight_layout(); plt.show()

    return n_valid > 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--bag", help="ros2 bag 디렉터리 또는 .db3 경로. "
                                  "주면 합성 대신 실측으로 검증한다")
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--stride", type=int, default=1, help="N개마다 1개만 처리")
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--px-per-meter", type=float, default=PX_PER_M)
    ap.add_argument("--x-max", type=float, default=2.2)
    ap.add_argument("--bin-size", type=float, default=0.15)
    ap.add_argument("--half-width", type=float, default=0.35)
    a = ap.parse_args()

    if a.bag:
        ok = run_bag(a.bag, a.topic, a.stride, a.limit, a.plot,
                     px_per_meter=a.px_per_meter, x_max=a.x_max,
                     bin_size=a.bin_size, nominal_half_width_m=a.half_width)
        return 0 if ok else 1

    print(f"px_per_meter = {PX_PER_M}  (임시값 — 실차에서 재계산 필요)")
    print(f"평가 지점: near {0.6}m, far {1.5}m\n")

    results = []

    print("[1] 직선 복도 — 차량이 중앙/좌/우에 있을 때 부호가 맞는가")
    results.append(run_case("중앙 (정답 0)", lambda x: 0.0)[0])
    # 복도가 왼쪽으로 치우침 = 차가 오른쪽에 있음 -> offset 음수여야
    results.append(run_case("복도 왼쪽 +0.2m", lambda x: 0.2)[0])
    results.append(run_case("복도 오른쪽 -0.2m", lambda x: -0.2)[0])

    print("\n[2] 곡선 복도 — 곡률을 따라가는가")
    results.append(run_case("좌커브", lambda x: 0.10 * x ** 2)[0])
    results.append(run_case("우커브", lambda x: -0.10 * x ** 2)[0])

    print("\n[3] S자 복도 — 실제 트랙 형태")
    results.append(run_case("S자", lambda x: 0.25 * np.sin(1.6 * x))[0])

    print("\n[4] 한쪽 벽 결측 — 콘이 끊긴 구간")
    results.append(run_case("좌벽 1.0~1.8m 없음", lambda x: 0.0,
                            drop_left=[(1.0, 1.8)])[0])
    results.append(run_case("우벽 절반 없음", lambda x: 0.0,
                            drop_right=[(0.8, 2.5)])[0])

    print("\n[5] 노이즈 내성")
    for j in (0.01, 0.03, 0.06):
        results.append(run_case(f"콘 위치 지터 {j*100:.0f}cm",
                                lambda x: 0.15 * np.sin(1.5 * x), jitter=j)[0])

    print("\n[6] 복도 폭 변화")
    for hw in (0.25, 0.35, 0.50):
        results.append(run_case(f"반폭 {hw}m", lambda x: 0.0, half_width=hw)[0])

    print()
    n_ok = sum(results)
    print(f"{n_ok}/{len(results)} 통과")
    if n_ok < len(results):
        print("→ '확인필요' 항목은 파라미터(bin_size, min_bins, 평가지점)를 조정하거나")
        print("   합성 조건이 비현실적인지 검토할 것")

    if a.plot:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        cases = [("직선", lambda x: 0.0),
                 ("좌커브", lambda x: 0.10 * x ** 2),
                 ("S자", lambda x: 0.25 * np.sin(1.6 * x))]
        for ax, (nm, fn) in zip(axes, cases):
            _, pts, res, est = run_case(nm, fn)
            ax.scatter(pts[:, 0], pts[:, 1], c="orange", s=40, label="콘")
            gx = np.linspace(0.2, 3.0, 50)
            ax.plot(gx, [fn(v) for v in gx], "g--", label="정답 중앙선")
            for xe, off in ((est.eval_near_m, res.offset_near),
                            (est.eval_far_m, res.offset_far)):
                ax.plot(xe, -off / PX_PER_M, "bo", ms=10)
            ax.set_title(nm)
            ax.set_xlabel("x 전방 (m)")
            ax.set_ylabel("y 좌측 (m)")
            ax.axhline(0, color="gray", lw=0.5)
            ax.set_aspect("equal")
            ax.legend(fontsize=8)
        plt.tight_layout()
        plt.show()

    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
