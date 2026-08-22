#!/usr/bin/env python3
"""합성 S자 코스로 geometry.py 를 검증한다. ROS 없이 지금 돌릴 수 있다.

    python3 tools/synth_check.py

────────────────────────────────────────────────────────────────────────
왜 합성인가

실차 rosbag 이 **없다**(2026-08-22). 그런데 이번 개편은 좌우 사슬 배정과
중심선 생성이라는, 눈으로 확인하기 어려운 기하 로직을 통째로 바꾼다.
검증 없이 실차에 올리면 콘을 친 뒤에야 틀린 걸 안다.

그래서 실측 치수로 S자 코스를 **직접 만들고**, 참값(중심선)을 알고 있는
상태에서 알고리즘이 그걸 되찾아내는지 본다. 라이다 노이즈·결측도 넣는다.

여기서 통과한다고 실차가 된다는 뜻은 아니다. 다만 **여기서 실패하면
실차에서도 반드시 실패한다** — 그 필터 역할이다.

측정 대상:
    1. 사슬에 반대 벽 콘이 섞이지 않는가   (S자 커브의 핵심 실패 모드)
    2. 중심선이 참 중심선에서 얼마나 벗어나는가
    3. 목표점 오차가 좌우 여유(0.20m) 안에 드는가
    4. 콘을 하나씩 빼도(검출 실패) 회복하는가
    5. 뒤축 변환 유무로 조향각이 얼마나 달라지는가
    6. 명령 조향각이 참 곡률이 요구하는 값과 맞는가
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

if getattr(sys.stdout, 'encoding', '') and sys.stdout.encoding.lower() not in ('utf-8', 'utf8'):
    try:
        sys.stdout.reconfigure(encoding='utf-8')
    except Exception:  # noqa: BLE001
        pass

from my_obstacle import geometry as geo  # noqa: E402

# ── 실측 치수 ──
W = geo.CORRIDOR_WIDTH_M          # 0.80 콘 중심 간 복도 폭
P = geo.CONE_PITCH_M              # 0.425 같은 줄 콘 간격 (한 칸 건너 0.85)
AXLE = geo.LIDAR_TO_REAR_AXLE_M   # 0.41 라이다 -> 뒤축
CLEARANCE = 0.20                  # 좌우 여유 (통과폭 0.70 - 차폭 0.30) / 2

# ── 알고리즘 파라미터 (rubbercone_node 기본값과 같아야 한다) ──
CHAIN_MAX = 0.60
EXTEND = 0.55
REASSIGN = 0.30
MIN_CONES = 2
MIN_GAP = 0.70          # drive_params.yaml min_gap_m 와 같아야 한다 (2026-08-22: 0.60 -> 0.70)
MAX_GAP = 0.82          # ★ P*2(0.85) 보다 작아야 한다 (2026-08-22: 1.00 -> 0.82)
TANGENT_COS = 0.50      # centerline 방향 검사 (같은 벽 짝짓기 차단)
SIDE_HALF = 2.50        # [2026-08-22] 1.00 -> 2.50 (= RANGE_MAX, 사실상 무효)
RANGE_MAX = 2.50        # [2026-08-22] 1.40 -> 2.50. 근거는 drive_params.yaml
FORWARD_MIN = 0.15
LOOKAHEAD = 1.10        # [2026-08-22] 0.85 -> 1.10 (rubbercone_node 기본값과 동일)
WHEELBASE = 0.333
GAIN = 57.29578
ANGLE_LIMIT = 35.0
PREV_TARGET_MAX_AGE = 5


def s_course(half_wave_len=1.5, lateral=0.54, n_wave=3, ds=0.01):
    """S자 중심선을 만든다. 반파장 경로장과 횡변위로 형상을 정한다.

    사용자 확인(2026-08-22): 진입 -> 좌 -> 우 -> 좌, 반파장 약 1.5m.
    **일정 반경 원이 아니다** — sin 형상으로 곡률이 연속적으로 변하게 만든다.
    알고리즘이 곡률을 상수로 가정하지 않는다는 것을 여기서 확인한다.
    """
    pts = []
    s = 0.0
    total = half_wave_len * n_wave
    x = 0.0
    y = 0.0
    while s < total:
        # 진행 방향 heading 을 sin 으로 흔든다 -> 곡률이 cos 으로 변한다
        theta = (lateral * math.pi / (2.0 * half_wave_len)) * \
            math.sin(math.pi * s / half_wave_len)
        x += ds * math.cos(theta)
        y += ds * math.sin(theta)
        pts.append((x, y))
        s += ds
    return pts


def walls_from_center(center, pitch=P, width=W):
    """중심선에서 좌/우 콘 위치를 만든다. 각 벽을 따라 pitch 간격."""
    left, right = [], []
    for side, out in ((+1, left), (-1, right)):
        acc = 0.0
        last = None
        for i in range(1, len(center)):
            px, py = center[i - 1]
            qx, qy = center[i]
            tx, ty = qx - px, qy - py
            n = math.hypot(tx, ty)
            if n < 1e-12:
                continue
            nx, ny = -ty / n, tx / n           # 좌측 법선
            wx = qx + side * (width / 2.0) * nx
            wy = qy + side * (width / 2.0) * ny
            if last is None:
                out.append((wx, wy))
                last = (wx, wy)
                continue
            acc = math.hypot(wx - last[0], wy - last[1])
            if acc >= pitch:
                out.append((wx, wy))
                last = (wx, wy)
    return left, right


def curvature_at(center, i, span=20):
    """중심선의 i 지점 곡률(1/m). 세 점 외접원으로 구한다."""
    a = center[max(i - span, 0)]
    b = center[i]
    c = center[min(i + span, len(center) - 1)]
    d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
    if abs(d) < 1e-12:
        return 0.0
    ux = ((a[0] ** 2 + a[1] ** 2) * (b[1] - c[1]) + (b[0] ** 2 + b[1] ** 2) * (c[1] - a[1])
          + (c[0] ** 2 + c[1] ** 2) * (a[1] - b[1])) / d
    uy = ((a[0] ** 2 + a[1] ** 2) * (c[0] - b[0]) + (b[0] ** 2 + b[1] ** 2) * (a[0] - c[0])
          + (c[0] ** 2 + c[1] ** 2) * (b[0] - a[0])) / d
    rad = math.hypot(b[0] - ux, b[1] - uy)
    return 0.0 if rad < 1e-9 else 1.0 / rad


def to_car_frame(pts, pose):
    """월드 좌표 -> 라이다 좌표. pose = (x, y, theta) 는 라이다의 월드 위치."""
    px, py, th = pose
    c, s = math.cos(-th), math.sin(-th)
    out = []
    for (x, y) in pts:
        dx, dy = x - px, y - py
        out.append((dx * c - dy * s, dx * s + dy * c))
    return out


def in_roi(p):
    x, y = p
    return (x >= FORWARD_MIN and abs(y) <= SIDE_HALF
            and math.hypot(x, y) <= RANGE_MAX)


def true_center_in_frame(center, pose):
    return [p for p in to_car_frame(center, pose) if 0.0 <= p[0] <= RANGE_MAX]


def dist_to_polyline(poly, p):
    return geo.polyline_closest_point(poly, p, 0.0)[1]


def run(drop_rate=0.0, noise=0.0, seed=1, verbose=False, lateral=0.54):
    random.seed(seed)
    center = s_course(lateral=lateral)
    wl, wr = walls_from_center(center)

    flips = 0
    frames = 0
    center_err = []
    target_err = []
    no_path = 0
    single = 0
    naive_delta = []
    steer_gap = []
    prev_target = None
    prev_age = 0

    # 중심선을 따라 차를 움직이며 매 지점에서 스캔을 흉내낸다.
    step = 10  # 0.1m 마다
    for i in range(0, len(center) - step, step):
        px, py = center[i]
        nx, ny = center[i + step]
        th = math.atan2(ny - py, nx - px)
        # ★ **뒤축**을 중심선 위에 놓는다 (라이다가 아니다).
        #   차가 경로를 따른다는 것은 뒤축이 경로 위에 있다는 뜻이고, Pure
        #   Pursuit 의 기하도 뒤축 기준이다. 라이다를 중심선에 놓으면 뒤축이
        #   커브 바깥으로 0.41^2/(2R) ~ 0.10m 밀려난 자세가 되어, 조향
        #   명령이 참 곡률보다 작게 나온다(실측 5.3도 vs 요구 21도).
        pose = (px + AXLE * math.cos(th), py + AXLE * math.sin(th), th)

        cones = []
        truth = {}
        for tag, wall in (('L', wl), ('R', wr)):
            for p in to_car_frame(wall, pose):
                if not in_roi(p):
                    continue
                if drop_rate and random.random() < drop_rate:
                    continue
                q = (p[0] + random.gauss(0, noise), p[1] + random.gauss(0, noise))
                cones.append(q)
                truth[(round(q[0], 6), round(q[1], 6))] = tag

        if not cones:
            continue
        frames += 1

        left, right = geo.build_chains(cones, CHAIN_MAX, EXTEND, REASSIGN,
                                       MIN_CONES, MIN_GAP, MAX_GAP)

        # 1) 사슬에 **반대 벽 콘이 섞였는가** — 이것이 진짜 사슬 실패다.
        #    (좌/우 라벨이 바뀌는 것 자체는 주행에 영향이 없다. 중심선은
        #     대칭이고, 편측 폴백의 방향은 resolve_side 가 따로 정한다.
        #     그 결과는 아래 목표점 오차에 그대로 드러난다.)
        for chain in (left, right):
            tags = {truth.get((round(c[0], 6), round(c[1], 6))) for c in chain}
            tags.discard(None)
            if len(tags) > 1:
                flips += 1
                if verbose:
                    print('  mix @%.2fm  L%d R%d' % (i * 0.01, len(left), len(right)))
                break

        # 2) 중심선 오차
        path, src = _plan(left, right, prev_target)
        if not path:
            no_path += 1
            prev_age += 1
            if prev_age > PREV_TARGET_MAX_AGE:
                prev_target = None
            continue
        if src == 'wall':
            single += 1
        truth_c = true_center_in_frame(center, pose)
        if len(truth_c) >= 2:
            for p in path:
                center_err.append(dist_to_polyline(truth_c, p))

        # 3) 목표점 오차
        tgt, eff, _ = geo.target_at_lookahead(path, LOOKAHEAD, AXLE)
        if tgt is not None:
            prev_target = tgt
            prev_age = 0
        if tgt and len(truth_c) >= 2:
            target_err.append(dist_to_polyline(truth_c, tgt))

            # 5) 뒤축 변환을 빼면 조향이 얼마나 달라지나 (옛 코드 재현)
            good = geo.steer_pure_pursuit(tgt, AXLE, WHEELBASE, GAIN, 0.3, ANGLE_LIMIT)
            naive = geo.steer_pure_pursuit(tgt, 0.0, WHEELBASE, GAIN, 0.3, ANGLE_LIMIT)
            naive_delta.append(abs(naive) - abs(good))

            # 6) 참 곡률이 요구하는 조향각과 얼마나 맞는가
            #    delta_true = atan(wheelbase / R). Pure Pursuit 은 원래 코너를
            #    lookahead^2/(8R) 만큼 깎으므로 **약간 작게 나오는 것이 정상**이다.
            #    크게 나오면 과조향, 많이 작으면 코너를 못 돈다는 뜻.
            kappa = curvature_at(center, i)
            if abs(kappa) > 1e-6:
                want = math.degrees(math.atan(WHEELBASE * abs(kappa)))
                steer_gap.append(abs(good) - want)

    return {
        'frames': frames, 'flips': flips, 'no_path': no_path, 'single': single,
        'center_err': center_err, 'target_err': target_err,
        'naive_delta': naive_delta, 'steer_gap': steer_gap,
    }


def _plan(left, right, prev_target=None):
    """rubbercone_node._plan_path() 와 같은 결정 순서를 재현한다."""
    if left and right:
        path = geo.centerline(left, right, MIN_GAP, MAX_GAP, EXTEND, TANGENT_COS)
        if path:
            return path, 'center'
    for chain in (left, right):
        if len(chain) >= 2:
            cs, conf = geo.corridor_side(chain)
            side = geo.resolve_side(chain, W / 2.0, cs, conf, prev_target)
            if side is None:
                continue
            path = geo.offset_from_single_wall(chain, W / 2.0, side)
            if path:
                return path, 'wall'
    return [], 'none'


def stat(v):
    if not v:
        return 'n/a'
    v = sorted(v)
    return 'mean %.3f  p95 %.3f  max %.3f' % (
        sum(v) / len(v), v[int(0.95 * (len(v) - 1))], v[-1])


def main():
    print('=' * 72)
    print('합성 S자 코스 검증  (복도폭 %.2fm, 콘간격 %.3fm, 여유 %.2fm)'
          % (W, P, CLEARANCE))
    print('=' * 72)

    fail = 0
    for name, kw in (
        ('이상적 (결측 0%, 노이즈 0)', dict()),
        ('노이즈 2cm', dict(noise=0.02)),
        ('콘 20% 결측', dict(drop_rate=0.20)),
        ('노이즈 2cm + 결측 20%', dict(noise=0.02, drop_rate=0.20)),
        ('가혹: 노이즈 3cm + 결측 35%', dict(noise=0.03, drop_rate=0.35)),
    ):
        r = run(**kw)
        print('\n[%s]' % name)
        print('  프레임 %d | 사슬혼입 %d | 경로없음 %d | 편측폴백 %d'
              % (r['frames'], r['flips'], r['no_path'], r['single']))
        print('  중심선 오차 : %s' % stat(r['center_err']))
        print('  목표점 오차 : %s' % stat(r['target_err']))
        if r['steer_gap']:
            g = r['steer_gap']
            print('  참 곡률 대비 조향 : 평균 %+.1f도  (음수 = 코너 컷, 정상)'
                  % (sum(g) / len(g)))
        if r['naive_delta']:
            d = r['naive_delta']
            print('  뒤축변환 없을 때 조향 초과: 평균 +%.1f도  최대 +%.1f도'
                  % (sum(d) / len(d), max(d)))

        if r['flips']:
            print('  >>> FAIL 사슬에 반대 벽 콘이 섞였다')
            fail += 1
        if r['target_err'] and max(r['target_err']) > CLEARANCE:
            print('  >>> FAIL 목표점 오차가 좌우 여유(%.2fm)를 넘는다' % CLEARANCE)
            fail += 1

    # ── 코스 곡률 민감도 ──
    # 실제 반경을 모른다(사용자 확인: "훨씬 크고, 완전한 원이 아니다").
    # 알고리즘은 곡률을 상수로 가정하지 않지만, 급할수록 **양쪽 벽이 동시에
    # 보이는 프레임이 줄어** 편측 폴백과 FTG 비율이 오른다. 그 민감도를 본다.
    #   R_min = 2*L^2 / (pi^2 * lateral),  L = 반파장 1.5m
    print('\n[코스 곡률 민감도]  (노이즈 2cm + 결측 20%)')
    print('  %-8s %-8s %-11s %-11s %s'
          % ('횡변위', 'R_min', '양벽프레임', '경로없음(FTG)', '목표점p95'))
    for lat in (0.30, 0.40, 0.54, 0.70):
        rmin = 2.0 * 1.5 ** 2 / (math.pi ** 2 * lat)
        r = run(noise=0.02, drop_rate=0.20, lateral=lat)
        both = r['frames'] - r['no_path'] - r['single']
        te = sorted(r['target_err'])
        p95 = te[int(0.95 * (len(te) - 1))] if te else 0.0
        bad = r['flips'] or (te and max(te) > CLEARANCE)
        print('  %-8.2f %-8.2f %-11s %-11s %.3f%s'
              % (lat, rmin, '%d/%d' % (both, r['frames']),
                 '%d/%d' % (r['no_path'], r['frames']), p95,
                 '  <- FAIL' if bad else ''))
        if bad:
            fail += 1

    print('\n' + '=' * 72)
    print('FAIL %d 건' % fail if fail else '전부 통과')
    return 1 if fail else 0


if __name__ == '__main__':
    sys.exit(main())
