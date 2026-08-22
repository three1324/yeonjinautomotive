"""라바콘 복도 기하 — ROS 의존성 없음, 순수 함수.

rubbercone_node 에서 떼어낸 이유: 실차 bag 이 **없다**. 합성 데이터로만
검증할 수 있으므로, ROS 를 띄우지 않고 tools/synth_check.py 가 그대로
부를 수 있어야 한다.

────────────────────────────────────────────────────────────────────────
전제가 되는 실측 치수 (2026-08-22 확정)

    콘 단면 지름(라이다 높이)      D = 0.10 m
    같은 줄 콘 중심 간격            P = 0.434 m   (A4 297mm + 베이스 137mm)
    좌우 콘 중심 간 거리            W = 0.80 m    (통과폭 0.70 + D)
    차폭                            0.30 m  ->  좌우 여유 0.20 m 씩
    라이다 -> 뒤축                  0.41 m

이 숫자들이 아래 모든 문턱값의 근거다. **코스가 바뀌면 여기부터 다시 잰다.**

★ 곡률은 **가정하지 않는다.** 코스는 S자이고 일정 반경 원이 아니다.
  아래 알고리즘은 어느 것도 반경을 상수로 쓰지 않는다 — 중심선을 좌우 벽의
  최근접 대응으로 만들기 때문에 곡률이 구간마다 달라도 그대로 성립한다.
  문서에 나오는 R=0.90 은 **최악 케이스 검산값**이지 가정이 아니다
  (안쪽 콘 벽 반경 0.50 을 가정했을 때의 중심선. 실제 코스는 이보다 완만하다).

────────────────────────────────────────────────────────────────────────
왜 y 부호로 좌우를 나누지 않는가 — S자에서 깨진다

최악 케이스(중심선 R=0.90) 좌향 커브를 라이다 좌표로 전개하면:

    내벽(좌)  (0.382, +0.577)  (0.493, +0.982)
    외벽(우)  (0.426, -0.328)  (0.805, -0.121)  (1.095, +0.199)  <- y 가 +
                                                (1.264, +0.597)  <- y 가 +

**외벽(우측) 콘이 r=1.1m 부터 y>0 으로 넘어온다.** 옛 _find_pair_target() 의
`y > 0.1 / y < -0.1` 분류로는 이것들이 전부 좌측 줄로 들어가 목표점이
반대로 튄다. 그래서 좌우를 **콘 사이 거리(사슬)** 로 나눈다:

    콘 간격 0.434  <  사슬 문턱 0.60  <  복도 폭 0.80

두 벽 사이의 반경 방향 간격은 커브 어디서나 W 이므로, 이 부등식은
직선에서든 커브에서든 곡률과 무관하게 성립한다.
"""

import math

# ── 실측 치수 (위 docstring 참고). 파라미터 기본값의 근거이기도 하다 ──
CONE_DIAMETER_M = 0.10
CONE_PITCH_M = 0.434
CORRIDOR_WIDTH_M = 0.80
LIDAR_TO_REAR_AXLE_M = 0.41


def polyline_closest_point(poly, p, extend=0.0):
    """폴리라인 위에서 p 에 가장 가까운 점과 그 거리를 돌려준다.

    extend > 0 이면 **양 끝 세그먼트를 그만큼 직선 연장**한 뒤 투영한다.
    이게 없으면 콘 하나가 검출에 실패했을 때 그 너머 콘이 폴리라인 끝점
    으로부터 P*2 = 0.868m 로 잡혀, 반대편 벽(0.80m)보다 **멀어진다** —
    즉 반대편 벽으로 오배정된다. 연장이 그걸 막는다.

    반환: (점, 거리) 또는 (None, inf)
    """
    if not poly:
        return None, float("inf")
    if len(poly) == 1:
        return poly[0], math.hypot(p[0] - poly[0][0], p[1] - poly[0][1])

    best = None
    best_d = float("inf")
    last = len(poly) - 2
    for i in range(len(poly) - 1):
        ax, ay = poly[i]
        bx, by = poly[i + 1]
        dx = bx - ax
        dy = by - ay
        seg = math.hypot(dx, dy)
        if seg < 1e-9:
            continue
        t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / (seg * seg)
        # 첫/마지막 세그먼트만 바깥으로 연장한다. 중간은 clamp.
        lo = -extend / seg if i == 0 else 0.0
        hi = 1.0 + extend / seg if i == last else 1.0
        if t < lo:
            t = lo
        elif t > hi:
            t = hi
        qx = ax + t * dx
        qy = ay + t * dy
        d = math.hypot(p[0] - qx, p[1] - qy)
        if d < best_d:
            best = (qx, qy)
            best_d = d
    return best, best_d


#: corridor_side() 의 수직 판정을 신뢰할 최소 |sin| (약 33도).
#: 이보다 작으면 벽이 차에서 거의 **방사 방향**이라 좌우 판정이 무의미하다.
#: [합성검증 2026-08-22] 처음에 0.30 으로 뒀더니 conf 0.34~0.46 구간에서
#: **자신 있게 틀리는** 프레임이 남았다(목표점 오차 0.69~0.80m). 확신하는
#: 프레임들의 실제 conf 는 0.68~1.0 이라 0.55 는 그것들을 안 건드린다.
SIDE_MIN_SIN = 0.55


def corridor_side(chain):
    """사슬(벽) 기준으로 **차가 어느 쪽에 있는지**. (side, 확신도) 를 준다.
    side: +1 = 차가 벽의 왼쪽, -1 = 오른쪽. 확신도: 0.0~1.0.

    차는 언제나 복도 안이고 벽은 언제나 복도 바깥이므로, 이 값이 곧
    "복도가 벽의 어느 쪽인가"다. 좌/우 라벨과 편측 폴백의 미는 방향을
    전부 이것으로 정한다.

    ★ 왜 y 부호로는 안 되는가
      S자 커브에서는 **우측 벽 콘도 y>0 이 된다**(모듈 docstring 전개표).
      합성검증에서 실제로 편측 폴백이 벽의 반대쪽으로 밀어 목표점이 0.80m
      — 정확히 복도 폭만큼 — 어긋났다 (2026-08-22).

    ★ 왜 수직 판정만으로도 안 되는가
      벽이 차에서 거의 **방사 방향**으로 보이면(급커브에서 벽을 옆이 아니라
      정면으로 보는 순간) 원점이 접선 위에 얹혀 cross 가 0 에 수렴한다.
      합성검증 실측: 사슬 [(0.47,-0.61),(0.74,-0.96)] 에서 cross=0.0005 —
      부호가 노이즈로 결정된다.

    그래서 둘을 **겹쳐 쓴다**: 수직 판정이 SIDE_MIN_SIN 이상으로 뚜렷하면
    그걸 믿고, 아니면 y 부호로 떨어진다. 두 단서가 서로의 실패 구간을
    덮는다 — y 부호는 먼 콘에서 깨지고, 수직 판정은 방사 방향에서 깨진다.
    """
    if not chain:
        return 1, 0.0
    if len(chain) < 2:
        return (-1 if chain[0][1] > 0.0 else 1), 0.0

    # 원점에 가장 가까운 정점의 접선을 기준으로 원점의 좌우를 본다.
    best_i = 0
    best_d = float("inf")
    for i, (cx, cy) in enumerate(chain):
        d = math.hypot(cx, cy)
        if d < best_d:
            best_d = d
            best_i = i
    j = min(best_i + 1, len(chain) - 1)
    k = max(best_i - 1, 0)
    tx = chain[j][0] - chain[k][0]
    ty = chain[j][1] - chain[k][1]
    qx, qy = chain[best_i]
    tn = math.hypot(tx, ty)
    qn = math.hypot(qx, qy)
    if tn < 1e-9 or qn < 1e-9:
        return (-1 if qy > 0.0 else 1), 0.0

    # cross(t, origin - q) > 0 이면 원점이 진행방향 왼쪽에 있다.
    cross = tx * (-qy) - ty * (-qx)
    conf = abs(cross) / (tn * qn)          # |sin(끼인각)|
    if conf >= SIDE_MIN_SIN:
        return (1 if cross > 0.0 else -1), conf
    return (-1 if qy > 0.0 else 1), conf   # 방사 방향 — y 부호로 폴백


def build_chains(cones, chain_max_dist, extend, reassign_dist, min_cones,
                 min_gap, max_gap):
    """콘을 좌/우 두 사슬로 나눈다. 반환 (left, right) — 각각 차에서 가까운 순.

    3단계다:
      1) 시드   원점 최근접 콘 A, 그리고 A 와 반대 부호쪽 최근접 콘 B.
                **차 바로 옆**은 커브에서도 좌우 부호가 안 뒤집힌다
                (뒤집히는 건 위 docstring 처럼 r>1.0 의 먼 콘이다).
                반대 부호가 아예 없으면 폭 창 [min_gap, max_gap] 안의
                최근접 콘을 B 로 쓴다.
      2) 성장   각 시드에서 문턱 chain_max_dist(0.60) 로 greedy 확장.
                0.60 < W(0.80) 이므로 반대편 벽까지 건너뛸 수 없다.
      3) 복구   남은 콘을 두 폴리라인 중 **수직거리**가 가까운 쪽에 편입.
                자기 벽이면 0.03~0.16m, 반대 벽이면 >=0.80m 라 5배 여유.
                둘 다 reassign_dist 를 넘으면 **버린다** — 콘 하나를 잃는
                것이 벽을 뒤집는 것보다 훨씬 싸다.

    ※ "가장 가까운 콘이 있는 사슬에 붙인다"로는 안 된다. 콘 하나가 빠지면
      자기 벽까지 0.868m, 반대 벽까지 0.80m 라 **반대 벽이 이긴다.**
      반드시 방향(폴리라인)을 봐야 한다.
    """
    pool = list(cones)
    if not pool:
        return [], []

    def r(p):
        return math.hypot(p[0], p[1])

    seed_a = min(pool, key=r)
    pool.remove(seed_a)

    opposite = [c for c in pool if c[1] * seed_a[1] < 0.0]
    if opposite:
        seed_b = min(opposite, key=r)
    else:
        window = [c for c in pool
                  if min_gap <= math.hypot(c[0] - seed_a[0],
                                           c[1] - seed_a[1]) <= max_gap]
        seed_b = min(window, key=r) if window else None
    if seed_b is not None:
        pool.remove(seed_b)

    def grow(chain):
        while pool:
            nxt = min(pool, key=lambda c: math.hypot(c[0] - chain[-1][0],
                                                     c[1] - chain[-1][1]))
            if math.hypot(nxt[0] - chain[-1][0],
                          nxt[1] - chain[-1][1]) > chain_max_dist:
                return
            pool.remove(nxt)
            chain.append(nxt)

    chain_a = [seed_a]
    grow(chain_a)
    chain_b = [seed_b] if seed_b is not None else []
    if chain_b:
        grow(chain_b)

    # 3) 복구 — 끊긴 조각 되붙이기
    for c in list(pool):
        da = polyline_closest_point(chain_a, c, extend)[1] if chain_a else float("inf")
        db = polyline_closest_point(chain_b, c, extend)[1] if chain_b else float("inf")
        if min(da, db) > reassign_dist:
            continue                      # 애매하면 버린다
        (chain_a if da <= db else chain_b).append(c)
        pool.remove(c)

    # 편입된 콘 때문에 순서가 깨졌으니 차에서 가까운 순으로 다시 세운다.
    # (x 순이 아니다 — S자에서 x 는 단조가 아니다.)
    chain_a.sort(key=r)
    chain_b.sort(key=r)

    if len(chain_a) < min_cones:
        chain_a = []
    if len(chain_b) < min_cones:
        chain_b = []

    # 좌/우 배정은 **차가 벽의 어느 쪽에 있는가**로 한다 (콘의 y 부호가 아니라).
    # 차가 벽의 오른쪽이면(side<0) 그 벽은 좌측 벽이다. corridor_side() 참고.
    if chain_a and chain_b:
        # 벽이 둘이면 라벨은 반드시 서로 반대다. 확신도가 높은 쪽이 정하고
        # 나머지는 그 반대를 받는다 — 방사 방향으로 보이는 벽이 애매한 판정을
        # 내려 **둘 다 같은 라벨이 되는 것**을 막는다(그러면 하나가 덮여
        # 사라지고 멀쩡한 양벽 프레임이 편측 폴백으로 떨어진다).
        sa, ca = corridor_side(chain_a)
        sb, cb = corridor_side(chain_b)
        if ca >= cb:
            side_a = sa
        else:
            side_a = -sb
        return (chain_a, chain_b) if side_a < 0 else (chain_b, chain_a)

    for ch in (chain_a, chain_b):
        if ch:
            return (ch, []) if corridor_side(ch)[0] < 0 else ([], ch)
    return [], []


def centerline(left, right, min_gap, max_gap, extend):
    """좌우 사슬에서 복도 중심선을 만든다. 차에서 가까운 순.

    x 격자 보간이 **아니라** 최근접 대응이다. S자에서 좌우 벽의 x 범위가
    거의 겹치지 않기 때문이다(위 전개 예: 내벽 x 0.38~0.49 동안 외벽은
    0.43~1.26 — 공통 x 구간이 거의 없다). 대응 방식은 커브에서 오히려
    정확하다: 두 벽 사이의 최근접 방향이 곧 복도의 법선 방향이고, 그 중점이
    정확히 중심선 위다. 곡률이 일정할 필요도 없다.
    (R=0.90 검산: 계산값 (0.688,0.319) vs 참값 (0.687,0.318))

    폭 창 [min_gap, max_gap] 을 벗어난 대응은 버린다. **상한이 특히
    중요하다** — 옛 코드에는 상한이 없어서, 한쪽 벽이 비면 반대편 트랙 콘과
    짝지어 폭 1.5m 짜리 엉뚱한 중심선이 생길 수 있었다.
    """
    mids = []
    for src, dst in ((left, right), (right, left)):
        for c in src:
            q, d = polyline_closest_point(dst, c, extend)
            if q is None or not (min_gap <= d <= max_gap):
                continue
            mids.append(((c[0] + q[0]) / 2.0, (c[1] + q[1]) / 2.0))

    mids.sort(key=lambda p: math.hypot(p[0], p[1]))
    out = []
    for m in mids:
        if out and math.hypot(m[0] - out[-1][0], m[1] - out[-1][1]) < 0.05:
            continue                      # 양방향 대응이 만든 중복 제거
        out.append(m)
    return out


def offset_from_single_wall(chain, offset, side=None):
    """한쪽 벽만 보일 때 — 폴리라인 **법선 방향**으로 offset 만큼 민 선.

    옛 코드는 y 축으로만 밀었다(`ly - offset`). 직선에서만 맞고 커브에서는
    크게 틀린다. R=0.90 외벽 검산: 법선 방향이면 (0.558,0.193) 로 참값
    (0.557,0.193) 과 일치하지만, y 축으로 밀면 (0.805,0.279) 로 **0.26m**
    벗어난다 — 좌우 여유가 0.20m 인 복도에서는 그대로 콘 접촉이다.

    커브 안쪽 벽은 콘이 원래 성기다(같은 각도 구간에 바깥 벽보다 적게 놓인다).
    그래서 이 경로는 예외가 아니라 **커브에서 정상적으로 쓰이는 경로**다.
    벽이 하나뿐일 때의 품질이 곧 커브 주행 품질이다.
    """
    if len(chain) < 2:
        return []
    # 미는 방향은 **차가 있는 쪽**이다. 콘의 y 부호가 아니다 —
    # corridor_side() docstring 의 0.80m 오차 사례 참고.
    # side 를 넘겨받으면 그대로 쓴다 (호출자가 시간 연속성으로 정한 경우).
    if side is None:
        side = corridor_side(chain)[0]
    out = []
    n = len(chain)
    for i in range(n):
        px, py = chain[i]
        j = min(i + 1, n - 1)
        k = max(i - 1, 0)
        tx = chain[j][0] - chain[k][0]
        ty = chain[j][1] - chain[k][1]
        norm = math.hypot(tx, ty)
        if norm < 1e-9:
            continue
        tx /= norm
        ty /= norm
        # 진행방향 기준 좌측 법선은 (-ty, tx). side=+1(차가 왼쪽)이면 그쪽으로.
        if side > 0:
            nx, ny = -ty, tx
        else:
            nx, ny = ty, -tx
        out.append((px + offset * nx, py + offset * ny))
    return out


def resolve_side(chain, offset, conf_side, conf, prev_target):
    """편측 폴백에서 **어느 쪽으로 밀지**를 최종 결정한다.

    반환: side(+1/-1). 근거가 하나도 없으면 None (호출자는 이 벽을 쓰지 않는다).

    ── 판단 순서 ──
    1순위는 **직전 프레임의 목표점**이다: 두 후보 경로 중 그쪽에 가까운 것을
    고른다. 없을 때만 corridor_side() 의 수직 판정으로 떨어지고, 그것마저
    애매하면(conf < SIDE_MIN_SIN) None 이다.

    ── 왜 시간 연속성이 1순위인가 ──
    판별력이 비교가 안 된다. 두 후보는 **복도 폭만큼 = 0.80m** 떨어져 있는데,
    복도 중심은 한 프레임(10Hz, 저속)에 0.06m 남짓 움직인다 — 13배 차이다.
    반면 수직 판정은 벽이 차에서 거의 **방사 방향**으로 보이는 순간 무너진다.
    [합성검증 2026-08-22] conf 0.34~0.46 구간에서 **자신 있게 틀려** 목표점이
    0.69~0.80m 어긋났다. 그게 SIDE_MIN_SIN 을 0.55 로 올린 이유다.

    잘못 물려도 자기교정된다: 양쪽 벽이 보이는 프레임의 중심선은 side 를 아예
    쓰지 않고, 콘을 오래 놓치면 호출자가 prev_target 을 버린다.

    ── 왜 애매하면 None 인가 ──
    틀린 방향으로 밀면 목표점이 복도 폭만큼 반대로 간다. 좌우 여유가 0.20m 인
    복도에서 그건 그대로 콘 접촉이다. 한 프레임 경로를 못 내는 것(-> FTG)이
    훨씬 싸다.
    """
    if prev_target is not None:
        best = conf_side
        best_d = float("inf")
        for cand in (1, -1):
            path = offset_from_single_wall(chain, offset, cand)
            if not path:
                continue
            d = polyline_closest_point(path, prev_target, 0.0)[1]
            if d < best_d:
                best_d = d
                best = cand
        return best

    # 직전 목표점이 없다 = 첫 프레임이거나 콘을 오래 놓친 직후.
    if conf >= SIDE_MIN_SIN:
        return conf_side
    return None


def target_at_lookahead(path, lookahead, axle_offset):
    """뒤축에서 유클리드 거리가 lookahead 인 path 위의 점 (라이다 좌표).

    ★ 기준점은 **뒤축**이다. 옛 코드는 라이다 원점을 뒤축으로 취급해
      Pure Pursuit 을 돌렸다. 라이다가 뒤축보다 0.41m 앞에 있으므로
      각도 alpha 가 과대평가되어 **약 2배 과조향**이 났다
      (목표 (1.0,0.3) 기준: 10.4도 vs 올바른 5.5도). 좌우 여유가 0.20m 인
      복도에서 그 크기는 그대로 지그재그 접촉이다.

    반환: (목표점(라이다 좌표), 실제 lookahead, clamp 되었는지)

    실제 lookahead 를 같이 돌려주는 이유: 중심선이 차에서 멀리서 시작하면
    요청값보다 커지고, 코너 컷이 lookahead^2/(8R) 로 커진다. 그 사실이
    디버그에 안 보이면 실차에서 "왜 코너를 깎지" 의 원인을 못 찾는다.
    """
    if not path:
        return None, 0.0, False
    pts = [(x + axle_offset, y) for (x, y) in path]

    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        dx = bx - ax
        dy = by - ay
        a = dx * dx + dy * dy
        if a < 1e-12:
            continue
        b = 2.0 * (ax * dx + ay * dy)
        c = ax * ax + ay * ay - lookahead * lookahead
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            continue
        sq = math.sqrt(disc)
        for t in ((-b + sq) / (2.0 * a), (-b - sq) / (2.0 * a)):
            if 0.0 <= t <= 1.0:
                return (ax + t * dx - axle_offset, ay + t * dy), lookahead, False

    # 교점이 없다 = 중심선 전체가 원 안이거나 밖이다.
    near = pts[0]
    far = pts[-1]
    pick = near if math.hypot(near[0], near[1]) > lookahead else far
    return (pick[0] - axle_offset, pick[1]), math.hypot(pick[0], pick[1]), True


def steer_pure_pursuit(target, axle_offset, wheelbase, gain, lookahead_min,
                       angle_limit):
    """Pure Pursuit. target 은 라이다 좌표, 내부에서 뒤축으로 옮긴다."""
    bx = target[0] + axle_offset
    by = target[1]
    dist = math.hypot(bx, by)
    if dist < 1e-3:
        return 0.0
    ld = max(dist, lookahead_min)
    angle = math.atan2(2.0 * wheelbase * (by / dist), ld) * gain
    return max(-angle_limit, min(angle_limit, angle))
