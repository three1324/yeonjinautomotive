"""라이다 스캔 -> 라바콘 복도 중앙 추정.

ROS 의존성 없음 (numpy만). lane.py 와 같은 방식으로 오프라인 검증이 가능하다.

────────────────────────────────────────────────────────────────────────
왜 필요한가

라바콘이 좌우로 촘촘히 늘어서 **복도(벽)** 를 이루고, 그 복도가 S자로 굽어 있다.
**우측 콘 벽이 흰 실선보다 안쪽**에 있어서, 페인트 차선 중심을 따라가면
우측 콘 벽으로 들어간다. 즉 콘 구간에서는 콘 벽이 실제 주행 가능 경계다.

라바콘은 입체물이라 라이다에 잘 보인다 (바닥 테이프는 안 보이는 것과 대조).
그래서 이 구간만큼은 카메라가 아니라 라이다가 주 센서가 된다.

────────────────────────────────────────────────────────────────────────
설계 — 차선 추정과 완전히 대칭

    차선:  YOLO 마스크 → 픽셀 좌표 → polyfit x=f(y) → 두 행 평가   → offset 2개
    복도:  라이다 스캔  → 콘 좌표   → polyfit y=f(x) → 두 거리 평가 → offset 2개
                                                                       ↑ 출력 형식 동일

출력이 LaneResult 와 같은 형식(픽셀 오프셋)이므로 **제어기를 하나도 안 고쳐도 된다.**

────────────────────────────────────────────────────────────────────────
좌표계와 부호 (헷갈리기 쉬우니 주의)

    라이다:  x = 전방(+),  y = 좌측(+)        [ROS 표준]
    화면:    offset > 0 = 트랙 중앙이 화면 중심보다 **오른쪽**

복도 중앙이 y = +0.2 (차량 왼쪽)이면 왼쪽으로 가야 하므로 offset 은 **음수**다.
따라서 변환 시 부호를 뒤집는다:  offset_px = -y_center_m * px_per_meter
"""

from dataclasses import dataclass

import numpy as np


def _as_points(pair):
    """(xs, ys) ndarray 쌍 -> 순수 파이썬 [(x, y), ...]. 없으면 빈 튜플."""
    if pair is None:
        return ()
    xs, ys = pair
    return tuple((float(a), float(b)) for a, b in zip(xs, ys))


@dataclass
class CorridorResult:
    """LaneResult 와 같은 형식. 단위도 픽셀로 맞춰 제어기에 그대로 넣을 수 있다."""

    offset_near: float   # px. + 면 복도 중앙이 화면 중심보다 오른쪽
    offset_far: float    # px
    valid: bool
    quality: float       # 0~1
    width_m: float       # 측정된 복도 폭 (진단용). 0이면 미측정
    n_bins: int          # 피팅에 쓴 구간 수 (진단용)

    # --- 시각화/진단용. 제어에는 쓰지 않는다 ---
    method: str = "none"        # "walls" | "refine" | "bins" | "hold"
    # 중앙선을 샘플링한 라이다 좌표 [(x, y), ...]. RViz 에 그대로 그린다.
    centerline: tuple = ()
    # 벽으로 채택된 점 [(x, y), ...]. "왜 저 값이 나왔는가"를 보려면 이게 필요하다.
    left_pts: tuple = ()
    right_pts: tuple = ()


class CorridorEstimator:
    """라이다 점군에서 좌우 벽 사이 복도 중앙선을 추정한다.

    처리 흐름:
      1. 극좌표 → 직교좌표 (x 전방, y 좌측+)
      2. 전방 거리로 구간 분할 (bin)
      3. 각 구간에서 좌/우 최근접 벽점 탐색
      4. 중앙선 곡선 추정 — **아래 사다리** (정확한 순서대로 시도)
      5. 두 전방거리에서 평가 → offset 2개 (픽셀로 변환)

    ────────────────────────────────────────────────────────────────
    중앙선 추정 사다리 (lane.py 의 우선순위 구조와 대칭)

      1) walls  : 좌/우 벽을 **각각** 폴리핏한 뒤 두 곡선의 중점   quality 1.0
      2) refine : 1차 추정 곡선에 **수직**으로 다시 좌우 벽을 재측정  quality 0.8
      3) bins   : 구간별 중점을 바로 피팅 (종전 방식)              quality 0.4~

    왜 이 순서인가 — 종전 방식(3)은 **x축에 수직인 슬라이스**로 좌우 벽을 찾는다.
    복도가 굽으면 그 슬라이스가 복도를 비스듬히 자르므로 중점이 곡선 안쪽으로
    편향된다. 곡률이 클수록 커지고, 먼 평가지점에서 크게 나타난다. S자 코스가
    정확히 그 구간이다.

    [실측 2026-08-19, 합성 S자 코스] far 평가지점 평균 절대오차:
        완만 S(0.25·sin1.6x)   bins 91.7px   walls 15.0px   refine 17.2px
        중간 S(0.30·sin2.0x)   bins 148.7px  walls 11.0px   refine 97.0px
    walls 가 가장 정확하지만 양쪽 벽이 각각 충분히 보여야 해서 가용성이 낮고
    (15회 중 5회/3회), refine 은 가용성이 100% 다. 그래서 사다리로 엮는다.
    3) 이 남아 있으므로 **어떤 경우에도 종전보다 나빠지지 않는다.**

    이 오차가 중요한 이유: SteeringController 는 (offset_far - offset_near) 를
    곡률 선행보상으로 쓴다. far 가 과소평가되면 S자에서 선행보상이 부족해
    안쪽으로 파고들거나 반응이 늦다.
    """

    def __init__(
        self,
        # --- 관심 영역 ---
        x_min=0.25,              # 이보다 가까운 점은 무시 (차체/노이즈)
        x_max=2.2,               # 이보다 먼 점은 무시. eval_far(1.5m) 보다 조금만
                                 # 크게 잡는다. 멀리까지 넣으면 그 구간의 부정확한
                                 # 추정이 2차 피팅 전체를 끌어당긴다 (실측: 곡선에서
                                 # x_max=3.0 일 때 far 오차 -31px, 2.2 로 줄이면 개선)
        max_lateral=1.5,         # |y| 가 이보다 크면 복도 벽이 아니라고 본다
        min_lateral=0.06,        # |y| 가 이보다 작으면 벽이 아니라 정면 장애물
        # --- 구간 분할 ---
        bin_size=0.15,           # 전방 구간 폭 (m).
                                 # x_max 를 2.2m 로 줄인 만큼 구간을 촘촘히 나눠야
                                 # 피팅에 쓸 표본이 확보된다. 0.25 로 두면 구간이
                                 # 4개뿐이라 한쪽 벽만 보이는 구간을 버리는 순간
                                 # min_bins 를 못 채워 추정이 통째로 실패한다.
        min_bins=4,              # 피팅에 필요한 최소 구간 수
        min_span_m=0.5,          # 구간들이 이만큼은 앞뒤로 퍼져 있어야 피팅
        min_points_per_side=1,   # 한 구간에서 벽으로 인정할 최소 점 수
        # --- 평가 지점 ---
        eval_near_m=0.6,
        eval_far_m=1.5,
        # --- 단위 변환 ---
        px_per_meter=300.0,
        # --- 한쪽 벽만 보일 때 ---
        nominal_half_width_m=0.35,
        # 양쪽 벽이 다 보이는 구간이 이 개수 이상이면, 한쪽만 보이는 구간은
        # 피팅에서 아예 뺀다. 곡선에서 바깥쪽 벽이 max_lateral 을 벗어나면
        # 공칭 반폭 보정이 크게 틀리는데(실측 0.7m 오차), 그 값이 피팅에 들어가면
        # 곡선 전체가 망가진다. 양쪽 근거가 충분할 때는 굳이 쓸 이유가 없다.
        min_both_bins=3,
        # --- 중앙선 추정 방식 (아래 update() 의 사다리 참고) ---
        wall_min_bins=4,
        # ↑ 한쪽 벽을 독립적으로 피팅하는 데 필요한 최소 구간 수.
        #   [실측 2026-08-19] 2/3/4 를 비교했을 때 4 가 가장 정확했다
        #   (완만S far 51.5 -> 31.2px, 중간S 103.6 -> 97.4px). 3 이하로 낮추면
        #   2차 피팅이 사실상 보간이 되어(점 3개, 계수 3개) 노이즈를 그대로 탄다.
        refine_iters=2,          # 2패스 법선 재추정 반복 횟수. 0이면 비활성
        refine_samples=14,       # 재추정 시 곡선을 몇 지점에서 자를지
        # 재추정 밴드 폭은 bin_size 를 그대로 쓴다. [실측] 1.5배/2배로 넓히면
        # 곡선 구간에서 반대편 벽 점까지 빨려들어와 오히려 크게 나빠졌다
        # (중간S far 97 -> 169 -> 192px).
        path_samples=12,         # 발행용 중앙선 샘플 개수
        # --- 안정화 (lane.py 에서 얻은 교훈) ---
        hold_frames=10,
        max_jump_px=250.0,
    ):
        self.x_min = x_min
        self.x_max = x_max
        self.max_lateral = max_lateral
        self.min_lateral = min_lateral
        self.bin_size = bin_size
        self.min_bins = min_bins
        self.min_span_m = min_span_m
        self.min_points_per_side = min_points_per_side
        self.eval_near_m = eval_near_m
        self.eval_far_m = eval_far_m
        self.px_per_meter = px_per_meter
        self.nominal_half_width_m = nominal_half_width_m
        self.min_both_bins = min_both_bins
        self.wall_min_bins = wall_min_bins
        self.refine_iters = refine_iters
        self.refine_samples = refine_samples
        self.path_samples = path_samples
        self.hold_frames = hold_frames
        self.max_jump_px = max_jump_px

        self._last = CorridorResult(0.0, 0.0, False, 0.0, 0.0, 0)
        self._last_near_px = None
        self._miss = 0
        self._rejected = 0

    def reset(self):
        self._last = CorridorResult(0.0, 0.0, False, 0.0, 0.0, 0)
        self._last_near_px = None
        self._miss = 0
        self._rejected = 0

    @property
    def rejected_count(self):
        return self._rejected

    # ------------------------------------------------------------------
    # 1단계: 스캔 → 직교좌표
    # ------------------------------------------------------------------
    def to_points(self, ranges, angle_min, angle_increment,
                  range_min=0.0, range_max=np.inf):
        """LaserScan 형식 -> (x, y) 점군. 관심 영역 밖은 걸러낸다."""
        r = np.asarray(ranges, dtype=np.float64)
        if r.size == 0:
            return np.empty(0), np.empty(0)

        ang = angle_min + np.arange(r.size) * angle_increment
        ok = np.isfinite(r) & (r >= max(range_min, 1e-3)) & (r <= range_max)
        r, ang = r[ok], ang[ok]

        x = r * np.cos(ang)      # 전방 +
        y = r * np.sin(ang)      # 좌측 +

        sel = (
            (x >= self.x_min) & (x <= self.x_max)
            & (np.abs(y) <= self.max_lateral)
            & (np.abs(y) >= self.min_lateral)
        )
        return x[sel], y[sel]

    # ------------------------------------------------------------------
    # 2~4단계: 구간별 좌우 벽 → 복도 중앙점
    # ------------------------------------------------------------------
    def centerline_points(self, x, y):
        """구간별 복도 중앙점 목록과 폭 목록을 낸다.

        각 구간에서 **중앙선에 가장 가까운 점**을 벽으로 본다.
        (콘 뒤에 벽이 더 있어도 우리를 막는 것은 앞쪽 콘이다)
        """
        if x.size == 0:
            return np.empty(0), np.empty(0), np.empty(0)

        n_bins = max(1, int(np.ceil((self.x_max - self.x_min) / self.bin_size)))
        idx = np.clip(((x - self.x_min) / self.bin_size).astype(int), 0, n_bins - 1)

        cx, cy, widths = [], [], []
        for b in range(n_bins):
            m = idx == b
            if not np.any(m):
                continue
            xb, yb = x[m], y[m]

            left = yb[yb > 0]
            right = yb[yb < 0]

            has_l = left.size >= self.min_points_per_side
            has_r = right.size >= self.min_points_per_side
            if not has_l and not has_r:
                continue

            # 중앙선에 가장 가까운 점 = 실질적인 벽
            y_l = float(np.min(left)) if has_l else None      # 양수 중 최소
            y_r = float(np.max(right)) if has_r else None     # 음수 중 최대

            if has_l and has_r:
                mid = (y_l + y_r) / 2.0
                w = y_l - y_r
            elif has_l:
                # 왼쪽 벽만 보임 → 공칭 반폭만큼 오른쪽으로
                mid = y_l - self.nominal_half_width_m
                w = 0.0
            else:
                mid = y_r + self.nominal_half_width_m
                w = 0.0

            cx.append(float(np.mean(xb)))
            cy.append(mid)
            widths.append(w)

        return np.array(cx), np.array(cy), np.array(widths)

    def _wall_points(self, x, y, side):
        """한쪽 벽의 구간별 대표점. side=+1 왼쪽(y>0), -1 오른쪽(y<0).

        각 구간에서 **중앙선에 가장 가까운 점**을 그 구간의 벽으로 본다
        (콘 뒤에 뭐가 더 있어도 우리를 막는 것은 앞쪽 콘이다).
        """
        m = (y > 0) if side > 0 else (y < 0)
        xs, ys = x[m], y[m]
        if xs.size == 0:
            return np.empty(0), np.empty(0)

        n_bins = max(1, int(np.ceil((self.x_max - self.x_min) / self.bin_size)))
        idx = np.clip(((xs - self.x_min) / self.bin_size).astype(int), 0, n_bins - 1)

        px, py = [], []
        for b in range(n_bins):
            sel = idx == b
            if np.count_nonzero(sel) < self.min_points_per_side:
                continue
            yb = ys[sel]
            px.append(float(np.mean(xs[sel])))
            py.append(float(np.min(yb)) if side > 0 else float(np.max(yb)))
        return np.array(px), np.array(py)

    def _fit_walls(self, x, y):
        """사다리 1) 좌/우 벽을 각각 피팅해 두 곡선의 중점을 낸다.

        구간별로 먼저 중점을 내는 방식과 달리, 한쪽 벽이 빠진 구간이 있어도
        나머지 구간들이 그 벽의 곡선을 지탱한다. 그래서 편향이 훨씬 작다.
        """
        lx, ly = self._wall_points(x, y, +1)
        rx, ry = self._wall_points(x, y, -1)
        if lx.size < self.wall_min_bins or rx.size < self.wall_min_bins:
            return None, None, None
        if (float(lx.max() - lx.min()) < self.min_span_m
                or float(rx.max() - rx.min()) < self.min_span_m):
            return None, None, None
        cl = np.polyfit(lx, ly, 2)
        cr = np.polyfit(rx, ry, 2)
        return (cl + cr) / 2.0, (lx, ly), (rx, ry)

    def _refine(self, coef, x, y):
        """사다리 2) 곡선에 **수직**으로 좌우 벽을 다시 찾아 중앙선을 고친다.

        x축 수직 슬라이스가 굽은 복도를 비스듬히 자르는 편향을 없앤다.
        수렴하지 않거나 표본이 모자라면 들어온 곡선을 그대로 돌려준다.
        """
        for _ in range(max(0, self.refine_iters)):
            xs = np.linspace(self.x_min, self.x_max, self.refine_samples)
            ys = np.polyval(coef, xs)
            dv = np.polyval(np.polyder(coef), xs)
            norm = np.hypot(1.0, dv)
            nx, ny = -dv / norm, 1.0 / norm      # 곡선의 법선 (좌측 +)

            cxs, cys = [], []
            for xi, yi, nxi, nyi in zip(xs, ys, nx, ny):
                dx, dy = x - xi, y - yi
                t = dx * nxi + dy * nyi          # 법선 성분 (좌 +)
                u = dx * nyi - dy * nxi          # 접선 성분
                near = np.abs(u) <= self.bin_size
                if not np.any(near):
                    continue
                tn = t[near]
                tl, tr = tn[tn > 0], tn[tn < 0]
                if tl.size == 0 or tr.size == 0:
                    continue
                mid = (float(np.min(tl)) + float(np.max(tr))) / 2.0
                cxs.append(xi + mid * nxi)
                cys.append(yi + mid * nyi)

            if len(cxs) < self.min_bins:
                return coef
            cxs, cys = np.array(cxs), np.array(cys)
            if float(cxs.max() - cxs.min()) < self.min_span_m:
                return coef
            coef = np.polyfit(cxs, cys, 2)
        return coef

    def _sample(self, coef):
        """발행용 중앙선 샘플. 라이다 좌표 [(x, y), ...]."""
        xs = np.linspace(self.x_min, self.x_max, self.path_samples)
        return tuple((float(a), float(b)) for a, b in zip(xs, np.polyval(coef, xs)))

    # ------------------------------------------------------------------
    # 5~6단계: 피팅 및 평가
    # ------------------------------------------------------------------
    def update(self, ranges, angle_min, angle_increment,
               range_min=0.0, range_max=np.inf):
        """프레임당 1회 호출. LaneResult 와 같은 형식의 결과를 낸다."""
        x, y = self.to_points(ranges, angle_min, angle_increment, range_min, range_max)
        cx, cy, widths = self.centerline_points(x, y)

        if cx.size < self.min_bins:
            return self._hold()

        both = widths > 0
        n_both = int(np.sum(both))
        width_m = float(np.median(widths[both])) if n_both else 0.0

        # 양쪽 벽이 다 보인 구간만으로 **피팅이 성립할 때에만** 그것만 쓴다.
        # 성립 조건을 확인하지 않고 걸러내면, 양쪽 구간이 min_bins 에 못 미칠 때
        # 멀쩡한 한쪽-벽 구간까지 버려서 추정이 통째로 실패한다.
        bx, by = cx[both], cy[both]
        use_both_only = (
            n_both >= self.min_both_bins
            and bx.size >= self.min_bins
            and float(bx.max() - bx.min()) >= self.min_span_m
        )

        # ---- 중앙선 사다리 (클래스 docstring 의 근거 참고) ----
        # 1) walls: 좌/우 벽을 각각 피팅한 뒤 중점. 가장 정확하지만 조건이 까다롭다.
        coef, lpts, rpts = self._fit_walls(x, y)
        method = "walls"
        quality = 1.0

        if coef is None:
            # 2)/3) 은 구간별 중점에서 출발한다.
            if use_both_only:
                fx, fy = bx, by
                quality = 1.0
            else:
                # 한쪽 벽 추정이 섞인다. 그 비율만큼 신뢰도를 낮춘다.
                fx, fy = cx, cy
                quality = 0.4 + 0.6 * (n_both / max(cx.size, 1))

            if fx.size < self.min_bins or float(fx.max() - fx.min()) < self.min_span_m:
                return self._hold()

            coef = np.polyfit(fx, fy, 2)
            lpts = rpts = None
            method = "bins"

            # 2) refine: 곡선에 수직으로 재측정해 굽은 구간의 편향을 없앤다.
            if self.refine_iters > 0:
                refined = self._refine(coef, x, y)
                if refined is not coef:
                    coef = refined
                    method = "refine"
                    quality = max(quality, 0.8)

        y_near = float(np.polyval(coef, self.eval_near_m))
        y_far = float(np.polyval(coef, self.eval_far_m))

        # 라이다 y(좌측+) → 화면 offset(우측+) 이므로 부호를 뒤집는다
        near_px = -y_near * self.px_per_meter
        far_px = -y_far * self.px_per_meter

        # 급변 방어 — lane.py 와 같은 교훈. 피팅이 튀면 말도 안 되는 값이 나온다.
        if self._last_near_px is not None and \
                abs(near_px - self._last_near_px) > self.max_jump_px:
            self._rejected += 1
            return self._hold()

        self._miss = 0
        self._last_near_px = near_px
        n_used = (len(lpts[0]) + len(rpts[0])) if lpts is not None else int(cx.size)
        self._last = CorridorResult(
            near_px, far_px, True, quality, width_m, int(n_used),
            method=method,
            centerline=self._sample(coef),
            left_pts=_as_points(lpts),
            right_pts=_as_points(rpts),
        )
        return self._last

    def _hold(self):
        """근거 부족 → 직전 값 유지. 너무 오래 지속되면 무효로 알린다."""
        self._miss += 1
        if self._miss > self.hold_frames:
            self._last = CorridorResult(
                self._last.offset_near, self._last.offset_far,
                False, 0.0, self._last.width_m, self._last.n_bins,
                method="hold")
        return self._last
