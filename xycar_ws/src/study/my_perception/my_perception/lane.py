"""차선 마스크 -> 조향용 횡방향 오프셋.

ROS/ultralytics 의존성 없음 (numpy만). 영상으로 오프라인 검증하려고 일부러 분리했다.
검증 도구: tools/offline_check.py

────────────────────────────────────────────────────────────────────────
설계 근거 (전부 실제 주행영상 실측. reference/perception_analysis.md 참고)

1. 목표는 "차로 중심"이 아니라 "트랙 중앙"이다.
   좌/우 차로 중심을 목표로 두면 오프셋 중앙값이 -151px / +206px로 대칭으로
   크게 벌어졌다. 반면 좌우 흰 실선의 중점을 기준으로 하면 +39px / -40px로
   0에 가까웠다. 즉 차량은 차로를 나눠 달리는 게 아니라 트랙 중앙을 따라간다.
   노란 점선은 차로 구분선이 아니라 트랙 중앙 표시로 취급해야 한다.

2. 노란 점선(dashed_line)이 곧 트랙 중앙이다.
   실측 338표본에서 (dashed) - (좌우경계 중점) = 중앙값 +3.0px.
   게다가 dashed 피팅 성공률 97% > 좌우경계 동시검출 69~91%.
   따라서 dashed를 1순위로 쓰고, 좌우 경계는 보완/정밀화에 쓴다.

3. 행 단위 샘플링은 쓰지 않는다.
   dashed는 물리적으로 끊긴 점선이라 특정 행을 찍으면 빈 구간에 걸린다.
   실측: 프레임 단위 96% vs 행 단위(y=340) 55%. 조각 픽셀을 전부 모아
   2차 다항식 x=f(y)로 피팅하면 97%로 회복된다.

4. solid_line은 좌/우를 반드시 분리한다.
   합쳐서 중앙값을 내면 좌우 경계선이 섞여 엉뚱한 값이 나온다.

5. 차로폭은 상수가 아니다 — 다만 **학습값을 계속 갱신하지도 않는다.**
   원근 때문에 행마다 크게 변한다. 양쪽이 보이는 프레임에서 행별 반폭을
   모아 **중앙값으로 잠그고**(half_lock_frames), 한쪽만 보일 때 그 값으로
   보완한다. 예전에는 EMA 로 계속 갱신했는데, 이 값은 한쪽만 보일 때
   중앙 추정에 **그대로 편향으로 들어가므로** 누적되면 위험하다
   (2026-08-24). half_near_px/half_far_px 로 완전 고정도 가능하다.
────────────────────────────────────────────────────────────────────────
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class LaneResult:
    offset_near: float  # 가까운 평가행에서 (트랙중앙 - 화면중심), 픽셀. +면 트랙이 오른쪽
    offset_far: float   # 전방주시행에서의 같은 값. near와의 차이가 곡률/헤딩을 뜻한다
    valid: bool         # 이번 프레임에 실제로 차선을 봤는지 (hold된 값이면 False)
    quality: float      # 0~1. 근거가 많을수록 높다

    # 학습된 행별 트랙 반폭(픽셀). 좌우 흰 실선을 동시에 본 프레임마다 EMA 로 갱신된다.
    # 0.0 이면 아직 한 번도 학습하지 못했다는 뜻.
    #
    # 왜 밖으로 내보내나: 회피 기동에서 "트랙의 반쪽 중앙"(= 트랙중앙 ± 반폭/2)을
    # 목표로 삼기 위해서다. 고정 픽셀로 옆으로 밀면 원근·트랙폭 변화를 무시하게 되는데,
    # 이 값은 이미 그 둘을 반영해 학습돼 있다. 판단은 my_driver 의 몫이므로 여기서는
    # 값만 실어 보낸다.
    half_near: float = 0.0
    half_far: float = 0.0

    # 선행 곱률 신호(px). 2차 피팅 x=a*y^2+b*y+c 의 a 로 만든다:
    #     curve_px = a * (curve_preview_row - eval_near)^2
    #   = "선행행까지 갔을 때 차선이 접선에서 몇 px 휘는가"
    #
    # ★ bend(offset_far-offset_near) 와 결정적으로 다른 점:
    #   x(v) 를 분해하면 횟편차 y 는 v 에 **선형**, 헤딩은 **상수**,
    #   곱률만 **2차**로 들어간다. 그래서 a 는 횟편차·헤딩에 불변이다.
    #   (합성검증: 같은 곱률에서 횟편차 ±20cm 흔들기 —
    #    a 변동 0.0% / bend 변동 108.8%)
    #   또 bend 는 eval_far(전방 ~1.25m) 한 점만 보지만, 피팅은
    #   y_lo~y_hi 전구간(약 0.46~3.75m)을 쓴다 = 훨씬 멀리 본다.
    # 부호가 있다(좌/우). 속도 계획은 abs 를 쓴다.
    curve_px: float = 0.0

    # --- 좌회전(지름길) 전용: **가장 왼쪽 노란픽셀 밴드**만 쓴 추정 ---
    # 분기점에서는 dashed 마스크에 직진 노란선과 좌회전 노란선이 **둘 다**
    # 들어온다. 전부 평균내면 그 사이 어딘가를 겨냥해 어느 쪽도 못 탄다.
    # 행별로 가장 왼쪽 픽셀에서 left_band_px 안쪽만 남기면 좌회전 가지가
    # 선택되고, 좌회전을 마친 뒤에는 합류 차선을 그대로 따라간다.
    #
    # 평소 주행은 이 값을 **쓰지 않는다** — driver_node 가 SHORTCUT 상태에서만
    # 골라 쓴다. 판단은 my_driver 의 몫이므로 여기서는 값만 실어 보낸다.
    offset_near_left: float = 0.0
    offset_far_left: float = 0.0
    valid_left: bool = False


def _fit(instances, y_lo, y_hi, min_pts, min_span):
    """(xs, ys) 인스턴스들을 합쳐 x = f(y) 2차 다항식으로 피팅. 실패 시 None."""
    if not instances:
        return None
    xs = np.concatenate([p[0] for p in instances])
    ys = np.concatenate([p[1] for p in instances])
    sel = (ys >= y_lo) & (ys <= y_hi)
    xs, ys = xs[sel], ys[sel]
    if xs.size < min_pts:
        return None
    # y 범위가 좁으면 2차 피팅이 폭주한다 (numpy 2.x 호환: .ptp() 대신 max-min)
    if float(ys.max() - ys.min()) < min_span:
        return None
    return np.polyfit(ys, xs, 2)


def _left_band(instances, band_px):
    """행마다 **가장 왼쪽 픽셀에서 band_px 이내**만 남긴다.

    분기점에서 직진 노란선과 좌회전 노란선이 같이 잡힐 때, 왼쪽 가지만
    고르기 위한 필터다. 행별로 자르는 이유: 전체 최소 x 하나로 자르면
    원근 때문에 먼 행이 통째로 탈락한다(먼 행일수록 x 가 중앙에 몰린다).

    반환: [(xs, ys)] 형태의 인스턴스 1개. 남은 픽셀이 없으면 [].
    """
    if band_px <= 0 or not instances:
        return instances
    xs = np.concatenate([p[0] for p in instances])
    ys = np.concatenate([p[1] for p in instances])
    if xs.size == 0:
        return []
    rows = ys.astype(np.int64)
    idx = rows - rows.min()
    min_x = np.full(int(idx.max()) + 1, np.inf)
    np.minimum.at(min_x, idx, xs.astype(np.float64))
    keep = xs <= min_x[idx] + band_px
    if not keep.any():
        return []
    return [(xs[keep], ys[keep])]


def horizontal_dashed_left_strip(dashed_instances, strip_width_px=30.0):
    """세로형 dashed 는 그대로 두고, **가로형만** 왼쪽 strip_width_px 로 자른다.

    [2026-08-23 이식] 팀원 구현(race_perception/branch_detector.py)을
    그대로 옮겼다. 좌회전을 마치고 **본선으로 합류할 때** 쓴다.

    ──────────────────────────────────────────────────────────
    문제: 합류 지점에서는 본선의 노란선이 화면을 **가로로** 가로질러
    보인다. 그것까지 합쳐서 피팅하면 x = f(y) 가 퇴화해 조향이 튀다.

    해법: 인스턴스를 **가로형 / 세로형으로 먼저 분류**한다.
        x 퍼짐 <= y 퍼짐   ->  세로형 = 내가 가는 방향의 차선  ->  그대로 둔다
        x 퍼짐 >  y 퍼짐   ->  가로형 = 앞을 가로지르는 선    ->  왼쪽 끝만 남긴다

    ★ 이것이 _left_band() 와 다른 점이다. _left_band 는 가로/세로를
      구분하지 않고 **모든** dashed 를 행별로 깎으므로, 진행 방향 차선까지
      왼쪽으로 편향시킨다(합성검증: 단일선 -3px, 가로선 -36px).
      분기점에서는 _left_band, 합류에서는 이 함수를 쓴다.

    가로형을 자를 때 화면 x=0 이 아니라 **가로형 픽셀을 전부 합친 뒤
    그 최소 x** 에서 strip_width_px 만큼만 남긴다.

    반환: 필터를 거친 dashed 인스턴스 리스트.
    """
    filtered = []
    horizontal = []
    for xs, ys in dashed_instances:
        xs = np.asarray(xs)
        ys = np.asarray(ys)
        if xs.size == 0 or ys.size == 0:
            continue
        if float(np.ptp(xs)) <= float(np.ptp(ys)):
            filtered.append((xs, ys))
            continue
        horizontal.append((xs, ys))
    if horizontal:
        xs = np.concatenate([item[0] for item in horizontal])
        ys = np.concatenate([item[1] for item in horizontal])
        left = float(np.min(xs))
        selected = xs <= left + max(1.0, float(strip_width_px))
        if np.any(selected):
            filtered.append((xs[selected], ys[selected]))
    return filtered


class LaneEstimator:
    """차선 마스크에서 트랙 중앙을 추정한다.

    트랙 중앙 산출 우선순위:
      1. dashed + 좌우 solid 모두   -> dashed와 좌우중점의 평균, quality 1.0
      2. dashed만                   -> dashed 곡선 (트랙 중앙과 3px 일치), quality 0.9
      3. 좌우 solid만               -> 두 곡선의 중점, quality 0.8
      4. 한쪽 solid만               -> 학습된 행별 반폭으로 보완, quality 0.4
                                       (+ 이상치 방어. 아래 설명 참고)
      5. 아무것도 없음              -> 직전 값 유지(hold), valid=False

    이상치 방어 (모든 경로 공통):
        피팅이 실패하면 말도 안 되는 값이 나온다. 실측(테스트영상 428프레임):
            |offset| 최대 746px, 프레임간 변화 최대 1158px
        화면 폭이 632px인데 746px 오차면 트랙이 화면 밖 한참 너머라는 뜻이고,
        0.5초 만에 1158px 이동은 물리적으로 불가능하다.

        **극단값은 quality 0.9(노란선만 보임) 경로에서 나온다** — 저신뢰 경로인
        4번(한쪽 흰선만, 최대 242px)이 아니다. 급커브에서 좌우 경계선이 번갈아
        사라질 때 노란선 하나로 피팅하면서, 화면 밖으로 나가는 부분을 **외삽**
        하다가 2차 다항식이 발산하기 때문이다.

        그래서 두 가지 상한을 둔다:
          - max_center_offset_px : 화면 중심에서 이만큼 넘게 벗어난 추정은 기각
          - max_jump_px          : 직전 채택값에서 이만큼 넘게 튄 추정은 기각
        기각되면 hold 로 넘어간다 (직전 값 유지).
    """

    def __init__(
        self,
        width,
        height,
        y_lo=270,
        y_hi=412,   # [2026-08-23] 425 -> 412. 차량 커버가 가리는 하단 위
        eval_near=400,
        eval_far=310,
        center_bias_px=0.0,
        min_pts=40,   # [2026-08-23] 50 -> 40. y_hi 축소로 표본이 줄었다
        min_span=20,
        hold_frames=15,
        half_alpha=0.05,
        half_lock_frames=30,
        half_near_px=0.0,
        half_far_px=0.0,
        max_center_offset_px=480.0,
        max_jump_px=250.0,
        left_band_px=30.0,
        curve_preview_row=280,
    ):
        self.width = width
        self.height = height
        self.y_lo = y_lo                    # 지평선 아래
        self.y_hi = y_hi                    # 차체(검은 원형)에 가려지는 하단 위
        self.eval_near = eval_near
        self.eval_far = eval_far
        # 카메라가 차량 중심선에서 벗어나 장착된 경우의 보정. 실차에서 정지 상태로
        # 트랙 중앙에 세워놓고 offset을 읽어 그 값을 여기 넣으면 0이 목표가 된다.
        self.center_bias_px = center_bias_px
        self.min_pts = min_pts
        self.min_span = min_span
        self.hold_frames = hold_frames
        self.half_alpha = half_alpha        # 행별 반폭 EMA 계수 (lock 전에는 안 쓴다)
        # ── 반폭을 **고정**한다 (2026-08-24) ────────────────────────────
        # 왜: _half 는 한쪽 흰선만 보일 때 중앙을 외삽하는 기준이라
        # (center = xl + half), 값이 틀리면 **출력에 그대로 편향으로 들어간다.**
        # EMA(alpha 0.05)로 계속 갱신하면 그 편향이 프레임을 넘어 누적되고,
        # 트랙 폭이 다른 구간을 지날 때마다 기준이 흔들린다.
        #
        #   half_near_px / half_far_px > 0  -> 그 값으로 **완전 고정**. 학습 안 함.
        #   half_lock_frames > 0            -> 좌우 흰선을 동시에 본 프레임을
        #                                      이만큼 모아 **중앙값**으로 잠근다.
        #                                      잠긴 뒤에는 절대 안 바뀐다.
        #   둘 다 0                          -> 옛 EMA 동작 (누적됨)
        #
        # 중앙값을 쓰는 이유: EMA 는 초반 이상치를 오래 끌고 가지만 중앙값은
        # 표본 절반이 망가져도 버틴다. 잠그기 전에도 지금까지의 중앙값을 쓰므로
        # 초기 프레임부터 바로 사용 가능하다.
        self.half_lock_frames = half_lock_frames
        self.half_fixed = {eval_near: half_near_px, eval_far: half_far_px}
        # 화면 중심에서 이만큼 넘게 벗어난 추정은 피팅 실패로 보고 기각
        self.max_center_offset_px = max_center_offset_px
        # 직전 채택값에서 이만큼 넘게 튄 추정은 기각 (물리적으로 불가능한 이동)
        self.max_jump_px = max_jump_px
        # 좌회전 전용 밴드 폭(px). 0 이면 좌측밴드 추정을 끈다.
        self.left_band_px = left_band_px
        # 선행 곱률을 평가할 행. **낮을수록 멀리 본다.**
        # y_lo 보다 낮게 두면 표본 밖 외삽이 된다.
        self.curve_preview_row = curve_preview_row

        self._half = {eval_near: None, eval_far: None}
        # 잠그기 전까지 모으는 표본과, 잠금 여부.
        self._half_samples = {eval_near: [], eval_far: []}
        self._half_locked = {eval_near: False, eval_far: False}
        # 행별로 마지막에 채택된 중앙 x. 이상치 판정의 기준.
        self._last_center = {eval_near: None, eval_far: None}
        # 좌측밴드 경로는 **별도의** 이상치 기준을 쓴다. 같은 딕셔너리를
        # 공유하면 두 경로가 서로의 max_jump 판정을 오염시킨다.
        self._last_center_lb = {eval_near: None, eval_far: None}
        self._last = LaneResult(0.0, 0.0, False, 0.0)
        self._miss = 0
        self._rejected = 0                  # 이상치로 버린 횟수 (진단용)
        self._curve_px = 0.0                # 선행 곱률 직전값

    def reset(self):
        self._half = {self.eval_near: None, self.eval_far: None}
        self._half_samples = {self.eval_near: [], self.eval_far: []}
        self._half_locked = {self.eval_near: False, self.eval_far: False}
        self._last_center = {self.eval_near: None, self.eval_far: None}
        self._last_center_lb = {self.eval_near: None, self.eval_far: None}
        self._last = LaneResult(0.0, 0.0, False, 0.0)
        self._miss = 0
        self._rejected = 0

    @property
    def rejected_count(self):
        """이상치로 버려진 fallback 추정 횟수. 로그/진단용."""
        return self._rejected

    def _split_solid(self, solid_instances, f_dashed):
        """중앙선 곡선(없으면 화면 중심)을 기준으로 경계선을 좌/우로 가른다."""
        left, right = [], []
        for xs, ys in solid_instances:
            if xs.size == 0:
                continue
            y_ref = float(np.median(ys))
            x_ref = np.polyval(f_dashed, y_ref) if f_dashed is not None else self.width / 2.0
            (left if float(np.median(xs)) < x_ref else right).append((xs, ys))
        return left, right

    def _update_half(self, row, half):
        """좌우 흰선을 동시에 본 프레임에서만 호출. 반폭 기준을 갱신/잠근다.

        고정값이 주어졌거나 이미 잠겼으면 **아무것도 하지 않는다** — 그게
        이 함수의 존재 이유다(누적 편향 차단).
        """
        if self.half_fixed.get(row, 0.0) > 0.0 or self._half_locked[row]:
            return

        if self.half_lock_frames > 0:
            s = self._half_samples[row]
            s.append(half)
            # 잠그기 전에도 바로 쓸 수 있게 지금까지의 중앙값을 넣어 둔다.
            self._half[row] = float(np.median(s))
            if len(s) >= self.half_lock_frames:
                self._half_locked[row] = True
                s.clear()               # 잠근 뒤에는 표본을 들고 있을 이유가 없다
            return

        # half_lock_frames = 0 -> 옛 EMA 동작 (값이 계속 누적된다)
        prev = self._half[row]
        self._half[row] = half if prev is None else (
            (1 - self.half_alpha) * prev + self.half_alpha * half
        )

    def _half_at(self, row):
        """이 행에서 쓸 반폭. 고정값이 있으면 그것이 최우선."""
        fixed = self.half_fixed.get(row, 0.0)
        return fixed if fixed > 0.0 else self._half[row]

    def _curvature_px(self, f_dashed, f_left, f_right):
        """선행행에서의 곱률 신호(px). 근거가 없으면 직전값 유지.

        dashed 가 1순위다(성공률 97%, 트랙중앙과 3px 일치). 없으면
        좌/우 흰선의 a 를 평균낸다 — 둘은 평행이라 곱률이 같다.
        """
        a = None
        if f_dashed is not None:
            a = float(f_dashed[0])
        elif f_left is not None and f_right is not None:
            a = (float(f_left[0]) + float(f_right[0])) / 2.0
        elif f_left is not None:
            a = float(f_left[0])
        elif f_right is not None:
            a = float(f_right[0])
        if a is None:
            return self._curve_px
        d = float(self.curve_preview_row - self.eval_near)
        return a * d * d

    def _center_at(self, row, f_dashed, f_left, f_right):
        """평가행에서 트랙 중앙 x와 quality. 행별 반폭 EMA도 갱신한다."""
        xd = float(np.polyval(f_dashed, row)) if f_dashed is not None else None
        xl = float(np.polyval(f_left, row)) if f_left is not None else None
        xr = float(np.polyval(f_right, row)) if f_right is not None else None

        mid = None
        if xl is not None and xr is not None:
            mid = (xl + xr) / 2.0
            self._update_half(row, abs(xr - xl) / 2.0)

        if xd is not None and mid is not None:
            return self._accept(row, (xd + mid) / 2.0, 1.0)
        if xd is not None:
            return self._accept(row, xd, 0.9)
        if mid is not None:
            return self._accept(row, mid, 0.8)

        # 한쪽 흰선만 보임. 학습된 반폭으로 중앙을 추정한다.
        half = self._half_at(row)
        if half is None:
            # 반폭을 아직 한 번도 학습하지 못했다 (좌우 흰선을 동시에 본 적 없음).
            # 근거 없이 추정하면 위험하므로 hold 로 넘긴다.
            return None, 0.0
        if xl is not None:
            return self._accept(row, xl + half, 0.4)
        if xr is not None:
            return self._accept(row, xr - half, 0.4)
        return None, 0.0

    def _accept(self, row, center_x, quality, store=None):
        """이상치를 걸러낸 뒤 채택한다. 기각되면 (None, 0.0) 을 돌려 hold 로 넘긴다.

        모든 경로에 공통 적용한다. 극단값이 저신뢰 경로가 아니라 quality 0.9
        (노란선만 보임) 에서 나오는 것이 실측으로 확인됐기 때문이다.
        """
        # ① 물리적 상한 — 트랙 중앙이 화면 중심에서 이만큼 벗어날 수는 없다.
        #    다항식 외삽이 발산하면 수백~수천 px 이 나온다.
        if abs(center_x - self.width / 2.0) > self.max_center_offset_px:
            self._rejected += 1
            return None, 0.0

        # ② 급변 상한 — 직전 채택값에서 이만큼 튀는 것은 물리적으로 불가능하다.
        store = self._last_center if store is None else store
        prev = store[row]
        if prev is not None and abs(center_x - prev) > self.max_jump_px:
            self._rejected += 1
            return None, 0.0

        store[row] = center_x
        return center_x, quality

    def _dashed_center_at(self, row, f_dashed_lb):
        """좌측밴드 노란선만으로 목표 x 를 낸다 (흰 실선을 쓰지 않는다).

        좌회전 분기에서는 흰 실선이 **다른 차로의 경계**라 섞으면 오히려
        직진 차로 쪽으로 끌린다. 그래서 이 경로는 노란선 단독이다.
        """
        if f_dashed_lb is None:
            return None, 0.0
        x = float(np.polyval(f_dashed_lb, row))
        return self._accept(row, x, 0.9, store=self._last_center_lb)

    def update(self, dashed_instances, solid_instances):
        """프레임당 1회 호출.

        dashed_instances / solid_instances:
            [(xs, ys), ...] 리스트. xs, ys는 마스크가 켜진 픽셀 좌표 ndarray.
            신뢰도 필터링은 호출부(ROS 노드/오프라인 도구)에서 끝난 상태로 넘어온다.
        """
        f_dashed = _fit(dashed_instances, self.y_lo, self.y_hi, self.min_pts, self.min_span)
        # 좌회전 전용 경로. 평소 주행에는 영향이 없다 — 값만 같이 실어 보내고
        # 쓸지 말지는 driver_node 가 FSM 상태로 정한다.
        f_dashed_lb = _fit(_left_band(dashed_instances, self.left_band_px),
                           self.y_lo, self.y_hi, self.min_pts, self.min_span)
        cn_lb, _ = self._dashed_center_at(self.eval_near, f_dashed_lb)
        cf_lb, _ = self._dashed_center_at(self.eval_far, f_dashed_lb)
        left, right = self._split_solid(solid_instances, f_dashed)
        f_left = _fit(left, self.y_lo, self.y_hi, self.min_pts, self.min_span)
        f_right = _fit(right, self.y_lo, self.y_hi, self.min_pts, self.min_span)

        self._curve_px = self._curvature_px(f_dashed, f_left, f_right)
        c_near, q_near = self._center_at(self.eval_near, f_dashed, f_left, f_right)
        c_far, q_far = self._center_at(self.eval_far, f_dashed, f_left, f_right)

        if c_near is None and c_far is None:
            self._miss += 1
            if self._miss > self.hold_frames:
                # ★ [2026-08-24] 급변 게이트 기준을 **버린다.**
                #   _last_center 는 출력에 더해지는 값이 아니라 max_jump_px
                #   판정의 기준일 뿐이다. 그런데 hold 를 포기할 만큼 오래
                #   놓쳤다면 그 사이 차는 계속 움직였으므로, 그 기준은 이미
                #   현실과 무관한 **stale 값**이다. 그대로 두면 차선이 정상적으로
                #   다시 보여도 "직전 값에서 250px 넘게 튀었다"고 기각해
                #   hold 에서 못 빠져나온다(자기 잠금).
                #   버리면 다음 유효 관측이 무조건 채택되고, 거기서부터 다시
                #   게이트가 선다.
                self._last_center = {self.eval_near: None, self.eval_far: None}
                self._last_center_lb = {self.eval_near: None, self.eval_far: None}
                # 오프셋은 무효로 떨어뜨리되 반폭은 살려둔다 — 반폭은 이 프레임의
                # 관측이 아니라 누적 학습값이라 차선을 놓쳤다고 사라지지 않는다.
                self._last = LaneResult(
                    self._last.offset_near, self._last.offset_far, False, 0.0,
                    half_near=self._half_at(self.eval_near) or 0.0,
                    half_far=self._half_at(self.eval_far) or 0.0,
                    offset_near_left=self._last.offset_near_left,
                    offset_far_left=self._last.offset_far_left,
                    valid_left=False,
                    curve_px=self._curve_px,
                )
            return self._last

        # 한쪽 평가행만 잡히면 다른 행은 그 값으로 대체한다 (곡률 정보는 포기)
        if c_near is None:
            c_near, q_near = c_far, q_far * 0.7
        if c_far is None:
            c_far, q_far = c_near, q_near * 0.7

        target = self.width / 2.0 + self.center_bias_px

        # 좌측밴드: 한쪽 행만 잡히면 다른 행을 그 값으로 대체한다(본 경로와 동일).
        valid_left = cn_lb is not None or cf_lb is not None
        if cn_lb is None:
            cn_lb = cf_lb
        if cf_lb is None:
            cf_lb = cn_lb
        off_n_lb = (cn_lb - target) if valid_left else self._last.offset_near_left
        off_f_lb = (cf_lb - target) if valid_left else self._last.offset_far_left

        self._miss = 0
        self._last = LaneResult(
            offset_near=c_near - target,
            offset_far=c_far - target,
            valid=True,
            quality=min(q_near, q_far),
            half_near=self._half_at(self.eval_near) or 0.0,
            half_far=self._half_at(self.eval_far) or 0.0,
            offset_near_left=off_n_lb,
            offset_far_left=off_f_lb,
            valid_left=valid_left,
            curve_px=self._curve_px,
        )
        return self._last
