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

5. 차로폭은 상수가 아니다.
   원근 때문에 행마다 크게 변한다. 양쪽이 보이는 프레임에서 행별 반폭을
   EMA로 학습해두고, 한쪽만 보일 때 그 값으로 보완한다.
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


class LaneEstimator:
    """차선 마스크에서 트랙 중앙을 추정한다.

    트랙 중앙 산출 우선순위:
      1. dashed + 좌우 solid 모두   -> dashed와 좌우중점의 평균, quality 1.0
      2. dashed만                   -> dashed 곡선 (트랙 중앙과 3px 일치), quality 0.9
      3. 좌우 solid만               -> 두 곡선의 중점, quality 0.8
      4. 한쪽 solid만               -> 학습된 행별 반폭으로 보완, quality 0.4
      5. 아무것도 없음              -> 직전 값 유지(hold), valid=False
    """

    def __init__(
        self,
        width,
        height,
        y_lo=270,
        y_hi=425,
        eval_near=400,
        eval_far=310,
        center_bias_px=0.0,
        min_pts=50,
        min_span=20,
        hold_frames=15,
        half_alpha=0.05,
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
        self.half_alpha = half_alpha        # 행별 반폭 EMA 계수

        self._half = {eval_near: None, eval_far: None}
        self._last = LaneResult(0.0, 0.0, False, 0.0)
        self._miss = 0

    def reset(self):
        self._half = {self.eval_near: None, self.eval_far: None}
        self._last = LaneResult(0.0, 0.0, False, 0.0)
        self._miss = 0

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

    def _center_at(self, row, f_dashed, f_left, f_right):
        """평가행에서 트랙 중앙 x와 quality. 행별 반폭 EMA도 갱신한다."""
        xd = float(np.polyval(f_dashed, row)) if f_dashed is not None else None
        xl = float(np.polyval(f_left, row)) if f_left is not None else None
        xr = float(np.polyval(f_right, row)) if f_right is not None else None

        mid = None
        if xl is not None and xr is not None:
            mid = (xl + xr) / 2.0
            half = abs(xr - xl) / 2.0
            prev = self._half[row]
            self._half[row] = half if prev is None else (
                (1 - self.half_alpha) * prev + self.half_alpha * half
            )

        if xd is not None and mid is not None:
            return (xd + mid) / 2.0, 1.0
        if xd is not None:
            return xd, 0.9
        if mid is not None:
            return mid, 0.8

        half = self._half[row]
        if half is None:
            return None, 0.0
        if xl is not None:
            return xl + half, 0.4
        if xr is not None:
            return xr - half, 0.4
        return None, 0.0

    def update(self, dashed_instances, solid_instances):
        """프레임당 1회 호출.

        dashed_instances / solid_instances:
            [(xs, ys), ...] 리스트. xs, ys는 마스크가 켜진 픽셀 좌표 ndarray.
            신뢰도 필터링은 호출부(ROS 노드/오프라인 도구)에서 끝난 상태로 넘어온다.
        """
        f_dashed = _fit(dashed_instances, self.y_lo, self.y_hi, self.min_pts, self.min_span)
        left, right = self._split_solid(solid_instances, f_dashed)
        f_left = _fit(left, self.y_lo, self.y_hi, self.min_pts, self.min_span)
        f_right = _fit(right, self.y_lo, self.y_hi, self.min_pts, self.min_span)

        c_near, q_near = self._center_at(self.eval_near, f_dashed, f_left, f_right)
        c_far, q_far = self._center_at(self.eval_far, f_dashed, f_left, f_right)

        if c_near is None and c_far is None:
            self._miss += 1
            if self._miss > self.hold_frames:
                self._last = LaneResult(
                    self._last.offset_near, self._last.offset_far, False, 0.0
                )
            return self._last

        # 한쪽 평가행만 잡히면 다른 행은 그 값으로 대체한다 (곡률 정보는 포기)
        if c_near is None:
            c_near, q_near = c_far, q_far * 0.7
        if c_far is None:
            c_far, q_far = c_near, q_near * 0.7

        target = self.width / 2.0 + self.center_bias_px
        self._miss = 0
        self._last = LaneResult(
            offset_near=c_near - target,
            offset_far=c_far - target,
            valid=True,
            quality=min(q_near, q_far),
        )
        return self._last
