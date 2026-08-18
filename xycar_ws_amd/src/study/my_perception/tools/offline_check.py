#!/usr/bin/env python3
"""영상 파일로 인식 모듈을 검증/튜닝하는 오프라인 도구.

ROS 없이 Windows/PC에서 바로 돌아간다. 젯슨 오리진이 준비되기 전까지
차선 추정과 신호등 투표 로직을 실제 주행영상으로 다듬는 용도.

    python3 tools/offline_check.py <video> [--model models/best5.pt]
                                           [--lane left|right]
                                           [--every 2] [--viz out_dir]

--viz 를 주면 피팅 곡선과 차로중심을 그린 프레임을 저장한다.
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from my_perception.detect import extract  # noqa: E402
from my_perception.lane import LaneEstimator  # noqa: E402
from my_perception.light_vote import LightVoter  # noqa: E402

# 노드와 동일한 기본 임계값. 바꿔가며 튜닝할 때는 여기와
# my_bringup/config/drive_params.yaml 을 함께 맞출 것.
DASHED_CONF = 0.40
SOLID_CONF = 0.25


LIGHT_COLOR = {
    "NONE": (160, 160, 160), "RED": (0, 0, 255), "YELLOW": (0, 220, 220),
    "GREEN": (0, 220, 0), "LEFT": (0, 220, 120),
}


def draw(img, est, dashed, solid, res, state_name="NONE", cone_n=0, car=None):
    """피팅 곡선과 트랙 중앙을 그린다 (디버깅/시각화용).

    노랑=중앙선(dashed) 파랑=좌경계 빨강=우경계 초록=추정 트랙중앙 흰=화면중심
    """
    from my_perception.lane import _fit

    h, w = img.shape[:2]
    fd = _fit(dashed, est.y_lo, est.y_hi, est.min_pts, est.min_span)
    left, right = est._split_solid(solid, fd)
    fl = _fit(left, est.y_lo, est.y_hi, est.min_pts, est.min_span)
    fr = _fit(right, est.y_lo, est.y_hi, est.min_pts, est.min_span)

    for f, col in ((fd, (0, 255, 255)), (fl, (255, 60, 60)), (fr, (60, 60, 255))):
        if f is None:
            continue
        for y in range(est.y_lo, est.y_hi):
            x = int(np.polyval(f, y))
            if 0 <= x < w:
                cv2.circle(img, (x, y), 2, col, -1)

    # 화면 중심(=카메라 위치)
    cv2.line(img, (w // 2, est.y_lo), (w // 2, est.y_hi), (255, 255, 255), 1)

    # 추정한 트랙 중앙: 두 평가행 사이를 이어 그린다
    x_near = int(w / 2 + est.center_bias_px + res.offset_near)
    x_far = int(w / 2 + est.center_bias_px + res.offset_far)
    cv2.line(img, (x_far, est.eval_far), (x_near, est.eval_near), (0, 255, 0), 2)
    for x, y, r in ((x_near, est.eval_near, 7), (x_far, est.eval_far, 5)):
        cv2.circle(img, (x, y), r, (0, 255, 0), -1)
        cv2.circle(img, (x, y), r, (0, 0, 0), 1)

    # 조향 오차 막대: 화면 중심에서 트랙 중앙까지
    cv2.arrowedLine(img, (w // 2, est.eval_near), (x_near, est.eval_near),
                    (0, 255, 0), 2, tipLength=0.3)

    # 상단 정보 패널
    cv2.rectangle(img, (0, 0), (w, 30), (0, 0, 0), -1)
    cv2.putText(img, f"offset {res.offset_near:+5.0f}px  q{res.quality:.2f}"
                     f"  {'OK' if res.valid else 'HOLD'}",
                (6, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
    cv2.putText(img, f"LIGHT {state_name}", (w - 190, 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, LIGHT_COLOR.get(state_name, (200, 200, 200)), 2)
    if cone_n:
        cv2.putText(img, f"cone x{cone_n}", (w - 320, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
    if car is not None:
        cv2.putText(img, "CAR", (w - 390, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--model", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "models", "best5.pt"))
    ap.add_argument("--bias", type=float, default=0.0,
                    help="center_bias_px. 카메라 장착 편차 보정값")
    ap.add_argument("--every", type=int, default=2, help="N프레임마다 1장 처리")
    ap.add_argument("--limit", type=int, default=0, help="최대 처리 프레임 수 (0=전체)")
    ap.add_argument("--viz", default="", help="시각화 프레임 저장 디렉토리")
    ap.add_argument("--out-video", default="", help="시각화 영상 저장 경로 (.mp4)")
    ap.add_argument("--out-fps", type=float, default=15.0, help="출력 영상 fps")
    args = ap.parse_args()

    from ultralytics import YOLO

    model = YOLO(args.model)
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"영상을 열 수 없음: {args.video}")

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    est = LaneEstimator(width, height, center_bias_px=args.bias)
    voter = LightVoter(window=30, min_weight=3.0, min_ratio=0.5, miss_tolerance=10)

    if args.viz:
        os.makedirs(args.viz, exist_ok=True)

    writer = None
    if args.out_video:
        d = os.path.dirname(os.path.abspath(args.out_video))
        if d:
            os.makedirs(d, exist_ok=True)
        writer = cv2.VideoWriter(args.out_video,
                                 cv2.VideoWriter_fourcc(*"mp4v"),
                                 args.out_fps, (width, height))

    idx = done = 0
    valid_n = 0
    offs, quals, states = [], [], []
    jumps = []  # 프레임 간 오프셋 급변량. 튐(추정오류)과 실제 주행편차를 가르는 단서
    prev_off = None

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        if idx % args.every:
            continue

        result = model.predict(frame, conf=0.20, verbose=False)[0]
        det = extract(result, model.names, width, height,
                      dashed_conf=DASHED_CONF, solid_conf=SOLID_CONF)
        dashed, solid = det.dashed, det.solid
        cone_n = det.cone_n
        car = (det.car_cx, det.car_conf) if det.car_present else None

        res = est.update(dashed, solid)
        state = voter.update(det.lamp, det.lamp_conf, det.light_width)

        done += 1
        valid_n += res.valid
        if res.valid:
            offs.append(res.offset_near)
            quals.append(res.quality)
            if prev_off is not None:
                jumps.append(abs(res.offset_near - prev_off))
            prev_off = res.offset_near
        states.append(state)

        if writer is not None or (args.viz and done % 10 == 0):
            from my_perception.light_vote import STATE_TO_NAME as _S
            vis = draw(frame.copy(), est, dashed, solid, res,
                       _S[state], cone_n, car)
            if writer is not None:
                writer.write(vis)
            if args.viz and done % 10 == 0:
                cv2.imwrite(os.path.join(args.viz, f"v_{done:04d}.jpg"), vis)

        if args.limit and done >= args.limit:
            break

    cap.release()
    if writer is not None:
        writer.release()
        print(f"시각화 영상 저장 : {args.out_video}")

    o = np.array(offs) if offs else np.zeros(1)
    q = np.array(quals) if quals else np.zeros(1)
    print(f"처리 프레임        : {done}")
    print(f"차선 유효율        : {valid_n / max(done, 1) * 100:.0f}%")
    print(f"offset_near 중앙값 : {np.median(o):+.0f}px  "
          f"(5~95%tile {np.percentile(o, 5):+.0f} ~ {np.percentile(o, 95):+.0f})")
    print(f"quality 중앙값     : {np.median(q):.2f}")
    if jumps:
        j = np.array(jumps)
        # 30fps에서 every=N이면 샘플 간격은 N/30초. 그 사이 오프셋이 크게 뛰면
        # 물리적 이동이 아니라 추정이 튄 것으로 봐야 한다.
        print(f"프레임간 변화량    : 중앙값 {np.median(j):.0f}px, "
              f"95%tile {np.percentile(j, 95):.0f}px, 최대 {j.max():.0f}px")
        print(f"  100px 초과 급변  : {(j > 100).mean() * 100:.1f}%  "
              f"(높으면 추정 튐, 낮으면 실제 주행편차)")
    from collections import Counter
    from my_perception.light_vote import STATE_TO_NAME
    c = Counter(states)
    dist = ", ".join(f"{STATE_TO_NAME[k]} {v / done * 100:.0f}%"
                     for k, v in sorted(c.items(), key=lambda kv: -kv[1]))
    print(f"신호등 확정상태 분포: {dist}")


if __name__ == "__main__":
    main()
