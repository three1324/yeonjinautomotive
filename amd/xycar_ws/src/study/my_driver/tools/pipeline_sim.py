#!/usr/bin/env python3
"""영상 파일로 전체 파이프라인(YOLO 검출 -> 인지 -> 판단 -> planning -> 제어)을
단계별로 확인하는 오프라인 도구. ROS 없이 PC/젯슨에서 바로 돌아간다.

판단/planning/제어는 driver_node.py 가 실제로 쓰는 것과 **같은 모듈**
(my_driver.fsm/fusion/lateral/longitudinal/control)을 그대로 가져다 쓴다.
로직이 갈라지면 오프라인 검증의 의미가 없기 때문이다 (my_perception 쪽
detect.py/offline_check.py 와 같은 원칙).

    python3 tools/pipeline_sim.py <video>
    python3 tools/pipeline_sim.py <video> --out-video out.mp4 --every 2
    python3 tools/pipeline_sim.py <video> --model ../my_perception/models/best5.engine

파라미터(임계값·게인)는 기본적으로 my_bringup/config/drive_params.yaml 을
그대로 읽는다 — 실차에 쓰는 값과 다른 값을 검증하면 의미가 없기 때문이다.
--params 로 다른 yaml을 줄 수 있다.

이 도구가 실제 노드와 다른 점 (반드시 감안할 것):
    - 라이다 복도(/corridor)가 없다. 항상 invalid 로 취급하므로 카메라
      차선만으로 판단한다 — 라바콘 구간의 실제 동작과 다를 수 있다.
    - /drive_enable, 신호등 대기는 기본적으로 건너뛴다 (--wait-light 로 켬).
    - "모터 발행"은 실제 토픽 publish 가 아니라 driver_node 와 같은 형식의
      로그 줄로 콘솔에 찍는다 (동일한 각도/속도 계산 결과).

출력 영상은 좌: YOLO 원시 검출, 우: 인지(차선/신호등/장애물) + 판단·planning·
제어 HUD 를 합친 화면이다.

추가로 --traj-out 을 주면(기본은 --out-video 와 같이 켬) 매 프레임 계산된
angle/speed 명령을 자전거모델(bicycle model)로 적분해 "이 명령대로 움직였다면
어떤 경로를 그렸을지"를 별도 PNG 로 그린다. 축간거리 0.333m(실측,
xycar_motor/config/vesc.yaml), speed_weight 0.08, angle_limit=50 -> 약 19.5도
(drive_params.yaml 주석)를 쓴다.

⚠️ 이건 영상 속 카메라가 실제로 지나간 경로가 **아니다** — 오프라인 개루프
   시뮬레이션(적분값이 실제 타이어 미끄러짐·관성을 무시)이므로 절대 위치가
   아니라 "제어 판단이 대체로 맞는 방향인지" 정도의 정성적 확인용이다.
"""

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml

# Windows 기본 콘솔은 cp949 라 한글 출력에서 죽는다. 젯슨(UTF-8)에서는 무해.
if getattr(sys.stdout, "encoding", "") and sys.stdout.encoding.lower() not in ("utf-8", "utf8"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001 - 출력 인코딩 때문에 도구가 죽으면 안 된다
        pass

_HERE = os.path.dirname(os.path.abspath(__file__))
# my_driver/tools -> my_driver 패키지 루트 (my_driver.control 등 임포트용)
sys.path.insert(0, os.path.join(_HERE, ".."))
# my_driver/tools -> src/study -> my_perception 패키지 루트 (my_perception.detect 등 임포트용)
sys.path.insert(0, os.path.join(_HERE, "..", "..", "my_perception"))
# offline_check.py 의 draw() 를 재사용 (인지 시각화 중복 방지)
sys.path.insert(0, os.path.join(_HERE, "..", "..", "my_perception", "tools"))

from my_perception.detect import extract  # noqa: E402
from my_perception.lane import LaneEstimator  # noqa: E402
from my_perception.light_vote import STATE_TO_NAME, LightVoter  # noqa: E402

from my_driver.control import SpeedLimiter, SteeringController  # noqa: E402
from my_driver.fsm import DriveFSM  # noqa: E402
from my_driver.fusion import FusedResult, LateralFusion, LateralRef  # noqa: E402
from my_driver.lateral import LateralPlanner, OvertakeBehavior  # noqa: E402
from my_driver.longitudinal import LongitudinalPlanner  # noqa: E402

from offline_check import draw as draw_perception  # noqa: E402

LIGHT_NAME = {0: "NONE", 1: "RED", 2: "YELLOW", 3: "GREEN", 4: "LEFT"}

DEFAULT_MODEL = os.path.join(_HERE, "..", "..", "my_perception", "models", "best5.pt")
DEFAULT_PARAMS = os.path.join(_HERE, "..", "..", "my_bringup", "config", "drive_params.yaml")

# --- 경로 적분(dead-reckoning)용 물리 상수 ---
# [실측] xycar_motor/config/vesc.yaml:62, 2026-08-16 줄자 측정
WHEELBASE_M = 0.333
# xycar_motor speed 단위 -> m/s. drive_params.yaml 주석: speed 12 -> 약 0.96m/s
SPEED_WEIGHT = 0.08
# xycar_motor angle 단위 -> 실제 앞바퀴 조향각(deg). drive_params.yaml 주석:
# angle_limit=50.0 [고정] -> 약 19.5도
ANGLE_UNIT_TO_DEG = 19.5 / 50.0


class Trajectory:
    """angle/speed 명령을 자전거모델로 적분해 개루프 경로를 만든다.

    heading=0 을 "전방"으로 두고, +heading 은 오른쪽으로 도는 방향
    (angle>0 = 우조향, driver_node/lateral.py 의 부호 규약과 일치).
    """

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.xs = [0.0]
        self.ys = [0.0]
        self.ts = [0.0]
        self.speeds = [0.0]

    def step(self, dt, angle_unit, speed_unit, t):
        v = speed_unit * SPEED_WEIGHT
        steer_rad = math.radians(angle_unit * ANGLE_UNIT_TO_DEG)
        self.heading += (v / WHEELBASE_M) * math.tan(steer_rad) * dt
        self.x += v * math.cos(self.heading) * dt
        self.y += v * math.sin(self.heading) * dt
        self.xs.append(self.x)
        self.ys.append(self.y)
        self.ts.append(t)
        self.speeds.append(v)

    def save_plot(self, path):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(8, 8))
        sc = ax.scatter(self.ys, self.xs, c=self.ts, cmap="viridis", s=4)
        ax.plot(self.ys, self.xs, color="gray", linewidth=0.5, alpha=0.5)
        ax.plot(self.ys[0], self.xs[0], "go", markersize=10, label="start")
        ax.plot(self.ys[-1], self.xs[-1], "rs", markersize=10, label="end")
        ax.set_xlabel("lateral y (m, +right)")
        ax.set_ylabel("forward x (m)")
        ax.set_title(
            "open-loop dead-reckoning path from control commands\n"
            f"(wheelbase={WHEELBASE_M}m, speed_weight={SPEED_WEIGHT}, "
            "NOT the camera's real path)"
        )
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
        ax.legend()
        cbar = fig.colorbar(sc, ax=ax)
        cbar.set_label("t (s)")
        fig.tight_layout()
        fig.savefig(path, dpi=130)
        plt.close(fig)


class Obs:
    """driver_node.Obs 와 같은 필드. ROS 의존성 없이 여기서 다시 정의한다
    (원본은 driver_node.py 참고 — 필드가 늘면 거기와 맞출 것)."""

    def __init__(self):
        self.offset_near = 0.0
        self.offset_far = 0.0
        self.lane_valid = False
        self.quality = 0.0
        self.half_near = 0.0
        self.half_far = 0.0

        self.cor_near = 0.0
        self.cor_far = 0.0
        self.cor_valid = False
        self.cor_quality = 0.0

        self.light = 0

        self.cone_n = 0
        self.cone_near_y = 0.0
        self.car_present = False
        self.car_cx = 0.0
        self.car_bottom_y = 0.0

        self.front_dist = 99.0
        self.left_free = 99.0
        self.right_free = 99.0


def g(d, dotted, default):
    """drive_params.yaml 의 중첩 dict에서 'lane.y_lo' 같은 점표기 키를 읽는다."""
    cur = d
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def load_section(params_path, node_name):
    with open(params_path) as f:
        doc = yaml.safe_load(f)
    return doc.get(node_name, {}).get("ros__parameters", {})


def build_pipeline(perc_p, drv_p, width, height, auto_start, bias_override,
                   lidar_confirm=False):
    center_bias = bias_override if bias_override is not None else g(perc_p, "lane.center_bias_px", 0.0)
    est = LaneEstimator(
        width=width, height=height,
        y_lo=g(perc_p, "lane.y_lo", 270), y_hi=g(perc_p, "lane.y_hi", 425),
        eval_near=g(perc_p, "lane.eval_near", 400), eval_far=g(perc_p, "lane.eval_far", 310),
        center_bias_px=center_bias,
        min_pts=g(perc_p, "lane.min_pts", 50), min_span=g(perc_p, "lane.min_span", 20),
        hold_frames=g(perc_p, "lane.hold_frames", 15), half_alpha=g(perc_p, "lane.half_alpha", 0.05),
        max_center_offset_px=g(perc_p, "lane.max_center_offset_px", 480.0),
        max_jump_px=g(perc_p, "lane.max_jump_px", 250.0),
    )
    voter = LightVoter(
        window=g(perc_p, "light.window", 30), min_weight=g(perc_p, "light.min_weight", 3.0),
        min_ratio=g(perc_p, "light.min_ratio", 0.6), miss_tolerance=g(perc_p, "light.miss_tolerance", 10),
    )

    fsm = DriveFSM(
        start_confirm_frames=g(drv_p, "fsm.start_confirm_frames", 5),
        enable_shortcut=g(drv_p, "fsm.enable_shortcut", False),
        auto_start=auto_start,
    )
    fusion = LateralFusion(
        cone_n_lo=g(drv_p, "fusion.cone_n_lo", 2.0), cone_n_hi=g(drv_p, "fusion.cone_n_hi", 6.0),
        max_corridor_weight=g(drv_p, "fusion.max_corridor_weight", 0.9),
        weight_rate_per_sec=g(drv_p, "fusion.weight_rate_per_sec", 1.5),
        min_corridor_quality=g(drv_p, "fusion.min_corridor_quality", 0.4),
    )
    lateral = LateralPlanner(
        OvertakeBehavior(
            shift_px=g(drv_p, "lateral.shift_px", 120.0),
            trigger_bottom_y=g(drv_p, "lateral.trigger_bottom_y", 300.0),
            trigger_front_dist=g(drv_p, "lateral.trigger_front_dist", 1.5),
            side_clearance=g(drv_p, "lateral.side_clearance", 0.6),
            shift_sec=g(drv_p, "lateral.shift_sec", 0.8),
            pass_sec=g(drv_p, "lateral.pass_sec", 1.5),
            return_sec=g(drv_p, "lateral.return_sec", 1.0),
            cooldown_sec=g(drv_p, "lateral.cooldown_sec", 1.0),
            pass_exit_ratio=g(drv_p, "lateral.pass_exit_ratio", 0.85),
            pass_exit_cx_ratio=g(drv_p, "lateral.pass_exit_cx_ratio", 0.85),
            # 이 도구에는 라이다가 없어 front_dist 가 항상 99.0 이다. yaml 기본값
            # (true)을 그대로 쓰면 회피가 **절대** 발동하지 않아 검증이 불가능하다.
            # 그래서 여기서는 기본적으로 끄고, --lidar-confirm 으로 켤 수 있게 한다.
            require_lidar_confirm=lidar_confirm,
        ),
        enable_overtake=g(drv_p, "lateral.enable_overtake", True),
    )
    longitudinal = LongitudinalPlanner(
        base_speed=g(drv_p, "speed.base", 12.0), min_speed=g(drv_p, "speed.min", 4.0),
        curve_px_lo=g(drv_p, "speed.curve_px_lo", 30.0), curve_px_hi=g(drv_p, "speed.curve_px_hi", 150.0),
        curve_factor_min=g(drv_p, "speed.curve_factor_min", 0.45),
        quality_lo=g(drv_p, "speed.quality_lo", 0.4), quality_factor_min=g(drv_p, "speed.quality_factor_min", 0.5),
        cone_n_lo=g(drv_p, "speed.cone_n_lo", 2.0), cone_n_hi=g(drv_p, "speed.cone_n_hi", 8.0),
        cone_factor_min=g(drv_p, "speed.cone_factor_min", 0.6),
        stop_dist=g(drv_p, "speed.stop_dist", 0.35), slow_dist=g(drv_p, "speed.slow_dist", 1.2),
        overtake_factor=g(drv_p, "speed.overtake_factor", 0.7),
    )
    steering = SteeringController(
        k_lat=g(drv_p, "steer.k_lat", 0.10), k_curve=g(drv_p, "steer.k_curve", 0.15),
        k_damp=g(drv_p, "steer.k_damp", 0.004), angle_limit=g(drv_p, "steer.angle_limit", 50.0),
        rate_limit_per_sec=g(drv_p, "steer.rate_limit_per_sec", 180.0),
        lpf_alpha=g(drv_p, "steer.lpf_alpha", 0.45),
        speed_gain_ref=g(drv_p, "steer.speed_gain_ref", 0.0), invert=g(drv_p, "steer.invert", False),
    )
    speed_limiter = SpeedLimiter(
        accel_per_sec=g(drv_p, "speed.accel_per_sec", 20.0),
        decel_per_sec=g(drv_p, "speed.decel_per_sec", 60.0),
        speed_limit=g(drv_p, "speed.limit", 27.0),
    )
    return est, voter, fsm, fusion, lateral, longitudinal, steering, speed_limiter


def draw_hud(img, stage_state, ref, target_offset, angle, speed, reason, overtake_phase):
    """판단(FSM) / planning(융합·추월) / 제어(각도·속도) 를 하단 바에 표시한다."""
    h, w = img.shape[:2]
    bar_h = 78
    out = cv2.copyMakeBorder(img, 0, bar_h, 0, 0, cv2.BORDER_CONSTANT, value=(20, 20, 20))
    y0 = h + 18
    # cv2.putText 는 한글(non-ASCII)을 그리지 못하고 ????? 로 깨진다 — 라벨은 영문 고정.
    cv2.putText(out, f"3) decision(fsm)  state={stage_state}", (8, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
    cv2.putText(out, f"4) planning  target_off={target_offset:+6.1f}px  "
                     f"ref[{ref.source}] w={ref.corridor_weight:.2f} "
                     f"{'OK' if ref.valid else 'HOLD'}  overtake={overtake_phase}",
                (8, y0 + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1)
    cv2.putText(out, f"5) control  angle={angle:+6.1f}  speed={speed:5.1f}  ({reason})",
                (8, y0 + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 220, 0), 1)
    return out


def label_top(img, left_w, text_left, text_right):
    h, w = img.shape[:2]
    cv2.rectangle(img, (0, 0), (w, 24), (0, 0, 0), -1)
    cv2.putText(img, text_left, (8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
    cv2.putText(img, text_right, (left_w + 8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("--model", default=DEFAULT_MODEL, help="best5.pt 또는 best5.engine")
    ap.add_argument("--params", default=DEFAULT_PARAMS, help="drive_params.yaml 경로")
    ap.add_argument("--bias", type=float, default=None, help="lane.center_bias_px 덮어쓰기")
    ap.add_argument("--every", type=int, default=1, help="N프레임마다 1장 처리")
    ap.add_argument("--limit", type=int, default=0, help="최대 처리 프레임 수 (0=전체)")
    ap.add_argument("--out-video", default="", help="단계별 시각화 영상 저장 경로 (.mp4)")
    ap.add_argument("--viz", default="", help="10프레임마다 정지 이미지를 저장할 디렉토리")
    ap.add_argument("--traj-out", default="", help="경로 적분 결과 PNG 저장 경로. 기본은 "
                                                    "--out-video 와 같은 이름에 _trajectory.png")
    ap.add_argument("--no-traj", action="store_true", help="경로 적분을 끈다")
    ap.add_argument("--wait-light", dest="auto_start", action="store_false", default=True,
                     help="기본은 신호등 대기를 건너뛰고 바로 주행(auto_start). 이 플래그를 주면 "
                          "실전처럼 초록불 연속검출을 기다린다 (영상에 신호등이 있어야 출발함)")
    ap.add_argument(
        "--lidar-confirm", action="store_true",
        help="회피 트리거에 라이다 교차확인을 요구한다. 이 도구에는 라이다가 없어 "
             "켜면 회피가 발동하지 않는다 (실차 동작 재현용)")
    ap.add_argument("--log-every-frame", action="store_true",
                     help="기본은 drive_params.yaml 의 log_period_sec 주기로 로그. 매 프레임 로그하려면 지정")
    a = ap.parse_args()

    perc_p = load_section(a.params, "perception_node")
    drv_p = load_section(a.params, "driver_node")

    from ultralytics import YOLO

    print(f"모델 로드: {a.model}")
    # .engine 은 task 메타데이터가 안 남아 기본값(detect)으로 잘못 추측된다.
    # 이 프로젝트 모델은 항상 세그멘테이션(YOLO11n-seg)이므로 명시한다.
    model = YOLO(a.model, task="segment")

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        raise SystemExit(f"영상을 열 수 없음: {a.video}")

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    dt = a.every / fps
    print(f"영상 {width}x{height} @ {fps:.1f}fps, every={a.every} -> dt={dt:.3f}s/frame, "
          f"auto_start={a.auto_start}")
    if not a.auto_start:
        print("  (--wait-light: 신호등 GREEN 이 start_confirm_frames 만큼 연속돼야 출발합니다)")
    print("  (라이다 /corridor 없음 — 항상 카메라 차선만으로 판단)\n")

    (est, voter, fsm, fusion, lateral, longitudinal,
     steering, speed_limiter) = build_pipeline(
         perc_p, drv_p, width, height, a.auto_start, a.bias, a.lidar_confirm)

    conf = g(perc_p, "infer_conf", 0.20)
    dashed_conf = g(perc_p, "dashed_conf", 0.40)
    solid_conf = g(perc_p, "solid_conf", 0.25)
    cone_conf = g(perc_p, "cone_conf", 0.30)
    car_conf = g(perc_p, "car_conf", 0.40)
    lane_lost_stop = g(drv_p, "lane_lost_stop_sec", 2.0)
    log_period = g(drv_p, "log_period_sec", 1.0)

    if a.viz:
        os.makedirs(a.viz, exist_ok=True)
    writer = None
    if a.out_video:
        d = os.path.dirname(os.path.abspath(a.out_video))
        if d:
            os.makedirs(d, exist_ok=True)
        out_fps = fps / a.every
        writer = cv2.VideoWriter(a.out_video, cv2.VideoWriter_fourcc(*"mp4v"),
                                 out_fps, (width * 2, height + 78))

    obs = Obs()
    ref = FusedResult()
    lane_lost_since = None
    last_log = -1e9
    idx = done = 0
    state_counts = {}
    traj = None if a.no_traj else Trajectory()

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        if idx % a.every:
            continue
        t = idx / fps

        # ---- 1) YOLO 검출 ----
        result = model.predict(frame, conf=conf, verbose=False)[0]

        # ---- 2) 인지: YOLO 결과 -> 구조화된 관측값 -> 차선추정/신호등투표 ----
        det = extract(result, model.names, width, height,
                      dashed_conf=dashed_conf, solid_conf=solid_conf,
                      cone_conf=cone_conf, car_conf=car_conf)
        lane_res = est.update(det.dashed, det.solid)
        light_state = voter.update(det.lamp, det.lamp_conf, det.light_width)

        obs.offset_near, obs.offset_far = lane_res.offset_near, lane_res.offset_far
        obs.lane_valid, obs.quality = lane_res.valid, lane_res.quality
        obs.half_near, obs.half_far = lane_res.half_near, lane_res.half_far
        obs.light = light_state
        obs.cone_n, obs.cone_near_y = det.cone_n, det.cone_near_y
        obs.car_present, obs.car_cx, obs.car_bottom_y = det.car_present, det.car_cx, det.car_bottom_y
        # 라이다 복도 없음 -> 항상 invalid (카메라 차선만으로 판단)
        obs.cor_near = obs.cor_far = obs.cor_quality = 0.0
        obs.cor_valid = False
        obs.front_dist = obs.left_free = obs.right_free = 99.0

        # ---- 3) 판단: 상태기계 ----
        state = fsm.update(obs.light, obs.lane_valid)
        state_counts[state.value] = state_counts.get(state.value, 0) + 1

        reason = "wait"
        target_offset = 0.0
        angle = speed = 0.0

        if fsm.should_drive:
            ref = fusion.update(
                dt,
                LateralRef(obs.offset_near, obs.offset_far, obs.lane_valid, obs.quality),
                LateralRef(obs.cor_near, obs.cor_far, obs.cor_valid, obs.cor_quality),
                obs.cone_n,
            )
            if ref.valid:
                lane_lost_since = None
            elif lane_lost_since is None:
                lane_lost_since = t

            if lane_lost_since is not None and (t - lane_lost_since) > lane_lost_stop:
                reason = "ref lost"
            else:
                # ---- 4) planning: 횡방향/종방향 목표 ----
                target_offset = lateral.update(dt, obs, image_width=width)
                target_speed = longitudinal.update(
                    ref.valid, ref.offset_near, ref.offset_far, ref.quality,
                    obs.cone_n, obs.front_dist,
                    overtake_active=lateral.overtake.active,
                )
                speed = speed_limiter.update(dt, target_speed)

                # ---- 5) 제어: 조향각 계산 ----
                if ref.valid:
                    angle = steering.update(dt, ref.offset_near, ref.offset_far, target_offset, speed)
                else:
                    angle = steering.hold(dt)
                reason = longitudinal.last_reason
        else:
            ref = FusedResult()

        done += 1

        if traj is not None:
            traj.step(dt, angle, speed, t)

        # ---- 시각화 ----
        if writer is not None or (a.viz and done % 10 == 0):
            yolo_vis = result.plot()
            if yolo_vis.shape[:2] != (height, width):
                yolo_vis = cv2.resize(yolo_vis, (width, height))
            car = (det.car_cx, det.car_conf) if det.car_present else None
            perc_vis = draw_perception(frame.copy(), est, det.dashed, det.solid, lane_res,
                                       STATE_TO_NAME[light_state], det.cone_n, car)
            overtake_phase = lateral.overtake.phase.value if lateral.overtake.active else "-"
            perc_vis = draw_hud(perc_vis, state.value, ref, target_offset, angle, speed,
                                reason, overtake_phase)
            yolo_vis = cv2.copyMakeBorder(yolo_vis, 0, 78, 0, 0, cv2.BORDER_CONSTANT, value=(20, 20, 20))
            combo = np.hstack([yolo_vis, perc_vis])
            label_top(combo, width, "1) YOLO detect", "2) perception (lane/light/objects)")
            if writer is not None:
                writer.write(combo)
            if a.viz and done % 10 == 0:
                cv2.imwrite(os.path.join(a.viz, f"v_{done:04d}.jpg"), combo)

        # ---- 모터 발행 로그 (driver_node._log 와 같은 형식) ----
        if a.log_every_frame or (t - last_log) >= log_period:
            last_log = t
            ov = lateral.overtake.phase.value if lateral.overtake.active else "-"
            print(
                f"t={t:6.2f}s [{state.value:10s}] angle={angle:+6.1f} speed={speed:5.1f} | "
                f"off={ref.offset_near:+6.1f}/{ref.offset_far:+6.1f} q={ref.quality:.2f} "
                f"{'OK' if ref.valid else 'HOLD'} [{ref.source} w{ref.corridor_weight:.2f}] | "
                f"light={LIGHT_NAME.get(obs.light, '?')} cone={obs.cone_n} ov={ov} | {reason}"
            )

        if a.limit and done >= a.limit:
            break

    cap.release()
    if writer is not None:
        writer.release()
        print(f"\n시각화 영상 저장: {a.out_video}")

    print(f"\n처리 프레임: {done}")
    dist = ", ".join(f"{k} {v / max(done,1) * 100:.0f}%" for k, v in sorted(state_counts.items(), key=lambda kv: -kv[1]))
    print(f"판단 상태 분포: {dist}")

    if traj is not None:
        traj_out = a.traj_out
        if not traj_out:
            base = a.out_video or a.video
            traj_out = os.path.splitext(base)[0] + "_trajectory.png"
        d = os.path.dirname(os.path.abspath(traj_out))
        if d:
            os.makedirs(d, exist_ok=True)
        traj.save_plot(traj_out)
        span_x = max(traj.xs) - min(traj.xs)
        span_y = max(traj.ys) - min(traj.ys)
        print(f"경로 적분 그림 저장: {traj_out}  (전진 {span_x:.1f}m x 횡 {span_y:.1f}m 범위, "
              f"제어명령 개루프 적분 — 실제 카메라 경로 아님)")


if __name__ == "__main__":
    main()
