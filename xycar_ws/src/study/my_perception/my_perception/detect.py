"""YOLO 결과 -> 구조화된 관측값.

ROS 의존성 없음. perception_node(온라인)와 tools/offline_check(오프라인)가
같은 코드를 쓰도록 여기로 분리했다. 둘이 갈라지면 오프라인 검증의 의미가 없어진다.

거리 추정에 대하여:
    YOLO는 거리를 모른다. 하지만 카메라가 고정 장착이고 라바콘/차량이 지면에
    놓여 있으므로, **bbox 하단 y좌표가 거리의 단조 함수**가 된다 (아래쪽일수록 가깝다).
    호모그래피 캘리브레이션 없이 "가까워지는 중"을 판단할 수 있어서 이 방식을 쓴다.
    절대 거리가 필요하면 라이다(my_obstacle)를 볼 것.
"""

from dataclasses import dataclass, field

import numpy as np

LAMP_NAMES = ("RED", "YELLOW", "GREEN", "LEFT")
CAR_NAMES = ("AvanteN", "ionic5")


@dataclass
class Detections:
    """한 프레임의 관측 결과."""

    # 차선: 마스크가 켜진 픽셀 좌표 (xs, ys) 인스턴스 리스트
    dashed: list = field(default_factory=list)
    solid: list = field(default_factory=list)

    # 신호등
    lamp: str = None          # 이번 프레임에 보인 램프 클래스명 (없으면 None)
    lamp_conf: float = 0.0
    light_width: float = 0.0  # traffic_light 박스 폭(px). 0이면 신호등이 화면에 없음

    # 라바콘
    cone_n: int = 0
    cone_near_y: float = 0.0  # 가장 가까운 콘의 bbox 하단 y (클수록 가깝다)
    cone_max_h: float = 0.0
    # ↑ 가장 큰 콘의 bbox 높이(px). **구간 진입 트리거의 거리 판단**에 쓴다.
    #   왜 하단 y(cone_near_y)가 아니라 높이인가: 하단 y 는 카메라 피치와 노면
    #   기울기에 흔들리고, 콘이 화면 아래로 잘리면 오히려 작아진다. bbox 높이는
    #   거의 순수하게 거리의 함수라 "얼마나 가까운가"를 더 곧게 나타낸다.
    #   멀리서 콘 무리가 보이자마자 구간으로 전환되면 아직 차선 구간인데
    #   라이다 주행으로 넘어가므로, 개수와 **함께** 이 값으로 문턱을 만든다.

    # 방해차량
    car_present: bool = False
    car_cx: float = 0.0        # 화면상 x중심 (좌/우 판단용)
    car_bottom_y: float = 0.0  # bbox 하단 y (진단용. 트리거에는 안 쓴다)
    car_h: float = 0.0
    # ↑ bbox 높이(px). **회피 트리거의 거리 판단**에 쓴다.
    #   cone_max_h 와 같은 이유다(위 참고): 하단 y 는 카메라 피치·노면
    #   기울기에 흔들리고 차가 화면 아래로 잘리면 오히려 작아진다.
    #   높이는 거의 순수하게 거리의 함수다.
    car_conf: float = 0.0
    car_cls: int = 0
    # ↑ 방해차량 모델 번호. CAR_NAMES 의 인덱스 + 1 이다
    #   (0 = 없음, 1 = AvanteN, 2 = ionic5).
    #   [2026-08-23] 모델마다 차체 크기가 달라 같은 거리에서도
    #   bbox 높이가 다르다. 회피 트리거 문턱을 모델별로 나누려면
    #   어느 모델인지를 판단 쪽으로 알려줘야 한다.

    # ── 모델별 차량 슬롯 (2026-08-24 팀원 구현 이식) ──────────────────
    # 위의 car_* 는 "가장 conf 높은 차 하나"만 담는다. 팀원 회피는
    # **모델별로** 문턱과 회피량이 다르고, 둘이 동시에 보일 때
    # height_ratio 가 큰 쪽을 고르므로 슬롯을 나눠 담아야 한다.
    #
    # vehicles[cls_index] = {
    #     "cls": 1|2, "cx": float, "h": float, "h_ratio": float,
    #     "conf": float, "lane": 0|1|2,
    # }
    #   lane 1 = 차가 dashed **왼쪽**  -> 오른쪽으로 피한다
    #   lane 2 = 차가 dashed **오른쪽** -> 왼쪽으로 피한다
    #   lane 0 = 그 프레임에 dashed 를 못 봤다 -> 화면중앙 폴백
    vehicles: dict = field(default_factory=dict)


def merged_instance_x_at_y(instances, target_y):
    """여러 segmentation 조각을 합친 2차 곡선의 target_y 에서의 x.

    [2026-08-24 이식] 팀원 race_perception/detect.py 그대로.
    회피 방향을 정할 때 **차량 bbox 하단**에서 중앙 dashed 가 어디인지
    보려고 쓴다 — 화면 중앙이 아니라 실제 차선 기준으로 좌/우를 가른다.

    lane.py 의 _fit 과 달리 y_lo/y_hi 로 자르지 않는다. 차량 bbox 하단은
    평가행 범위 밖(화면 아래쪽)일 수 있는데, 거기서 값을 읽어야 하기
    때문이다. 대신 실제 관측된 y 범위로 clip 해서 외삽을 막는다.

    반환: (x, 실제 평가한 y) 또는 (None, None).
    """
    if not instances:
        return None, None
    xs = np.concatenate([np.asarray(item[0], dtype=float) for item in instances])
    ys = np.concatenate([np.asarray(item[1], dtype=float) for item in instances])
    finite = np.isfinite(xs) & np.isfinite(ys)
    xs, ys = xs[finite], ys[finite]
    if xs.size < 3 or float(ys.max() - ys.min()) < 2.0:
        return None, None
    eval_y = float(np.clip(target_y, ys.min(), ys.max()))
    return float(np.polyval(np.polyfit(ys, xs, 2), eval_y)), eval_y


def extract(result, names, width, height, dashed_conf, solid_conf,
            cone_conf=0.30, car_conf=0.40, resize_fn=None):
    """ultralytics Result -> Detections.

    result   : ultralytics 결과 1개 (masks/boxes 포함)
    names    : model.names (인덱스 -> 클래스명)
    resize_fn: 마스크를 (width, height)로 맞추는 함수. 기본은 cv2.resize.
               cv2 import를 이 모듈에 강제하지 않으려고 주입 가능하게 뒀다.
    """
    if resize_fn is None:
        import cv2

        def resize_fn(mask):
            return cv2.resize(mask, (width, height)) > 0.5

    det = Detections()

    # --- 마스크가 필요한 클래스 (차선) ---
    if result.masks is not None:
        for mk, box in zip(result.masks.data.cpu().numpy(), result.boxes):
            cls = names[int(box.cls)]
            conf = float(box.conf)
            if cls == "dashed_line":
                if conf < dashed_conf:
                    continue
                target = det.dashed
            elif cls == "solid_line":
                if conf < solid_conf:
                    continue
                target = det.solid
            else:
                continue
            m = resize_fn(mk)
            ys, xs = np.nonzero(m)
            if xs.size:
                target.append((xs, ys))

    # --- 박스만 쓰는 클래스 (신호등/라바콘/차량) ---
    for box in result.boxes:
        cls = names[int(box.cls)]
        conf = float(box.conf)
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]

        if cls == "traffic_light":
            det.light_width = max(det.light_width, x2 - x1)
        elif cls in LAMP_NAMES:
            if conf > det.lamp_conf:
                det.lamp, det.lamp_conf = cls, conf
        elif cls == "traffic_cone":
            if conf >= cone_conf:
                det.cone_n += 1
                det.cone_near_y = max(det.cone_near_y, y2)
                det.cone_max_h = max(det.cone_max_h, y2 - y1)
        elif cls in CAR_NAMES:
            # 방해차량은 **항상 한 대**다(사용자 확인 2026-08-22). 여러 박스가
            # 나오면 conf 가 가장 높은 것 하나만 남긴다 — 원래 규칙 그대로다.
            #
            # ※ 한때 "높이가 가장 큰 것"으로 바꿨다가 되돌렸다. 근거로 삼았던
            #   두 박스가 같은 프레임의 중복 검출이 아니라 **서로 다른 시점의
            #   사진**이었다. 한 대뿐이면 두 규칙은 같은 것을 고르므로, 검증된
            #   쪽을 유지한다.
            if conf >= car_conf and conf > det.car_conf:
                det.car_present = True
                det.car_cx = (x1 + x2) / 2.0
                det.car_bottom_y = y2
                det.car_h = y2 - y1      # 회피 트리거의 거리 판단에 쓴다
                det.car_conf = conf
                det.car_cls = CAR_NAMES.index(cls) + 1

            # ── 모델별 슬롯 (2026-08-24 팀원 구현 이식) ──
            # car_conf 게이트를 **여기서는 안 건다.** 팀원의 진입 판정은
            # staged_vehicle_entry(신뢰도 2단 + height_ratio)로 판단 쪽에서
            # 하므로, 인지는 원본 conf 를 그대로 넘겨야 한다.
            cls_idx = CAR_NAMES.index(cls) + 1
            prev = det.vehicles.get(cls_idx)
            if prev is None or conf > prev["conf"]:
                det.vehicles[cls_idx] = {
                    "cls": cls_idx,
                    "cx": (x1 + x2) / 2.0,
                    "h": y2 - y1,
                    "h_ratio": (y2 - y1) / max(1.0, float(height)),
                    "conf": conf,
                    "bottom_y": y2,
                    "lane": 0,          # dashed 를 본 뒤 아래에서 채운다
                }

    # ── 회피 방향의 기준: 차량 bbox **하단**에서의 중앙 dashed 위치 ──
    # [2026-08-24 팀원 구현 이식] 화면 중앙이 아니라 실제 차선으로 좌/우를
    # 가른다. 차가 트랙 어디에 있든 "저 차는 1차선인가 2차선인가"가
    # 정확해진다 — 화면 중앙 기준은 우리 차가 이미 옆으로 치우쳐 있으면
    # 엉뚱한 쪽을 고른다.
    #   lane 1 = 차가 dashed 왼쪽  -> 오른쪽으로 피한다
    #   lane 2 = 차가 dashed 오른쪽 -> 왼쪽으로 피한다
    # dashed 를 못 본 프레임은 lane 0 으로 두고 판단 쪽이 화면중앙으로 폴백.
    if det.vehicles and det.dashed:
        for v in det.vehicles.values():
            dashed_x, _ = merged_instance_x_at_y(det.dashed, v["bottom_y"])
            if dashed_x is not None:
                v["lane"] = 1 if v["cx"] < dashed_x else 2

    return det
