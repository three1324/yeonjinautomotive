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
    car_bottom_y: float = 0.0  # bbox 하단 y (접근도)
    car_conf: float = 0.0


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
            if conf >= car_conf and conf > det.car_conf:
                det.car_present = True
                det.car_cx = (x1 + x2) / 2.0
                det.car_bottom_y = y2
                det.car_conf = conf

    return det
