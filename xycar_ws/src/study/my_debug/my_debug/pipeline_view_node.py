#!/usr/bin/env python3
"""파이프라인 시각화 뷰어.

perception_node(publish_debug_image=true 일 때)가 내는 /debug_image 와
driver_node 가 항상 내는 /debug_state 를 합쳐 화면에 띄운다.

이 노드 자신은 아무것도 계산하지 않는다 — 각 노드가 이미 계산해 둔 결과를
받아 그리기만 한다(판단 로직 중복 금지). 그래서 인지/판단 노드가 죽어도
이 노드가 뭘 잘못 계산할 일은 없고, 그냥 화면이 멈출 뿐이다.

    ros2 run my_debug pipeline_view_node

/debug_image 는 perception_node 가 publish_debug_image:=true 일 때만 낸다
(CPU 추론 병목이라 기본은 off — 이 노드를 띄운다고 자동으로 켜지지 않는다.
 drive.launch.py debug:=true 로 띄우면 같이 켜준다).
"""

import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

_DIR_NAME = {1: "RIGHT", -1: "LEFT", 0: "-"}

_BAR_HEIGHT = 142
_BAR_BG = (20, 20, 20)
_TXT = (0, 255, 255)

# 회피(추월) 기동 색. 주황 = "지금 인지가 아니라 기동이 목표를 밀고 있다".
# 노랑(차선)·빨강(라바콘)과 겹치지 않는 색을 골랐다 — 세 상태가 한 화면에서
# 구분돼야 "왜 저쪽으로 가지"를 오해하지 않는다.
_OT_COLOR = (0, 140, 255)
_OT_DIM = (0, 90, 170)
# 방해차량이 있다고 판단한 쪽 반화면에 씌우는 틴트의 진하기.
_SIDE_TINT_ALPHA = 0.18


class PipelineViewNode(Node):

    def __init__(self):
        super().__init__("pipeline_view_node")

        self.declare_parameter("window_name", "xycar pipeline")
        self.window_name = self.get_parameter("window_name").value

        self.bridge = CvBridge()
        self._frame = None
        self._state = None

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, "debug_image", self.on_image, sensor_qos)
        self.create_subscription(String, "debug_state", self.on_state, 10)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # 30Hz 로 다시 그린다 — 이미지는 YOLO 속도(느릴 수 있음)로만 갱신되지만
        # 텍스트(판단/제어)는 driver_node 주기(기본 30Hz)로 계속 최신 상태를 반영한다.
        self.create_timer(1.0 / 30.0, self._redraw)
        self.get_logger().info(
            "pipeline_view_node 시작 — /debug_image, /debug_state 대기 중 "
            "(perception_node 를 publish_debug_image:=true 로 띄웠는지 확인할 것)"
        )

    def on_image(self, msg):
        try:
            self._frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:  # noqa: BLE001 - 뷰어가 죽으면 디버깅 자체가 안 된다
            self.get_logger().error(f"디버그 이미지 변환 실패: {exc}")

    def on_state(self, msg):
        try:
            self._state = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            pass

    def _draw_overtake(self, frame, s):
        """영상 위에 회피(추월) 기동 상태를 겹쳐 그린다.

        계산은 하지 않는다 — driver_node 가 이미 정한 값을 그리기만 한다
        (ot_car_side / ot_dir / overtake / car_cx). 이 노드가 좌우를 다시
        판단하면 실제 제어와 어긋난 그림이 나와 진단이 더 어려워진다.

        보여주는 것 두 가지:
          1) **지금 추월 기동 중인가** — 상단 배너 + 위상(SHIFT/PASS/RETURN/
             OVERSHOOT) + 테두리.
          2) **차가 어느 쪽에 있다고 판단했는가** — 그쪽 반화면 틴트 +
             판단 근거였던 x중심 세로선. 피하는 방향은 그 반대쪽 화살표로.
        """
        h, w = frame.shape[:2]
        phase = s.get("overtake", "-")
        active = phase != "-"

        # (a) 기동과 무관하게, 지금 보이는 차량 위치는 항상 얇게 표시한다.
        #     기동이 안 걸릴 때 "못 본 건지 / 봤는데 아직 먼 건지"를 가른다.
        if s.get("car_present"):
            cx = int(round(s.get("car_cx", 0.0)))
            by = int(round(s.get("car_bottom_y", 0.0)))
            ch = s.get("car_h", 0.0)
            if 0 <= cx < w:
                cv2.line(frame, (cx, 0), (cx, h), (120, 120, 120), 1, cv2.LINE_AA)
            if 0 <= by < h:
                cv2.line(frame, (cx - 25, by), (cx + 25, by), (120, 120, 120), 1,
                         cv2.LINE_AA)
            # 높이(h)가 트리거 기준이다 — 화면에서 바로 읽혀야 "왜 아직
            # 안 피하지"를 가를 수 있다.
            cv2.putText(frame, f"car cx{cx} h{ch:.0f}", (max(4, cx - 45), h - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (160, 160, 160), 1,
                        cv2.LINE_AA)

        if not active:
            return frame

        side = s.get("ot_car_side", 0)      # -1 왼쪽 / +1 오른쪽 (차가 있던 쪽)
        adir = s.get("ot_dir", 0)           # +1 오른쪽 / -1 왼쪽 (피하는 쪽)

        # (b) 차가 있다고 판단한 쪽 반화면을 틴트. 색으로 먼저 눈에 들어와야
        #     한 프레임만 스쳐도 방향을 놓치지 않는다.
        if side != 0:
            x0, x1 = (0, w // 2) if side < 0 else (w // 2, w)
            tint = frame.copy()
            cv2.rectangle(tint, (x0, 0), (x1, h), _OT_COLOR, -1)
            cv2.addWeighted(tint, _SIDE_TINT_ALPHA, frame,
                            1.0 - _SIDE_TINT_ALPHA, 0.0, frame)

        # (c) 판단 근거가 된 x중심(기동 시작 시점에 고정된 값) 세로선.
        cx0 = int(round(s.get("ot_car_cx", 0.0)))
        if 0 < cx0 < w:
            cv2.line(frame, (cx0, 0), (cx0, h), _OT_COLOR, 2, cv2.LINE_AA)

        # (d) 상단 배너 — 기동 중임을 놓칠 수 없게.
        cv2.rectangle(frame, (0, 0), (w, 34), _OT_DIM, -1)
        cv2.rectangle(frame, (0, 0), (w - 1, h - 1), _OT_COLOR, 3)
        cv2.putText(
            frame,
            f"OVERTAKING [{phase}]  car {_DIR_NAME.get(side, '?')}"
            f" -> avoid {_DIR_NAME.get(adir, '?')}",
            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2,
            cv2.LINE_AA)

        # (e) 피하는 방향 화살표 — 화면 중앙 높이에서 그쪽으로.
        if adir != 0:
            cy = h // 2
            mid = w // 2
            tip = mid + adir * min(140, w // 3)
            cv2.arrowedLine(frame, (mid, cy), (tip, cy), _OT_COLOR, 4,
                            cv2.LINE_AA, tipLength=0.3)
        return frame

    def _redraw(self):
        if self._frame is None:
            placeholder = np.zeros((_BAR_HEIGHT * 3, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, "waiting for /debug_image ...", (20, _BAR_HEIGHT),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(placeholder,
                        "perception_node 를 publish_debug_image:=true 로 띄웠는지 확인",
                        (20, _BAR_HEIGHT + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (200, 200, 200), 1, cv2.LINE_AA)
            cv2.imshow(self.window_name, placeholder)
            cv2.waitKey(1)
            return

        width = self._frame.shape[1]
        bar = np.full((_BAR_HEIGHT, width, 3), _BAR_BG, dtype=np.uint8)
        s = self._state
        # 원본을 건드리지 않는다 — 다음 프레임이 안 와도 오버레이가 누적되면
        # 화면이 점점 물든다(같은 배열에 계속 덧그리게 된다).
        frame = self._frame.copy()
        if s is not None:
            frame = self._draw_overtake(frame, s)
        if s is None:
            cv2.putText(bar, "waiting for /debug_state ...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(
                bar, f"3) decision(fsm)  state={s['state']}",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1, cv2.LINE_AA)
            cv2.putText(
                bar,
                f"4) planning  off={s['off_near']:+.0f}/{s['off_far']:+.0f}px "
                f"-> target={s.get('target_off', 0):+.0f}px "
                f"{'CONE(lidar)' if s.get('cone_zone') else 'LANE(cam)'} "
                f"{'OK' if s['valid'] else 'HOLD'}"
                + (f"  | avoid {_DIR_NAME.get(s.get('ot_dir', 0), '?')} "
                   f"{s.get('ot_amount', 0):.0f}px"
                   if s.get('overtake', '-') != '-' else ""),
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(
                bar,
                f"5) control  angle={s['angle']:+.1f}  speed={s['speed']:.1f}  "
                f"light={s['light']} "
                # 라이다(rubbercone_node)가 몰고 있는 구간인지 한눈에 보이게 한다.
                f"cone={s['cone_n']}{'[ZONE]' if s.get('cone_zone') else ''}"
                f"{'' if s.get('use_lidar', True) else ' [LIDAR OFF]'} "
                f"| {s['reason']}",
                (10, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.55, _TXT, 1, cv2.LINE_AA)
            # 회피 전용 줄. 기동 중이면 위상·좌우 판단·근거를, 아니면 왜
            # 안 하는지(차단 남은 시간 / 차량 관측)를 같은 자리에 쓴다.
            # 자리를 고정해야 눈이 매번 같은 곳을 본다.
            if s.get('overtake', '-') != '-':
                ot_line = (
                    f"6) OVERTAKE [{s['overtake']}]  "
                    f"car={_DIR_NAME.get(s.get('ot_car_side', 0), '?')}"
                    f"(cx{s.get('ot_car_cx', 0):.0f}) -> "
                    f"avoid={_DIR_NAME.get(s.get('ot_dir', 0), '?')} "
                    f"{s.get('ot_amount', 0):.0f}px  {s.get('ot_reason', '')}")
                ot_color = _OT_COLOR
            else:
                blocked = s.get('ot_block', 0)
                ot_line = (
                    "6) OVERTAKE  idle  "
                    + (f"BLOCKED {blocked:.1f}s  " if blocked > 0 else "")
                    + (f"car seen cx{s.get('car_cx', 0):.0f} "
                       f"h{s.get('car_h', 0):.0f}"
                       if s.get('car_present') else "no car"))
                ot_color = (140, 140, 140)
            cv2.putText(bar, ot_line, (10, 124), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, ot_color, 1, cv2.LINE_AA)

        combined = np.vstack([frame, bar])
        cv2.imshow(self.window_name, combined)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
