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
 drive_amd.launch.py debug:=true 로 띄우면 같이 켜준다).
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

_BAR_HEIGHT = 110
_BAR_BG = (20, 20, 20)
_TXT = (0, 255, 255)


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
                f"ref[{s['source']}] w={s['corridor_weight']:.2f} "
                f"{'OK' if s['valid'] else 'HOLD'}"
                + (f"  | avoid {_DIR_NAME.get(s.get('ot_dir', 0), '?')} "
                   f"{s.get('ot_amount', 0):.0f}px {s.get('ot_reason', '')}"
                   if s.get('overtake', '-') != '-' else ""),
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(
                bar,
                f"5) control  angle={s['angle']:+.1f}  speed={s['speed']:.1f}  "
                f"light={s['light']} front={s['front_dist']:.2f}m cone={s['cone_n']} "
                f"| {s['reason']}",
                (10, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.55, _TXT, 1, cv2.LINE_AA)

        combined = np.vstack([self._frame, bar])
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
