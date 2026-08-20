#!/usr/bin/env python3
"""영상 파일을 카메라인 척 실시간으로 흘려보내는 노드.

    ros2 run my_debug video_pub_node --ros-args -p video_path:=/home/user/test.mp4

카메라 드라이버(`xycar_cam`) 대신 이 노드를 띄우면 **파이프라인 전체가 그대로**
돈다 — perception_node 는 /image_raw 를 구독할 뿐이라 그것이 카메라인지 파일인지
알지 못한다. 그래서 인지→판단→제어→시각화까지 실차와 같은 경로로 검증할 수 있다.

왜 오프라인 도구(my_perception/tools/offline_check.py)로 부족한가:
    offline_check.py 는 인지 모듈만 최대 속도로 돌린다(ROS 없음). 반면 이 노드는
    **영상의 원래 fps 로 실시간 재생**하므로, 젯슨이 그 프레임률을 실제로
    따라가는지, 못 따라갈 때 driver_node 가 stale 판정으로 멈추는지까지 드러난다.
    그건 인지 정확도와는 다른 문제고, 실차에서 실제로 문제가 됐던 쪽이다.

파라미터:
    video_path   재생할 영상 파일 (필수)
    topic        발행 토픽                         (기본 /image_raw)
    frame_id     header.frame_id                   (기본 camera)
    fps          0.0 이면 영상 파일의 fps 를 그대로 쓴다 (기본 0.0)
    rate_scale   재생 배속. 1.0=실시간, 0.5=절반 속도 (기본 1.0)
    loop         끝나면 처음부터 다시              (기본 true)
    start_frame  이 프레임부터 시작                (기본 0)
    width/height 발행 전 리사이즈. 0 이면 원본 그대로 (기본 640x480)

⚠️ width/height 기본값이 640x480 인 이유:
    driver_node 의 `image_width`(기본 640)가 화면 중심을 정하는 값이라, 영상 해상도가
    그와 다르면 조향이 계통적으로 한쪽으로 치우친다. 테스트 영상은 632px 인 경우가
    있어(README §6) 기본적으로 실차 해상도로 맞춰 내보낸다. 원본 그대로 보내고
    싶으면 width:=0 height:=0 으로 두고 driver_node 의 image_width 를 그 값에 맞출 것.

⚠️ 이 노드는 영상을 낼 뿐 차를 멈추지 않는다. **모터 스택이 떠 있는 상태에서
   /drive_enable 을 켜면 차는 영상만 보고 실제로 달린다.** 벤치 테스트라면 모터
   스택을 띄우지 말 것 (my_bringup/launch/replay.launch.py 는 아예 띄우지 않는다).
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class VideoPubNode(Node):

    def __init__(self):
        super().__init__("video_pub_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("video_path", ""),
                ("topic", "/image_raw"),
                ("frame_id", "camera"),
                ("fps", 0.0),
                ("rate_scale", 1.0),
                ("loop", True),
                ("start_frame", 0),
                ("width", 640),
                ("height", 480),
                ("log_period_sec", 2.0),
            ],
        )
        g = self.get_parameter

        self.path = g("video_path").value
        if not self.path:
            raise RuntimeError("video_path 파라미터가 비어 있다. 재생할 영상 경로를 지정할 것.")

        self.cap = cv2.VideoCapture(self.path)
        if not self.cap.isOpened():
            raise RuntimeError(f"영상을 열 수 없다: {self.path}")

        self.loop = g("loop").value
        self.width = int(g("width").value)
        self.height = int(g("height").value)
        self.frame_id = g("frame_id").value

        src_fps = self.cap.get(cv2.CAP_PROP_FPS)
        fps = float(g("fps").value) or (src_fps if src_fps and src_fps > 0.0 else 30.0)
        scale = float(g("rate_scale").value) or 1.0
        self.fps = fps * scale

        start = int(g("start_frame").value)
        if start > 0:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, start)

        src_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        src_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        total = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        self.bridge = CvBridge()
        # perception_node 의 구독 QoS 와 같아야 한다 (BEST_EFFORT / depth 1).
        # depth 1 이라 소비자가 느리면 프레임이 그냥 버려진다 — 실제 카메라와 같은 거동이다.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Image, g("topic").value, qos)

        self._sent = 0
        self._log_period = g("log_period_sec").value
        self._last_log = self.get_clock().now()
        self.create_timer(1.0 / self.fps, self._tick)

        out = (f"{self.width}x{self.height}" if self.width and self.height else "원본 그대로")
        self.get_logger().info(
            f"video_pub_node 시작 — {self.path}\n"
            f"  원본 {src_w}x{src_h} {src_fps:.2f}fps {total}프레임 "
            f"-> 발행 {out} {self.fps:.2f}fps loop={self.loop} "
            f"topic={g('topic').value}")

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            if not self.loop:
                self.get_logger().info("영상 끝. 노드를 종료한다.")
                raise SystemExit
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().error("영상을 되감을 수 없다. 노드를 종료한다.")
                raise SystemExit

        if self.width and self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

        self._sent += 1
        now = self.get_clock().now()
        elapsed = (now - self._last_log).nanoseconds / 1e9
        if elapsed >= self._log_period:
            pos = int(self.cap.get(cv2.CAP_PROP_POS_FRAMES))
            self.get_logger().info(f"{self._sent / elapsed:4.1f}fps 발행 중 (frame {pos})")
            self._sent = 0
            self._last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = VideoPubNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
