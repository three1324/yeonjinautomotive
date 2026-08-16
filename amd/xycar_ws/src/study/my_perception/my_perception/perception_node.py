#!/usr/bin/env python3
"""카메라 인식 노드.

/image_raw 를 받아 YOLO11n-seg 를 **1회만** 추론하고, 그 결과를 세 토픽으로 나눠 발행한다.
인식을 미션별 노드로 쪼개지 않는 이유는 같은 이미지에 YOLO를 여러 번 돌릴 수 없기
때문이다. (자세한 근거는 프로젝트 README 4장)

발행:
    /lane    Float32MultiArray [offset_near, offset_far, valid, quality]
    /light   Int32             0=NONE 1=RED 2=YELLOW 3=GREEN 4=LEFT (투표 확정값)
    /objects Float32MultiArray [cone_n, cone_near_y, car_present, car_cx, car_bottom_y]

핵심 계산(lane/light_vote/detect)은 ROS 의존성이 없는 모듈로 분리돼 있고,
tools/offline_check.py 가 같은 모듈을 써서 영상으로 검증한다.
"""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32

from my_perception.detect import extract
from my_perception.lane import LaneEstimator
from my_perception.light_vote import STATE_TO_NAME, LightVoter


class PerceptionNode(Node):

    def __init__(self):
        super().__init__("perception_node")

        p = self.declare_parameters(
            namespace="",
            parameters=[
                ("model_path", ""),
                ("image_topic", "/image_raw"),
                ("infer_conf", 0.20),      # YOLO 자체 임계값. 클래스별 임계값은 아래에서 따로
                ("dashed_conf", 0.40),
                ("solid_conf", 0.25),      # 어두운 구간에서 0.33까지 떨어져 낮게 잡는다
                ("cone_conf", 0.30),
                ("car_conf", 0.40),
                # 차선 추정
                ("lane.y_lo", 270),        # 지평선 아래
                ("lane.y_hi", 425),        # 차체에 가려지는 하단 위
                ("lane.eval_near", 400),
                ("lane.eval_far", 310),
                ("lane.center_bias_px", 0.0),
                ("lane.min_pts", 50),
                ("lane.min_span", 20),
                ("lane.hold_frames", 15),
                ("lane.half_alpha", 0.05),
                # 신호등 투표
                ("light.window", 30),
                ("light.min_weight", 3.0),
                ("light.min_ratio", 0.5),
                ("light.miss_tolerance", 10),
                # 기타
                ("log_period_sec", 2.0),
                ("publish_debug_image", False),
            ],
        )
        del p

        self.model_path = self.get_parameter("model_path").value
        if not self.model_path:
            raise RuntimeError("model_path 파라미터가 비어 있다. best5.pt 경로를 지정할 것.")

        self.infer_conf = self.get_parameter("infer_conf").value
        self.dashed_conf = self.get_parameter("dashed_conf").value
        self.solid_conf = self.get_parameter("solid_conf").value
        self.cone_conf = self.get_parameter("cone_conf").value
        self.car_conf = self.get_parameter("car_conf").value

        self.bridge = CvBridge()
        self.estimator = None   # 첫 프레임에서 이미지 크기를 알고 나서 생성
        self.voter = LightVoter(
            window=self.get_parameter("light.window").value,
            min_weight=self.get_parameter("light.min_weight").value,
            min_ratio=self.get_parameter("light.min_ratio").value,
            miss_tolerance=self.get_parameter("light.miss_tolerance").value,
        )

        # 모델 로드는 무겁다. 생성자에서 한 번만.
        from ultralytics import YOLO

        self.get_logger().info(f"YOLO 모델 로드: {self.model_path}")
        self.model = YOLO(self.model_path)
        self.names = self.model.names
        self.get_logger().info(f"클래스: {self.names}")

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_lane = self.create_publisher(Float32MultiArray, "lane", reliable_qos)
        self.pub_light = self.create_publisher(Int32, "light", reliable_qos)
        self.pub_objects = self.create_publisher(Float32MultiArray, "objects", reliable_qos)

        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.on_image, sensor_qos
        )

        self._frames = 0
        self._log_period = self.get_parameter("log_period_sec").value
        self._last_log = self.get_clock().now()
        self.get_logger().info("perception_node 시작")

    def _make_estimator(self, width, height):
        g = self.get_parameter
        return LaneEstimator(
            width=width,
            height=height,
            y_lo=g("lane.y_lo").value,
            y_hi=g("lane.y_hi").value,
            eval_near=g("lane.eval_near").value,
            eval_far=g("lane.eval_far").value,
            center_bias_px=g("lane.center_bias_px").value,
            min_pts=g("lane.min_pts").value,
            min_span=g("lane.min_span").value,
            hold_frames=g("lane.hold_frames").value,
            half_alpha=g("lane.half_alpha").value,
        )

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:  # noqa: BLE001 - 콜백이 죽으면 노드 전체가 멈춘다
            self.get_logger().error(f"이미지 변환 실패: {exc}")
            return

        height, width = frame.shape[:2]
        if self.estimator is None:
            self.estimator = self._make_estimator(width, height)
            self.get_logger().info(f"입력 해상도 {width}x{height}")

        result = self.model.predict(frame, conf=self.infer_conf, verbose=False)[0]
        det = extract(
            result, self.names, width, height,
            dashed_conf=self.dashed_conf,
            solid_conf=self.solid_conf,
            cone_conf=self.cone_conf,
            car_conf=self.car_conf,
        )

        lane = self.estimator.update(det.dashed, det.solid)
        state = self.voter.update(det.lamp, det.lamp_conf, det.light_width)

        self.pub_lane.publish(Float32MultiArray(data=[
            float(lane.offset_near),
            float(lane.offset_far),
            1.0 if lane.valid else 0.0,
            float(lane.quality),
        ]))
        self.pub_light.publish(Int32(data=int(state)))
        self.pub_objects.publish(Float32MultiArray(data=[
            float(det.cone_n),
            float(det.cone_near_y),
            1.0 if det.car_present else 0.0,
            float(det.car_cx),
            float(det.car_bottom_y),
        ]))

        self._frames += 1
        now = self.get_clock().now()
        elapsed = (now - self._last_log).nanoseconds / 1e9
        if elapsed >= self._log_period:
            fps = self._frames / elapsed
            self.get_logger().info(
                f"{fps:4.1f}fps  offset={lane.offset_near:+6.1f} q={lane.quality:.2f} "
                f"{'OK' if lane.valid else 'HOLD'}  light={STATE_TO_NAME[state]}  "
                f"cone={det.cone_n} car={int(det.car_present)}"
            )
            self._frames = 0
            self._last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
