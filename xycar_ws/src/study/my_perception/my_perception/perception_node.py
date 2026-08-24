#!/usr/bin/env python3
"""카메라 인식 노드.

/image_raw 를 받아 YOLO11n-seg 를 **1회만** 추론하고, 그 결과를 세 토픽으로 나눠 발행한다.
인식을 미션별 노드로 쪼개지 않는 이유는 같은 이미지에 YOLO를 여러 번 돌릴 수 없기
때문이다. (자세한 근거는 프로젝트 README 4장)

발행:
    /lane    Float32MultiArray [offset_near, offset_far, valid, quality,
                               half_near, half_far,
                               offset_near_left, offset_far_left, valid_left,
                               curve_px]
             curve_px 는 **선행 곱률**(2차피팅 a 계수 기반). 횟편차·헤딩에
             불변이고 bend 보다 훨씬 멀리 본다. 곱선 진입 전 감속용.
             *_left 는 **가장 왼쪽 노란픽셀 밴드**(lane.left_band_px)만으로
             만든 추정이다. 좌회전 분기에서 직진 가지와 좌회전 가지가 같이
             보일 때 좌회전 쪽을 고른다. SHORTCUT 상태에서만 쓰인다.
             half_* 는 학습된 트랙 반폭(px). my_driver 가 회피 목표
             (트랙중앙 ± 반폭/2 = 트랙 반쪽의 중앙)를 만드는 데 쓴다. 0이면 미학습.
    /light   Int32             0=NONE 1=RED 2=YELLOW 3=GREEN 4=LEFT (투표 확정값)
    /objects Float32MultiArray [cone_n, cone_near_y, car_present, car_cx, car_bottom_y,
                                cone_max_h, car_h, car_cls, light_width,
                                (모델별 슬롯 x2) cls, cx, h_ratio, conf, lane]
             마지막 10개는 AvanteN / ionic5 슬롯이다(각 5개).
             회피는 모델별로 문턱·회피량이 다르고, 둘이 동시에
             보일 때 height_ratio 가 큰 쪽을 고르므로 슬롯을 나눈다.
             lane: 1=차가 dashed 왼쪽(오른쪽으로 피함), 2=오른쪽, 0=dashed 못 봄.
             car_cls 는 방해차량 모델 번호(0=없음, 1=AvanteN, 2=ionic5).
             모델마다 차체가 달라 같은 거리에서도 bbox 높이가 다르므로,
             회피 트리거 문턱을 모델별로 나누려고 판단 쪽으로 넘긴다.

구독:
    /left_exit Int32   1이면 **좌회전 합류(EXIT) 모드**. 이때만 차선 추정을
             horizontal_dashed_left_strip 으로 거른 노란 점선만으로 한다
             (흰 실선은 아예 버린다). 합류 지점에서 앞을 가로지르는 본선
             노란선과 본선 흰 실선이 같이 잡히면 중앙 추정이 무너지기
             때문이다. 판단(언제 EXIT 인가)은 my_driver 의 몫이므로
             여기서는 스위치만 받는다.
    /debug_image Image         publish_debug_image=true 일 때만. YOLO 박스 + 차선 시각화
                 (my_debug/pipeline_view_node 전용, 기본 off — CPU 추론 병목 때문)

핵심 계산(lane/light_vote/detect)은 ROS 의존성이 없는 모듈로 분리돼 있고,
tools/offline_check.py 가 같은 모듈을 써서 영상으로 검증한다.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32

from my_perception.detect import extract
from my_perception.lane import LaneEstimator, horizontal_dashed_left_strip
from my_perception.light_vote import STATE_TO_NAME, LightVoter

# 디버그 패널 박스 색상 (BGR). YOLO 결과 시각화용이라 여기서만 쓴다.
_DEBUG_BOX_COLOR = {
    "dashed_line": (200, 0, 200),
    "solid_line": (0, 200, 200),
    "traffic_cone": (0, 180, 0),
}
_DEBUG_BOX_DEFAULT = (0, 140, 255)


def _vehicle_slots(det):
    """모델별 차량 슬롯을 평탄한 float 리스트로. 순서는 항상 cls 1,2.

    순서를 고정하는 이유: 수신쪽이 인덱스로 읽는다. 없는 모델은
    cls=0 으로 비워 두어 길이가 항상 같게 유지된다.
    """
    out = []
    for cls_idx in (1, 2):
        v = det.vehicles.get(cls_idx)
        if v is None:
            out += [0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            out += [float(v["cls"]), float(v["cx"]), float(v["h_ratio"]),
                    float(v["conf"]), float(v["lane"])]
    return out


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
                ("lane.left_band_px", 30.0),
                ("lane.left_exit_strip_px", 30.0),
                # 선행 곱률을 평가할 행. 낮을수록 멀리 본다.
                # 선행거리를 **미터**로 지정한다. 행은 아래 카메라 상수로
                # 환산한다 — 카메라를 재장착하면 horizon_row/cam_fh 만 고치면 된다.
                ("lane.curve_preview_m", 1.7),
                # 0 이면 위 미터값으로 자동 계산. >0 이면 그 행을 그대로 쓴다.
                ("lane.curve_preview_row", 0),
                # 핀홀 근사: X = cam_fh / (row - horizon_row)
                # ⚠️ 둘 다 **추정치**다. 실측하려면 직선에 정지해
                #    debug_state 의 half_near/half_far 를 읽고(트랙 반폭=0.40m):
                #      p_near = half_near/0.40,  p_far = half_far/0.40   [px/m]
                #      horizon_row = (380*p_far - 310*p_near)/(p_far - p_near)
                #      cam_fh      = (380 - horizon_row) / p_near * 0.40 ... 의 역수관계
                #    또는 더 간단히: 앞 1.7m 지점에 물체를 놓고 그게 몇 행에
                #    찍히는지 본 다음 curve_preview_row 에 직접 넣으면 된다.
                ("lane.horizon_row", 250.0),
                ("lane.cam_fh", 75.0),
                ("lane.min_pts", 50),
                ("lane.min_span", 20),
                ("lane.hold_frames", 15),
                ("lane.half_alpha", 0.05),
                # 반폭 고정 (2026-08-24). lock_frames 만큼 모아 중앙값으로
                # 잠그고, 그 뒤로는 절대 갱신하지 않는다. *_px > 0 이면
                # 학습 없이 그 값으로 완전 고정.
                ("lane.half_lock_frames", 30),
                ("lane.half_near_px", 0.0),
                ("lane.half_far_px", 0.0),
                ("lane.max_center_offset_px", 480.0),
                ("lane.max_jump_px", 250.0),
                # 신호등 투표
                ("light.window", 30),
                ("light.min_weight", 3.0),
                ("light.min_ratio", 0.5),
                ("light.miss_tolerance", 10),
                ("light.min_weight_left", 1.5),
                ("light.min_ratio_left", 0.45),
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
            min_weight_left=self.get_parameter("light.min_weight_left").value,
            min_ratio_left=self.get_parameter("light.min_ratio_left").value,
        )

        # 모델 로드는 무겁다. 생성자에서 한 번만.
        from ultralytics import YOLO

        self.get_logger().info(f"YOLO 모델 로드: {self.model_path}")
        # task="segment" 를 **명시해야 한다.** .pt 는 파일 안에 태스크가 들어 있어
        # 생략해도 되지만, TensorRT 엔진(.engine)은 생략하면 ultralytics 가
        # detect 로 로드해 출력(1,46,8400)의 32개 마스크 계수를 클래스 점수로
        # 오독한다 — 그러면 "클래스 24가 300개" 같은 쓰레기가 나오고 콘·차선이
        # 하나도 안 잡힌다(2026-08-19 실제로 겪음). 차선은 마스크가 필요하므로
        # 이 인자가 빠지면 주행 자체가 불가능하다.
        self.model = YOLO(self.model_path, task="segment")
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

        # publish_debug_image=true 일 때만 만든다 — YOLO 는 이미 이 노드의 병목이라
        # 기본값에서는 그리기 비용을 아예 안 낸다.
        self.publish_debug_image = self.get_parameter("publish_debug_image").value
        self.pub_debug_image = None
        if self.publish_debug_image:
            self.pub_debug_image = self.create_publisher(Image, "debug_image", sensor_qos)

        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.on_image, sensor_qos
        )

        # 좌회전 합류(EXIT) 스위치. driver_node 의 TimedLeftDrive 가 EXIT 위상에
        # 들어가면 1을 보낸다. 판단은 my_driver 가 하고 여기는 따르기만 한다.
        self._left_exit = False
        self.create_subscription(Int32, "left_exit", self.on_left_exit, reliable_qos)

        self._frames = 0
        self._log_period = self.get_parameter("log_period_sec").value
        self._last_log = self.get_clock().now()
        self.get_logger().info("perception_node 시작")

    def _publish_debug_image(self, msg, frame, result, lane, state, det, width, height):
        """디버그용 2분할 시각화: 1) YOLO 원시 검출  2) 차선 추정 결과.

        driver_node 의 판단/제어 텍스트는 별개 노드(my_debug/pipeline_view_node)가
        /debug_state 를 받아 이 이미지 아래에 합성한다 — 이 노드는 자기가 이미
        계산해 둔 것만 그린다(판단 로직을 여기로 끌어오지 않는다).
        """
        panel_yolo = frame.copy()
        if result.boxes is not None:
            for box in result.boxes:
                cls = self.names[int(box.cls)]
                conf = float(box.conf)
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                color = _DEBUG_BOX_COLOR.get(cls, _DEBUG_BOX_DEFAULT)
                cv2.rectangle(panel_yolo, (x1, y1), (x2, y2), color, 2)
                cv2.putText(panel_yolo, f"{cls} {conf:.2f}", (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        panel_lane = frame.copy()
        y_near = self.get_parameter("lane.eval_near").value
        y_far = self.get_parameter("lane.eval_far").value
        # 기준선은 화면 중심이 아니라 **차량 중심**이다.
        #   target = width/2 + center_bias_px   (lane.py 와 같은 식)
        # 카메라가 차량 정중앙에 안 달려 있으면 둘이 어긋난다. 예전에는 여기서
        # width//2 를 그려서, center_bias_px 를 아무리 바꿔도 선이 안 움직였다
        # (값이 반영 안 되는 줄 알기 딱 좋다). 오프셋도 이 기준으로 재야
        # 화면의 초록 점이 실제 트랙 중앙(c_near) 위치에 찍힌다.
        bias = self.estimator.center_bias_px
        ref_x = int(width / 2.0 + bias)
        cv2.line(panel_lane, (ref_x, 0), (ref_x, height), (255, 0, 0), 1)
        if lane.valid:
            near_pt = (int(ref_x + lane.offset_near), int(y_near))
            far_pt = (int(ref_x + lane.offset_far), int(y_far))
            cv2.line(panel_lane, near_pt, far_pt, (0, 255, 0), 2)
            cv2.circle(panel_lane, near_pt, 6, (0, 255, 0), -1)
            cv2.circle(panel_lane, far_pt, 6, (0, 255, 255), -1)
        cv2.putText(
            panel_lane,
            f"off={lane.offset_near:+.0f}/{lane.offset_far:+.0f} q={lane.quality:.2f} "
            f"{'OK' if lane.valid else 'HOLD'} light={STATE_TO_NAME[state]} cone={det.cone_n}",
            (10, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
        )

        combined = np.hstack([panel_yolo, panel_lane])
        out = self.bridge.cv2_to_imgmsg(combined, "bgr8")
        out.header = msg.header
        self.pub_debug_image.publish(out)

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
            left_band_px=g("lane.left_band_px").value,
            curve_preview_row=self._preview_row(g),
            min_pts=g("lane.min_pts").value,
            min_span=g("lane.min_span").value,
            hold_frames=g("lane.hold_frames").value,
            half_alpha=g("lane.half_alpha").value,
            half_lock_frames=g("lane.half_lock_frames").value,
            half_near_px=g("lane.half_near_px").value,
            half_far_px=g("lane.half_far_px").value,
            max_center_offset_px=g("lane.max_center_offset_px").value,
            max_jump_px=g("lane.max_jump_px").value,
        )

    def _preview_row(self, g):
        """선행거리(m) -> 평가행. curve_preview_row > 0 이면 그것을 쓴다.

        핀홀 근사 X = cam_fh / (row - horizon_row) 을 뒤집은 것이다.
        y_lo 보다 위로 가면(=더 멀리) 표본 밖 외삽이라 2차항이
        발산하므로 y_lo 로 클램프하고 경고한다.
        """
        row = int(g("lane.curve_preview_row").value)
        y_lo = int(g("lane.y_lo").value)
        if row <= 0:
            hz = float(g("lane.horizon_row").value)
            fh = float(g("lane.cam_fh").value)
            m = max(0.1, float(g("lane.curve_preview_m").value))
            row = int(round(hz + fh / m))
            self.get_logger().info(
                f"선행곱률: {m:.2f}m -> {row}행 "
                f"(horizon {hz:.0f}, cam_fh {fh:.0f})")
        if row < y_lo:
            self.get_logger().warn(
                f"curve_preview_row {row} 이 y_lo {y_lo} 보다 멀다 — "
                f"표본 밖 외삽이므로 {y_lo} 로 자른다")
            row = y_lo
        return row

    def on_left_exit(self, msg):
        self._left_exit = bool(int(msg.data))

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

        # center_bias_px 만 매 프레임 다시 읽는다 — 실차에서 파란 기준선을 보며
        # 값을 넣어 맞추는 작업이라 재시작을 끼우면 감을 놓친다:
        #     ros2 param set /perception_node lane.center_bias_px 20.0
        # 나머지 lane.* 는 피팅 구조에 얽혀 있어 estimator 재생성이 필요하므로
        # 여기서 갱신하지 않는다(단순 오프셋인 이 값만 예외).
        self.estimator.center_bias_px = self.get_parameter("lane.center_bias_px").value

        result = self.model.predict(frame, conf=self.infer_conf, verbose=False)[0]
        det = extract(
            result, self.names, width, height,
            dashed_conf=self.dashed_conf,
            solid_conf=self.solid_conf,
            cone_conf=self.cone_conf,
            car_conf=self.car_conf,
        )

        # 좌회전 합류(EXIT) 모드에서는 입력을 바꾼다.
        #   - 노란 점선: 가로형만 왼쪽 strip 으로 자른다(세로형은 그대로)
        #   - 흰 실선: **아예 안 쓴다**
        # 합류 지점에서 앞을 가로지르는 본선 노란선과 본선 흰 실선을 그대로
        # 넣으면 좌우 중점이 엉뚱한 곳에 잡히고 x=f(y) 피팅이 퇴화한다.
        if self._left_exit:
            dashed_in = horizontal_dashed_left_strip(
                det.dashed,
                self.get_parameter("lane.left_exit_strip_px").value,
            )
            solid_in = []
        else:
            dashed_in, solid_in = det.dashed, det.solid

        lane = self.estimator.update(dashed_in, solid_in)
        state = self.voter.update(det.lamp, det.lamp_conf, det.light_width)

        self.pub_lane.publish(Float32MultiArray(data=[
            float(lane.offset_near),
            float(lane.offset_far),
            1.0 if lane.valid else 0.0,
            float(lane.quality),
            float(lane.half_near),
            float(lane.half_far),
            # [확장 필드] 좌회전(지름길) 전용 — 가장 왼쪽 노란픽셀 밴드만 쓴 추정.
            # 평소 주행은 이 값을 안 쓴다. driver_node 가 SHORTCUT 상태에서만
            # 골라 쓴다. 뒤에 붙였으므로 옛 driver_node 와 섞어도 무시된다.
            float(lane.offset_near_left),
            float(lane.offset_far_left),
            1.0 if lane.valid_left else 0.0,
            # [확장] 선행 곱률(px). 없으면 0.0 이라 선행 감속이 안 걸린다.
            float(lane.curve_px),
        ]))
        self.pub_light.publish(Int32(data=int(state)))
        self.pub_objects.publish(Float32MultiArray(data=[
            float(det.cone_n),
            float(det.cone_near_y),
            1.0 if det.car_present else 0.0,
            float(det.car_cx),
            float(det.car_bottom_y),
            # [확장 필드] 가장 큰 콘의 bbox 높이(px). driver_node 가 라바콘 구간
            # 진입 트리거의 "충분히 가까운가" 판단에 쓴다. 뒤에 붙였으므로
            # 옛 driver_node 와 섞어 써도 길이 검사로 무시된다.
            float(det.cone_max_h),
            # [확장 필드] 방해차량 bbox 높이(px). 회피 트리거의 거리 판단.
            # 하단 y(위 5번째)는 진단용으로만 남긴다 — detect.py 주석 참고.
            float(det.car_h),
            # [확장 필드] 방해차량 모델 번호(0=없음, 1=AvanteN, 2=ionic5).
            # 모델마다 차체 크기가 달라 같은 거리에서도 bbox 높이가 다르다.
            # 회피 트리거 문턱을 모델별로 나누려고 판단 쪽으로 넘긴다.
            float(det.car_cls),
            # [확장 필드] traffic_light 박스 폭(px). 0 이면 신호등이
            # 화면에 없다. driver_node 가 "신호등에 접근 중"을 판단해
            # 미리 감속하는 데 쓴다. 램프 클래스가 아니라 **신호등 본체**라
            # 색을 못 읽는 거리에서도 잡힐다 — 그게 핵심이다.
            float(det.light_width),
            # [확장] 모델별 차량 슬롯 2개 × 5필드 (2026-08-24).
            # 없는 모델은 cls=0 으로 비워 보낸다.
            *_vehicle_slots(det),
        ]))

        if self.publish_debug_image:
            self._publish_debug_image(msg, frame, result, lane, state, det, width, height)

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
