#!/usr/bin/env python3
"""
라이다 원본 데이터가 제대로 들어오는지 확인하기 위한 진단용 구독 노드.

race_manager, yolo_perception 등 기존 주행 코드는 전혀 거치지 않고,
이 패키지 안에서 /scan을 직접(새로) 구독한다. rf2o/slam_toolbox에 데이터를
넘기기 전에, 하드웨어에서 값이 정상적으로 나오는지 눈으로 먼저 확인하는 용도.

TODO(1): SCAN_TOPIC 값이 실제 라이다 드라이버가 발행하는 토픽 이름과 같은지 확인.
         (확인 방법: ros2 topic list)
TODO(2): EXPECTED_FRAME_ID 값이 실제 라이다 메시지의 frame_id와 같은지 확인.
         (확인 방법: ros2 topic echo /scan --once  -> header.frame_id 확인)
         이 값은 launch 파일의 static_transform_publisher child frame과도 맞춰야 한다.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SCAN_TOPIC = '/scan'                # TODO(1): 실제 토픽 이름 확인 후 수정
EXPECTED_FRAME_ID = 'laser_frame'   # TODO(2): 실제 라이다 frame_id 확인 후 수정


class ScanCheckNode(Node):

    def __init__(self):
        super().__init__('scan_check_node')
        self.sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self.on_scan, qos_profile_sensor_data)
        self._count = 0
        self._last_log = time.time()
        self._warned_frame_mismatch = False
        self.get_logger().info(f'Subscribed to {SCAN_TOPIC}. Waiting for data...')

    def on_scan(self, msg: LaserScan):
        self._count += 1

        if msg.header.frame_id != EXPECTED_FRAME_ID and not self._warned_frame_mismatch:
            self.get_logger().warn(
                f'frame_id mismatch: got "{msg.header.frame_id}", '
                f'expected "{EXPECTED_FRAME_ID}". '
                f'scan_check_node.py의 EXPECTED_FRAME_ID, 그리고 launch 파일의 '
                f'static_transform_publisher child frame 값을 갱신하세요 (TODO 2).')
            self._warned_frame_mismatch = True

        now = time.time()
        if now - self._last_log >= 1.0:
            valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            min_r = min(valid) if valid else float('nan')
            max_r = max(valid) if valid else float('nan')
            self.get_logger().info(
                f'rate={self._count}Hz points={len(msg.ranges)} '
                f'valid={len(valid)} min={min_r:.2f} max={max_r:.2f} '
                f'frame_id={msg.header.frame_id}')
            self._count = 0
            self._last_log = now


def main():
    rclpy.init()
    node = ScanCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
