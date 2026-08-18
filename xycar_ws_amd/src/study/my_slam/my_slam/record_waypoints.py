#!/usr/bin/env python3
"""주행 경로를 waypoint CSV 로 기록한다. (2단계 준비물)

측위가 돌아가는 상태에서 트랙을 한 바퀴 주행하면, map 좌표계 기준 궤적이 저장된다.
저장된 CSV 는 tools/clean_waypoints.py 로 평활화한 뒤 속도계획/레이싱라인에 쓴다.

사용:
    # 1) 측위를 먼저 띄운다
    ros2 launch my_slam localization.launch.py map:=~/track_map
    # 2) 기록 시작 (수동 주행 or 차선추종 주행 중에 켜두면 된다)
    ros2 run my_slam record_waypoints --ros-args -p out_path:=$HOME/wp.csv

출력 형식은 f1tenth 관례를 따른다 (헤더 '# x_m, y_m, yaw_rad').
"""

import math
import os

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_from_quat(q):
    """쿼터니언 -> yaw. tf_transformations 의존성을 피하려고 직접 계산한다."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class WaypointRecorder(Node):

    def __init__(self):
        super().__init__('record_waypoints')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('out_path', os.path.expanduser('~/waypoints.csv')),
                ('map_frame', 'map'),
                ('base_frame', 'base_link'),
                ('sample_hz', 20.0),
                # 이 거리 이상 움직였을 때만 점을 남긴다. 정지 중 같은 점이
                # 수천 개 쌓이는 것을 막는다.
                ('min_distance_m', 0.05),
            ],
        )

        g = self.get_parameter
        self.out_path = g('out_path').value
        self.map_frame = g('map_frame').value
        self.base_frame = g('base_frame').value
        self.min_dist = g('min_distance_m').value

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.points = []
        self._last = None
        self._warned = False

        self.create_timer(1.0 / g('sample_hz').value, self.on_tick)
        self.get_logger().info(
            f"waypoint 기록 시작 -> {self.out_path}  (Ctrl+C 로 종료하면 저장됨)")

    def on_tick(self):
        try:
            tf = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001 - TF 는 초반에 자주 실패한다
            if not self._warned:
                self.get_logger().warn(
                    f"TF {self.map_frame}->{self.base_frame} 대기중: {exc}")
                self._warned = True
            return

        self._warned = False
        t = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)

        if self._last is not None:
            dx, dy = t.x - self._last[0], t.y - self._last[1]
            if math.hypot(dx, dy) < self.min_dist:
                return

        self._last = (t.x, t.y)
        self.points.append((t.x, t.y, yaw))

        if len(self.points) % 100 == 0:
            self.get_logger().info(f"{len(self.points)} points")

    def save(self):
        if not self.points:
            self.get_logger().warn("기록된 점이 없어 저장하지 않는다")
            return
        d = os.path.dirname(os.path.abspath(self.out_path))
        if d:
            os.makedirs(d, exist_ok=True)
        with open(self.out_path, 'w', encoding='utf-8') as f:
            f.write('# x_m,y_m,yaw_rad\n')
            for x, y, yaw in self.points:
                f.write(f'{x:.4f},{y:.4f},{yaw:.4f}\n')
        self.get_logger().info(f"{len(self.points)} points 저장 -> {self.out_path}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
