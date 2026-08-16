#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# xycar_motor 토픽(Float32MultiArray [angle, speed])을 받아 캘리브레이션을 적용하고
# AckermannDriveStamped로 변환해 vesc_ackermann(ackermann_to_vesc_node)로 넘긴다.
# ROS1 noetic_ws/src/xycar_motor/src/xycar_motor.py 의 VESC 경로를 ROS2로 이식한 것.
# motor_type=1(아두이노 시리얼 경로)은 항상 motor_type=0으로 하드코딩되어 실사용되지
# 않았으므로 이식하지 않았다.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float32MultiArray

FAULT_CODE = [
    "FAULT_CODE_NONE",
    "FAULT_CODE_OVER_VOLTAGE",
    "FAULT_CODE_UNDER_VOLTAGE",
    "FAULT_CODE_DRV8302",
    "FAULT_CODE_ABS_OVER_CURRENT",
    "FAULT_CODE_OVER_TEMP_FET",
    "FAULT_CODE_OVER_TEMP_MOTOR",
]

# auto_drive_vesc()의 속도 램핑(가속/감속 계단) 스텝 사이 대기 시간(초).
# ROS1 원본과 동일한 값 — 콜백 안에서 블로킹되므로 spin_once 주기에 영향을 준다.
CHANGE_VECTOR_TERM = 0.07
RAMP_STEPS = 10


class XycarMotor(Node):

    def __init__(self):
        super().__init__('xycar_motor')

        self.declare_parameter('angle_offset', 0.0)
        self.declare_parameter('angle_bias_1', 0.0)
        self.declare_parameter('angle_bias_2', 0.0)
        self.declare_parameter('angle_weight', 0.0068)
        self.declare_parameter('speed_bias_1', 0.0)
        self.declare_parameter('speed_bias_2', 0.0)
        self.declare_parameter('speed_weight', 0.08)
        # ROS1에서는 /vesc_driver/speed_max 등을 전역 파라미터 서버에서 직접 읽었으나
        # ROS2는 전역 파라미터 서버가 없어 같은 값을 이 노드에도 선언해 config/vesc.yaml
        # 하나로 vesc_driver와 함께 채운다.
        self.declare_parameter('speed_max', 40000.0)
        self.declare_parameter('speed_min', -20000.0)
        self.declare_parameter('speed_to_erpm_gain', 4614.0)

        self.angle_offset = self.get_parameter('angle_offset').value
        self.angle_bias_1 = self.get_parameter('angle_bias_1').value
        self.angle_bias_2 = self.get_parameter('angle_bias_2').value
        self.angle_weight = self.get_parameter('angle_weight').value
        self.speed_bias_1 = self.get_parameter('speed_bias_1').value
        self.speed_bias_2 = self.get_parameter('speed_bias_2').value
        self.speed_weight = self.get_parameter('speed_weight').value

        speed_max = self.get_parameter('speed_max').value
        speed_min = self.get_parameter('speed_min').value
        speed_to_erpm_gain = self.get_parameter('speed_to_erpm_gain').value
        self.vesc_smax = speed_max / speed_to_erpm_gain
        self.vesc_smin = speed_min / speed_to_erpm_gain

        self.last_speed = 0.0
        self.start_time = self.get_clock().now()
        self.battery_low_logged_at = self.start_time

        self.ack_msg = AckermannDriveStamped()
        self.ack_msg.header.frame_id = ''

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.ackerm_publisher = self.create_publisher(
            AckermannDriveStamped, 'ackermann_cmd', reliable_qos)
        self.create_subscription(
            Float32MultiArray, 'xycar_motor', self.motor_callback, reliable_qos)
        self.create_subscription(
            VescStateStamped, '/sensors/core', self.vesc_state_callback, sensor_qos)

        self.get_logger().info('xycar_motor node started')

    def vesc_state_callback(self, msg):
        battery = msg.state.voltage_input
        fault_code = msg.state.fault_code
        if fault_code:
            self.get_logger().fatal(FAULT_CODE[fault_code])

        now = self.get_clock().now()
        if battery < 5 and (now - self.battery_low_logged_at).nanoseconds > 1e9:
            self.get_logger().warn('The current battery voltage is lower than 8V.')
            self.battery_low_logged_at = now

    def motor_callback(self, msg):
        angle = max(-50.0, min(msg.data[0], 100.0))
        speed = max(-50.0, min(msg.data[1], 100.0))

        calibrated_angle = (
            (angle + self.angle_offset + self.angle_bias_1) * self.angle_weight
        ) + self.angle_bias_2
        calibrated_speed = (
            (speed + self.speed_bias_1) * self.speed_weight
        ) + self.speed_bias_2

        self.drive(calibrated_angle, calibrated_speed)

    def drive(self, steer_val, car_run_speed):
        self.ack_msg.header.stamp = self.get_clock().now().to_msg()
        self.ack_msg.drive.steering_angle = -steer_val
        target_speed = max(min(car_run_speed, self.vesc_smax), self.vesc_smin)

        if self._crosses_zero(self.last_speed, target_speed):
            self._ramp_through_zero(target_speed)
        elif self.last_speed == 0.0:
            self._ramp_from_zero(target_speed)

        self.ack_msg.drive.speed = float(target_speed)
        self.ackerm_publisher.publish(self.ack_msg)
        self.last_speed = float(target_speed)

    @staticmethod
    def _crosses_zero(last_speed, target_speed):
        return (last_speed > 0 and target_speed <= 0) or (last_speed < 0 and target_speed >= 0)

    def _ramp_through_zero(self, target_speed):
        min_v_piece = float(self.vesc_smax) / (2.0 * float(RAMP_STEPS))
        reduce_v = max(abs(target_speed - self.last_speed) / float(RAMP_STEPS), min_v_piece)
        vector = -(self.last_speed / abs(self.last_speed))
        for step in range(RAMP_STEPS):
            step_speed = self.last_speed + (vector * step * reduce_v)
            if vector > 0:
                self.ack_msg.drive.speed = min(step_speed, target_speed)
            else:
                self.ack_msg.drive.speed = max(step_speed, target_speed)
            self.ackerm_publisher.publish(self.ack_msg)
            self._sleep(CHANGE_VECTOR_TERM)
        self._sleep(CHANGE_VECTOR_TERM)

    def _ramp_from_zero(self, target_speed):
        min_v_piece = float(self.vesc_smax) / (2.0 * float(RAMP_STEPS))
        if target_speed < 0:
            reduce_v = min(target_speed / float(RAMP_STEPS), min_v_piece)
            steps = RAMP_STEPS
        elif target_speed > 0:
            reduce_v = max(target_speed / float(RAMP_STEPS), min_v_piece)
            steps = RAMP_STEPS
        else:
            return
        for step in range(steps):
            step_speed = self.last_speed + (reduce_v * step)
            if target_speed < 0:
                self.ack_msg.drive.speed = max(step_speed, target_speed)
            else:
                self.ack_msg.drive.speed = min(step_speed, target_speed)
            self.ackerm_publisher.publish(self.ack_msg)
            self._sleep(CHANGE_VECTOR_TERM)
        self._sleep(CHANGE_VECTOR_TERM)

    @staticmethod
    def _sleep(seconds):
        # ROS1 원본과 동일하게 콜백 안에서 블로킹 sleep으로 램핑한다.
        # (단일 스레드 executor 기준으로 spin이 그동안 멈추는 것도 원본과 동일한 동작)
        import time
        time.sleep(seconds)


def main(args=None):
    rclpy.init(args=args)
    node = XycarMotor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
