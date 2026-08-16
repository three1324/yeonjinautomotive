#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Float32MultiArray

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None
motor_msg = Float32MultiArray()

#=============================================
# 모터 토픽을 발행하는 함수.
#=============================================
def drive(angle, speed):
    motor_msg.data = [float(angle), float(speed)] 
    motor.publish(motor_msg)

#=============================================
# 실질적인 메인 함수
#=============================================
def start():
    global motor
    #=========================================
    # ROS 노드를 생성하고 초기화
    # 토픽의 구독과 발행을 선언
    #=========================================
    rospy.init_node('driver')
    motor = rospy.Publisher('xycar_motor', Float32MultiArray, queue_size=1)
    print ("----- Xycar self driving -----")
    
    # 핸들 똑바로 정렬, 속도 0.0으로 정지, 3초 대기
    drive(angle=0.0, speed=0.0)
    time.sleep(3)
    delta = 10
 
    for i in range(10):
        angle = 0
        speed = 0
        drive(angle, speed)
        time.sleep(0.1)

    #=========================================
    # 메인루프 - ROS가 종료될 때까지 반복
    #=========================================
    while not rospy.is_shutdown():

        angle = angle+delta
        speed = speed+delta
        
        for i in range(20):            
            drive(angle, speed)
            time.sleep(0.1)

        if (speed == 50):
            delta = -5
        elif (speed == -50):
            delta = +5

#=============================================
# 메인 함수 - start() 함수를 호출.
#=============================================
if __name__ == '__main__':
    start()
