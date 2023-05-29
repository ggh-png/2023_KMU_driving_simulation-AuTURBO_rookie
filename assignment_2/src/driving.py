#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import rospy, time
import numpy as np

from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

from math import *
import signal
import sys
import os

#=============================================
# 터미널에서 Ctrl-c 키입력이로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


# =============================================
# 차선을 따라가는 알고리즘을 구현한 클래스
class ControlLane():
    def __init__(self):
        self.motor_msg = xycar_motor()
        # 중앙 차선을 받아옴
        self.sub_lane = rospy.Subscriber('/xycar/center_lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_max_vel = rospy.Subscriber('xycar_motor', xycar_motor, self.cbGetMaxVel, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)
        
        # 초기 오차값을 0으로 설정
        self.lastError = 0
        # 최대 속도를 0.1로 설정
        self.MAX_VEL = 50

        # ROS 종료시 속도를 0으로 설정
        rospy.on_shutdown(self.fnShutDown)
    # 최대 속도를 받아옴
    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.speed

    # lane center를 받아옴 & PID 제어
    def cbFollowLane(self, desired_center):
        #  현제 차선의 중앙값을 받아옴
        center = desired_center.data
        #  오차값을 계산
        error = center
        #  PID 제어
        Kp = 0.025
        # Ki = 0.0001 + Ki * (error + self.lastError)
        Kd = 0.00
        # 제어값을 계산
        angular_z = Kp * error + Kd * (error - self.lastError) 
        # 오차값을 저장
        self.lastError = error
        
        # 모터 메세지를 생성하고 발행
        motor_msg = xycar_motor()
        # 최대 속도를 넘지 않도록 설정
        motor_msg.angle = int(angular_z)
        # int(-max(angular_z, -50) if angular_z < 0 else -min(angular_z, 50))
        # motor_msg.speed = int(min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05))
        motor_msg.speed = 10
        self.pub_cmd_vel.publish(motor_msg)

    # ROS 종료시 속도를 0으로 설정
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        motor_msg = xycar_motor()
        motor_msg.angle = 0
        motor_msg.speed = 0
        self.pub_cmd_vel.publish(motor_msg)
    # 메인 함수
    def main(self):
        rospy.spin()
    
        
        

# 메인 함수
#=============================================
if __name__ == '__main__':
    rospy.init_node('control_lane')

    node = ControlLane()
    node.main()

