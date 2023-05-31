#!/usr/bin/env python
#-- coding:utf-8 --

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
from xycar_msgs.msg import xycar_motor

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#============================================= 
rx, ry = [300, 350, 400, 450], [300, 350, 400, 450]

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

# 
def pid_control(target, current, prev_error, integral, Kp, Ki, Kd):
    error = target - current
    integral += error
    derivative = error - prev_error

    output = Kp * error + Ki * integral + Kd * derivative

    return output, error, integral

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================

def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    target_x = 1036  # 목표 x 위치
    target_y = 162  # 목표 y 위치
    target_yaw = 0  # 목표 각도

    # PID 제어 게인 설정
    Kp = 0.4
    Ki = 0.01
    Kd = 0.1

    prev_error = 0.0
    integral = 0.0


    # 현재 위치와 각도 정보를 업데이트
    current_x = x
    current_y = y
    current_yaw = yaw

    # 목표 위치와 현재 위치 간의 거리와 각도 계산
    distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
    angle = math.atan2(target_y - current_y, target_x - current_x)
    error_yaw = target_yaw - current_yaw

    # PID 제어를 통해 주행 각도 계산
    angle_output, prev_error, integral = pid_control(angle, error_yaw, prev_error, integral, Kp, Ki, Kd)

    # PID 제어를 통해 주행 속도 계산
    speed_output, prev_error, integral = pid_control(0, distance, prev_error, integral, Kp, Ki, Kd)

    # 주행 각도와 속도를 모터에 전달하여 주행
    drive(angle_output, speed_output)


#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry
    print("Start Planning")

    return rx, ry


# 메인 함수
def main():
    pygame.init()

    # ROS 초기화
    rospy.init_node('path_planning', anonymous=True)


    # 초기 위치 및 각도 설정
    start_x = 300
    start_y = 300
    start_yaw = -1.57

    # 초기 속도 및 최대 가속도 설정
    velocity = 0
    max_acceleration = 10

    # 단위 시간 설정
    dt = 0.01

    # 주행 제어 시작
    tracking(screen, start_x, start_y, start_yaw, velocity, max_acceleration, dt)

    pygame.quit()

if __name__ == '__main__':
    main()