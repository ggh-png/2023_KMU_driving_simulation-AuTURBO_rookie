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

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry
    print("Start Planning")
    print("sx = %d sy = %d syaw = %d max_acceleration = %d dt = %f" % (sx, sy, syaw, max_acceleration, dt))

    # 출발점과 도착점 연결을 위한 직선 경로 생성
    dx = sx - P_END[0]
    dy = sy - P_END[1]
    distance = math.sqrt(dx ** 2 + dy ** 2)
    num_points = int(distance / 100) + 1  # 100 단위로 등간격으로 점 생성
    rx = np.linspace(P_END[0], sx)
    ry = np.linspace(P_END[1], sy)

    # 시작각도와 종료각도 사이를 천천히 이동하는 경로 추가
    yaw_diff = 45 - syaw
    num_yaw_points = int(abs(yaw_diff) / 5) + 1  # 5도 단위로 등간격으로 각도 생성
    yaw_values = np.linspace(syaw, 45, num_yaw_points)

    if syaw > 45:
        rx = np.append(rx, P_END[0])
        ry = np.append(ry, P_END[1])
    else:
        rx = np.insert(rx, 0, P_END[0])
        ry = np.insert(ry, 0, P_END[1])

    for yaw in yaw_values:
        dx = 10 * math.cos(math.radians(yaw))
        dy = 10 * math.sin(math.radians(yaw))
        new_x = rx[-1] + dx
        new_y = ry[-1] + dy
        rx = np.append(rx, new_x)
        ry = np.append(ry, new_y)

    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    global rx, ry

    # 현재 위치에서 가장 가까운 경로 상의 점을 찾음
    current_point = np.array([x, y])
    distances = np.linalg.norm(np.array([rx, ry]) - current_point, axis=1)
    closest_index = np.argmin(distances)
    target_point = np.array([rx[closest_index], ry[closest_index]])

    # 차량의 방향 벡터와 타겟 점을 연결한 벡터를 계산
    direction_vector = target_point - current_point

    # 타겟 각도 계산
    target_yaw = math.atan2(direction_vector[1], direction_vector[0])
    target_yaw = math.degrees(target_yaw)

    # 현재 각도와 타겟 각도의 차이 계산
    angle_diff = target_yaw - yaw

    # 각도 제한 (-50 ~ 50)
    angle_diff = np.clip(angle_diff, -50, 50)

    # 속도 제한 (-50 ~ 50)
    speed = 50

    drive(angle_diff, speed)

    # global rx, ry
    # angle = 0  # -50 ~ 50
    # speed = 50 # -50 ~ 50
    
    # drive(angle, speed)

