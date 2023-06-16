#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

# 초음파 센서 데이터를 저장하는 리스트 
ultrasonicData = [0.0, 0.0, 0.0, 0.0, 0.0]

# 초음파 센서 데이터를 저장하는 콜백 함수
def callback(msg): 
    global ultrasonicData
    ultrasonicData = msg.data  

# 노드 초기화 및 구독자, 발행자 설정
rospy.init_node('driver')
# 초음파 센서 데이터를 받아오는 구독자 설정
rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
# 조향각 및 속도 제어 메시지를 발행하는 발행자 설정
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
# 조향각 및 속도 제어 메시지 객체 생성
xycar_msg = xycar_motor()

# PID 제어 게인 값
Kp = 0.2  # 비례 제어 게인

# 외란이 없는 시뮬레이션 환경이므로 적분 및 미분 제어는 고려하지 않음
Ki = 0.0  # 적분 제어 게인
Kd = 0.0  # 미분 제어 게인

# global error_sum, prev_error
# PID 제어 변수
error_sum = 0  # 오차의 적분 값
prev_error = 0  # 이전 오차 값

def pid_control(error):
    global error_sum, prev_error
    # 오차의 적분 값 업데이트
    error_sum += error
    # 오차의 변화율 계산
    error_diff = error - prev_error
    # PID 제어값 계산
    pid_output = Kp * error + Ki * error_sum + Kd * error_diff
    # 이전 오차 값 업데이트
    prev_error = error
    return pid_output


while not rospy.is_shutdown():
    # 센서값을 사용하여 조향각을 결정하는 로직을 구현
    target_angle = 0  # 목표 조향각 (-50에서 50)
    
    # 초음파 센서 데이터를 활용하여 오차 계산
    error = 0
    # 자동차에 부착된 센서의 오차를 부착된 위치에 따라 계산
    # 왼쪽 >> 오른쪽으로 이동할 때 오차가 증가
    # 오른쪽 >> 왼쪽으로 이동할 때 오차가 감소  
    # 따라서 왼쪽 센서의 오차를 빼고 오른쪽 센서의 오차를 더함
    error -= ultrasonicData[0] # 왼쪽 센서 오차 계산
    error -= ultrasonicData[1] # 왼쪽 센서 오차 계산
    error += ultrasonicData[3] # 오른쪽 센서 오차 계산
    error += ultrasonicData[4] # 오른쪽 센서 오차 계산

    # 위에서 계산한 오차를 이용하여
    # PID 제어를 통해 조향각 계산
    steer = pid_control(error)
    # 조향각이 -50에서 50 사이가 되도록 값 조정
    if steer > 50:
        steer = 50
    elif steer < -50:
        steer = -50
    # 조향각, 속도 결정
    xycar_msg.angle = int(steer)
    print("angle: ", xycar_msg.angle)
    xycar_msg.speed = 50
    # 조향각 및 속도 제어 메시지 전송
    motor_pub.publish(xycar_msg)