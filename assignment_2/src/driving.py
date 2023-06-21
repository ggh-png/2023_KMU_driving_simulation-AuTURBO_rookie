#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2
import rospy, time
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os

from preprocessor import PreProcessor


#=============================================
# 터미널에서 Ctrl-c 키입력이로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    time.sleep(1)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
#bridge = CvBridge()
motor = None # 모터 토픽을 담을 변수
errorPrev = 0 
proportional_gain = 0.50  # 0.52
integral_gain = 0.0008 #  0.0000002
derivative_gain = 10.00 # 15
error_sum = 0

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
# 카메라 이미지 토픽이 도착하면 자동으로 호출되는 함수
# 토픽에서 이미지 정보를 꺼내 image 변수에 옮겨 담음.
#=============================================
def img_callback(data):
    global image
    image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # R

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

#=============================================
# PID 제어 함수
# 목표 값과 현재 값을 받아
# 최종 PID 제어 출력을 생성한다.
#=============================================

def calculate_PID(reference_input, feedback_input):
    # 위에서 선언한 변수를 calculate_PID() 안에서 사용하고자 함
    global error_sum
    global errorPrev
    
    error = reference_input - feedback_input   # 에러 계산
    error_sum = error_sum + error            # 에러합 계산(적분)
    proportional_output = proportional_gain * error  # 비례(P) 제어 값
    integral_output = integral_gain*error_sum   # 적분(I) 제어 값
    derivative_output = derivative_gain * (error - errorPrev) # 미분(D) 제어 값
    errorPrev = error 

    # 최종 PID 제어 출력 값 
    output = int(proportional_output + integral_output + derivative_output)
    return output

#=============================================
# 실질적인 메인 함수
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함.
#=============================================

def start():

    roi_height = 200 # roi 높이
    roi_width = 640 # roi 폭

    pre_module = PreProcessor(roi_height, roi_width) # 영상처리 객체 생성

    # 위에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image, img_callback)

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    rospy.wait_for_message("/usb_cam/image_raw/", Image)

    #=========================================
    # 메인 루프
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서
    # "이미지처리 + 차선위치찾기 + 조향각 결정 + 모터토픽 발행"
    # 작업을 반복적으로 수행함.
    #========================================

    while not rospy.is_shutdown():
       
        # 이미지 처리를 위해 카메라 원본 이미지를 img에 복사 저장한다.
        img = image.copy()

        rows, cols = img.shape[:2] # 이미지의 height, width 정보를 받아온다.
        
        cv2.imshow("origin", img) # 원본 영상 디스플레이
        cv2.waitKey(1)

        warped_img = pre_module.warp_perspect(img) # Inverse Perspective Mapping(IPM)을 이용해 Bird Eye View 변환
        #cv2.imshow("warped_img", warped_img) # Bird Eye View 영상 디스플레이

        white_parts = pre_module.color_filter(warped_img) # HSV 변환후 흰색 픽셀만 필터링
        #cv2.imshow("white_parts", white_parts) # 흰색 픽셀 필터링 영상 디스플레이

        msk, lx, ly, rx, ry = pre_module.sliding_window(white_parts) # Sliding window 알고리즘으로 차선 검출
        
        #overlay_img = pre_module.overlay_line(warped_img, lx, ly, rx, ry) # 2차 다항식으로 차선 피팅후 오버레이 

        #=========================================
        # 제어기가 추종할 값인 target값 정하기.
        # 차선 검출의 결과에 따라 추종할 목표 값을 설정함.
        #=========================================

        if len(lx) > 2 and len(rx) > 2 : # 왼쪽 차선과 오른쪽 차선 둘다 있을떄는 직선 구간으로 판단
            target = (lx[len(lx) // 2] + rx[len(rx) // 2]) // 2 # 왼쪽차선과 오른쪽 차선의 중앙값을 추종하는 목표 값으로 설정
            speed = 30 # 직선 구간은 속도를 43으로 설정
        elif len(lx) > 2 and len(rx) == 0: # 왼쪽 차선만 있을때
            if len(lx) > 2 and len(ly) > 2: # 왼쪽 차선 각도 구하기
                w = lx[-1] - lx[0] 
                h = ly[-1] - ly[0]
                rad = atan2(h, w)
                line_angle = abs(degrees(rad))

                print(f"left angle:{line_angle}")

                if line_angle > 90 + 45: # 왼쪽으로 일정 각도 이상 기울어져 있으면 급격한 코너로 판단
                    target = 300 # 왼쪽으로 회전
                    print("left!!!")
                else: # 코너가 아닌 왼쪽 차선만 있으면 
                    target = lx[len(lx) // 2] + 50 # 왼쪽 차선과 일정 간격 유지
            speed = 20 # 속도를 42으로 설정
        elif len(lx) == 0 and len(rx) > 2: # 오른쪽 차선만 있을때
            if len(rx) > 2 and len(ry) > 2: # 오른쪽 차선 각도 구하기
                w = rx[-1] - rx[0]
                h = ry[-1] - ry[0]
                rad = atan2(h, w)
                line_angle = abs(degrees(rad))

                print(f"right angle:{line_angle}")

                if line_angle < 90 - 45: # 오른쪽으로 일정 각도 이상 기울어져 있으면 급격한 코너로 판단 57
                    target = 340 # 오른쪽으로 회전  
                    print("right!!!")
                else: # 코너가 아닌 오른쪽 차선만 있으면 
                    target = rx[len(rx) // 2] - 50 # 오른쪽 차선과 일정 간격 유지
            speed = 20 # 속도를 42으로 설정

        print(f"target: {target}")

        if target != None:
            cv2.circle(msk, (target, 120), 3, (255, 0, 0), 4) # 추종 목표 값 점으로 표시 
        cv2.circle(msk, (320, 120), 3, (0, 0, 255), 2) # 차량 중앙 점으로 표시
        cv2.imshow("Lane Detection - Sliding Windows", msk) # Sliding window 디스플레이
        
        #=========================================
        # 핸들 조향각 값인 angle값 정하기.
        # 차선의 위치 정보와 PID 제어기를 이용해서 angle값을 설정함.
        #=========================================

        angle = calculate_PID(target, 320) # 핸들을 얼마나 꺾을지 결정
        #print(f"angle: {angle}")

        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(angle, speed)


#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함
# start() 함수가 실질적인 메인 함수임.
#=============================================
if __name__ == '__main__':
    start()