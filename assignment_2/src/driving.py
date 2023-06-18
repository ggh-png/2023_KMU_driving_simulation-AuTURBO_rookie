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
import matplotlib.pyplot as plt

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
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #image = bridge.imgmsg_to_cv2(data, "bgr8")

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
# 실질적인 메인 함수
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함.
#=============================================

errorPrev = 0
proportional_gain = 0.8 #0.7
integral_gain = 0 #0.0001
derivative_gain = 2.00#8.5
error_sum = 0

def calculate_PID(reference_input, feedback_input):
    global error_sum
    global errorPrev
    
    error = reference_input - feedback_input   # Compute the error
    error_sum = error_sum + error            # Compute the error sum
    proportional_output = proportional_gain * error  # Compute the proportional output
    integral_output = integral_gain*error_sum   # Compute the integral output
    derivative_output = derivative_gain * (error - errorPrev) # Compute the derivative output
    errorPrev = error

    # Compute the pre-saturated output
    presaturated_output = int(proportional_output + integral_output + derivative_output)
    return presaturated_output

def start():

    roi_height = 200
    roi_width = 640

    pre_module = PreProcessor(roi_height, roi_width)

    window_base_find_flag = False

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

    last_error = 0

    while not rospy.is_shutdown():
       
        # 이미지 처리를 위해 카메라 원본 이미지를 img에 복사 저장한다.
        img = image.copy()

        rows, cols = img.shape[:2] # 이미지의 height, width 정보를 받아온다.
        
        cv2.imshow("origin", img)
        cv2.waitKey(1)

        warped_img = pre_module.warp_perspect(img)
        #cv2.imshow("warped_img", warped_img)

        white_parts = pre_module.color_filter(warped_img)
        #cv2.imshow("white_parts", white_parts)

        msk, lx, ly, rx, ry = pre_module.sliding_window(white_parts)
        

        overlay_img = pre_module.overlay_line(warped_img, lx, ly, rx, ry)

        if len(lx) != 0 and len(rx) != 0 :
            target = (lx[0] + rx[0]) // 2
            speed = 50
        elif len(lx) != 0 and len(rx) == 0:
            target = lx[0] + 50
            #print("left only")
            speed = 50
        elif len(lx) == 0 and len(rx) != 0:
            target = rx[0] - 50
            speed = 50
            #print("right only")
        #print(f"target: {target}")
        cv2.circle(msk, (target, 120), 3, (255, 0, 0), 4)
        cv2.circle(msk, (320, 120), 3, (0, 0, 255), 2)
        cv2.imshow("Lane Detection - Sliding Windows", msk)
        
        # error = target - 320
        #print(f"error: {error}")
        #cv2.imshow("Lane Detection - Overlay", overlay_img)

        # kp = 1.1
        # kd = 1.1

        angle = calculate_PID(target, 320)
        print(f"angle: {angle}")
        
        #=========================================
        # 핸들 조향각 값인 angle값 정하기.
        # 차선의 위치 정보를 이용해서 angle값을 설정함.
        #=========================================

        # 핸들을 얼마나 꺾을지 결정
        #angle = 0

        #=========================================
        # 차량의 속도 값인 speed값 정하기.
        # 주행 속도를 조절하기 위해 speed값을 설정함.
        #=========================================

        # 주행 속도를 결정
        #speed = 50

        # drive() 호출. drive()함수 안에서 모터 토픽이 발행됨.
        drive(angle, speed)


#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함
# start() 함수가 실질적인 메인 함수임.
#=============================================
if __name__ == '__main__':
    start()

