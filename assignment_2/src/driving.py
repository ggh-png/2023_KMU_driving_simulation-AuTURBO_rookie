#!/usr/bin/env python
# -*- coding: utf-8 -*-

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # ros와 opencv 연결해주는 bridge 역할을 해줌
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os
import random

#=============================================
# 터미널에서 Ctrl-C 키입력으로 프로그램 실행을 끝낼 때
# 그 처리시간을 줄이기 위한 함수
#=============================================
def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0]) # 카메라 이미지를 담을 변수
bridge = CvBridge() 
motor = None # 모터 토픽을 담을 변수
wb = 0 # 가중치
searching = True # 카메라의 오른쪽 차선 여러개 보일 경우를 대비해서 xycar가 안쪽의 차선만을 따라가도록 제어해야한다. 이를 위해서 평소에 이미지의 중앙부터 오른쪽으로 차선을 다시 확인할 것인데
                 # 그럴 필요가 없다면(오른쪽 차선이 아예 보이지 않을 경우) False, 평소에는 True

pre_mask_left = np.empty(shape=[0]) # 카메라에 흰색 선이 보이지 않을 때 이전의 mask 이미지를 담을 변수
pre_mask_right = np.empty(shape=[0]) 
pre_cx_left = 0; pre_cy_left = 400; pre_cx_right = 640; pre_cy_right = 400

# PID 제어 게인 값
Kp = 0.2  # 비례 제어 게인

# 외란이 없는 시뮬레이션 환경이므로 적분 및 미분 제어는 고려하지 않음
Ki = 0.0  # 적분 제어 게인
Kd = 0.0  # 미분 제어 게인

# global error_sum, prev_error
# PID 제어 변수
error_sum = 0  # 오차의 적분 값
prev_error = 0  # 이전 오차 값

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
    image = bridge.imgmsg_to_cv2(data, "bgr8") # 받은 이미지메세지를 opencv로 넘겨주는 함수(rgb 8bit로) 
    # 이후 opencv 함수를 사용하면 됨

#=============================================
# 모터 토픽을 발행하는 함수  
# 입력으로 받은 angle과 speed 값을 
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):

    global motor

    motor_msg = xycar_motor() # motor_msg라는 xycar_motor class의 성질을 가지는 객체 생성
                              # /xycar_motor 토픽에는 angle과 speed라는 int32형 데이터가 있기 때문에 
    motor_msg.angle = angle # angle 값과
    motor_msg.speed = speed # speed 값을 넣어주고

    motor.publish(motor_msg) # publish해주면 xycar는 지정해준 angle과 speed로 움직이게 된다.

def searchingCenterOfMask(mask, cx, cy):
    global image

    h, w, d = image.shape
    if searching:
        if mask[cy,cx] == 0: # 무게중심이 흰색 차선 위가 아닌 다른 곳에 잡힌다면
            cx = w/2 # 무게중심의 cx를 w/2부터 시작해서 증가시키면서 
            while mask[cy,cx] == 0: # 가장 안쪽에 있는 흰색 차선을 찾고 이를 인식하도록 함.
                cx += 1

    return (cx, cy)

def centerFromMask(mask, contour_left, contour_right):
    global wb, searching
    global pre_mask_left, pre_mask_right
    global pre_cx_left, pre_cy_left, pre_cx_right, pre_cy_right

    mask_left = mask
    mask_right = mask

    mask_left[:, contour_left:] = 0
    mask_right[:, 0:contour_right] = 0

    if np.all(mask_left + mask_right == 0): # xycar가 급격한 우회전을 하는 경우에는 오른쪽 차선이 카메라를 벗어나 보이지 않을 것이므로 mask에는 0만 저장되어 있을 것이다. 
                          # 따라서 mask의 모든 numpy 배열 값이 0이라면, 
        mask_left, mask_right = pre_mask_left, pre_mask_right # 이전 mask를 그대로 사용하고
        wb += 1 # 바뀌지 않을 angle을 점점 더 크게 해주기 위해서 가중치 값을 더해줌
        searching = False # 흰색 선이 여러개 감지된 경우가 아니므로 searching은 False
            
    else: # 차선이 인식되고 있는 상태라면
        pre_mask_left, pre_mask_right = mask_left, mask_right # 흰색선이 보이지 않을 때 사용하기 위한 pre_mask에 현재의 mask값 저장
        wb = 0 # 가중치는 초기화시켜줌
        searching = True # searching은 다시 True로 설정
    
    moments_left = cv2.moments(mask_left)
    moments_right = cv2.moments(mask_right)

    if moments_left['m00'] <= 0:
        moments_left = cv2.moments(pre_mask_left)

    if moments_right['m00'] <= 0:
        moments_right = cv2.moments(pre_mask_right)
    
    if moments_right['m00'] > 0:
        cx_left = int(moments_left['m10'] / moments_left['m00'])
        cy_left = int(moments_left['m01'] / moments_left['m00'])
        pre_cx_left, pre_cy_left = cx_left, cy_left
    else:
        cx_left, cy_left = pre_cx_left, pre_cy_left
    
    if moments_right['m00'] > 0:
        cx_right = int(moments_right['m10'] / moments_right['m00'])
        cy_right = int(moments_right['m01'] / moments_right['m00'])
        pre_cx_right, pre_cy_right = cx_right, cy_right
    else:
        cx_right, cy_right = pre_cx_right, pre_cy_right


    return (searchingCenterOfMask(mask_left, cx_left, cy_left), searchingCenterOfMask(mask_right, cx_right, cy_right))

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

#=============================================
# 실질적인 메인 함수 
# 카메라 토픽을 받아 각종 영상처리와 알고리즘을 통해
# 차선의 위치를 파악한 후에 조향각을 결정하고,
# 최종적으로 모터 토픽을 발행하는 일을 수행함. 
#=============================================
def start():

    # 아래에서 선언한 변수를 start() 안에서 사용하고자 함
    global motor, image

    #=========================================
    # ROS 노드를 생성하고 초기화 함.
    # 카메라 토픽을 구독하고 모터 토픽을 발행할 것임을 선언
    #=========================================
    rospy.init_node('driving')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1) # 'xycar_motor'를 publish해주기 위한 변수
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback) # /usb_cam/image_raw/ 데이터를 받는데 그 데이터의 형태는 Image이고 데이터를 받으면
                                                                           # img_callback 함수 실행, 즉 xycar는 Image 형태의 데이터를 계속해서 보낼텐데 우리는 
                                                                           # subsciber 노드를 통해서 이를 받을 때마다 img_callback함수를 실행시켜 opencv로 넘겨
                                                                           # 주고 있다.

    print ("----- Xycar self driving -----")

    # 첫번째 카메라 토픽이 도착할 때까지 기다림.
    while not image.size == (WIDTH * HEIGHT * 3):
        continue

    #=========================================
    # 메인 루프 
    # 카메라 토픽이 도착하는 주기에 맞춰 한번씩 루프를 돌면서 
    # "이미지처리 +차선위치찾기 +조향각결정 +모터토픽발행" 
    # 작업을 반복적으로 수행함.
    #=========================================
    while not rospy.is_shutdown():
        img = image.copy() # img_callback 함수에 의해서 opencv로 바뀐 카메라 원본이미지를 img에 복사하여 저장
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # bgr을 hsv로 바꿔줌
        # 색상 : H ) 0 - 360 의 범위를 가지고, 가장 파장이 긴 빨간색을 0도로 지정한다.
        # 채도 : S ) 0 - 100 의 범위를 가지고, 색상이 가장 진한 상태를 100으로 하며, 진함의 정도를 나타낸다.
        # 명도 : V ) 0 - 100 의 범위를 가지고, 흰색, 빨간색을 100, 검은색이 0이며, 밝은 정도를 나타낸다.
        angle = 0 # 초기 xycar의 각도 설정

        lower_white = np.array([0,0,200]) # 흰색의 범위 설정 (hsv의 하한과 상한 결정)
        upper_white = np.array([180,20,255])
        mask = cv2.inRange(hsv, lower_white, upper_white) # 흰색의 범위 안에 들어가 있는 것만 image를 다시 땀 

        h, w, d = img.shape # img의 높이(h) 너비(w)를 저장(깊이(d)도 2차원 이미지를 사용하므로 여기선 사용하지 않는다.) 있지만), 영상에서는 항상 왼쪽 위가 (0,0)이고 오르쪽 아래로 갈수록 숫자가 커짐

        #=========================================
        # 반 위쪽은 사용하지 않기 때문에 이미지의 위쪽 3/4만큼은 자르고 높이가 20인 image 사용 
        # 흰색만 딴 mask에서 searching_top ~ searching_bot에 해당하지 않는 부분은 다 0으로 바꿔서 버림
        # 오른쪽 차선을 따라갈 것이기 때문에 필요없는 왼쪽 이미지를 4/7만큼 자름
        # 즉 xycar는 높이: searching_top~searching_bot, 너비: 4*w/7 ~ w 이미지에 있는 흰색 선만 인식
        #=========================================
        search_top = h * 3/4 
        search_bot = h * 3/4 + 20 
        mask[0:search_top, 0:w] = 0 
        mask[search_bot:h, 0:w] = 0
        
        (cx_left, cy_left), (cx_right, cy_right) = centerFromMask(mask, 2/7*w, 5/7*w)
        print("(cx_left, cy_left), (cx_right, cy_right) = ({}, {}), ({}, {}))", cx_left, cy_left, cx_right, cy_right)

        cv2.circle(img, (cx_left, cy_left), 10, (0, 0, 255), -1) # 흰색 이미지의 무게중심을 빨간색 원으로 찍어 표시
        cv2.circle(img, (cx_right, cy_right), 10, (0, 0, 255), -1) # 흰색 이미지의 무게중심을 빨간색 원으로 찍어 표시
        middle = (cx_left + cx_right) / 2
        cv2.circle(img, (middle, cy_left), 10, (0, 255, 0), -1) # 도로의 중앙을 초록색 원으로 표시
        cv2.circle(img, (w/2, cy_left), 10, (255, 0, 0), -1) # 화면의 중앙을 파란색으로 표시
        err = middle - w/2 # 화면의 중앙과 도로의 중앙의 차이를 err에 저장, 이 err에 비례해서 회전 각도 결정

        # # 만약 xycar가 우회전(angle > 0)을 한다면 오른쪽 차선은 인코스에 있고, 좌회전(angle < 0)을 한다면 오른쪽 차선이 아웃코스에 있기 때문에 우회전 할때에 비해서 좌회전 할 때의 err값이 더 작을
        # # 것이기 때문에 우회전 시에는 err을 8로 나눠주고 좌회전 시에는 err을 4로 나눠준다. 
        # if angle >= 0: 
        #     angle = float(err)/8 + wb/165 # 오른쪽 차선이 안보일 경우 가중치를 추가
        #     if angle > 19: # angle이 너무 커지지 않도록 19로 제한
        #         angle = 19
        # else:
        #     angle = float(err)/4 # 좌회전을 할 때는 오른쪽 차선이 아웃코스로 차선이 안보일 경우가 없기 때문에 가중치가 필요 없음
        # print(angle)

        # 위에서 계산한 오차를 이용하여
        # PID 제어를 통해 조향각 계산
        angle = pid_control(err)
        # 조향각이 -50에서 50 사이가 되도록 값 조정
        if angle > 50:
            angle = 50
        elif angle < -50:
            angle = -50

        cv2.imshow("CAM View", img) # 창을 띄워서 이미지 보여주기
        cv2.imshow("mask",mask) # 이미지에서 딴 mask를 창을 띄워서 보여주기
        cv2.waitKey(1) 
		
        # xycar의 default 속도 설정
        speed = 11
		
        # drive() 호출. drive()함수 안에서 모터 토픽이 publish됨.
        drive(angle, speed)


#=============================================
# 메인 함수
# 가장 먼저 호출되는 함수로 여기서 start() 함수를 호출함.
# start() 함수가 실질적인 메인 함수임. 
#=============================================
if __name__ == '__main__':
    start()

