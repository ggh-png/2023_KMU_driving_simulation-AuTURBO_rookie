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
import motion_planning as pl

import matplotlib.pyplot as plt


#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

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


# planning
#=============================================
# -- planning 함수에서 사용할 전역변수들 --
# 경로 및 주행 정보를 저장할 변수들 (경로의 좌표와, 해당 경로의 주행 방향-전진/후진-을 저장)
# 2차원 배열로 저장하며, 각각의 배열은 각각의 경로를 의미한다.
# path_x, path_y, rdirect는 각각 경로의 x좌표, y좌표, 주행 방향을 의미한다.
# -- 이게 맞는지 모르겠음 --
# rx, rt는 motion planning에서 waypoint를 탐색할 때 사용하는 변수이다. 
# 예를 들어, path_x[0]은 첫번째 경로의 x좌표를 의미하고, path_y[0]은 첫번째 경로의 y좌표를 의미한다.
# rdirect[0]은 첫번째 경로의 주행 방향을 의미한다.
#============================================= 

# -- planning 함수에서 전역 변수로 사용하는 이유 --
# planning 함수에서 생성한 경로를 tracking 함수에서 사용해야 하기 때문에 전역변수로 선언한다.
rx, ry, ryaw, rdirect, path_x, path_y = [], [], [], [], [], []
# tracking 함수를 시작할 때 1회만 실행되는 flag, planning 함수가 실행될 때 1로 초기화된다.
# 마찬가지로 전역변수로 선언한다.
tracking_start = 1


#=============================================
# 경로를 생성하는 함수 - path_planning.py 에서 사용
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
# 주어진 정보(차량의 시작위치, 단위시간)를 바탕으로 경로를 생성한다.
# 관계자의 문의를 통해 휠 베이스(WB)와 차량의 너비(W)를 알 수 있었고, 휠 베이스(WB)를 path planning 알고리즘에 적용하여 회전 반경을 계산하였다.
# 경로 생성에 사용된 path planning 알고리즘은 reeds_shepp 알고리즘을 사용하였다.
# path planning 함수는 motion_planning.py에 정의되어 있으며, 자세한 설명은 path_planning.py를 참고한다.
#=============================================-

def planning(sx, sy, syaw, max_acceleration, dt):
    # planning 함수에서 생성한 경로를 tracking 함수에서 사용해야 하기 때문에 전역변수로 선언한다.
    global rx, ry, ryaw, rdirect, path_x, path_y
    # tracking 함수를 시작할 때 1회만 실행되는 flag, planning 함수가 실행될 때 1로 초기화된다.
    # 마찬가지로 전역변수로 선언한다.
    global tracking_start
    print("경로를 생성합니다.")
    # print("sx : ", sx, "sy : ", sy, "syaw : ", syaw, "max_acceleration : ", max_acceleration, "dt : ", dt)
    tracking_start = 1
    # AR = (1142, 62) # AR 태그의 위치
    # P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
    # P_END = (1129, 69) # 주차라인 끝의 좌표

    # 경로 좌표를 저장할 리스트
    # 초기 각도에 90도를 더해주는 이유는, 주어진 차량의 초기 각도와 path planning 알고리즘에서 사용하는 각도의 기준이 90 정도의 틀어져 있기 때문에 90도를 더해주었다.
    # 좀 더 정확한 경로를 생성하기 위해 아래와 같이 3개의 단계로 나누었다. 
    # 1. 주자 라인 진입 100 픽셀 전에 경로를 생성한다.
    # 2. AR 태그의 위치에서 60 픽셀 전에 경로를 생성한다.
    states = [(sx, sy, syaw+90), (P_ENTRY[0]-100, P_ENTRY[1]+100, -45), (AR[0]-57, AR[1]+57, -45)]
    # path planning 알고리즘을 사용하여 경로를 생성한다. reeds_shepp path planning algorithm 사용 
    # motion_planning.py 에서 제공하는 함수를 사용한다.
    # reeds_shepp 알고리즘의 workflow는 다음과 같다.
    # -- path planning workflow --
    # 1. 차량의 초기 위치와 목표 위치를 입력받는다.
    # 2. 차량의 초기 위치와 목표 위치를 바탕으로 차량의 초기 각도를 계산한다.
    # 3. 차량의 초기 각도와 휠 베이스, 최대 조향각을 바탕으로 차량의 회전 반경을 계산한다.
    # 4. 차량의 초기 위치, 목표 위치, 차량의 회전 반경을 바탕으로 경로를 생성한다.
    # --------------------------------
    # 자세한 내용은 path_planning.py를 참고한다.
    rx, ry, ryaw, rdirect, path_x, path_y = pl.generate_path(states)

    # 생성된 경로를 시각화 하기 위해 생성된 경로를 반환한다.
    return path_x, path_y

# tracking
#=================================================================================================
# -- tracking 함수에서 전역 변수로 사용하는 이유 --
# dt마다 실행되는 tracking 함수에선 while문과 for문을 사용하게 되면 무한 루프와 같은 현상이 발생한다.
# 이를 방지하기 위해 while문과 for문을 사용하지 않고, if문을 사용하여 while문과 for문을 대체하였다.
# if문을 사용하여 while문과 for문을 대체하였기 때문에, 블록이 달랐고, 이에 따라서 전역변수로 선언하였다. 
# 전역변수로 선언한 변수는 다음과 같다.

# yaw_old = 0.0

# 차량의 주행 했던 경로를 저장하는 리스트 - 디버깅시 사용
x_rec, y_rec = [], []

# path planning 알고리즘에서 생성된 리스트는 2차원 리스트로 생성되어 있기 때문에 - 방향이 바뀔때 마다 다른 경로를 생성하기 때문에 여러개의 경로가 저장 되기 때문
# 1차원 리스트로 변환하여 사용해야 한다.
# 1차원 리스트로 변환하기 위해 사용되는 리스트 - cx(경로의 x좌표), cy(경로의 y좌표), cyaw(경로의 yaw), cdirect(경로의 전 후진 방향)
cx, cy, cyaw, cdirect = [], [], [], []

# n번째 경로를 저장하기 위한 cnt 변수 
cnt = 0

# 차량의 현재 정보를 저장하는 클래스 - x, y, yaw, v(속도), direct(전 후진 방향)
node = pl.Node(x=0.0, y=0.0, yaw=0.0, v=0.0, direct=0.0)
# 차량의 현재 정보를 저장하는 클래스를 저장하는 리스트
nodes = pl.Nodes()

# 차량이 제어되는 시간을 저장하는 변수
t = 0.0

# 경로 추적을 위해 PATH 클래스의 인스턴스를 생성한다.
# 이 클래스는 경로의 좌표와 목표 인덱스를 계산하는 기능을 제공한다.
ref_trajectory = pl.PATH(cx, cy)
# 목표 인덱스를 저장하는 변수
target_ind, _ = 0, 0

# n번째 경로를 시작할 때 1회만 실행 되로록 하기 위한 flag
flag_for = 1

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
# 위 planning 함수에서 생성된 경로를 따라가는 함수이다.
# tracking 즉 motion planning을 구현하기 위해 사용된 알고리즘은 Pure Pursuit 알고리즘을 사용하였다.
# Pure Pursuit 알고리즘은 차량의 현재 위치에서 경로상의 목표 지점을 계산하고, 목표 지점을 따라가는 알고리즘이다.
# 관계자의 문의를 통해 휠 베이스(WB)와 차량의 너비(W)를 알 수 있었고, 휠 베이스(WB)를 통해 차량의 크기를 비례하여 구했고 이를 통해 목표 지점을 계산하였다.
# 자세한 내용은 motion_planning.py를 참고한다.
#=============================================

def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    # print("x ", x, "y ", y, "yaw ", yaw)
    # planning 함수에서 생성한 경로를 tracking 함수에서 사용해야 하기 때문에 전역변수로 선언한다.
    global rx, ry, ryaw, rdirect, path_x, path_y
    # tracking 함수를 시작할 때 1회만 실행되는 flag
    global tracking_start

    # 차량이 지나온 경로를 저장하는 리스트 - 디버깅시 사용
    global x_rec, y_rec
    
    # 차량이 제어되는 시간을 저장하는 변수
    global t
    
    # 경로 추적을 위해 PATH 클래스의 인스턴스를 생성한다.
    # 이 클래스는 경로의 좌표와 목표 인덱스를 계산하는 기능을 제공한다.
    global ref_trajectory
  
    # 목표 인덱스를 저장하는 변수
    global target_ind, _
    
    # 차량의 현재 정보를 저장하는 클래스 - x, y, yaw, v(속도), direct(전 후진 방향)
    # 차량의 현재 정보를 저장하는 클래스를 저장하는 리스트
    global node, nodes

    # path planning 알고리즘에서 생성된 리스트는 2차원 리스트로 생성되어 있기 때문에 - 방향이 바뀔때 마다 다른 경로를 생성하기 때문에 여러개의 경로가 저장 되기 때문
    # 1차원 리스트로 변환하여 사용해야 한다.
    # 1차원 리스트로 변환하기 위해 사용되는 리스트 - cx(경로의 x좌표), cy(경로의 y좌표), cyaw(경로의 yaw), cdirect(경로의 전 후진 방향)
    global cx, cy, cyaw, cdirect
    # n번째 경로를 저장하기 위한 cnt 변수 
    global cnt
   
    # n번째 경로를 시작할 때 1회만 실행 되로록 하기 위한 flag
    global flag_for

    # 차량의 현재 각도를 radian으로 변환한다.- Pure Pursuit 알고리즘에서 radian을 사용하기 때문에
    # yaw에 -1을 곱하는 이유는 알고리즘에서 사용되는 좌표계와 맞추기 위함이다.
    yaw = np.deg2rad(-yaw)

    # 차량의 제어 알고리즘에서 사용될 parameter를 저장하는 클래스를 생성한다.
    config = pl.C
    # 차량의 제어주기를 설정한다.
    config.dt = dt
    # 차량의 휠 베이스를 설정한다.
    config.WB = 84
    # 차량의 너비를 설정한다.
    config.W = 64

    # tracking 함수가 실행되면 1회만 실행되도록 하는 flag
    if tracking_start:
        # flag off - 1회만 실행
        tracking_start = 0
        # 차량의 주행기록 리스트를 초기화한다.
        x_rec, y_rec = [], []
        # 0번째 경로를 시작 하도록 cnt를 0으로 초기화한다.
        cnt = 0
        # N번째 경로를 시작할 때 1회만 실행 되도록 하기 위한 flag
        flag_for = 1
        print("tracking을 시작합니다.")
        
    # 2차원 리스트의 개수를 구한다.
    # 2차원 리스트의 개수는 생성된 경로의 개수와 같다.
    turn_direct = len(list(zip(rx, ry, ryaw, rdirect)))

    # 생성된 경로의 개수인 turn_direct 보다 cnt가 크면 실행되는 if문 - 경로가 끝났을 때
    # n번째 경로가 정해지고 1회만 실행되도록 하기 위한 flag
    # 위 두가지 조건을 만족하면 실행된다.
    if cnt < turn_direct and flag_for:
        # flag off - n번쨰 경로당 1회만 실행
        flag_for = 0
        # 차량의 제어시간을 초기화한다.
        t = 0.0
        # 2차원 리스트의 n번째 경로를 1차원 리스트로 변환한다.
        cx, cy, cyaw, cdirect = rx[cnt], ry[cnt], ryaw[cnt], rdirect[cnt]
        # 차량의 현재 정보를 저장하는 클래스를 생성하고 현재 차량의 정보를 저장한다.
        node = pl.Node(x=x, y=y, yaw=-yaw, v=velocity, direct=cdirect)
        # 차량의 현재 정보를 저장하는 클래스를 저장하는 리스트를 생성하고 초기화한다.
        nodes = pl.Nodes()
        # 경로 추적을 위해 PATH 클래스의 인스턴스를 생성한다.
        ref_trajectory = pl.PATH(cx, cy)
        # 목표 인덱스를 저장한다.
        # work flow는 아래와 같다.
        # -- motion planning target index work flow --
        # 1. 차량의 현재 위치를 기준으로 가장 가까운 경로의 좌표를 찾는다.
        # 2. 찾은 경로좌표의 인덱스를 찾는다.
        # 3. 찾은 인덱스를 기준으로 목표 인덱스를 찾는다.
        # 4. 목표 인덱스를 저장한다.
        # --------------------------------------------
        target_ind, _ = ref_trajectory.target_index(node)
        print(cnt, " 번째 경로를 시작합니다.")

    # 생성된 경로의 개수인 turn_direct 보다 cnt가 크면 실행되는 if문 - 경로가 끝났을 때
    if cnt < turn_direct:
        # n번째 경로의 방향 리스트의 첫번째 값이 1이면 전진
        if cdirect[0] > 0:
            # 차량제어의 목표 속도를 설정한다.
            target_speed = 50.0 
            # 차량의 제어 알고리즘에서 사용될 parameter를 설정한다.
            # 전방 주행 차량이나 추종하고자 하는 경로와의 거리를 설정한다. 
            config.Ld = 120.0
            # 차량의 정지거리
            config.dist_stop = 10
            # 전방 주행 idx와의 거리
            config.dc = -8.4
        # n번째 경로의 방향 리스트의 첫번째 값이 -1이면 후진
        else:
            # 차량제어의 목표 속도를 설정한다.
            target_speed = 30.0 
            # 차량의 제어 알고리즘에서 사용될 parameter를 설정한다.
            # 전방 주행 차량이나 추종하고자 하는 경로와의 거리를 설정한다.
            config.Ld = 70
            # 차량의 정지거리
            config.dist_stop = 10
            # 전방 주행 idx와의 거리
            config.dc = 1.68

        # 현재 지점에서 목표 지점까지의 거리를 계산한다.
        xt = node.x + config.dc * math.cos(node.yaw)
        yt = node.y + config.dc * math.sin(node.yaw)
        dist = math.hypot(xt - cx[-1], yt - cy[-1])

        # 남은 거리가 정지거리보다 작으면 정지한다.
        if dist < config.dist_stop:
            # 다음 경로로 넘어가기 위해 cnt를 1 증가시킨다.
            cnt += 1
            # 다음 경로에서 1회만 실행되도록 flag를 1로 설정한다.
            flag_for = 1
            # 차량을 정지시킨다.
            drive(0, 0)
            # 마지막 경로에 도착했을 때 도착했음을 알리고 차량을 정지시킨다.
            if cnt == turn_direct:
                print("마지막 경로 도착")
                drive(0, 0)
                return
       
        # 차량의 현재 정보를 저장한다.  
        node = pl.Node(x=x, y=y, yaw=yaw, v=velocity, direct=cdirect[0])
        # 현재 차량 정보와 추종 경로 그리고 목표 인덱스를 이용하여 차량의 조향각을 계산한다.
        # 조향각을 계산하고, 목표 인덱스를 반환한다.
        # 알고리즘은 pure pursuit을 사용하였고 알고리즘의 work flow는 아래와 같다.
        # -- pure pursuit work flow --
        # 1. 차량의 현재 위치를 기준으로 가장 가까운 경로의 좌표를 찾는다.
        # 2. 찾은 경로좌표의 인덱스를 찾는다.
        # 3. 찾은 인덱스를 기준으로 목표 인덱스를 찾는다.
        # 4. 목표 인덱스를 기준으로 조향각을 계산한다.
        # 5. 계산된 조향각을 반환한다.
        # --------------------------------------------
        # 자세한 알고리즘은 pure_pursuit.py를 참고한다.
        delta, target_ind = pl.pure_pursuit(node, ref_trajectory, target_ind)
        # 차량을 제어하기 위해 조향각을 degree로 변환한다.
        delta = np.rad2deg(delta)
        # 조향각의 범위를 -20 ~ 20도로 제한한다. - 차량의 조향각은 -20 ~ 20도로 제한되어 있다.
        if delta > 20:
            delta = 20
        elif delta < -20:
            delta = -20

        # 제어 시간을 업데이트한다.
        t += config.dt
        # 차량을 제어한다. - 차량의 조향각과 목표 속도를 입력한다.
        # target_ind*cdirect[0] - 목표 인덱스를 기준으로 차량의 방향을 결정한다.
        drive(delta, target_ind*cdirect[0])
        # 차량의 현재 정보를 리스트에 추가한다.
        nodes.add(t, node)
        # 차량의 지나온 경로를 리스트에 추가한다. - 디버깅용
        x_rec.append(node.x)
        y_rec.append(node.y)


#         #=================== 그래프 그리기 ===================#
#         # graph를 그린다.
#         plt.cla()
#         # 차량의 현 위치를 그린다.
#         plt.plot(node.x, node.y, marker='.', color='k')
#         # 경로를 그린다.
#         plt.plot(path_x, path_y, color='gray', linewidth=2)
#         # 차량의 지나온 경로를 그린다. - 디버깅용
#         plt.plot(x_rec, y_rec, color='darkviolet', linewidth=2)
#         # 추종 waypoint를 그린다.
#         plt.plot(cx[target_ind], cy[target_ind], ".b")

#         plt.axis("equal")
#         plt.title("PurePursuit: v=" + str(node.v )[:4] + "km/h")
#         # 키 입력시 종료
#         plt.gcf().canvas.mpl_connect('key_release_event',
#                                         lambda event:
#                                         [exit(0) if event.key == 'escape' else None])
#         # 그래프 업데이트 주기
#         plt.pause(0.001)

# plt.show()