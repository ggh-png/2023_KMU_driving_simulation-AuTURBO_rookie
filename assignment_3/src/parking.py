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
import planning as pl

import draw as draw
import matplotlib.pyplot as plt

import time

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

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
# planning
rx, ry = [], []
ryaw, rdirect, path_x, path_y = [], [], [], []
tracking_start = 1
# tracking

maxTime = 100.0
yaw_old = 0.0
rx, ry = [], []
ryaw, rdirect, path_x, path_y = [], [], [], []
x_rec, y_rec = [], []
x0, y0, yaw0, direct0 = 0.0, 0.0, 0.0, 0.0

cnt = 0
t = 0.0

node = pl.Node(x=0.0, y=0.0, yaw=0.0, v=0.0, direct=0.0)
nodes = pl.Nodes()

cx, cy, cyaw, cdirect = [], [], [], []

ref_trajectory = pl.PATH(cx, cy)
target_ind, _ = 0, 0

flag_for = 1


def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry, ryaw, rdirect, path_x, path_y

    global tracking_start
    print("Start Planning")
    print("sx : ", sx, "sy : ", sy, "syaw : ", syaw, "max_acceleration : ", max_acceleration, "dt : ", dt)
    tracking_start = 1
    # AR = (1142, 62) # AR 태그의 위치
    # P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
    # P_END = (1129, 69) # 주차라인 끝의 좌표

    states = [(sx, sy, syaw+90), (P_ENTRY[0], P_ENTRY[1], -45), (1083, 118, -45)]
    rx, ry, ryaw, rdirect, path_x, path_y = pl.generate_path(states)


    return path_x, path_y


def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    # print("x ", x, "y ", y, "yaw ", yaw)
    global rx, ry, ryaw, rdirect, path_x, path_y
    global tracking_start

    global maxTime
    global yaw_old
    global x_rec, y_rec
    global x0, y0, yaw0, direct0

    global cnt
    global t

    global target_ind, _
    global node, nodes
    global cx, cy, cyaw, cdirect
    global ref_trajectory

    global flag_for

    yaw = np.deg2rad(yaw)

    config = pl.C
    config.dt = dt
    # main 1회만 실행
    if tracking_start:
        tracking_start = 0
        maxTime = 100.0
        yaw_old = 0.0
        x0, y0, yaw0, direct0 = rx[0][0], ry[0][0], ryaw[0][0], rdirect[0][0]
        x_rec, y_rec = [], []
        cnt = 0
        flag_for = 1

        print("1회만 실행")
        
    
    turn_direct = len(list(zip(rx, ry, ryaw, rdirect)))

    if cnt < turn_direct and flag_for:
        flag_for = 0
        t = 0.0
        cx, cy, cyaw, cdirect = rx[cnt], ry[cnt], ryaw[cnt], rdirect[cnt]
        node = pl.Node(x=x, y=y, yaw=-yaw, v=velocity, direct=cdirect)
        nodes = pl.Nodes()
        ref_trajectory = pl.PATH(cx, cy)
        target_ind, _ = ref_trajectory.target_index(node)
        print("새로운 경로 시작 : ", cnt, "번째")
    
    if cnt < turn_direct:
        if cdirect[0] > 0:
            target_speed = 30.0 
            config.Ld = 100.0
            config.dist_stop = 10
            config.dc = -8.4
        else:
            target_speed = 20.0 
            config.Ld = 70
            config.dist_stop = 10
            config.dc = 1.

        xt = node.x + config.dc * math.cos(node.yaw)
        yt = node.y + config.dc * math.sin(node.yaw)
        dist = math.hypot(xt - cx[-1], yt - cy[-1])

        if dist < config.dist_stop:
            cnt += 1
            flag_for = 1
            drive(0, 0)
            if cnt == turn_direct:
                print("마지막 경로 도착")
                drive(0, 0)
                return
            # return
        
        node = pl.Node(x=x, y=y, yaw=-yaw, v=velocity, direct=cdirect[0])
        delta, target_ind = pl.pure_pursuit(node, ref_trajectory, target_ind)
        delta = np.rad2deg(delta)
        if delta > 20:
            delta = 20
        elif delta < -20:
            delta = -20

        t += config.dt

        drive(delta, target_speed*cdirect[0])
        print("delta : ", delta, "target_speed : ", target_speed*cdirect[0])
        nodes.add(t, node)
        x_rec.append(node.x)
        y_rec.append(node.y)

        dy = (node.yaw - yaw_old) / (node.v * config.dt)
        steer = pl.rs.pi_2_pi(-math.atan(config.WB * dy))
        yaw_old = node.yaw
        x0 = nodes.x[-1]
        y0 = nodes.y[-1]
        yaw0 = nodes.yaw[-1]
        direct0 = nodes.direct[-1]
        # animation
        
        plt.cla()
        plt.plot(node.x, node.y, marker='.', color='k')
        plt.plot(path_x, path_y, color='gray', linewidth=2)
        plt.plot(x_rec, y_rec, color='darkviolet', linewidth=2)
        plt.plot(cx[target_ind], cy[target_ind], ".b")
        draw.draw_car(node.x, node.y, yaw_old, steer, config)

        # for m in range(len(states)):
        #     draw.Arrow(states[m][0], states[m][1], np.deg2rad(states[m][2]), 2, 'blue')

        plt.axis("equal")
        plt.title("PurePursuit: v=" + str(node.v * 3.6)[:4] + "km/h")
        plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event:
                                        [exit(0) if event.key == 'escape' else None])
        plt.pause(0.001)

plt.show()