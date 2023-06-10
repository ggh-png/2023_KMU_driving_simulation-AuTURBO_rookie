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
rx, ry = [], []
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
# parameter
MAX_T = 100.0  # maximum time to the goal [s]
MIN_T = 5.0  # minimum time to the goal[s]

#=============================================

# 5차 다항식을 이용하여 경로를 생성하는 함수
class QuinticPolynomial:
    # 생성자 
    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        # 계수를 저장할 변수 선언
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0
        # 5차 다항식의 계수를 계산
        A = np.array([[time ** 3, time ** 4, time ** 5],
                      [3 * time ** 2, 4 * time ** 3, 5 * time ** 4],
                      [6 * time, 12 * time ** 2, 20 * time ** 3]])
        b = np.array([xe - self.a0 - self.a1 * time - self.a2 * time ** 2,
                      vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)
        # 계수를 저장
        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5
               
        return xt
    
    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

        return xt


def quintic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt):
    """
    quintic polynomial planner

    input
        s_x: start x position [m]
        s_y: start y position [m]
        s_yaw: start yaw angle [rad]
        sa: start accel [m/ss]
        gx: goal x position [m]
        gy: goal y position [m]
        gyaw: goal yaw angle [rad]
        ga: goal accel [m/ss]
        max_accel: maximum accel [m/ss]
        max_jerk: maximum jerk [m/sss]
        dt: time tick [s]

    return
        time: time result
        rx: x position result list
        ry: y position result list
        ryaw: yaw angle result list
        rv: velocity result list
        ra: accel result list

    """

    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

    for T in np.arange(MIN_T, MAX_T, MIN_T):
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T)
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)

        time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

        for t in np.arange(0.0, T + dt, dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))

            vx = xqp.calc_first_derivative(t)
            vy = yqp.calc_first_derivative(t)
            v = np.hypot(vx, vy)
            yaw = math.atan2(vy, vx)
            rv.append(v)
            ryaw.append(yaw)

            ax = xqp.calc_second_derivative(t)
            ay = yqp.calc_second_derivative(t)
            a = np.hypot(ax, ay)
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a *= -1
            ra.append(a)

            jx = xqp.calc_third_derivative(t)
            jy = yqp.calc_third_derivative(t)
            j = np.hypot(jx, jy)
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j)

        if max([abs(i) for i in ra]) <= max_accel and max([abs(i) for i in rj]) <= max_jerk:
            print("find path!!")
            break

   
    return rx, ry



#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================


# Parameters
k = 0.1  # look forward gain
Lfc = 2.  # [m] look-ahead distance

Kp = 1.  # speed proportional gain
Ki = 0.00  # speed integral gain
Kd = 0.01  # speed derivative gain

# dt = 0.1  # [s] time tick
WB = 20 # [m] wheel base of vehicle


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        self.dt = dt

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.v / WB * math.tan(delta) * self.dt
        self.v += a * self.dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current) + Ki * target + Kd * (target - current)

    return a


class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

# pure pursuit path tracking simulation
def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)
    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
    
    return delta, ind


# def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
#     """
#     Plot arrow
#     """

#     if not isinstance(x, float):
#         for ix, iy, iyaw in zip(x, y, yaw):
#             plot_arrow(ix, iy, iyaw)
#     else:
#         plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
#                   fc=fc, ec=ec, head_width=width, head_length=width)
#         plt.plot(x, y)



#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
# sx: 시작 위치의 x 좌표 [미터]
# sy: 시작 위치의 y 좌표 [미터]
# syaw: 시작 상태의 선회각 (yaw angle) [라디안]
# sv: 시작 속도 [미터/초]
# sa: 시작 가속도 [미터/초^2]
# gx: 목표 위치의 x 좌표 [미터]
# gy: 목표 위치의 y 좌표 [미터]
# gyaw: 목표 상태의 선회각 (yaw angle) [라디안]
# gv: 목표 속도 [미터/초]
# ga: 목표 가속도 [미터/초^2]
# max_accel: 최대 가속도 [미터/초^2]
# max_jerk: 최대 제동도 [미터/초^3]
# dt: 시간 간격 [초]


target_course = TargetCourse(rx, ry)
time_cnt = 0.0
# target_course = TargetCourse(rx, ry)
# initial state
states = States()
target_ind = 0
state = State()


def planning(sx, sy, syaw, max_acceleration, dt):
    global rx, ry
    global target_course
    global state
    global states
    global time_cnt
    print("acc", max_acceleration)
    #=============================================
    # 경로를 생성하는 함수
    # 차량의 시작위치 sx, sy, 시작각도 syaw
    # 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
    # 경로를 리스트를 생성하여 반환한다.
    #=============================================
    # sx: 시작 위치의 x 좌표 [미터]
    # sy: 시작 위치의 y 좌표 [미터]
    # syaw: 시작 상태의 선회각 (yaw angle) [라디안]
    # sv: 시작 속도 [미터/초]
    # sa: 시작 가속도 [미터/초^2]
    # gx: 목표 위치의 x 좌표 [미터]
    # gy: 목표 위치의 y 좌표 [미터]
    # gyaw: 목표 상태의 선회각 (yaw angle) [라디안]
    # gv: 목표 속도 [미터/초]
    # ga: 목표 가속도 [미터/초^2]
    # max_accel: 최대 가속도 [미터/초^2]
    # max_jerk: 최대 제동도 [미터/초^3]
    # dt: 시간 간격 [초]   

    print("Start Planning")
    print("sx: ", sx, "sy: ", sy, "syaw: ", syaw)
    syaw = np.deg2rad(syaw+90)  # start yaw angle [rad]
    sv = 8.5  # start speed [m/s]
    sa = 0.1 # start accel [m/ss]
    gx = P_END[0]  # goal x position [m]
    gy = P_END[1]  # goal y position [m]
    gyaw = np.deg2rad(-45)  # goal yaw angle [rad]
    gv = 4.5  # goal speed [m/s]
    ga = 0.1  # goal accel [m/ss]
    max_accel = 1.0  # max accel [m/ss]
    max_jerk = 10.5  # max jerk [m/sss]


    # rx, ry = [], []
    # state = State()
    time_cnt = 0.0
    state = State(sx, sy, np.deg2rad(syaw), 20, dt)
    rx, ry = quintic_polynomials_planner(
        sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, max_jerk, dt)
    
    # tracking
    target_course = TargetCourse(rx, ry)
    
    return rx, ry
    
    



def tracking(screen, x, y, yaw, velocity, max_acceleration, dt):
    print("x: ", x, "y: ", y, "yaw: ", yaw, "velocity: ", velocity, "max_acceleration: ", max_acceleration, "dt: ", dt)
    global rx, ry

    global target_course
    global states
    global time_cnt
    global target_ind

    target_speed = 50  # -50 ~ 50
    state = State(x, y, np.deg2rad(yaw), velocity, dt)

    # Calc control input
    ai = proportional_control(target_speed, state.v)
    di, target_ind = pure_pursuit_steer_control(state, target_course, target_ind)
    state.update(ai, di)  # Control vehicle
    # print("ai: ", ai, "di: ", di)
    
    drive(np.clip(np.rad2deg(di), -20, 20), ai)
    time_cnt += dt
    states.append(time_cnt, state)
    
    # 1. 현재위치와 각도를 전달받아서
    # 2. 경로를 생성하고
    # 3. 경로를 따라가도록 각도와 속도를 결정한다.
    # 4. 각도와 속도를 결정하여 전달한다.
    # 5. 전달받은 각도와 속도를 이용하여 주행한다.
    # 6. 1로 돌아간다.