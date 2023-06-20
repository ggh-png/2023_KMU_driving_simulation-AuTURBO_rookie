"""
Pure Pursuit
author: huiming zhou
"""

import os
import sys
import math
import numpy as np

# reeds_shepp path planning algorithm 라이브러리 
import path_planning as rs

# 알고리즘에 사용하는 상수들을 모아놓은 클래스
class C:
    # PID 파라미터
    Kp = 0.8  # proportional gain
    Ki = 0.0  # Integral gain
    Kd = 0.01  # derivative gain
    
    # 차량의 제어 알고리즘에서 사용될 parameter를 설정한다.
    # 전방 주행 차량이나 추종하고자 하는 경로와의 거리를 설정한다. 
    Ld = 2.6 
    # Lf = Kf * v + Ld
    # Lf : 경로를 따라가며 추적할 목표 지점과의 거리
    kf = 0.1  # 목표 지점과의 거리를 계산할 때 사용되는 상수
    dt = 0.1  # 제어 주기
    dist_stop = 0.7  # 정지 거리
    # 전방 주행 idx와의 거리
    dc = 0.0

    # 차량 파라미터
    RF = 3.3 * 33  # [m] 차량의 전방에서 차량의 전면까지의 거리
    RB = 0.8 * 33 # [m] 차량의 전방에서 차량의 후면까지의 거리
    W = 2.4 * 33  # [m] 차량의 너비
    WD = 0.7 * W  # [m] 차량의 후륜의 너비
    WB = 84  # [m] 차량의 축 거리 - 전륜과 후륜의 중간(휠 베이스)
    TR = 0.44 * 33  # [m] 차량의 타이어 반지름
    TW = 0.7 * 33  # [m] 차량의 타이어 너비
    MAX_STEER = 0.3491 * 0.75 # [rad] 최대 조향각 [약 20deg]
    MAX_ACCELERATION = 5.0 # [m/s^2] 최대 가속도

# 차량의 상태를 저장하는 클래스
class Node:
    # 차량의 위치와 각도, 속도, 진행 방향을 저장한다.
    def __init__(self, x, y, yaw, v, direct):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct
    # 주어진 속도와 방향, 그리고 조향각을 이용하여 차량의 상태를 업데이트하는 함수.
    # 단 이번 미션에서 현재 차량을 상태를 dt마다 제공하기에 이 함수는 사용하지 않는다.
    def update(self, a, delta, direct):
        # delta = self.limit_input(delta)
        self.x += self.v * math.cos(self.yaw) * C.dt
        self.y += self.v * math.sin(self.yaw) * C.dt
        self.yaw += self.v / C.WB * math.tan(delta) * C.dt
        self.direct = direct
        self.v = self.direct * a 

    @staticmethod
    # 조향각을 제한한다. 마찬가지로 이번 미션에서는 사용하지 않는다.
    def limit_input(delta):
        if delta > 1.2 * C.MAX_STEER:
            return 1.2 * C.MAX_STEER

        if delta < -1.2 * C.MAX_STEER:
            return -1.2 * C.MAX_STEER

        return delta

# 차량의 상태의 클래스를 리스트로 저장하는 클래스
class Nodes:
    def __init__(self):
        # 차량의 위치와 각도, 속도, 진행 방향, 시간을 리스트로 저장한다.
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.direct = []
    # 현재 차량의 상태를 받아오면 리스트에 추가하는 함수.
    def add(self, t, node):
        self.x.append(node.x)
        self.y.append(node.y)
        self.yaw.append(node.yaw)
        self.v.append(node.v)
        self.t.append(t)
        self.direct.append(node.direct)

#경로를 저장하는 클래스
class PATH:
    # x, y 경로 리스트와 경로의 길이를 저장한다.
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        # 경로의 길이를 저장한다.
        self.ind_end = len(self.cx) - 1
        # 이전 목표로 설정한 경로의 인덱스를 저장한다.
        self.index_old = None

    # 목표 인덱스를 설정하는 함수
    # 현재 차량의 위치와 가장 가까운 경로의 인덱스를 찾는다.
    def target_index(self, node):
        # target index 함수의 work flow는 아래와 같다.
        # -- target index work flow --
        # 1. 현재 차량의 위치와 가장 가까운 경로의 인덱스를 찾는다.
        # 2. 현재 차량의 속도와 경로의 인덱스를 이용하여 경로를 따라가며 추적할 목표 지점과의 거리를 계산한다.
        # 3. 목표 지점과의 거리가 Lf보다 크면 다음 경로의 인덱스를 찾는다.
        # 4. 목표 지점과의 거리가 Lf보다 작으면 해당 경로의 인덱스를 반환한다.
        # ---------------------------
        
        # 목표로 해둔 이전 경로의 인덱스가 없으면 현재 차량의 위치와 가장 가까운 경로의 인덱스를 찾는다.
        if self.index_old is None:
            # 현재 차량의 위치와 가장 가까운 경로의 인덱스를 찾는 함수 
            # node : 현재 차량의 상태
            self.calc_nearest_ind(node)
        
        # 경로를 따라가며 추적할 목표 지점과의 거리 계산
        # Lf : kf(상수 게인값) * 차량의 속도 + Ld(전방 주시거리)
        Lf = C.kf * node.v + C.Ld

        # 이전 경로 인덱스와 경로 끝 인덱스까지의 고려하여 다음 경로의 인덱스를 찾는다.
        for ind in range(self.index_old, self.ind_end + 1):
            # 목표 지점과의 거리가 Lf보다 크면 다음 경로의 인덱스를 찾는다.
            if self.calc_distance(node, ind) > Lf:
                # 다음 경로의 인덱스를 찾았으면 해당 경로의 인덱스를 반환한다.
                self.index_old = ind
                # 다음 경로의 인덱스와 목표 지점과의 거리를 반환한다.
                return ind, Lf
        # 경로 끝까지 도달했으면 경로 끝 인덱스를 반환한다.
        self.index_old = self.ind_end
        return self.ind_end, Lf

    # 현재 차량의 위치와 가장 가까운 경로의 인덱스를 찾는 함수
    def calc_nearest_ind(self, node):
        # 이 함수의 work flow는 아래와 같다.
        # -- calc nearest index work flow --
        # 1. 현재 차량의 위치와 경로의 모든 점과의 거리를 계산한다.
        # 2. 가장 가까운 거리를 찾는다.
        # 3. 가장 가까운 거리의 인덱스를 반환한다.
        # ----------------------------------
        # 현재 차량의 위치와 경로의 모든 점과의 거리를 계산한다.
        dx = [node.x - x for x in self.cx]
        dy = [node.y - y for y in self.cy]
        # 가장 가까운 거리의 인덱스를 찾는다.
        ind = np.argmin(np.hypot(dx, dy))
        # 가장 가까운 거리의 인덱스를 반환한다.
        self.index_old = ind
    # 현재 차량의 위치와 경로의 인덱스를 이용하여 경로를 따라가며 추적할 목표 지점과의 거리를 계산하는 함수
    def calc_distance(self, node, ind):
        return math.hypot(node.x - self.cx[ind], node.y - self.cy[ind])

# pure pursuit 알고리즘을 이용하여 차량의 조향각을 계산하는 함수
def pure_pursuit(node, ref_path, index_old):
    # pure pursuit 알고리즘의 work flow는 아래와 같다.
    # -- pure pursuit work flow --
    # 1. 목표 인덱스를 설정한다.
    # 2. 목표 인덱스를 이용하여 목표 지점의 x, y 좌표를 구한다.
    # 3. 목표 지점과의 각도를 계산한다.
    # 4. 목표 지점과의 각도를 이용하여 조향각을 계산한다.
    # 5. 조향각과 목표 인덱스를 반환한다.
    # ----------------------------
    
    # 목표 인덱스와 목표 지점과의 거리를 계산한다.
    # 자세한 설명은 위의 target_index 함수를 참고하면 된다.
    ind, Lf = ref_path.target_index(node)  
    # 목표 인덱스와 이전의 목표 인덱스를 비교하여 큰 값을 선택한다.
    ind = max(ind, index_old)

    # 목표 인덱스를 이용하여 목표 지점의 x, y 좌표를 구한다.
    tx = ref_path.cx[ind]
    ty = ref_path.cy[ind]
    # 목표 지점과의 각도를 계산한다. - 현재 차량의 위치와 목표 지점의 각도 차이(오차 각도)
    alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw
    # 목표 지점과의 각도를 이용하여 조향각을 계산한다.
    # delta = atan2(2 * L * sin(alpha) / Lf)
    delta = math.atan2(2.0 * C.WB * math.sin(alpha), Lf)
    # 조향각과 목표 인덱스를 반환한다.
    return delta, ind

# 차량의 속도를 pid 제어를 이용하여 계산하는 함수
# 단 이번미션에서는 시뮬레이션 환경이기에 외부 환경의 영향을 받지 않기 때문에 pid 제어를 이용하지 않는다.
def pid_control(target_v, v, dist, direct):
    # pid 제어
    a = (C.Kp * (target_v - v) + C.Kd * (target_v - v) + C.Ki * (target_v - v)) * direct
    # 목표 지점과의 거리가 10m 이내이고 차량의 속도가 3m/s 이상이면 감속
    if dist < 10.0:
        if v > 3.0:
            a = -2.5
        elif v < -2.0:
            a = -1.0

    return a

# 경로를 생성하는 함수
# reeds_shepp 알고리즘을 이용하여 경로를 생성한다.
# reeds_shepp 알고리즘은 차량의 최소 회전 반경을 고려하여 경로를 생성하는 알고리즘이다.
# 차량의 최소 회전 반경은 차량의 최대 조향각을 고려하여 계산한다.
def generate_path(s):
    # reeds_shepp 알고리즘을 이용하여 경로를 생성한다.
    # reeds_shepp 알고리즘은 차량의 최소 회전 반경을 고려하여 경로를 생성하는 알고리즘이다.
    # 경로 생성에 필요한 정보는 차량의 최대 조향각, 차량의 바퀴 베이스, 차량의 최대 속도, 경로의 시작점, 경로의 끝점이다.
    # 차량의 최소 회전 반경을 차량의 최대 조향각을 고려하여 계산하여 경로를 생성한다.
    # 위에서 구한 정보를 이용하여 경로를 생성한다.
   
    # path_x, path_y : 경로의 x, y 좌표
    # x_all, y_all : 모든 경로의 x, y 좌표 
    # direct : 경로의 방향
    # yaw : 경로의 각도
    # 위 정보 리스트를 반환한다. 
 
    # 차량의 최대 조향각을 고려하여 차량의 최대 회전 반경을 계산한다.
    max_c = math.tan(C.MAX_STEER) / C.WB  
    # 경로 생성에 필요한 정보 리스트를 생성한다.
    path_x, path_y, yaw, direct = [], [], [], []
    
    # 경로 새성에 필요한 정보를 저장하는 임시 리스트를 생성한다.
    x_rec, y_rec, yaw_rec, direct_rec = [], [], [], []

    # flag를 설정한다. - 경로의 방향을 설정하기 위한 flag
    direct_flag = 1.0

    # 경로의 시작점과 끝점을 설정한다.
    for i in range(len(s) - 1):
        # 시작점과 끝점의 x, y 좌표, 각도를 설정한다.
        s_x, s_y, s_yaw = s[i][0], s[i][1], np.deg2rad(s[i][2])
        g_x, g_y, g_yaw = s[i + 1][0], s[i + 1][1], np.deg2rad(s[i + 1][2])

        # 시작점과 끝점의 x, y 좌표, 각도, 차량의 최대 회전 반경을 이용하여 경로를 생성한다.
        # 자세한 내용은 path_planning.py을 참조한다.
        path_i = rs.calc_optimal_path(s_x, s_y, s_yaw,
                                      g_x, g_y, g_yaw, max_c)
        # 경로의 x, y 좌표, 각도, 방향을 저장한다.
        ix = path_i.x
        iy = path_i.y
        iyaw = path_i.yaw
        idirect = path_i.directions
        # 경로의 x, y 좌표, 각도, 방향을 저장한다.
        for j in range(len(ix)):
            # 이전 경로의 방향과 현재 경로의 방향이 같으면 경로의 x, y 좌표, 각도, 방향을 리스트에 추가한다.
            if idirect[j] == direct_flag:
                # 경로의 x, y 좌표, 각도, 방향을 리스트에 추가한다.
                x_rec.append(ix[j])
                y_rec.append(iy[j])
                yaw_rec.append(iyaw[j])
                direct_rec.append(idirect[j])
            # 이전 경로의 방향과 현재 경로의 방향이 다르면 경로 리스트를 2차원 리스트에 추가하고
            # 경로의 x, y 좌표, 각도, 방향을 저장하는 임시 리스트를 초기화한다.
            else:
                
                if len(x_rec) == 0 or direct_rec[0] != direct_flag:
                    direct_flag = idirect[j]
                    continue
                # 경로 리스트를 2차원 리스트에 추가한다.
                path_x.append(x_rec)
                path_y.append(y_rec)
                yaw.append(yaw_rec)
                direct.append(direct_rec)
                # 경로의 x, y 좌표, 각도, 방향을 저장하는 임시 리스트를 초기화한다.
                x_rec, y_rec, yaw_rec, direct_rec = \
                    [x_rec[-1]], [y_rec[-1]], [yaw_rec[-1]], [-direct_rec[-1]]
    # 경로 리스트를 2차원 리스트에 추가한다.
    path_x.append(x_rec)
    path_y.append(y_rec)
    yaw.append(yaw_rec)
    direct.append(direct_rec)
    # 모든 경로의 x, y 리스트를 담을 리스트를 생성한다.
    x_all, y_all = [], []
    # 모든 경로의 x, y값을 리스트에 저장한다.
    for ix, iy in zip(path_x, path_y):
        x_all += ix
        y_all += iy
    # 경로의 x, y 좌표, 각도, 방향, 모든 경로의 x, y 좌표를 반환한다.
    return path_x, path_y, yaw, direct, x_all, y_all


