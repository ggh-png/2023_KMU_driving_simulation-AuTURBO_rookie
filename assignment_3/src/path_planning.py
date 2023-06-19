import time
import math
import numpy as np


# 파라미터 초기화
# 주행 경로 간격 설정 (단위: m)
STEP_SIZE = 0.2
# 최대 주행 거리 설정 (단위: m)
MAX_LENGTH = 1000.0
# 파이 값 설정
PI = math.pi


# class for PATH element
# 경로 요소 클래스
class PATH:
    def __init__(self, lengths, ctypes, L, x, y, yaw, directions):
        self.lengths = lengths              # 각 경로 부분의 길이 (+: 전진, -: 후진) [float]
        self.ctypes = ctypes                # 각 경로 부분의 유형 [문자열]
        self.L = L                          # 전체 경로 길이 [float]
        self.x = x                          # 최종 x 위치 [m]
        self.y = y                          # 최종 y 위치 [m]
        self.yaw = yaw                      # 최종 yaw 각도 [rad]
        self.directions = directions        # 전진: 1, 후진: -1

# 이 함수는 주어진 시작점과 끝점을 이용하여 경로를 생성한다.
def calc_optimal_path(sx, sy, syaw, gx, gy, gyaw, maxc, step_size=STEP_SIZE):
    # 시작점과 끝점을 이용하여 모든 경로를 생성한다.
    paths = calc_all_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size=step_size)
    # 생성된 경로 중 가장 짧은 경로를 선택한다.- 가장 짧은 경로를 선택하는 이유는 주행 경로를 최대한 짧게 가져가기 위함이다.
    minL = paths[0].L
    # 기준점 설정
    mini = 0

    # 경로의 길이를 비교하여 가장 짧은 경로 인덱스를 찾는다.
    for i in range(len(paths)):
        if paths[i].L <= minL:
            minL, mini = paths[i].L, i

    # 찾은 인덱스를 이용하여 최적의 경로를 반환한다.
    return paths[mini]


# 이 함수는 시작점과 끝점을 이용하여 모든 경로를 생성한다.
def calc_all_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size=STEP_SIZE):

    # 시작점, 끝점을 저장하는 리스트
    q0 = [sx, sy, syaw]
    q1 = [gx, gy, gyaw]

    # 시작점과 끝점 그리고 최대 회전 반경을 이용하여 전역 경로를 생성한다.
    paths = generate_path(q0, q1, maxc)

    # 생성된 전역 경로를 generate_local_course 함수를 이용하여 지역 경로로 변환하여 2차원 경로 리스트로 만든다. 
    for path in paths:
        x, y, yaw, directions = \
            generate_local_course(path.L, path.lengths,
                                  path.ctypes, maxc, step_size * maxc)

        # 전역 좌표계로 변환한다.
        path.x = [math.cos(-q0[2]) * ix + math.sin(-q0[2]) * iy + q0[0] for (ix, iy) in zip(x, y)]
        path.y = [-math.sin(-q0[2]) * ix + math.cos(-q0[2]) * iy + q0[1] for (ix, iy) in zip(x, y)]
        path.yaw = [pi_2_pi(iyaw + q0[2]) for iyaw in yaw]
        # 전진, 후진 방향을 저장한다.
        path.directions = directions
        # 경로의 길이를 저장한다.
        path.lengths = [l / maxc for l in path.lengths]
        path.L = path.L / maxc
    # 생성된 전역 경로를 반환한다.
    return paths


# 이 함수는 모든 경로, 길이, 유형을 이용하여 지역 경로를 생성한다.
def set_path(paths, lengths, ctypes):
    # 경로 요소 클래스를 생성한다.
    path = PATH([], [], 0.0, [], [], [], [])
    # 경로 요소 클래스에 경로, 길이, 유형을 저장한다.
    path.ctypes = ctypes
    path.lengths = lengths

    # 생성된 경로가 이미 존재하는 경로인지 확인한다.
    for path_e in paths:
        if path_e.ctypes == path.ctypes:
            # 경로가 이미 존재하는 경로라면 경로를 반환한다.
            if sum([x - y for x, y in zip(path_e.lengths, path.lengths)]) <= 0.01:
                return paths  # not insert path
    # 경로가 존재하지 않는다면 경로를 추가한다.
    path.L = sum([abs(i) for i in lengths])
    # 만약 경로의 길이가 최대 길이를 넘어간다면 경로를 반환한다.
    if path.L >= MAX_LENGTH:
        return paths
    
    # 길이가 0.01 이상이라면 경로를 추가한다.
    assert path.L >= 0.01
    paths.append(path)
    # 경로를 반환한다.
    return paths


def LSL(x, y, phi):
    # LSL 유형의 Reeds-Shepp 경로 생성
    # LSL 유형은 시작점에서 시작 방향으로 왼쪽으로 회전하고, 직진한 후 다시 왼쪽으로 회전하여 끝점에 도달하는 경로이다.
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 경로 생성에 필요한 중간 변수들을 계산한다.
    # 중간 변수 u, t를 계산하고, t가 0 이상인 경우에만 경로를 생성한다.
    u, t = R(x - math.sin(phi), y - 1.0 + math.cos(phi))

    # 중간 변수 v를 계산하고, v가 0 이상인 경우에만 경로를 생성한다.
    # 경로 생성이 불가능한 경우 False를 반환하고, 가능한 경우 True와 경로 생성에 필요한 변수들을 반환한다.
    if t >= 0.0:
        v = M(phi - t)
        if v >= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0

def LSR(x, y, phi):
    # LSR 유형의 Reeds-Shepp 경로 생성
    # LSR 유형은 시작점에서 시작 방향으로 왼쪽으로 회전하고, 직진한 후 다시 오른쪽으로 회전하여 끝점에 도달하는 경로이다.
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 경로 생성에 필요한 중간 변수들을 계산한다.
    u1, t1 = R(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1 = u1 ** 2
    # 중간 변수 u1, t1를 계산하고, u1이 4 이상인 경우에만 경로를 생성한다.
    # 중간 변수 u를 계산하고, u가 0 이상인 경우에만 경로를 생성한다.
    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        # 중간 변수 theta를 계산한다.
        theta = math.atan2(2.0, u)
        # 중간 변수 t를 계산하고, t가 0 이상인 경우에만 경로를 생성한다.
        t = M(t1 + theta)
        # 중간 변수 v를 계산하고, v가 0 이상인 경우에만 경로를 생성한다.
        v = M(t - phi)
        
        # 경로 생성이 불가능한 경우 False를 반환하고, 가능한 경우 True와 경로 생성에 필요한 변수들을 반환한다.
        if t >= 0.0 and v >= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0

def LRL(x, y, phi):
    # LRL 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 경로 생성에 필요한 중간 변수들을 계산한다.
    u1, t1 = R(x - math.sin(phi), y - 1.0 + math.cos(phi))
    # 중간 변수 u1, t1를 계산하고, u1이 4 이상인 경우에만 경로를 생성한다.
    if u1 <= 4.0:
        u = -2.0 * math.asin(0.25 * u1)
        t = M(t1 + 0.5 * u + PI)
        v = M(phi - t + u)

        if t >= 0.0 and u <= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0



def SCS(x, y, phi, paths):
    # SCS 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi, 경로 리스트 paths이다.
    # SCS 유형은 직선으로 시작하여 왼쪽으로 회전하고 다시 직선으로 이동하는 경로이다.
    # SLS 함수를 호출하여 경로 생성 가능 여부를 확인하고 경로 리스트를 업데이트한다.
    flag, t, u, v = SLS(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["S", "WB", "S"])

    flag, t, u, v = SLS(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["S", "R", "S"])

    return paths


def SLS(x, y, phi):
    # SLS 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 경로 생성에 필요한 중간 변수들을 계산한다.
    phi = M(phi)
    if y > 0.0 and 0.0 < phi < PI * 0.99:
        xd = -y / math.tan(phi) + x
        t = xd - math.tan(phi / 2.0)
        u = phi
        v = math.sqrt((x - xd) ** 2 + y ** 2) - math.tan(phi / 2.0)
        return True, t, u, v
    elif y < 0.0 and 0.0 < phi < PI * 0.99:
        xd = -y / math.tan(phi) + x
        t = xd - math.tan(phi / 2.0)
        u = phi
        v = -math.sqrt((x - xd) ** 2 + y ** 2) - math.tan(phi / 2.0)
        return True, t, u, v

    return False, 0.0, 0.0, 0.0



def CSC(x, y, phi, paths):
    # CSC 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi, 경로 리스트 paths이다.
    # LSL, LSR 함수를 호출하여 경로 생성 가능 여부를 확인하고 경로 리스트를 업데이트한다.
    
    flag, t, u, v = LSL(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["WB", "S", "WB"])

    flag, t, u, v = LSL(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["WB", "S", "WB"])

    flag, t, u, v = LSL(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "S", "R"])

    flag, t, u, v = LSL(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "S", "R"])

    flag, t, u, v = LSR(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["WB", "S", "R"])

    flag, t, u, v = LSR(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["WB", "S", "R"])

    flag, t, u, v = LSR(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "S", "WB"])

    flag, t, u, v = LSR(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "S", "WB"])

    return paths


def CCC(x, y, phi, paths):
    # CCC 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi, 경로 리스트 paths이다.
    # LRL 함수를 호출하여 경로 생성 가능 여부를 확인하고 경로 리스트를 업데이트한다.
    
    flag, t, u, v = LRL(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["WB", "R", "WB"])

    flag, t, u, v = LRL(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["WB", "R", "WB"])

    flag, t, u, v = LRL(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, v], ["R", "WB", "R"])

    flag, t, u, v = LRL(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, -v], ["R", "WB", "R"])

    # backwards
    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)

    flag, t, u, v = LRL(xb, yb, phi)
    if flag:
        paths = set_path(paths, [v, u, t], ["WB", "R", "WB"])

    flag, t, u, v = LRL(-xb, yb, -phi)
    if flag:
        paths = set_path(paths, [-v, -u, -t], ["WB", "R", "WB"])

    flag, t, u, v = LRL(xb, -yb, -phi)
    if flag:
        paths = set_path(paths, [v, u, t], ["R", "WB", "R"])

    flag, t, u, v = LRL(-xb, -yb, phi)
    if flag:
        paths = set_path(paths, [-v, -u, -t], ["R", "WB", "R"])

    return paths


def calc_tauOmega(u, v, xi, eta, phi):
    # tau와 omega 계산
    # 경로 생성에 필요한 중간 변수들을 계산하여 tau와 omega를 반환한다.
    # u, v는 각각 시작점과 끝점의 각도이다.
    # xi, eta는 각각 시작점과 끝점의 x, y 좌표이다.
    # phi는 시작점에서 끝점을 향하는 방향이다.

    delta = M(u - v)
    A = math.sin(u) - math.sin(delta)
    B = math.cos(u) - math.cos(delta) - 1.0

    t1 = math.atan2(eta * A - xi * B, xi * A + eta * B)
    t2 = 2.0 * (math.cos(delta) - math.cos(v) - math.cos(u)) + 3.0

    if t2 < 0:
        tau = M(t1 + PI)
    else:
        tau = M(t1)

    omega = M(tau - u + v - phi)

    return tau, omega


def LRLRn(x, y, phi):
    # LRLRn 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 경로 생성에 필요한 중간 변수들을 계산하고, 경로 생성 가능 여부를 확인한다.
    
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho = 0.25 * (2.0 + math.sqrt(xi * xi + eta * eta))

    if rho <= 1.0:
        u = math.acos(rho)
        t, v = calc_tauOmega(u, -u, xi, eta, phi)
        if t >= 0.0 and v <= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0


def LRLRp(x, y, phi):
    # LRLRp 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 경로 생성에 필요한 중간 변수들을 계산하고, 경로 생성 가능 여부를 확인한다.
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho = (20.0 - xi * xi - eta * eta) / 16.0

    if 0.0 <= rho <= 1.0:
        u = -math.acos(rho)
        if u >= -0.5 * PI:
            t, v = calc_tauOmega(u, u, xi, eta, phi)
            if t >= 0.0 and v >= 0.0:
                return True, t, u, v

    return False, 0.0, 0.0, 0.0


def CCCC(x, y, phi, paths):
    # CCCC 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 각 유형별로 중간 변수들을 계산하고, 경로 생성 가능 여부를 확인한다.
    
    flag, t, u, v = LRLRn(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, -u, v], ["WB", "R", "WB", "R"])

    flag, t, u, v = LRLRn(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, u, -v], ["WB", "R", "WB", "R"])

    flag, t, u, v = LRLRn(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, -u, v], ["R", "WB", "R", "WB"])

    flag, t, u, v = LRLRn(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, u, -v], ["R", "WB", "R", "WB"])

    flag, t, u, v = LRLRp(x, y, phi)
    if flag:
        paths = set_path(paths, [t, u, u, v], ["WB", "R", "WB", "R"])

    flag, t, u, v = LRLRp(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, -u, -u, -v], ["WB", "R", "WB", "R"])

    flag, t, u, v = LRLRp(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, u, u, v], ["R", "WB", "R", "WB"])

    flag, t, u, v = LRLRp(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, -u, -u, -v], ["R", "WB", "R", "WB"])

    return paths


def LRSR(x, y, phi):
    # LRSR 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 중간 변수들을 계산하고, 경로 생성 가능 여부를 확인한다.
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, theta = R(-eta, xi)

    if rho >= 2.0:
        t = theta
        u = 2.0 - rho
        v = M(t + 0.5 * PI - phi)
        if t >= 0.0 and u <= 0.0 and v <= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0


def LRSL(x, y, phi):
    # LRSL 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 중간 변수들을 계산하고, 경로 생성 가능 여부를 확인한다.
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2.0:
        r = math.sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = M(theta + math.atan2(r, -2.0))
        v = M(phi - 0.5 * PI - t)
        if t >= 0.0 and u <= 0.0 and v <= 0.0:
            return True, t, u, v

    return False, 0.0, 0.0, 0.0


def CCSC(x, y, phi, paths):
    # CCSC 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 중간 변수들을 계산하고, 경로 생성 가능 여부를 확인한다.
    flag, t, u, v = LRSL(x, y, phi)
    if flag:
        paths = set_path(paths, [t, -0.5 * PI, u, v], ["WB", "R", "S", "WB"])

    flag, t, u, v = LRSL(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, 0.5 * PI, -u, -v], ["WB", "R", "S", "WB"])

    flag, t, u, v = LRSL(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, -0.5 * PI, u, v], ["R", "WB", "S", "R"])

    flag, t, u, v = LRSL(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, 0.5 * PI, -u, -v], ["R", "WB", "S", "R"])

    flag, t, u, v = LRSR(x, y, phi)
    if flag:
        paths = set_path(paths, [t, -0.5 * PI, u, v], ["WB", "R", "S", "R"])

    flag, t, u, v = LRSR(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, 0.5 * PI, -u, -v], ["WB", "R", "S", "R"])

    flag, t, u, v = LRSR(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, -0.5 * PI, u, v], ["R", "WB", "S", "WB"])

    flag, t, u, v = LRSR(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, 0.5 * PI, -u, -v], ["R", "WB", "S", "WB"])

    # backwards
    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)

    flag, t, u, v = LRSL(xb, yb, phi)
    if flag:
        paths = set_path(paths, [v, u, -0.5 * PI, t], ["WB", "S", "R", "WB"])

    flag, t, u, v = LRSL(-xb, yb, -phi)
    if flag:
        paths = set_path(paths, [-v, -u, 0.5 * PI, -t], ["WB", "S", "R", "WB"])

    flag, t, u, v = LRSL(xb, -yb, -phi)
    if flag:
        paths = set_path(paths, [v, u, -0.5 * PI, t], ["R", "S", "WB", "R"])

    flag, t, u, v = LRSL(-xb, -yb, phi)
    if flag:
        paths = set_path(paths, [-v, -u, 0.5 * PI, -t], ["R", "S", "WB", "R"])

    flag, t, u, v = LRSR(xb, yb, phi)
    if flag:
        paths = set_path(paths, [v, u, -0.5 * PI, t], ["R", "S", "R", "WB"])

    flag, t, u, v = LRSR(-xb, yb, -phi)
    if flag:
        paths = set_path(paths, [-v, -u, 0.5 * PI, -t], ["R", "S", "R", "WB"])

    flag, t, u, v = LRSR(xb, -yb, -phi)
    if flag:
        paths = set_path(paths, [v, u, -0.5 * PI, t], ["WB", "S", "WB", "R"])

    flag, t, u, v = LRSR(-xb, -yb, phi)
    if flag:
        paths = set_path(paths, [-v, -u, 0.5 * PI, -t], ["WB", "S", "WB", "R"])

    return paths


def LRSLR(x, y, phi):
    # LRSLR 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 중간 변수들을 계산하고, 경로 생성 가능 여부를 확인한다.
    # formula 8.11 *** TYPO IN PAPER ***
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2.0:
        u = 4.0 - math.sqrt(rho * rho - 4.0)
        if u <= 0.0:
            t = M(math.atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta))
            v = M(t - phi)

            if t >= 0.0 and v >= 0.0:
                return True, t, u, v

    return False, 0.0, 0.0, 0.0


def CCSCC(x, y, phi, paths):
    # CCSCC 유형의 Reeds-Shepp 경로 생성
    # 경로 생성에 필요한 정보는 시작점 (x, y)와 시작 방향 phi이다.
    # 중간 변수들을 계산하고, 경로 생성 가능 여부를 확인한다.
    flag, t, u, v = LRSLR(x, y, phi)
    if flag:
        paths = set_path(paths, [t, -0.5 * PI, u, -0.5 * PI, v], ["WB", "R", "S", "WB", "R"])

    flag, t, u, v = LRSLR(-x, y, -phi)
    if flag:
        paths = set_path(paths, [-t, 0.5 * PI, -u, 0.5 * PI, -v], ["WB", "R", "S", "WB", "R"])

    flag, t, u, v = LRSLR(x, -y, -phi)
    if flag:
        paths = set_path(paths, [t, -0.5 * PI, u, -0.5 * PI, v], ["R", "WB", "S", "R", "WB"])

    flag, t, u, v = LRSLR(-x, -y, phi)
    if flag:
        paths = set_path(paths, [-t, 0.5 * PI, -u, 0.5 * PI, -v], ["R", "WB", "S", "R", "WB"])

    return paths

def generate_local_course(L, lengths, mode, maxc, step_size):
    # 지역 경로를 생성하는 함수
    # 경로 생성에 필요한 정보는 경로의 총 길이 L, 각 부분 경로의 길이 lengths, 경로 모드 mode,
    # 최대 조향각 maxc, 스텝 크기 step_size이다.
    # 경로의 점의 개수를 계산한다.
    point_num = int(L / step_size) + len(lengths) + 3

    # 경로의 x, y, yaw, 방향 리스트를 초기화한다.
    px = [0.0 for _ in range(point_num)]
    py = [0.0 for _ in range(point_num)]
    pyaw = [0.0 for _ in range(point_num)]
    directions = [0 for _ in range(point_num)]
    ind = 1

    # 첫 번째 부분 경로의 방향 설정
    if lengths[0] > 0.0:
        directions[0] = 1
    else:
        directions[0] = -1

    # 첫 번째 부분 경로의 스텝 크기 계산
    if lengths[0] > 0.0:
        d = step_size
    else:
        d = -step_size

    ll = 0.0

    # 각 부분 경로에 대해 경로를 생성한다.
    for m, l, i in zip(mode, lengths, range(len(mode))):
        # 부분 경로의 스텝 크기 계산
        if l > 0.0:
            d = step_size
        else:
            d = -step_size

        ox, oy, oyaw = px[ind], py[ind], pyaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i - 1] * lengths[i]) > 0:
            pd = -d - ll
        else:
            pd = d - ll

        # 스텝별로 점 생성
        while abs(pd) <= abs(l):
            ind += 1
            px, py, pyaw, directions = \
                interpolate(ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
            pd += d

        ll = l - pd - d  # 남은 길이 계산

        ind += 1
        px, py, pyaw, directions = \
            interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)

    # 사용되지 않은 데이터 제거
    while len(px) >= 1 and px[-1] == 0.0:
        px.pop()
        py.pop()
        pyaw.pop()
        directions.pop()

    return px, py, pyaw, directions
def interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions):
    # 선형 보간 함수
    # ind: 현재 인덱스
    # l: 보간 거리
    # m: 모드 (S, WB, R)
    # maxc: 최대 조향각
    # ox, oy, oyaw: 이전 위치 및 방향
    # px, py, pyaw: 경로의 x, y, yaw 리스트
    # directions: 경로의 방향 리스트

    if m == "S":
        # 직선 이동
        px[ind] = ox + l / maxc * math.cos(oyaw)
        py[ind] = oy + l / maxc * math.sin(oyaw)
        pyaw[ind] = oyaw
    else:
        ldx = math.sin(l) / maxc
        if m == "WB":
            ldy = (1.0 - math.cos(l)) / maxc
        elif m == "R":
            ldy = (1.0 - math.cos(l)) / (-maxc)

        gdx = math.cos(-oyaw) * ldx + math.sin(-oyaw) * ldy
        gdy = -math.sin(-oyaw) * ldx + math.cos(-oyaw) * ldy
        px[ind] = ox + gdx
        py[ind] = oy + gdy

    if m == "WB":
        pyaw[ind] = oyaw + l
    elif m == "R":
        pyaw[ind] = oyaw - l

    if l > 0.0:
        directions[ind] = 1
    else:
        directions[ind] = -1

    return px, py, pyaw, directions

def generate_path(q0, q1, maxc):
    # 경로 생성 함수
    # q0: 시작 위치와 방향 (x, y, yaw)
    # q1: 끝 위치와 방향 (x, y, yaw)
    # maxc: 최대 조향각

    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    dth = q1[2] - q0[2]
    c = math.cos(q0[2])
    s = math.sin(q0[2])
    x = (c * dx + s * dy) * maxc
    y = (-s * dx + c * dy) * maxc

    paths = []
    paths = SCS(x, y, dth, paths)
    paths = CSC(x, y, dth, paths)
    paths = CCC(x, y, dth, paths)
    paths = CCCC(x, y, dth, paths)
    paths = CCSC(x, y, dth, paths)
    paths = CCSCC(x, y, dth, paths)

    return paths



# utils
# 세타를 -pi ~ pi 범위로 정규화
def pi_2_pi(theta):
    while theta > PI:
        theta -= 2.0 * PI

    while theta < -PI:
        theta += 2.0 * PI

    return theta

# 두 점 사이의 거리, 각도 계산
def R(x, y):
    """
    Return the polar coordinates (r, theta) of the point (x, y)
    """
    r = math.hypot(x, y)
    theta = math.atan2(y, x)

    return r, theta

# 각도 세타를 -pi ~ pi 범위로 정규화
def M(theta):
    """
    Regulate theta to -pi <= theta < pi
    """
    phi = theta % (2.0 * PI)

    if phi < -PI:
        phi += 2.0 * PI
    if phi > PI:
        phi -= 2.0 * PI

    return phi

# 경로 레이블 변환 
def get_label(path):
    label = ""

    for m, l in zip(path.ctypes, path.lengths):
        label = label + m
        if l > 0.0:
            label = label + "+"
        else:
            label = label + "-"

    return label


def calc_curvature(x, y, yaw, directions):
    # 주어진 x, y 좌표 및 yaw, 이동 방향을 기반으로 곡률(curvature)과 거리(ds)를 계산.
    c, ds = [], []
    # 곡률 계산
    for i in range(1, len(x) - 1):
        dxn = x[i] - x[i - 1]
        dxp = x[i + 1] - x[i]
        dyn = y[i] - y[i - 1]
        dyp = y[i + 1] - y[i]
        dn = math.hypot(dxn, dyn)
        dp = math.hypot(dxp, dyp)
        dx = 1.0 / (dn + dp) * (dp / dn * dxn + dn / dp * dxp)
        ddx = 2.0 / (dn + dp) * (dxp / dp - dxn / dn)
        dy = 1.0 / (dn + dp) * (dp / dn * dyn + dn / dp * dyp)
        ddy = 2.0 / (dn + dp) * (dyp / dp - dyn / dn)
        curvature = (ddy * dx - ddx * dy) / (dx ** 2 + dy ** 2)
        d = (dn + dp) / 2.0

        # 곡률이 nan이면 0으로 설정
        if np.isnan(curvature):
            curvature = 0.0
        # 이동 방향이 음수이면 곡률을 음수로 설정
        if directions[i] <= 0.0:
            curvature = -curvature
        # 곡률이 0이면 이전 곡률로 설정
        if len(c) == 0:
            ds.append(d)
            c.append(curvature)
        # 곡률이 0이 아니면 이전 곡률과 현재 곡률의 평균으로 설정
        ds.append(d)
        c.append(curvature)
    # 마지막 곡률은 이전 곡률로 설정
    ds.append(ds[-1])
    c.append(c[-1])
    # 곡률과 거리를 반환
    return c, ds

# 주어진 시작점과 목표점의 경로가 유효한지 확인하는 함수
def check_path(sx, sy, syaw, gx, gy, gyaw, maxc):
    # 모든 경로 생성 및 저장 
    paths = calc_all_paths(sx, sy, syaw, gx, gy, gyaw, maxc)
    # 경로가 없으면 False 반환
    assert len(paths) >= 1
    
    # 모든 경로에 대해 반복
    for path in paths:
        # 경로의 길이가 0이면 False 반환
        assert abs(path.x[0] - sx) <= 0.01
        assert abs(path.y[0] - sy) <= 0.01
        assert abs(path.yaw[0] - syaw) <= 0.01
        assert abs(path.x[-1] - gx) <= 0.01
        assert abs(path.y[-1] - gy) <= 0.01
        assert abs(path.yaw[-1] - gyaw) <= 0.01

        # 경로의 각 구간의 길이 확인
        d = [math.hypot(dx, dy)
             for dx, dy in zip(np.diff(path.x[0:len(path.x) - 1]),
                               np.diff(path.y[0:len(path.y) - 1]))]
        # 경로의 각 구간의 길이가 0이면 False 반환
        for i in range(len(d)):
            assert abs(d[i] - STEP_SIZE) <= 0.001


def main():
    start_x = 3.0  # [m]
    start_y = 10.0  # [m]
    start_yaw = np.deg2rad(40.0)  # [rad]
    end_x = 0.0  # [m]
    end_y = 1.0  # [m]
    end_yaw = np.deg2rad(0.0)  # [rad]
    max_curvature = 0.1

    t0 = time.time()

    for i in range(1000):
        _ = calc_optimal_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    t1 = time.time()
    print(t1 - t0)


if __name__ == '__main__':
    main()
