## Demo - assignment_1

> obstacle avoidance using pid control
> 

---

![Alt Text](etc/ass_1.gif)

## Work flow

---

1. 초음파 센서의 오차 값 중첩  
    1. 자동차에 부착된 센서의 오차를 부착된 위치에 따라 계산 
    2. 왼쪽 >> 오른쪽으로 이동할 때 오차가 증가, 오른쪽 >> 왼쪽으로 이동할 때 오차가 감소
2. 오차값을 이용한 조향각 pid 제어

## Demo - assignment_2

> lane control
> 

---

https://www.youtube.com/watch?v=UoUDTJGDGEE

## Work flow

---

1. Birde eye view
2. HSV filtering
3. sliding window 
4. 왼쪽, 오른쪽 차선 구분 
5. 차선 각도 예외처리 && 오차값 생성 
6. 위 오차값을 이용한 조향각 PID 제어 

## Demo - assignment_3

> path planning and motion planning using pure pursuit , reeds shepp
> 

---

![Alt Text](etc/ass_3_20230620.gif)

## Work flow

---

1. planning 
    1. reeds_shepp 알고리즘 사용 - 휠 베이스, 최대 조향각을 기반으로한 회전 반경에 따라 경로의 방향이 바뀜
2. tracking 
    1. Pure Pursuit 알고리즘 사용 
        1. 차량의 현재 위치에서 경로상의 목표 지점을 계산
        2. 목표 지점을 tracking