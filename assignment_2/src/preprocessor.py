#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================

import numpy as np
import cv2
import os
import matplotlib.pyplot as plt

#=============================================
# 영상처리를 위한 클래스
# Bird Eye View 변환
# HSV 필터링
# 차선 시작점 검출 
# Sliding window 알고리즘(차선 검출) 등 멤버 함수 존재 
#=============================================


class PreProcessor():
    def __init__(self, roi_height, roi_width):
        self.roi_height = roi_height # ROI 높이 설정
        self.roi_width = roi_width # ROI 폭 설정
        self.left_line_detect_flag = False 
        self.right_line_detect_flag = False

    #=============================================
    # Bird Eye View 변환 함수
    # src에서 dst로 변환하는 변환행렬을 구하고,
    # 이를 이용해 Bird Eye View를 생성한다.
    #=============================================

    def warp_perspect(self, img):
        src = np.float32([[0, 0], [0, self.roi_height-50 ],  [self.roi_width, 0], [640, self.roi_height-50 ]]) # Perspective transform을 위한 src 좌표 설정
        dst = np.float32([[0, 0], [272, self.roi_height-50 ], [self.roi_width, 0], [367, self.roi_height-50 ]]) # Perspective transform을 위한 dst 좌표 설정
        M = cv2.getPerspectiveTransform(src, dst) # 변환 행렬 생성 (src -> dst)
        #Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        # src, dst 시각화(디버깅 용)

        # cv2.circle(img, (int(src[0][0]), int(src[0][1])), 1, (255,0 ,0), 10)
        # cv2.circle(img, (int(src[1][0]), int(src[1][1])), 1, (0,255 ,0), 10)
        # cv2.circle(img, (int(src[2][0]), int(src[2][1])), 1, (0,0 ,255), 10)
        # cv2.circle(img, (int(src[3][0]), int(src[3][1])), 1, (255,255 ,0), 10)


        # cv2.circle(img, (int(dst[0][0]), int(dst[0][1])), 1, (255,0 ,0), 10)
        # cv2.circle(img, (int(dst[1][0]), int(dst[1][1])), 1, (0,255 ,0), 10)
        # cv2.circle(img, (int(dst[2][0]), int(dst[2][1])), 1, (0,0 ,255), 10)
        # cv2.circle(img, (int(dst[3][0]), int(dst[3][1])), 1, (255,255 ,0), 10)

        img = img[280:(280+self.roi_height-50), 0:self.roi_width] # ROI 적용

        warped_img = cv2.warpPerspective(img, M, (self.roi_width, self.roi_height-50), flags=cv2.INTER_LINEAR) # 이미지 워핑으로 Bird Eye View 생성

        return warped_img

    #=============================================
    # HSV 필터링 함수
    # 색공간을 BGR에서 HSV로 변환하고 흰색 픽셀의 마스크를 구한다.
    # 마스크를 이용해 흰색 픽셀만 추출하고 Gray 이미지로 변환함.  
    #=============================================

    def color_filter(self, img):
        # BGR to HSV 변환
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # HSV 범위를 설정하여 흰색을 감지합니다. 
        # 이 범위는 이미지에 따라 조정이 필요할 수 있습니다.
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])

        # 이미지에서 흰색만 마스크 생성
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 이미지에서 흰색 부분만 추출
        white_parts = cv2.bitwise_and(img, img, mask=mask)

        # BGR이미지를 Gray이미지로 변환
        white_parts = cv2.cvtColor(white_parts, cv2.COLOR_BGR2GRAY)

        return white_parts

    #=============================================
    # 차선 시작점 검출 함수
    # Sliding window 알고리즘의 초기 탐색의 시작점을 구하는 역할.
    # 차선이 있을것으로 추정되는 이미지 하단부분의 히스토그램을 이용해 초기 탐색점을 구한다.  
    #=============================================

    def hist_line_peak(self, img):
        histogram = np.sum(img[140:, :], axis=0) # X축 히스토그램 계산
        midpoint = np.int(histogram.shape[0]/2) # 중앙점 계산

        left_hist_result = np.argmax(histogram[:midpoint]) # 중앙점을 기준으로 히스토그램을 계산해 왼쪽 라인 시작점 구함
        right_hist_result = np.argmax(histogram[midpoint:]) # 중앙점을 기준으로 히스토그램을 계산해 오른쪽 라인 시작점 구함

        if right_hist_result == 0: # 오른쪽 차선이 없으면 차량이 왼쪽으로 치우쳐져 있다고 판단
            right_base = right_hist_result + midpoint +90 # 오른쪽 차선 탐색점을 더 오른쪽으로 설정
        else:
            right_base = midpoint + right_hist_result # 오른쪽 차선이 존재하면 히스토그램을 찾은 좌표 그대로 사용(중앙값을 기준으로 우측 이미지에서 구했으므로 중앙값을 더해줌)

        if left_hist_result == 0: # 왼쪽 차선이 없으면 차량이 오른쪽으로 치우쳐져 있다고 판단
            left_base = left_hist_result + midpoint -90 # 왼쪽 차선 탐색점을 더 왼쪽 설정
        else:
            left_base = left_hist_result # 왼쪽 차선이 존재하면 히스토그램을 찾은 좌표 그대로 사용 
        
        #print(left_base, right_base)
        return left_base, right_base

    #=============================================
    # 차선검출에 이용되는 Sliding window 알고리즘
    # 일정 크기의 윈도우를 위로 슬라이딩하면서 차선을 탐색하고, 
    # 누적된 감지를 기반으로 차선 위치를 결정함.
    #=============================================

    def sliding_window(self, img):
        prev_detect_flag_left = False # 직전 차선 검출 성공 플래그 False로 세팅 (왼쪽)
        prev_detect_flag_right = False # 직전 차선 검출 성공 플래그 False로 세팅 (오른쪽)
        line_detect_fail_count_left = 0 # 차선 검출 실패 카운트 0으로 초기화 (왼쪽)
        line_detect_fail_count_right = 0 # 차선 검출 실패 카운트 0으로 초기화 (오른쪽)

        left_base, right_base = self.hist_line_peak(img) # hist_line_peak 함수로 슬라이딩 윈도우의 초기 탐색점 결정

        #Sliding Window
        y = 140 # 탐색 시작 Y좌표 결정
        lx = [] # 왼쪽 차선 X좌표 저장 리스트
        ly = [] # 왼쪽 차선 Y좌표 저장 리스트
        rx = [] # 오른쪽 차선 X좌표 저장 리스트
        ry = [] # 오른쪽 차선 Y좌표 저장 리스트
        window_width = 10 # window 폭
        window_height = 3 # window 높이
        left_window_n = 0
        right_window_n = 0
        msk = img.copy() # 차선검출 결과를 디스플레이 하기 위한 이미지 복사
        msk = cv2.cvtColor(msk, cv2.COLOR_GRAY2BGR) # 컬러 표시를 위해 색공간을 Gray에서 BGR로 변환

        while y>0: # window가 이미지 상단에 도달할때까지 반복
            if left_window_n < 6: # 왼쪽 차선 검출을 위한 window를 5개까지만 허용 
                window = img[y-window_height:y, left_base-window_width:left_base+window_width] # left window 생성
                contours, _ = cv2.findContours(window, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # window 영역에서 contour 검출

                if(len(contours) == 0): # 차선을 못 찾았을떄
                    if prev_detect_flag_left == False: # 차선을 연속으로 못 찾았을떄
                        line_detect_fail_count_left+=1 # 차선 검출 실패 카운트 증가
                        cv2.circle(msk, (left_base-window_width-20, y-(window_height//2)), 1, (0,50,150), 1)

                    prev_detect_flag_left = False # 직전 차선 검출 성공 플래그 False로 세팅
                    cv2.circle(msk, (left_base-window_width, y-(window_height//2)), 1, (0,125,125), 1)
                
                if line_detect_fail_count_left < 5: # 차선 검출 실패 카운트가 5 이하일떄 차선으로 판단
                    
                    cv2.circle(msk, (left_base-window_width, y-(window_height//2)), 1, (0,255,0), 1)

                    for contour in contours: 
                        M = cv2.moments(contour) # contour 모멘트 계산
                        if M["m00"] != 0:
                            cx = int(M["m10"]/M["m00"]) # contour 모멘트 X 좌표 계산
                            cy = int(M["m01"]/M["m00"]) # contour 모멘트 Y 좌표 계산
                            lx.append(left_base-window_width + cx) # 왼쪽 차선 X좌표 저장 
                            ly.append(y-(window_height//2)) # 왼쪽 차선 Y좌표 저장 
                            cv2.circle(msk, (left_base-window_width + cx, y-(window_height//2)), 1, (255,0,0), 1)
                            left_base = left_base-window_width + cx # 다음 차선(위쪽 존재) 검출 윈도우 좌표를 현재 검출된 contour 모멘트를 고려해서 수정 
                            cv2.rectangle(msk, (left_base-window_width,y), (left_base+window_width,y-window_height), (255,0,0), 1)
                            prev_detect_flag_left = True  # 직전 차선 검출 성공 플래그 True로 세팅
                            self.left_line_detect_flag = True # 차선 검출 성공 플래그 True로 세팅
                            left_window_n += 1 # 왼쪽 윈도우 개수 증가        

                if len(lx) < 1: # 왼쪽 차선을 못찾았을뗴 
                    #print("Left line No")
                    self.left_line_detect_flag = False # 왼쪽 차선 검출 성공 플래그 False로 세팅

            if right_window_n < 6: # 오른쪽 차선 검출을 위한 window를 5개까지만 허용 
                window = img[y-window_height:y, right_base-window_width:right_base+window_width] # right window 생성
                contours, _ = cv2.findContours(window, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # window 영역에서 contour 검출

                if(len(contours) == 0): # 차선을 못 찾았을떄
                    if prev_detect_flag_right == False: # 차선을 연속으로 못 찾았을떄
                        line_detect_fail_count_right+=1 # 차선 검출 실패 카운트 증가
                        cv2.circle(msk, (right_base-window_width-20, y-(window_height//2)), 1, (0,255,0), 1)

                    prev_detect_flag_right = False # 직전 차선 검출 성공 플래그 False로 세팅
                    cv2.circle(msk, (right_base-window_width-10, y-(window_height//2)), 1, (0,255,255), 1)
                
                if line_detect_fail_count_right < 5: # 차선 검출 실패 카운트가 5 이하일떄 차선으로 판단
                    cv2.circle(msk, (right_base-window_width, y-(window_height//2)), 1, (0,0,255), 1)

                    for contour in contours:
                        M = cv2.moments(contour) # contour 모멘트 계산
                        if M["m00"] != 0:
                            cx = int(M["m10"]/M["m00"]) # contour 모멘트 X 좌표 계산
                            cy = int(M["m01"]/M["m00"]) # contour 모멘트 Y 좌표 계산
                            rx.append(right_base-window_width + cx) # 오른쪽 차선 X좌표 저장 
                            ry.append(y-(window_height//2)) # 오른쪽 차선 Y좌표 저장 
                            cv2.circle(msk, (right_base-window_width + cx, y-(window_height//2)), 1, (0,255,0), 1)
                            right_base = right_base-window_width + cx  # 다음 차선(위쪽 존재) 검출 윈도우 좌표를 현재 검출된 contour 모멘트를 고려해서 수정 
                            cv2.rectangle(msk, (right_base-window_width,y), (right_base+window_width,y-window_height), (0,255,0), 1)
                            prev_detect_flag_right = True  # 직전 차선 검출 성공 플래그 True로 세팅
                            self.right_line_detect_flag = True # 차선 검출 성공 플래그 True로 세팅
                            right_window_n += 1 # 오른쪽 윈도우 개수 증가        
                
                if len(rx) < 1: # 오른쪽 차선을 못찾았을뗴 
                    #print("Right line No")
                    self.right_line_detect_flag = False # 오른쪽 차선 검출 성공 플래그 False로 세팅

            # cv2.rectangle(msk, (left_base-window_width,y), (left_base+window_width,y-window_height), (0,0,255), 1)
            # cv2.rectangle(msk, (right_base-window_width,y), (right_base+window_width,y-window_height), (0,0,255), 1)
            y -= window_height # 윈도우 Y좌표 위로 이동
        return msk, lx, ly, rx, ry

    

    #=============================================
    # 검출된 차선을 화면에 표시
    # 3차곡선으로 피팅후 다각형으로 화면에 표시함.
    #=============================================

    def overlay_line(self, warped_img, lx, ly, rx, ry):
        #Fit a third order polynomial to each

        if len(lx) != 0 and len(ly) != 0: # 왼쪽 차선이 존재하면 
            left_fit = np.polyfit(ly, lx, 3) # 3차 곡선 피팅

            # 화면에 오버레이를 위한 X, Y값 생성 
            ploty = np.linspace(0, warped_img.shape[0] - 1, warped_img.shape[0]) 
            left_lane_fitx = left_fit[0] * ploty ** 3 + left_fit[1] * ploty ** 2 + left_fit[2] * ploty + left_fit[3]
            #left_lane_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2] # 2차 곡선일떄

            # 좌우 차선을 연결하기 위해 포인트 생성
            left_points = np.array([np.transpose(np.vstack([left_lane_fitx, ploty]))])

            # 찾은 라인을 이미지에 그리기
            cv2.polylines(warped_img, np.int32([left_points]), isClosed=False, color=(255, 0, 0), thickness=5)

        if len(rx) != 0 and len(ry) != 0: # 오른쪽 차선이 존재하면 
            right_fit = np.polyfit(ry, rx, 3) # 3차 곡선 피팅

            # 화면에 오버레이를 위한 X, Y값 생성
            ploty = np.linspace(0, warped_img.shape[0] - 1, warped_img.shape[0])
            right_lane_fitx = right_fit[0] * ploty ** 3 + right_fit[1] * ploty ** 2 + right_fit[2] * ploty + right_fit[3]
            #right_lane_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2] # 2차 곡선일떄

            #return lane_fitx, lane_fit

            # 좌우 차선을 연결하기 위해 포인트 생성
            right_points = np.array([np.flipud(np.transpose(np.vstack([right_lane_fitx, ploty])))])

            # 찾은 라인을 이미지에 그리기
            cv2.polylines(warped_img, np.int32([right_points]), isClosed=False, color=(0, 255, 0), thickness=5)
       
        return warped_img