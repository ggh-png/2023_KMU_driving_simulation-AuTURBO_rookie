import numpy as np
import cv2
import os
import matplotlib.pyplot as plt

# 이미지 전처리와 차선 검출에 관련된 메서드들을 포함하는 클래스
class PreProcessor():
    # 해당 클래스의 인스턴스를 초기화
    def __init__(self, roi_height, roi_width):
        self.roi_height = roi_height
        self.roi_width = roi_width
        self.left_line_detect_flag = False
        self.right_line_detect_flag = False

    # 입력 이미지를 원근 변환(perspective transformation)을 통해 변형시키는 메서드
    # src와 dst는 변환에 사용되는 좌표들을 나타내는 배열
    def warp_perspect(self, img):
        # src와 dst는 변환에 사용되는 좌표들을 나타내는 배열
        src = np.float32([[0, 0], [0, self.roi_height-50 ],  [self.roi_width, 0], [640, self.roi_height-50 ]])
        dst = np.float32([[0, 0], [272, self.roi_height-50 ], [self.roi_width, 0], [367, self.roi_height-50 ]])
        
        # 변환 행렬 M과 역변환 행렬 Minv를 생성
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        cv2.circle(img, (int(src[0][0]), int(src[0][1])), 1, (255,0 ,0), 10)
        cv2.circle(img, (int(src[1][0]), int(src[1][1])), 1, (0,255 ,0), 10)
        cv2.circle(img, (int(src[2][0]), int(src[2][1])), 1, (0,0 ,255), 10)
        cv2.circle(img, (int(src[3][0]), int(src[3][1])), 1, (255,255 ,0), 10)

        img = img[280:(280+self.roi_height-50), 0:self.roi_width] # Apply np slicing for ROI crop

        # 이미지를 원근 변환
        warped_img = cv2.warpPerspective(img, M, (self.roi_width, self.roi_height-50), flags=cv2.INTER_LINEAR) # Image warping
        return warped_img

    # 입력 이미지에서 흰색 부분을 추출하는 메서드
    # 입력 이미지를 BGR에서 HSV 색 공간으로 변환한 후, 미리 설정된 범위(흰색)에 해당하는 색상만 마스킹하여 추출
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

        # Convert the image to grayscale
        white_parts = cv2.cvtColor(white_parts, cv2.COLOR_BGR2GRAY)

        return white_parts

    # 입력 이미지에서 차선의 히스토그램을 생성하고, 히스토그램에서 왼쪽 차선과 오른쪽 차선의 기준점을 찾는 메서드
    # np.sum() 함수를 사용하여 이미지의 아래 절반부분에서 각 열의 합을 구한 뒤, 
    # 히스토그램의 중간 지점을 계산하여 이를 기준으로 왼쪽과 오른쪽 차선의 기준점을 계산하고 반환
    def hist_line_peak(self, img):
        histogram = np.sum(img[140:, :], axis=0)
        midpoint = np.int(histogram.shape[0]/2)

        left_hist_result = np.argmax(histogram[:midpoint])
        right_hist_result = np.argmax(histogram[midpoint:])

        if right_hist_result == 0:
            right_base = right_hist_result + midpoint +90
        else:
            right_base = midpoint + right_hist_result

        if left_hist_result == 0:
            left_base = left_hist_result + midpoint -90
        else:
            left_base = left_hist_result

        return left_base, right_base

    # 입력 이미지에서 슬라이딩 윈도우 방식을 사용하여 차선을 검출하는 메서드
    # 이전에 검출한 차선의 상태를 고려하고, 윈도우를 이용하여 차선을 검출
    # 검출된 차선의 좌표들을 저장하고, 결과 이미지와 함께 반환
    def sliding_window(self, img):
        prev_detect_flag_left = False
        prev_detect_flag_right = False
        line_detect_fail_count_left = 0
        line_detect_fail_count_right = 0

        left_base, right_base = self.hist_line_peak(img)

        #Sliding Window
        y = 140
        lx = []
        ly = []
        rx = []
        ry = []
        window_width = 12
        window_height = 4
        left_window_n = 0
        right_window_n = 0
        msk = img.copy()
        msk = cv2.cvtColor(msk, cv2.COLOR_GRAY2BGR)

        while y>0:
            
            if left_window_n < 6:
                ## Left threshold
                window = img[y-window_height:y, left_base-window_width:left_base+window_width]

                contours, _ = cv2.findContours(window, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                # print(len(contours))
                if(len(contours) == 0): # 차선을 못 찾았을떄
                    if prev_detect_flag_left == False: # 차선을 연속으로 못 찾았을떄
                        line_detect_fail_count_left+=1
                        cv2.circle(msk, (left_base-window_width-20, y-(window_height//2)), 1, (0,50,150), 1)

                    prev_detect_flag_left = False
                    cv2.circle(msk, (left_base-window_width, y-(window_height//2)), 1, (0,125,125), 1)
                
                if line_detect_fail_count_left < 5:
                    
                    cv2.circle(msk, (left_base-window_width, y-(window_height//2)), 1, (0,255,0), 1)

                    for contour in contours:
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"]/M["m00"])
                            cy = int(M["m01"]/M["m00"])
                            lx.append(left_base-window_width + cx)
                            ly.append(y-(window_height//2))
                            cv2.circle(msk, (left_base-window_width + cx, y-(window_height//2)), 1, (255,0,0), 1)
                            left_base = left_base-window_width + cx
                            cv2.rectangle(msk, (left_base-window_width,y), (left_base+window_width,y-window_height), (255,0,0), 1)
                            self.left_line_detect_flag = True
                            left_window_n += 1         

                if len(lx) < 1:
                    #print("Left line No")
                    self.left_line_detect_flag = False
            
            ## Right threshold
            
            if right_window_n < 6:
                window = img[y-window_height:y, right_base-window_width:right_base+window_width]
                contours, _ = cv2.findContours(window, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if(len(contours) == 0): # 차선을 못 찾았을떄
                    if prev_detect_flag_right == False: # 차선을 연속으로 못 찾았을떄
                        line_detect_fail_count_right+=1
                        cv2.circle(msk, (right_base-window_width-20, y-(window_height//2)), 1, (0,255,0), 1)

                    prev_detect_flag_right = False
                    cv2.circle(msk, (right_base-window_width-10, y-(window_height//2)), 1, (0,255,255), 1)
                
                if line_detect_fail_count_right < 5:
                    cv2.circle(msk, (right_base-window_width, y-(window_height//2)), 1, (0,0,255), 1)

                    for contour in contours:
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"]/M["m00"])
                            cy = int(M["m01"]/M["m00"])
                            rx.append(right_base-window_width + cx)
                            ry.append(y-(window_height//2))
                            cv2.circle(msk, (right_base-window_width + cx, y-(window_height//2)), 1, (0,255,0), 1)
                            right_base = right_base-window_width + cx
                            cv2.rectangle(msk, (right_base-window_width,y), (right_base+window_width,y-window_height), (0,255,0), 1)
                            prev_detect_flag_right = True
                            self.right_line_detect_flag = True
                
                if len(rx) < 1:
                    #print("Right line No")
                    self.right_line_detect_flag = False
                right_window_n += 1

            # cv2.rectangle(msk, (left_base-window_width,y), (left_base+window_width,y-window_height), (0,0,255), 1)
            # cv2.rectangle(msk, (right_base-window_width,y), (right_base+window_width,y-window_height), (0,0,255), 1)
            y -= window_height
        return msk, lx, ly, rx, ry