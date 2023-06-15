import numpy as np
import cv2
import os
import matplotlib.pyplot as plt

class PreProcessor():
    def __init__(self, roi_height, roi_width):
        self.roi_height = roi_height
        self.roi_width = roi_width
        self.left_line_detect_flag = False
        self.right_line_detect_flag = False

    def warp_perspect(self, img):
        src = np.float32([[0, 0], [0, self.roi_height-50 ],  [self.roi_width, 0], [640, self.roi_height-50 ]])
        dst = np.float32([[0, 0], [272, self.roi_height-50 ], [self.roi_width, 0], [367, self.roi_height-50 ]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

        cv2.circle(img, (int(src[0][0]), int(src[0][1])), 1, (255,0 ,0), 10)
        cv2.circle(img, (int(src[1][0]), int(src[1][1])), 1, (0,255 ,0), 10)
        cv2.circle(img, (int(src[2][0]), int(src[2][1])), 1, (0,0 ,255), 10)
        cv2.circle(img, (int(src[3][0]), int(src[3][1])), 1, (255,255 ,0), 10)


        # cv2.circle(img, (int(dst[0][0]), int(dst[0][1])), 1, (255,0 ,0), 10)
        # cv2.circle(img, (int(dst[1][0]), int(dst[1][1])), 1, (0,255 ,0), 10)
        # cv2.circle(img, (int(dst[2][0]), int(dst[2][1])), 1, (0,0 ,255), 10)
        # cv2.circle(img, (int(dst[3][0]), int(dst[3][1])), 1, (255,255 ,0), 10)

        img = img[280:(280+self.roi_height-50), 0:self.roi_width] # Apply np slicing for ROI crop

        warped_img = cv2.warpPerspective(img, M, (self.roi_width, self.roi_height-50), flags=cv2.INTER_LINEAR) # Image warping

        return warped_img

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

    def hist_line_peak(self, img):
        left_margin = 0 
        right_margin = 0 
        if self.left_line_detect_flag == True and self.right_line_detect_flag == False:
            right_margin = 50
        
        if self.left_line_detect_flag == False and self.right_line_detect_flag == True:
            left_margin = 50

        #print(f'left: {self.left_line_detect_flag}')
        #print(f'right: {self.right_line_detect_flag}')
        #Histogram

        histogram = np.sum(img[img.shape[0]//2:, :], axis=0)
        midpoint = np.int(histogram.shape[0]/2)
        
        left_base = np.argmax(histogram[:midpoint - left_margin])
        right_base = np.argmax(histogram[midpoint + right_margin:]) + midpoint
        print(midpoint + right_margin)
        return left_base, right_base

    def sliding_window(self, img):
        prev_detect_flag_left = False
        prev_detect_flag_right = False
        line_detect_fail_count_left = 0
        line_detect_fail_count_right = 0

        left_base, right_base = self.hist_line_peak(img)

        #Sliding Window
        y = 150
        lx = []
        ly = []
        rx = []
        ry = []
        window_width = 15
        window_height = 5
        msk = img.copy()
        msk = cv2.cvtColor(msk, cv2.COLOR_GRAY2BGR)

        while y>0:
            ## Left threshold
            window = img[y-window_height:y, left_base-window_width:left_base+window_width]

            #cv2.waitKey(0)

            contours, _ = cv2.findContours(window, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # print(len(contours))
            if(len(contours) == 0): # 차선을 못 찾았을떄
                if prev_detect_flag_left == False: # 차선을 연속으로 못 찾았을떄
                    line_detect_fail_count_left+=1
                    cv2.circle(msk, (left_base-window_width-20, y-(window_height//2)), 1, (0,50,150), 1)

                prev_detect_flag_left = False
                cv2.circle(msk, (left_base-window_width, y-(window_height//2)), 1, (0,125,125), 1)
            

            if line_detect_fail_count_left < 4:
                
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
                        

            if len(lx) < 1:
                #print("Left line No")
                self.left_line_detect_flag = False
            
            ## Right threshold
            
            window = img[y-window_height:y, right_base-window_width:right_base+window_width]
            contours, _ = cv2.findContours(window, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if(len(contours) == 0): # 차선을 못 찾았을떄
                if prev_detect_flag_right == False: # 차선을 연속으로 못 찾았을떄
                    line_detect_fail_count_right+=1
                    cv2.circle(msk, (right_base-window_width-20, y-(window_height//2)), 1, (0,255,0), 1)

                prev_detect_flag_right = False
                cv2.circle(msk, (right_base-window_width-10, y-(window_height//2)), 1, (0,255,255), 1)
            
            if line_detect_fail_count_right < 4:
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

            # cv2.rectangle(msk, (left_base-window_width,y), (left_base+window_width,y-window_height), (0,0,255), 1)
            # cv2.rectangle(msk, (right_base-window_width,y), (right_base+window_width,y-window_height), (0,0,255), 1)
            y -= window_height
        return msk, lx, ly, rx, ry
    
    def overlay_line(self, warped_img, lx, ly, rx, ry):
        # Fit a third order polynomial to each

        if len(lx) != 0 and len(ly) != 0:
            left_fit = np.polyfit(ly, lx, 2)

            ploty = np.linspace(0, warped_img.shape[0] - 1, warped_img.shape[0])
            # left_lane_fitx = left_fit[0] * ploty ** 3 + left_fit[1] * ploty ** 2 + left_fit[2] * ploty + left_fit[3]
            left_lane_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]

            # 좌우 차선을 연결하기 위해 포인트 생성
            left_points = np.array([np.transpose(np.vstack([left_lane_fitx, ploty]))])
            cv2.polylines(warped_img, np.int32([left_points]), isClosed=False, color=(255, 0, 0), thickness=5)

        if len(rx) != 0 and len(ry) != 0: 
            right_fit = np.polyfit(ry, rx, 2)

            # Generate x and y values for plotting

            ploty = np.linspace(0, warped_img.shape[0] - 1, warped_img.shape[0])
            # right_lane_fitx = right_fit[0] * ploty ** 3 + right_fit[1] * ploty ** 2 + right_fit[2] * ploty + right_fit[3]
            right_lane_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

            #return lane_fitx, lane_fit

            # 좌우 차선을 연결하기 위해 포인트 생성
            right_points = np.array([np.flipud(np.transpose(np.vstack([right_lane_fitx, ploty])))])
            #points = np.hstack((left_points, right_points))

            # 찾은 라인을 이미지에 그리기
            #cv2.polylines(warped_img, np.int32([points]), isClosed=False, color=(0, 255, 0), thickness=10)
            cv2.polylines(warped_img, np.int32([right_points]), isClosed=False, color=(0, 255, 0), thickness=5)

        return warped_img