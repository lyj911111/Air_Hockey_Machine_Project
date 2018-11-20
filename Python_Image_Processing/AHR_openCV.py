import cv2
import numpy as np
import time
import serial

NANOserial = serial.Serial(
    port='COM3',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
    timeout=0)

# Panel : 430 X 820
# Camera: 640 X 480
# Real_view :624 X 464 ############수정
MODE = 1
PIX_WIDTH = 615
PIX_HEIGHT = 415
PIX_CENTER = PIX_WIDTH/2
predict_y = 0
cx_origin = 0
cy_origin = 0
cx_1 = 0
cy_1 = 0
cx_2 = 0
cy_2 = 0
step1 = 10
a_flag = 0
cal_flag = 0
upper_y = None
lower_y = None


def nothing(x):
    pass


def mapping(val, from_max, from_min, to_max, to_min):
    return (val-from_min)*((to_max-to_min)/(from_max-from_min)) + to_min


def Reverse_mapping(map_val, max, min):
    return (max - map_val) + min


# y = m(x-a) + b
def x_coordinate(a, b, m):
    return a - b/m


def y_coordinate(a, b, m):
    return -m*a + b




cap = cv2.VideoCapture(0)
cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)

# FIND_BLACK_PUCK
cv2.createTrackbar("graybar", "Trackbars", 52, 255, nothing)
cv2.createTrackbar("bluebar", "Trackbars", 45,  255, nothing)
cv2.createTrackbar("greenbar", "Trackbars", 46, 255, nothing)
cv2.createTrackbar("redbar", "Trackbars", 63,  255, nothing)
cv2.createTrackbar("hsv hbar", "Trackbars", 220, 255, nothing)
cv2.createTrackbar("hsv sbar", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("hsv vbar", "Trackbars", 99, 255, nothing)
cv2.createTrackbar("hsl hbar", "Trackbars", 212, 255, nothing)
cv2.createTrackbar("hsl sbar", "Trackbars", 36, 255, nothing) # 75
cv2.createTrackbar("hsl lbar", "Trackbars", 255, 255, nothing)

#cv2.createTrackbar("graybar_", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("bluebar_", "Trackbars", 18,  255, nothing) # 0
cv2.createTrackbar("greenbar_", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("redbar_", "Trackbars", 0,  255, nothing)
cv2.createTrackbar("hsv hbar_", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("hsv sbar_", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("hsv vbar_", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("hsl hbar_", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("hsl sbar_", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("hsl lbar_", "Trackbars", 0, 255, nothing)


while True:
    _, frame = cap.read()
    # col,row,_ = frame.shape # 해상도
    # print(col,row)
    frame2 = frame.copy() # 영상원본

    # blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    frame = cv2.GaussianBlur(frame, (3, 3), 0)
    # frame = cv2.blur(frame,(3,3))
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # ret, thresh = cv2.threshold(gray_frame, 45, 255, cv2.THRESH_BINARY_INV)

    blue, green, red = cv2.split(frame)
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(frame_hsv)
    frame_hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    H, L, S = cv2.split(frame_hls)

    gray_c  = cv2.getTrackbarPos("graybar", "Trackbars")
    blue_c  = cv2.getTrackbarPos("bluebar",  "Trackbars")
    green_c = cv2.getTrackbarPos("greenbar", "Trackbars")
    red_c   = cv2.getTrackbarPos("redbar",   "Trackbars")
    hsv_h_c = cv2.getTrackbarPos("hsv hbar", "Trackbars")
    hsv_s_c = cv2.getTrackbarPos("hsv sbar", "Trackbars")
    hsv_v_c = cv2.getTrackbarPos("hsv vbar", "Trackbars")
    hsl_h_c = cv2.getTrackbarPos("hsl hbar", "Trackbars")
    hsl_s_c = cv2.getTrackbarPos("hsl sbar", "Trackbars")
    hsl_l_c = cv2.getTrackbarPos("hsl lbar", "Trackbars")

    gray_c_ = cv2.getTrackbarPos("graybar_", "Trackbars")
    blue_c_ = cv2.getTrackbarPos("bluebar_", "Trackbars")
    green_c_ = cv2.getTrackbarPos("greenbar_", "Trackbars")
    red_c_ = cv2.getTrackbarPos("redbar_", "Trackbars")
    hsv_h_c_ = cv2.getTrackbarPos("hsv hbar_", "Trackbars")
    hsv_s_c_ = cv2.getTrackbarPos("hsv sbar_", "Trackbars")
    hsv_v_c_ = cv2.getTrackbarPos("hsv vbar_", "Trackbars")
    hsl_h_c_ = cv2.getTrackbarPos("hsl hbar_", "Trackbars")
    hsl_s_c_ = cv2.getTrackbarPos("hsl sbar_", "Trackbars")
    hsl_l_c_ = cv2.getTrackbarPos("hsl lbar_", "Trackbars")

    _, gray1 = cv2.threshold(gray_frame,gray_c,255,cv2.THRESH_BINARY_INV)
    _, blue1 = cv2.threshold(blue, blue_c, 255, cv2.THRESH_BINARY_INV)
    _, green1 = cv2.threshold(green, green_c, 255, cv2.THRESH_BINARY_INV)
    _, red1   = cv2.threshold(red, red_c, 255, cv2.THRESH_BINARY_INV)
    _, h1 = cv2.threshold(h, hsv_h_c, 255, cv2.THRESH_BINARY_INV)
    _, s1 = cv2.threshold(s, hsv_s_c, 255, cv2.THRESH_BINARY_INV)
    _, v1 = cv2.threshold(v, hsv_v_c, 255, cv2.THRESH_BINARY_INV)
    _, H1 = cv2.threshold(H, hsl_h_c, 255, cv2.THRESH_BINARY_INV)
    _, L1 = cv2.threshold(L, hsl_s_c, 255, cv2.THRESH_BINARY_INV)
    _, S1 = cv2.threshold(S, hsl_l_c, 255, cv2.THRESH_BINARY_INV)

    _, gray_ = cv2.threshold(gray_frame, gray_c_, 255, cv2.THRESH_BINARY)
    _, blue_ = cv2.threshold(blue, blue_c_, 255, cv2.THRESH_BINARY)
    _, green_ = cv2.threshold(green, green_c_, 255, cv2.THRESH_BINARY)
    _, red_ = cv2.threshold(red, red_c_, 255, cv2.THRESH_BINARY)
    _, h_ = cv2.threshold(h, hsv_h_c_, 255, cv2.THRESH_BINARY)
    _, s_ = cv2.threshold(s, hsv_s_c_, 255, cv2.THRESH_BINARY)
    _, v_ = cv2.threshold(v, hsv_v_c_, 255, cv2.THRESH_BINARY)
    _, H_ = cv2.threshold(H, hsl_h_c_, 255, cv2.THRESH_BINARY)
    _, L_ = cv2.threshold(L, hsl_s_c_, 255, cv2.THRESH_BINARY)
    _, S_ = cv2.threshold(S, hsl_l_c_, 255, cv2.THRESH_BINARY)

    final_mask = gray1
    final_mask = cv2.bitwise_and(final_mask, blue1)
    final_mask = cv2.bitwise_and(final_mask, green1)
    final_mask = cv2.bitwise_and(final_mask, red1)
    final_mask = cv2.bitwise_and(final_mask, h1)
    final_mask = cv2.bitwise_and(final_mask, s1)
    final_mask = cv2.bitwise_and(final_mask, v1)
    final_mask = cv2.bitwise_and(final_mask, H1)
    final_mask = cv2.bitwise_and(final_mask, L1)
    final_mask = cv2.bitwise_and(final_mask, S1)

    final_mask = cv2.bitwise_and(final_mask, gray_)
    final_mask = cv2.bitwise_and(final_mask, blue_)
    final_mask = cv2.bitwise_and(final_mask, green_)
    final_mask = cv2.bitwise_and(final_mask, red_)
    #final_mask = cv2.bitwise_and(final_mask, h_)
    #final_mask = cv2.bitwise_and(final_mask, s_)
    final_mask = cv2.bitwise_and(final_mask, v_)
    #final_mask = cv2.bitwise_and(final_mask, H_)
    #final_mask = cv2.bitwise_and(final_mask, L_)
    #final_mask = cv2.bitwise_and(final_mask, S_)
    result = cv2.bitwise_and(frame2, frame2, mask=final_mask)
    cv2.imshow('result', final_mask)

    _, contours, _ = cv2.findContours(final_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # 컨투어 찾기

    if len(contours) != 0:
        for contour in contours:
            if (cv2.contourArea(contour) > 800) and (cv2.contourArea(contour) < 2200):
                ball_area = cv2.contourArea(contour)
                mom = contour
                M = cv2.moments(mom)
                cx_origin = int(M['m10'] / M['m00'])
                cy_origin = int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx_origin, cy_origin), 10, (0, 255, 0), -1) # Center of puck
                cy_origin = Reverse_mapping(cy_origin, 415, 70)
                # print("cx: ", cx_origin, "cy: ", cy_origin)

                if cx_origin > PIX_CENTER+50:
                    cx_1 = cx_origin
                    cy_1 = cy_origin
                    a_flag = 1
                if cx_origin < PIX_CENTER-10 and a_flag == 1:
                    cx_2 = cx_origin
                    cy_2 = cy_origin
                    a_flag = 0
                    cal_flag = 1

                if cal_flag == 1:
                    slope = (cy_1-cy_2)/(cx_1-cx_2) # 기울기
                    predict_y = int(y_coordinate(cx_2, cy_2, slope))
                    # print("slope1 : ", slope)

                    if (predict_y < 0) or (predict_y > PIX_HEIGHT): # 0~HEIGHT 범위 벗어남_1
                        if slope < 0: # 기울기 음수이면 y=0 / 양수이면 y = HEIGHT 대입
                            cx_3 = cx_2 - cy_2*(1/slope)
                            re_slope = -slope
                            cy_3 = -re_slope*cx_3

                        elif slope > 0:
                            cx_3 = cx_2 + (PIX_HEIGHT-cy_2)*(1/slope)
                            re_slope = -slope
                            cy_3 = -re_slope*cx_3 + PIX_HEIGHT

                        predict_y = int(cy_3) # 두번

                        if (predict_y < 0) or (predict_y > PIX_HEIGHT):  # 0~HEIGHT 범위 벗어남_2
                            predict_y = int(PIX_HEIGHT/2) # 두번 벗어나면 가운데 고정
                            print("It's center! CENTER:", int(PIX_HEIGHT / 2))

                    if MODE == 1:
                        step1 = int(mapping(predict_y, 70, 420, 1, 20))
                        NANOserial.write(bytes(step1, encoding='ascii'))
                        print(" prdict_y: ", predict_y, " map_y: ", step1)
                        cal_flag = 0

                    elif MODE == 2:
                        step2 = int(mapping(cy_origin, 70, 420, 1, 20))
                        # NANOserial.write(bytes(step2, encoding='ascii'))
                        print("It's  ball position")

                else:
                    pass

    # cv2.imshow('origin_frame', frame2)
    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()