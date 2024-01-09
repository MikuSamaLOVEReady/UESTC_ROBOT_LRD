import numpy as np
import cv2

cap = cv2.VideoCapture(0)

i = 0
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 680)
while(cap.isOpened()):
    ret, frame = cap.read()
    # height, width, channels = frame.shape
    # print(f"Image size: {width} x {height}")
    if ret==True:
        left = frame[:,0:640]
        right = frame[:,640:1280]
	#cv2.nameswindow(0)
        cv2.imshow('frame', frame)
        if cv2.waitKey(10)  == ord('q'):
            cv2.imwrite("D:\code\python\l\l_" + str(i) + ".jpg",left)
            cv2.imwrite("D:\code\python\s\s_" + str(i) + ".jpg",right)
            i = i +1
    else:
        break
