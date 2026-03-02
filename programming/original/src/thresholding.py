import cv2 as cv
import numpy as np
from get_dist_coef import mtx, dist

input_device = 1

cap = cv.VideoCapture(input_device)
cap.set(cv.CAP_PROP_FPS, 30.0)
if not cap.isOpened():
    raise RuntimeError("Not Found")

is_paused = False
while True:
    if is_paused:
        if cv.waitKey(1) == ord('c'):
            is_paused = False
        continue
    ok, frame = cap.read()
    if not ok:
        break
    original_h, original_w = frame.shape[:2]
    if input_device == 1:
        nmt, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (original_w, original_h), 1, (original_w, original_h))
        frame = cv.undistort(frame, mtx, dist, None, nmt)
        x, y, w, h = roi
        frame = frame[y:y+h, x:x+w]
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    thresh = cv.inRange(hsv, lowerb=np.array([20, 175, 0]),upperb=np.array([75, 255, 100]))
    contours, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        rect = cv.minAreaRect(cnt)
        w = rect[1][0]
        h = rect[1][1]
        rect_area = w * h
        area = cv.contourArea(cnt)
        if area < 10000:
            continue
        squareness = area / rect_area
        if squareness > 0.8:
            cv.drawContours(frame, [cnt], 0, (255, 0, 0), 2)
            rect_points = cv.boxPoints(rect).astype(np.int32)
            mask = np.zeros_like(frame).astype(np.uint8)
            cv.drawContours(mask, [rect_points], 0, (255, 255, 255), cv.FILLED)
            cv.drawContours(frame, [rect_points], 0, (0, 255, 0), 2)
            # print(squareness)
            masked = cv.bitwise_and(frame, mask)
            mean = cv.mean(masked)
            # cv.imshow('mask', masked)
            #print(mean)
            cx, cy = rect[0]
            cv.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), cv.FILLED)
    cv.imshow('thresh', thresh)
    cv.imshow('original', frame)
    if cv.waitKey(1) == ord('q'):
        break
    elif cv.waitKey(1) == ord('c'):
        is_paused = True

cv.waitKey(0)
cv.destroyAllWindows()