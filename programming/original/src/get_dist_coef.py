import cv2 as cv
import numpy as np
from pathlib import Path 

mtx, dist = None, None

chessboard_dir = Path('./chessboards')
tile_size = 0.028 # 2.7 cm

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((8*6,3), np.float32)
objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2) * tile_size
objpoints = []
imgpoints = []

chessboard_index = 0
while True:
    cur_path = chessboard_dir / f'{chessboard_index}.jpg'
    if not cur_path.is_file():
        break
    # print(cur_path.resolve())

    img = cv.imread(cur_path.resolve())
    h, w = img.shape[:2]
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    found, corners = cv.findChessboardCorners(gray, (8, 6))
    if found:
        # print('Found')
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        cv.drawChessboardCorners(img, (8, 6), corners2, found)
        # cv.imshow(f'Result {chessboard_index}', img)
    chessboard_index += 1

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

if __name__ == '__main__':
    chessboard_index = 0
    while True:
        cur_path = chessboard_dir / f'{chessboard_index}.jpg'
        if not cur_path.is_file():
            break
        print(f'reading {cur_path}')
        img = cv.imread(cur_path.resolve())
        h, w = img.shape[:2]
        ncm, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undist = cv.undistort(img, mtx, dist, None, ncm)
        x, y, w, h = roi
        undist = undist[y:y+h, x:x+w] 
        cv.imshow(f'Result {chessboard_index}', undist)
        chessboard_index += 1

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
        mean_error += error
    
    print('-----------------------')
    print(mtx)
    print(dist)
    print(f"total error: {mean_error/len(objpoints)}" )

    cv.waitKey(0)
    cv.destroyAllWindows()