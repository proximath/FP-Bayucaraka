import cv2 as cv
from pathlib import Path
import numpy as np

mtx = np.array([
    [942.97721311,   0.0        , 624.66453242],
    [  0.0       , 938.70785637 , 408.33161864],
    [  0.0       ,   0.0        ,   1.0       ]], dtype=np.float32)
dist = np.array([[-4.87330334e-01, 2.98268058e-01, -1.23741231e-02, 3.22547265e-04, -1.17394395e-01]], dtype=np.float32)

cap_device = 1
cap = cv.VideoCapture(cap_device)
if not cap.isOpened():
    raise RuntimeError("No camera found")

next_available_index = 0
raw_dataset_path = Path('raw-dataset-2/')
while True:
    if (raw_dataset_path / f"{next_available_index}.mp4").is_file():
        next_available_index += 1
    else:
        break

write_path = f'raw-dataset-2/{next_available_index}.mp4'
write_path_raw = f'raw-dataset-2/{next_available_index}_raw.mp4'

cap.set(cv.CAP_PROP_FPS, 30.0)
fourcc = cv.VideoWriter_fourcc(*'mp4v')
frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv.CAP_PROP_FPS)
vw = cv.VideoWriter(write_path, fourcc, fps, (frame_width, frame_height))
vw_raw = cv.VideoWriter(write_path_raw, fourcc, fps, (frame_width, frame_height))

if not vw or not vw_raw:
    raise RuntimeError(f"Cannot write to {write_path}")

while True:
    ok, frame = cap.read()
    if not ok:
        break
    cv.imshow('Original', frame)
    if cv.waitKey(1) == ord('q'):
        cv.destroyAllWindows()
    original_h, original_w = frame.shape[:2]
    ncm, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (original_w, original_h), 1, (original_w, original_h))
    undist = cv.undistort(frame, mtx, dist, None, ncm)
    bx, by, bw, bh = roi
    undist = undist[by:by+bh, bx:bx+bw]
    undist = cv.resize(undist, (original_w, original_h), None, cv.INTER_LINEAR)
    cv.imshow('Undistorted', undist)
    if cv.waitKey(1) == ord('q'):
        break
    vw.write(undist)
    vw_raw.write(frame)

cap.release()
cv.destroyAllWindows()
