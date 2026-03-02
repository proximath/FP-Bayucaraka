import math
from ultralytics import YOLO
import onnxruntime as ort
import cv2 as cv
import numpy as np
import numpy.typing as nt
from get_dist_coef import mtx, dist

IMG_SIZE = 640
ONNX_SOURCE = '../model/v6.onnx'
CONFIDENCE_THRESH = 0.8
PAYLOAD_WIDTH = 0.13 # 13 cm

def transform(res):
    x, y, w, h, conf1, conf2 = res[:]
    x1 = x - w/2
    y1 = y - h/2
    x2 = x + w/2
    y2 = y + h/2
    return [x1, y1, x2, y2, conf1, conf2]

def letterbox(img, original_w, original_h, target_dim) -> tuple[float, nt.NDArray, int, int]:
    scale = None
    if original_h > original_w:
        scale = target_dim/original_h
    else:
        scale = target_dim/original_w
    new_size = (int(original_w * scale), int(original_h * scale))
    rescaled = cv.resize(img, new_size, interpolation=cv.INTER_LINEAR)
    pad_w = target_dim - new_size[0]
    pad_h = target_dim - new_size[1]
    padded = cv.copyMakeBorder(
        rescaled,
        math.ceil(pad_h / 2),
        math.floor(pad_h / 2),
        math.ceil(pad_w / 2),
        math.floor(pad_w / 2),
        cv.BORDER_CONSTANT,
        value=(114, 114, 114)
    )
    return scale, padded, math.ceil(pad_w/2), math.ceil(pad_h/2)

def nms(x1, y1, x2, y2, conf):
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = conf.argsort()[::-1]

    keep: list[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)

        inds = np.where(iou <= 0.5)[0]
        order = order[inds + 1]
    stay = np.zeros((conf.shape[0]), dtype=np.bool)
    stay[keep] = True
    return stay

def recover(x1, y1, x2, y2, scale, px, py):
    x1 = x1 - px
    y1 = y1 - py
    x2 = x2 - px
    y2 = y2 - py
    map(lambda x: x/scale, [x1, y1, x2, y2])

def do_inference(img, sess, frame) -> tuple[tuple[int, int] | None, tuple[int, int] | None]:
    input_name = sess.get_inputs()[0].name
    out = sess.run(None, { input_name: img })
    out = np.squeeze(out[0])
    x1, y1, x2, y2, conf1, conf2 = transform(out)
    print(np.max(conf1), np.max(conf2))
    for conf, color in zip([conf1, conf2], [(0,0,255),(255,0,0)]):
        mask_thresh = conf > CONFIDENCE_THRESH
        mask_nms = nms(x1, y1, x2, y2, conf)
        mask = np.bitwise_and(mask_thresh, mask_nms)
        for (a, b, c, d, cnf) in zip(x1[mask], y1[mask], x2[mask], y2[mask], conf[mask]):
            a, b, c, d = map(int, [a, b, c, d])
            frame = cv.rectangle(frame, (a, b), (c, d), color, 2)
            cv.putText( frame, f"{round(float(cnf), 2)}", (a, max(0,b-6)), cv.FONT_HERSHEY_COMPLEX, 0.6, (0, 0, 255), 2)
    return (None, None)

cap = cv.VideoCapture(1)
if not cap.isOpened():
    raise RuntimeError("Camera is not found")
sess = ort.InferenceSession(ONNX_SOURCE, providers=["AzureExecutionProvider", "CPUExecutionProvider"])

while True:
    ok, frame = cap.read()
    if not ok:
        break
    cv.imshow('original', frame)
    original_h, original_w = frame.shape[:2]
    ncm, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (original_w, original_h), 1, (original_w, original_h))
    undist = cv.undistort(frame, mtx, dist, None, ncm)
    bx, by, bw, bh = roi
    undist = undist[by:by+bh, bx:bx+bw]
    scale, padded, px, py = letterbox(undist, original_w, original_h, IMG_SIZE)   
    cv.imshow('padded', padded)
    prep = cv.cvtColor(padded, cv.COLOR_BGR2RGB)
    prep = prep.astype(np.float32) / 255.0
    prep = np.transpose(prep, (2, 0, 1))
    prep = np.expand_dims(prep, axis=0)
    payload_position, drop_position = do_inference(prep, sess, padded)
    print(payload_position, drop_position)
    cv.imshow('w', padded)
    if cv.waitKey(1) == ord('q'):
        break

cv.destroyAllWindows()