import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
import onnxruntime as ort
import cv2 as cv
import numpy as np
import numpy.typing as nt
from pathlib import Path

mtx = np.array([[942.45855405,   0.0,         618.69564463],
    [  0.0,         938.95326623, 407.35429481],
    [  0.0,           0.0,           1.0        ]], dtype=np.float32)
dist = np.array([[-0.48063052,  0.28235575, -0.0114071,  0.00088903, -0.10498914]], dtype=np.float32)

ONNX_SOURCE = '../../model/v6.onnx'
IMG_SIZE = 512
CONFIDENCE_THRESH = 0.4
PAYLOAD_SIZE = 15
DROP_SIZE = 20
RW_W = 170.4046667
CAPTURE_DATASET = False
# CAMERA_HEIGHT = 0.83


def transform(res):
    x, y, w, h, conf1, conf2 = res[:]
    x1 = x - w/2
    y1 = y - h/2
    x2 = x + w/2
    y2 = y + h/2
    return [x1, y1, x2, y2, conf1, conf2]

def letterbox(img, target_dim) -> tuple[float, nt.NDArray, int, int]:
    original_h, original_w = img.shape[:2]
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

def recover(x1, y1, x2, y2, scale, px, py):
    x1 = x1 - px
    y1 = y1 - py
    x2 = x2 - px
    y2 = y2 - py
    return (x1/scale, y1/scale, x2/scale, y2/scale)

class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.create_subscription(Point, '/crane_position', self.update_crane_position, 20)
        self.payload_publisher = self.create_publisher(Point, '/payload_position', 20)
        self.calibrate_service = self.create_service(Empty, '/calibrate_offset', self.calibrate_offset)
        self.drop_publisher = self.create_publisher(Point, '/drop_position', 20)

        self.timer = self.create_timer(1/10, self.timer_callback)

        self.coord_offset = (0, 0)
        self.payload_coord = (0, 0)
        self.payload_y = 0
        self.camera_height = 1

        self.vid_cap = cv.VideoCapture(1)
        self.image_width = int(self.vid_cap.get(cv.CAP_PROP_FRAME_WIDTH))
        self.image_height = int(self.vid_cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        self.ncm, self.roi = cv.getOptimalNewCameraMatrix(mtx, dist, (self.image_width, self.image_height), 1, (self.image_width, self.image_height))
#a

        if not self.vid_cap.isOpened():
            raise RuntimeError("Camera is not found")

        next_available_index = 0
        raw_dataset_path = Path('../../raw-dataset-2/')
        while True:
            if (raw_dataset_path / f"{next_available_index}.mp4").is_file():
                next_available_index += 1
            else:
                break

        write_path = raw_dataset_path / f'{next_available_index}.mp4'
        write_path_raw = raw_dataset_path / f'{next_available_index}_raw.mp4'

        self.vid_cap.set(cv.CAP_PROP_FPS, 30.0)
        fourcc = cv.VideoWriter_fourcc(*'mp4v')
        frame_width = int(self.vid_cap.get(cv.CAP_PROP_FRAME_WIDTH))
        frame_height = int(self.vid_cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        fps = self.vid_cap.get(cv.CAP_PROP_FPS)
        self.vw = cv.VideoWriter(write_path.resolve(), fourcc, fps, (frame_width, frame_height))
        self.vw_raw = cv.VideoWriter(write_path_raw.resolve(), fourcc, fps, (frame_width, frame_height))

        print(write_path.resolve())
        print(write_path_raw.resolve())

        if not self.vw or not self.vw_raw:
            raise RuntimeError(f"Cannot write to {write_path}")

        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sess = ort.InferenceSession(ONNX_SOURCE, sess_options=so, providers=["CPUExecutionProvider"])
    
    def calibrate_offset(self, req, res):
        if self.payload_coord is None:
            self.get_logger().error("Payload not found")
        self.coord_offset = self.payload_coord
        self.camera_height = 512 / self.payload_width * PAYLOAD_SIZE / RW_W
        print(f'calibrate {512}/{self.payload_width}*{PAYLOAD_SIZE}/{RW_W} = {self.camera_height}')
        return res

    def do_inference(self, img, scale, px, py):
        input_name = self.sess.get_inputs()[0].name
        out = self.sess.run(None, { input_name: img })
        out = np.squeeze(out[0])
        x1, y1, x2, y2, conf2, conf1 = transform(out)
        for i in range(len(x1)):
            conf1[i] = self.validate(x1[i], y1[i], x2[i], y2[i], conf1[i])
            conf2[i] = self.validate(x1[i], y1[i], x2[i], y2[i], conf2[i])

        # print(np.max(conf1), np.max(conf2))
        res = []
        for conf in [conf1, conf2]:
            best = np.argmax(conf)
            if conf[best] < CONFIDENCE_THRESH:
                res.append(None)
                continue
            xx1, yy1, xx2, yy2 = recover(
                x1[best], 
                y1[best], 
                x2[best], 
                y2[best], 
                scale, 
                px, 
                py)
            res.append((xx1, yy1, xx2, yy2, conf[best]))

        return res
    
    def validate(self, x1, y1, x2, y2, conf):
        ratio = (x2 - x1) / (y2 - y1)
        if (ratio > 1.4 or ratio < 0.6):
            conf = conf * 0.5
        

        return conf

    # unused
    def bb_validate(self, payload_bb, drop_bb, img):
        px1, py1, px2, py2, _ = payload_bb
        dx1, dy1, dx2, dy2, _ = payload_bb
        px1, py1, px2, py2, dx1, dy1, dx2, dy2 = map(int, [px1, py1, px2, py2, dx1, dy1, dx2, dy2])
        drop = img[dy1:dy2, dx1:dx2]
        payload = img[py1:py2, px1:px2]
        hsv = cv.cvtColor(payload, cv.COLOR_BGR2HSV)
        blur = cv.GaussianBlur(payload, (3, 3), 0, None)
        gray = cv.cvtColor(blur, cv.COLOR_BGR2GRAY)
        contours, hierarchy = cv.findContours(gray, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area < 1000:
                continue
            canvas = np.zeros_like(payload)
            cv.drawContours(canvas, [cnt], 0, (255, 255, 255), cv.FILLED)
            mean_sat = np.mean(np.bitwise_and(canvas, hsv)[:,:,1])
            mean_val = np.mean(np.bitwise_and(canvas, hsv)[:,:,2])
            #print(mean_sat, mean_val)
        return (payload_bb, drop_bb)

    def timer_callback(self):
        ok, frame = self.vid_cap.read()
        if cv.waitKey(1) == ord('q'):
            cv.destroyAllWindows()
            rclpy.shutdown()
        if not ok:
            self.get_logger().warning("Video capture stopped")
            rclpy.shutdown()
            return
        # cv.imshow('Original', frame)
        original_h, original_w = frame.shape[:2]

        undist = cv.undistort(frame, mtx, dist, None, self.ncm)
        bx, by, bw, bh = self.roi
        undist = undist[by:by+bh, bx:bx+bw]

        scale, padded, px, py = letterbox(undist, IMG_SIZE)   
        #cv.imshow('Padded', padded)

        prep = cv.cvtColor(padded, cv.COLOR_BGR2RGB)
        prep = prep.astype(np.float32) / 255.0
        prep = np.transpose(prep, (2, 0, 1))
        prep = np.expand_dims(prep, axis=0)

        start = time.time()
        payload_bb, drop_bb = self.do_inference(prep, scale, px, py)
        print("Inference:", time.time() - start)

        self.draw_bb(payload_bb, undist, "Payload", (255, 0, 0), True)
        self.draw_bb(drop_bb, undist, "Drop", (0, 0, 255), False)

        cv.imshow('BB', undist)
        cv.resize(undist, (original_w, original_h), None, interpolation=cv.INTER_LINEAR)
        undist = cv.resize(undist, (original_w, original_h), None, cv.INTER_LINEAR)
        if CAPTURE_DATASET:
            self.vw.write(undist)
            self.vw_raw.write(frame)
    
    def draw_bb(self, bb, img, name, color, payload):
        if bb is not None:
            a, b, c, d = map(int, [bb[0], bb[1], bb[2], bb[3]])
            conf = bb[4]
            ix = (a + c) / 2
            iy = (b + d) / 2
            imw = img.shape[1]
            cv.circle(img, (int(ix), int(iy)), 5, color, cv.FILLED)
            img = cv.rectangle(img, (a, b), (c, d), color, 2)
            if payload:
                self.payload_width = c - a
                self.payload_coord = (ix, iy)
            ix = ix - self.coord_offset[0] 
            iy = iy - self.coord_offset[1] 
            msg = Point()
            msg.x = float(ix * RW_W / imw) * self.camera_height
            msg.y = float(iy * RW_W / imw) * self.camera_height
            if payload:
                self.payload_publisher.publish(msg)
            else:
                self.drop_publisher.publish(msg)
            cv.putText(img, f"{name} {round(float(conf), 2)}", (a, max(0,b-6)), cv.FONT_HERSHEY_COMPLEX, 0.6, color, 2)

        
    def update_crane_position(self, msg: Point):
        self.crane_position = (msg.x, msg.y)

def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    rclpy.shutdown()
