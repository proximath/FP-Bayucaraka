# REVISI:
# 1. Camera tidak perlu dikalibrasi. Yellow box dijadikan ground truth.
# 2. Code dirapikan dan disimplified
# 3. 
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import onnxruntime as ort
import cv2 as cv
import numpy as np
import numpy.typing as nt

# TODO: Sesuaikan dengan kamera yang Anda gunakan. Pakai script get-mtx-and-dist.py supaya gampang.
mtx = np.array([[942.45855405,   0.0,         618.69564463],
    [  0.0,         938.95326623, 407.35429481],
    [  0.0,           0.0,           1.0        ]], dtype=np.float32)
dist = np.array([[-0.48063052,  0.28235575, -0.0114071,  0.00088903, -0.10498914]], dtype=np.float32)

# TODO: Sesuaikan ini dengan tempat Anda menaruh model
ONNX_SOURCE = '../model/v6.onnx'
IMG_SIZE = 512
CONFIDENCE_THRESH = 0.5 # Batas bawah scores untuk mengatasi false positives
PAYLOAD_SIZE = 15 # Kami menggunakan ground support yang menambah dimensi payload menjadi 15x15 cm
DROP_SIZE = 20
REAL_WORLD_WIDTH = 170.4046667 # TODO: Sesuaikan konstanta ini dengan kamera yang Anda gunakan
                               # Bilangan apa ini? Jarak dari titik paling kiri ke titik paling kanan pada kamera
                               # di dunia nyata.
TIMER_PERIOD = 1/10
# Ini kotak kuning yang dipakai untuk membantu memposisikan kamera agar sejajar box.
GUIDING_RECTANGLE = [
    [136, 30],
    [136 + 133, 30 + 150]
]
CAMERA_INDEX = 1

def letterbox(img, target_dim) -> tuple[float, nt.NDArray, int, int]:
    original_h, original_w = img.shape[:2]
    scale = target_dim / (original_h if original_h > original_w else original_w)

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

class Camera(Node):
    def __init__(self):
        super().__init__('camera')

        self.create_subscription(Point, '/crane_position', self.update_crane_position, 20)
        self.payload_publisher = self.create_publisher(Point, '/payload_position', 20)
        self.drop_publisher = self.create_publisher(Point, '/drop_position', 20)

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.coord_offset = (0, 0)
        self.payload_coord = (0, 0)
        self.payload_y = 0
        self.camera_height = 1
        self.image_width = None
        self.msg = Point()

        self.vid_cap = cv.VideoCapture(CAMERA_INDEX)
        video_height = int(self.vid_cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        video_width = int(self.vid_cap.get(cv.CAP_PROP_FRAME_WIDTH))

        self.ncm, self.roi = cv.getOptimalNewCameraMatrix(mtx, dist, (video_width, video_height), 1, (video_width, video_height))

        if not self.vid_cap.isOpened():
            raise RuntimeError("Camera is not found")

        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sess = ort.InferenceSession(ONNX_SOURCE, sess_options=so, providers=["CPUExecutionProvider"])

    def do_inference(self, img, scale, px, py):
        input_name = self.sess.get_inputs()[0].name
        out = self.sess.run(None, { input_name: img })
        out = np.squeeze(out[0])

        x, y, w, h, conf1, conf2 = out[:]
        x1 = x - w/2
        x2 = x + w/2
        y1 = y - h/2
        y2 = y + h/2

        res = []

        # Cari bounding box dengan score terbaik
        for conf in [conf1, conf2]:
            best = np.argmax(conf)
            if conf[best] < CONFIDENCE_THRESH:
                res.append(None)
                res.append(None)
                continue
            
            xx1 = (x1 - px) / scale
            yy1 = (y1 - py) / scale
            xx2 = (x2 - px) / scale
            yy2 = (y2 - py) / scale

            res.append((xx1[best], yy1[best], xx2[best], yy2[best]))
            res.append(conf[best])

        return res

    def timer_callback(self):
        ok, frame = self.vid_cap.read()

        if cv.waitKey(1) == ord('q'):
            cv.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
            return

        if not ok:
            self.get_logger().error("Video capture stopped")
            rclpy.shutdown()
            return

        undist = cv.undistort(frame, mtx, dist, None, self.ncm)
        bx, by, bw, bh = self.roi
        undist = undist[by:by+bh, bx:bx+bw]
        self.image_width = bw

        scale, padded, px, py = letterbox(undist, IMG_SIZE)   

        prep = cv.cvtColor(padded, cv.COLOR_BGR2RGB)
        prep = prep.astype(np.float32) / 255.0
        prep = np.transpose(prep, (2, 0, 1))
        prep = np.expand_dims(prep, axis=0)

        drop_bb, drop_conf, payload_bb, payload_conf = self.do_inference(prep, scale, px, py)

        image_height = undist.shape[0]
        image_width = undist.shape[1]
        scale_factor = REAL_WORLD_WIDTH / image_width * self.camera_height

        if payload_bb:
            self.payload_width = payload_bb[2] - payload_bb[0]
            self.payload_coord = (payload_bb[0], payload_bb[1])
            self.publish(self.payload_publisher, payload_bb, scale_factor)
            self.camera_height = self.image_width / self.payload_width * PAYLOAD_SIZE / REAL_WORLD_WIDTH

        if drop_bb:
            self.publish(self.drop_publisher, drop_bb, scale_factor)
        
        self.draw_bb(payload_bb, payload_conf, undist, "Payload", (255, 0, 0))
        self.draw_bb(drop_bb, drop_conf, undist, "Drop", (0, 0, 255))
        cv.rectangle(undist, GUIDING_RECTANGLE[0], GUIDING_RECTANGLE[1], (0, 255, 255), 2)
        cv.putText(undist, f"H: {round(float(self.camera_height * 100), 3)} cm", (0, image_height - 6), cv.FONT_HERSHEY_COMPLEX, 0.6, (255, 255, 0), 2)

        cv.imshow('Result', undist)

    def publish(self, publisher, bb, scale_factor):
        BOTTOM_LIMIT = GUIDING_RECTANGLE[1][1]
        ix = (bb[0] + bb[2]) / 2 - GUIDING_RECTANGLE[0][0]
        iy = BOTTOM_LIMIT - (bb[1] + bb[3]) / 2
        self.msg.x = float(ix * scale_factor)
        self.msg.y = float(iy * scale_factor)
        publisher.publish(self.msg)
    
    def draw_bb(self, bb, conf, img, name, color):
        if bb is not None:
            a, b, c, d = map(int, [bb[0], bb[1], bb[2], bb[3]])
            ix = (a + c) / 2
            iy = (b + d) / 2

            cv.circle(img, (int(ix), int(iy)), 5, color, cv.FILLED)
            img = cv.rectangle(img, (a, b), (c, d), color, 2)

            cv.putText(img, f"{name} {round(float(conf), 2)}", (a, max(0,b-6)), cv.FONT_HERSHEY_COMPLEX, 0.6, color, 2)

    def update_crane_position(self, msg: Point):
        self.crane_position = (msg.x, msg.y)

def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()
