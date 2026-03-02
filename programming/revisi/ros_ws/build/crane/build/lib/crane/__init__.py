# Revisi: 
# 1. Sekarang kami menggunakan Int32MultiArray
# 2. Data cukup dikirim sekali setelah scanning

import rclpy
import math
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray
import time

SMOOTHING_COEF = 0.7
SCANNING_TIME = 2.0 # Jika dirasa kurang, bisa diadjust
FRAME_SCANNED_THRESH = 10 # Dalam rentang SCANNING_TIME detik, harus ada setidaknya FRAME_SCANNED_THRESH
                          # banyak posisi yang diterima lewat topic /drop_position dan /payload_position
MOVE_DURATION = 10.0 # Waktu gerak setelah scanning hingga selesai homing.
                     # Bisa disesuaikan apabila dirasa terlalu lama atau terlalu cepat
# Helper for computing Euclidean distance
def dist(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.sqrt(dx ** 2 + dy ** 2)

class Crane(Node):
    def __init__(self):
        super().__init__('crane')

        self.state = 'IDLE'
        self.payload_scanned_frames = 0
        self.drop_scanned_frames = 0
        self.payload_position = None
        self.drop_position = None

        self.move_msg = Int32MultiArray()
        self.move_publisher = self.create_publisher(Int32MultiArray, '/move', 20)

        self.create_subscription(Point, '/payload_position', lambda msg: self.update_position('PAYLOAD', msg), 20)
        self.create_subscription(Point, '/drop_position', lambda msg: self.update_position('DROPZONE', msg), 20)
        self.create_service(Empty, '/start', self.start_callback)

        self.scan_timer = None

        self.get_logger().info("Robot is ready to start!")

    def start_scanning(self):
        self.get_logger().info('START SCANNING')
        self.state = 'SCANNING'
        self.scan_timer = self.create_timer(SCANNING_TIME, self.stop_scanning)

    def stop_scanning(self):
        self.scan_timer.cancel()
        self.get_logger().info('STOPPED SCANNING')
        ok = True

        if self.payload_scanned_frames < FRAME_SCANNED_THRESH:
            self.get_logger().error(f'Payload NOT FOUND | Scanned {self.payload_scanned} | mission aborted')
            ok = False
        if self.drop_scanned_frames < FRAME_SCANNED_THRESH:
            self.get_logger().error(f'Dropzone NOT FOUND | Scanned {self.drop_scanned} | mission aborted')
            ok = False

        if not ok:
            self.state = 'IDLE'
            self.get_logger().info(f'Robot is ready to start!')
            return

        self.state = 'MOVING'

        self.get_logger().info('MOVING')

        self.move_msg.data = [
            int(self.payload_position[0]),
            int(self.payload_position[1]),
            int(self.drop_position[0]),
            int(self.drop_position[1])
        ]
        self.move_publisher.publish(self.move_msg)

        time.sleep(MOVE_DURATION)

        self.get_logger().info('Mission complete!')
        self.get_logger().info(f'Robot is ready to start!')

        self.state = 'IDLE'
    
    def update_position(self, object, msg: Point):
        if self.state == 'SCANNING':
            object_position = self.payload_position if object == 'PAYLOAD' else self.drop_position
            self.get_logger().info(f'Received {object} Position: ({msg.x}, {msg.y}) | SCANNED {self.payload_scanned_frames}')

            if object_position is None:
                object_position = (msg.x, msg.y)
            else:
                old_x, old_y = object_position
                object_position = (
                    SMOOTHING_COEF * old_x + (1 - SMOOTHING_COEF) * msg.x, 
                    SMOOTHING_COEF * old_y + (1 - SMOOTHING_COEF) * msg.y
                )
            if object == 'PAYLOAD':
                self.payload_scanned_frames += 1
                self.payload_position = object_position
            else:
                self.drop_scanned_frames += 1
                self.drop_position = object_position
   
    def start_callback(self, req, res):
        self.start_scanning()
        self.payload_scanned_frames = 0
        self.drop_scanned_frames = 0
        self.payload_position = None
        self.drop_position = None
        return res

def main(args=None):
    rclpy.init(args=args)
    crane = Crane()
    rclpy.spin(crane)
    rclpy.shutdown()
