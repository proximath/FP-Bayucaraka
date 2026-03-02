import rclpy
import math
from rclpy.node import Node
from std_srvs.srv import Empty as SrvEmpty
from geometry_msgs.msg import Point
import time

SMOOTHING_COEF = 0.7

def _dist(p1x, p1y, p2x, p2y):
    dx = p1x - p2x
    dy = p1y - p2y
    return math.sqrt(dx ** 2 + dy ** 2)

def dist(p1, p2):
    return _dist(p1[0], p1[1], p2[0], p2[1])

class Crane(Node):
    def __init__(self):
        super().__init__('crane')

        self.state = 'IDLE'
        self.payload_scanned = 0
        self.drop_scanned = 0
        self.payload_position = None

        self.drop_position = None
        self.move_publisher = self.create_publisher(Point, '/move', 20)
        self.create_subscription(Point, '/payload_position', self.update_payload_position, 20)
        self.create_subscription(Point, '/drop_position', self.update_drop_position, 20)
        self.create_service(SrvEmpty, '/start', self.start_callback)

        self.scan_timer = None

        self.get_logger().info("Robot is ready to start!")

    def start_scanning(self):
        self.get_logger().info('START SCANNING')
        self.state = 'SCANNING'
        self.scan_timer = self.create_timer(2.0, self.stop_scanning)

    def stop_scanning(self):
        self.state = 'MOVING'
        self.scan_timer.cancel()
        self.get_logger().info('STOPPED SCANNING')
        if self.payload_scanned < 10:
            self.get_logger().error(f'Payload NOT FOUND | Scanned {self.payload_scanned} | mission aborted')
            self.state = 'IDLE'
            self.get_logger().info(f'Robot is ready to start!')
            return
        if self.drop_scanned < 10:
            self.get_logger().error(f'Dropzone NOT FOUND | Scanned {self.drop_scanned} | mission aborted')
            self.state = 'IDLE'
            self.get_logger().info(f'Robot is ready to start!')
            return

        self.get_logger().info('GRABBING')
        msg = Point()
        msg.x = self.payload_position[0]
        msg.y = self.payload_position[1]
        self.move_publisher.publish(msg)

        time.sleep(10)

        self.get_logger().info('DROPPING')
        msg.x = self.drop_position[0]
        msg.y = self.drop_position[1]
        self.move_publisher.publish(msg)

        time.sleep(10)

        self.get_logger().info('HOMING')
        msg.x = 0.0
        msg.y = 0.0
        self.move_publisher.publish(msg)

        time.sleep(5)

        self.get_logger().info('Robot is ready to start!')

        self.state = 'IDLE'

    def update_payload_position(self, msg: Point):
        if self.state == 'SCANNING':
            self.get_logger().info(f'Received Payload Position: ({msg.x}, {msg.y}) | SCANNED {self.payload_scanned}')
            if self.payload_position is None:
                self.payload_position = (msg.x, msg.y)
            else:
                old_x, old_y = self.payload_position
                self.payload_position = ((old_x + msg.x) / 2, (old_y + msg.y) / 2)
            self.payload_scanned += 1

    def update_drop_position(self, msg: Point):
        if self.state == 'SCANNING':
            self.get_logger().info(f'Received Drop Position: ({msg.x}, {msg.y}) | SCANNED {self.drop_scanned}')
            if self.drop_position is None:
                self.drop_position = (msg.x, msg.y)
            else:
                old_x, old_y = self.drop_position
                self.drop_position = (
                    SMOOTHING_COEF * old_x + (1 - SMOOTHING_COEF) * msg.x, 
                    SMOOTHING_COEF * old_y + (1 - SMOOTHING_COEF) * msg.y)
            self.drop_scanned += 1
    
    def start_callback(self, req, res):
        self.start_scanning()
        self.payload_scanned = 0
        self.drop_scanned = 0
        self.payload_position = None
        self.drop_position = None
        return res

def main(args=None):
    rclpy.init(args=args)
    crane = Crane()
    rclpy.spin(crane)
    rclpy.shutdown()
