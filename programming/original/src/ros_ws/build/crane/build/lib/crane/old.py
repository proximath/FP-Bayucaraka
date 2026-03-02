
import rclpy
import math
from rclpy.node import Node
from std_srvs.srv import Empty as SrvEmpty
from geometry_msgs.msg import Point
import time

POSITION_TOLERANCE = 10 # +/- 10mm
CAMERA_HEIGHT = 0.82 # 62cm
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
        # self.crane_position = (1000000, 1000000)
        self.payload_scanned = 0
        self.drop_scanned = 0
        self.payload_position = None
        self.drop_position = None
        # self.target_publisher = self.create_publisher(Point, '/target_position', 20)
        # self.action_client = self.create_client(SrvEmpty, '/action')
        self.move_publisher = self.create_publisher(Point, '/move', 20)
        # self.create_subscription(Point, '/crane_position', self.update_crane_position, 20)
        self.create_subscription(Point, '/payload_position', self.update_payload_position, 20)
        self.create_subscription(Point, '/drop_position', self.update_drop_position, 20)
        self.create_service(SrvEmpty, '/start', self.start_callback)
        #self.create_timer(1/10, self.state_machine)
        self.scan_timer = None
        #while not self.action_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().error("Service /action not available")
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
            self.get_logger().info(f'Payload NOT FOUND | Scanned {self.payload_scanned} | assuming mission completed ;)')
            rclpy.shutdown()
        if self.drop_scanned < 10:
            self.get_logger().info(f'Dropzone NOT FOUND | Scanned {self.drop_scanned} | assuming mission completed ;)')
            rclpy.shutdown()

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
    '''
    def state_machine(self):
        if self.state == 'HOMING':
            msg = Point()
            msg.x = 0.0
            msg.y = 0.0
            self.target_publisher.publish(msg)
            self.get_logger().info(f"{ self.state } | CRANE {self.crane_position}")
            if dist(self.crane_position, (0, 0)) < POSITION_TOLERANCE:
                self.state = 'SCANNING'
                self.start_scanning()
        elif self.state == 'MOVE_TO_PAYLOAD':
            if self.payload_position is None:
                self.state = 'HOMING'
                return
            msg = Point()
            msg.x = self.payload_position[0]
            msg.y = self.payload_position[1]
            self.target_publisher.publish(msg)
            self.get_logger().info(f"{ self.state } | CRANE {self.crane_position} | PAYLOAD {self.payload_position}")
            if dist(self.crane_position, self.payload_position) < POSITION_TOLERANCE:
                self.action_client.call(SrvEmpty.Request())
                self.state = 'MOVE_TO_DROP'
        elif self.state == 'MOVE_TO_DROP':
            if self.drop_position is None:
                self.state = 'HOMING'
                return
            msg = Point()
            msg.x = self.drop_position[0]
            msg.y = self.drop_position[1]
            self.target_publisher.publish(msg)
            self.get_logger().info(f"{ self.state } | CRANE {self.crane_position} | DROP {self.payload_position}")
            if dist(self.crane_position, self.drop_position) < POSITION_TOLERANCE:
                self.action_client.call(SrvEmpty.Request())
                self.state = 'IDLE'
    
    def update_crane_position(self, msg: Point):
        if self.state in ['HOMING', 'MOVE_TO_PAYLOAD', 'MOVE_TO_DROP']:
            self.get_logger().info(f'Received Crane Position: ({msg.x}, {msg.y})')
            self.crane_position = (msg.x, msg.y)
    '''
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
        # self.state = 'HOMING'
        self.start_scanning()
        # self.crane_position = (1000000, 1000000)
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
