import rclpy
import math
from rclpy.node import Node
from std_srvs.srv import Empty as SrvEmpty
from geometry_msgs.msg import Point

POSITION_TOLERANCE = 10 # +/- 10mm
CAMERA_HEIGHT = 0.62 # 62cm

def _dist(p1x, p1y, p2x, p2y):
    dx = p1x - p2x
    dy = p1y - p2y
    return math.sqrt(dx ** 2 + dy ** 2)
def dist(p1, p2):
    return _dist(p1[0], p1[1], p2[0], p2[1])

class Crane(Node):
    def __init__(self):
        super().__init__('dummy_crane')
        self.state = 'HOMING'
        self.crane_position = (1000000, 1000000)
        self.frame_scanned = 0
        self.payload_position = None
        self.drop_position = None
        self.position_publisher = self.create_publisher(Point, '/crane_position', 20)
        self.action_client = self.create_service(SrvEmpty, '/action', self.action_callback)
        pos = Point()
        pos.x = 0.0
        pos.y = 0.0
        self.create_timer(1/10, lambda: self.position_publisher.publish(pos))

    def action_callback(self, req, res):
        self.state = 'HOMING'
        self.payload_position = None
        self.drop_position = None
        self.frame_scanned = 0
        self.crane_position = (1000000, 1000000)
        return res

def main(args=None):
    rclpy.init(args=args)
    crane = Crane()
    rclpy.spin(crane)
    rclpy.shutdown()
