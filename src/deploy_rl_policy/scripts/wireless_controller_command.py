import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from unitree_go.msg import WirelessController
import time
class Controller:
    def __init__(self, node: Node):
        self.node = node
        self.linear_x=0
        self.linear_y=0
        self.angular_z=0
        self.lx=0.0
        self.ly=0.0
        self.rx=0.0
        self.ry=0.0
        self.keys=0
        self._sub = node.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self._joy_callback,
            10
        )
        self.vel_scale=1

    def _joy_callback(self, msg: Joy):
        self.lx=msg.lx
        self.ly=msg.ly
        self.rx=msg.rx
        self.ry=msg.ry
        self.keys=msg.keys
        

    def get_left_stick(self):
        return (self.ly, -self.lx) 
    def get_right_stick(self):
        return -self.rx



