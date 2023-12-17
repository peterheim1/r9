import rclpy
import sys
import time
import tf2_ros
import math

from math import sin, cos, pi, radians, degrees
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from robbie_msg.msg import HeadJoint
from std_msgs.msg import String, Float32
from robbie_msg.srv import Say
class HeadPublisher(Node):

    def __init__(self):
        super().__init__('cortex_controller')
        self.get_logger().info('starting cortex')
         
        
        
        self.subscription = self.create_subscription(Float32,'battery', self._HandleMovment,10)
        self.client = self.create_client(Say, 'voice')
        self.client_futures = []
        #self.client = self.create_client(Empty, 'service')
        self.subscription  # prevent unused variable warning
        #self.publisher_ = self.create_publisher(HeadJoint, 'H_joints', 10)
        now = Node.get_clock(self).now()  
         
    def _HandleMovment(self, data):
        msg = Float32()
        msg = data.data
        req = Say.Request()
        req.sentence = "dont panic"
        if (msg < 10.5):
            self.get_logger().info(str("panic"))
            self.future = self.client.call_async(req)
        #self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)

    head_publisher = HeadPublisher()

    rclpy.spin(head_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    head_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
