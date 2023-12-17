import rclpy
import sys
import time
import tf2_ros
import math
from math import sin, cos, pi, radians, degrees
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

from SerialDataGateway3 import SerialDataGateway

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('starting arduino control')
        self.publisher_ = self.create_publisher(String, 'arm_serial', 10)
        self._OdometryPublisher = self.create_publisher(Odometry,'odom', 10)
        self._OdometryTransformBroadcaster = tf2_ros.TransformBroadcaster(self)
        
        self._SerialDataGateway = SerialDataGateway("/dev/ttyUSB0", 115200,  self._HandleReceivedLine)
        #self.rosNow = Node.get_clock().now().to_msg()
        #quaternion = Quaternion()
        self.Start()
        self.subscription = self.create_subscription(String,'topic', self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def _HandleReceivedLine(self,  line):
        msg = String()
        msg.data = line
        self.publisher_.publish(msg)
        #self._Counter = self._Counter + 1
        #x = len(line)
        #self.get_logger().info(str(x))
        #if (self._Counter % 50 == 0):
        
        if (len(line) > 0):
                        lineParts = line.split('\t')                 
                        if (lineParts[0] == 'o'):
                                self._Broadcast_Odom(lineParts)
                                return
                        if (lineParts[0] == 's'):
                                self._Broadcast_Servo(lineParts)
                                return
                                
    def _Broadcast_servos(self, lineParts):
        #self.get_logger().info(lineParts[1])
        partsCount = len(lineParts)
        #self.get_logger().info(str(partsCount))
        if (partsCount  < 2):
                pass
        try:
            p1 = (lineParts[4])
            self.get_logger().info(str(p1))
        except:
            self.get_logger().info("Unexpected error:left_lift_joint" + str(sys.exc_info()[0]))
            
    def _Broadcast_Odom(self, lineParts):
        partsCount = len(lineParts)
        if (partsCount  < 6):
            pass
        try:
            x = float(lineParts[1])
            y = float(lineParts[2])
            theta = float(lineParts[3])
                       
            vx = float(lineParts[4])
            omega = float(lineParts[5])
            
            #quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
            #tf2_ros.TransformBroadcaster()
            
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(theta / 2.0)
            quaternion.w = cos(theta / 2.0)		
			
            	
            rosNow = Node.get_clock(self).now().to_msg()
           
            t = TransformStamped()	
            t.header.stamp = rosNow
            t.header.frame_id = "odom"
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            q = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            
            self._OdometryTransformBroadcaster.sendTransform(t)
  
            odometry = Odometry()
            odometry.header.frame_id = "odom"
            odometry.header.stamp = rosNow
            odometry.pose.pose.position.x = x
            odometry.pose.pose.position.y = y
            #odometry.pose.pose.position.z = 0
            odometry.pose.pose.orientation = quaternion

            odometry.child_frame_id = "base_footprint"
            odometry.twist.twist.linear.x = vx
            #odometry.twist.twist.linear.y = 0
            odometry.twist.twist.angular.z = omega

            self._OdometryPublisher.publish(odometry)
            #self.get_logger().info('I heard: "%s"' % t)
		
        except:
            self.get_logger().info("Unexpected error odomfrom arduino.py   :" + str(sys.exc_info()[0]))

                    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
    def Start(self):
        #self.get_logger().info("Starting start function but wait")
        print('in start')
        self._SerialDataGateway.Start()
        message = 's \r'
        self._WriteSerial(message)
        
    def Stop(self):
        self.get_logger().info("Stopping")
        message = 'r \r'
        self._WriteSerial(message)
        sleep(5)
        self._SerialDataGateway.Stop()
        
    def _WriteSerial(self, message):
        #self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
         self._SerialDataGateway.Write(message)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
