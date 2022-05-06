import rclpy
import sys
import time
import tf2_ros
import math
from math import sin, cos, pi, radians, degrees
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

from SerialDataGateway3 import SerialDataGateway

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.get_logger().info('starting arduino control')
         #         Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = 0
        self.ticksPerMeter = int(25837) #68948
        self.wheel_track = float(0.27)
        self.lastTime = Node.get_clock(self).now()
        self.publisher_ = self.create_publisher(String, 'arm_serial', 10)
        self._OdometryPublisher = self.create_publisher(Odometry,'odom', 10)
        self._OdometryTransformBroadcaster = tf2_ros.TransformBroadcaster(self)
        self._batteryPublisher = self.create_publisher(BatteryState,'battery/state', 10)
        self._anglePublisher = self.create_publisher(Float32,'angle', 10)
        self._SerialDataGateway = SerialDataGateway("/dev/linorobot", 115200,  self._HandleReceivedLine)
        
        self.Start()
        self.subscription = self.create_subscription(Twist,'cmd_vel', self._HandleVelocityCommand,10)
        self.subscription  # prevent unused variable warning
        self.srv = self.create_service(Empty, 'autodock', self.AutoDock_callback)
        
        now = Node.get_clock(self).now()   
        #self.then = self.now # time for determining dx/dy
        self.last = (now.nanoseconds * 1e-6) + 10
        #self.t_next = now + self.t_delta
        self.odom_linear_scale_correction = 1
        self.odom_angular_scale_correction = 1
        
        

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
                        if (lineParts[0] == 'a'):
                                self._Broadcast_angle(lineParts)
                                return
                        if (lineParts[0] == 'b'):
                                self._Broadcast_battery(lineParts)
                                return                     
                                
    def _Broadcast_angle(self, lineParts):
        #self.get_logger().info(lineParts[1])
        partsCount = len(lineParts)
        #self.get_logger().info(str(partsCount))
        if (partsCount  < 2):
                pass
        msg = Float32()
        
        msg.data = float(lineParts[1])
        self._anglePublisher.publish(msg)
        #self.get_logger().info(str(msg))
       
            
    def _Broadcast_battery(self, lineParts):
        #self.get_logger().info(lineParts[1])
        partsCount = len(lineParts)
        #self.get_logger().info(str(partsCount))
        volt = float(lineParts[1])* 0.015867159
        per = int((volt / 13.4) * 100)
        msg = BatteryState()
        msg.header.stamp = Node.get_clock(self).now().to_msg()
        msg.voltage = float(lineParts[1])* 0.015867159
        msg.percentage = float(per)
        self._batteryPublisher.publish(msg)
        #self.get_logger().info(str(msg))
        
            
    def _Broadcast_Odom(self, lineParts):
        partsCount = len(lineParts)
        if (partsCount  < 6):
            pass
        try:
            self.left_enc_r = int(lineParts[1])
            self.right_enc_r = int(lineParts[2])
            
        except:
            self.get_logger().info("Unexpected error odomfrom arduino.py   :" + str(sys.exc_info()[0]))
        
        now = Node.get_clock(self).now()
        tt = now.nanoseconds * 1e-6
        #loop to publish data at 50hz
        if tt > self.last:
            dt = tt - self.last
            #print(dt)
            
            
            # Calculate odometry
            
            if self.enc_left == None:
                dright = 0
                dleft = 0
                
            else:
                dright = (self.right_enc_r - self.enc_right) / self.ticksPerMeter
                dleft = (self.left_enc_r - self.enc_left) / self.ticksPerMeter
                #print(dright)
            self.enc_right = self.right_enc_r
            self.enc_left = self.left_enc_r
            #print()
            dxy_ave = self.odom_linear_scale_correction * (dright + dleft) / 2.0
            dth = self.odom_angular_scale_correction * (dright - dleft) / float(self.wheel_track)
            vxy = dxy_ave / dt
            vth = dth / dt
            #print(dxy_ave)   
            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
            if (dth != 0):
                self.th += dth 
            
            
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)
            
            rosNow = Node.get_clock(self).now().to_msg()
            
            t = TransformStamped()
            	
            t.header.stamp = rosNow
            t.header.frame_id = "odom"
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
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
            odometry.pose.pose.position.x = float(self.x)
            odometry.pose.pose.position.y = float(self.y)
            #odometry.pose.pose.position.z = 0
            odometry.pose.pose.orientation = quaternion

            odometry.child_frame_id = "base_footprint"
            odometry.twist.twist.linear.x = vxy
            #odometry.twist.twist.linear.y = 0
            odometry.twist.twist.angular.z = vth

            self._OdometryPublisher.publish(odometry)
            
            #self.get_logger().info(self.pose.theta)
            
        self.last = tt + 70
        


        
    def Enc_reset(self):       
        message = 'c \r'
        self._WriteSerial(message)
        
    def Start(self):
        #self.get_logger().info("Starting start function but wait")
        
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
         
    def _HandleVelocityCommand(self, twistCommand):
        """ Handle movement requests. """
        x = twistCommand.linear.x        # m/s
        th = twistCommand.angular.z      # rad/s

        if x == 0:
            # Turn in place
            right = th * 0.25 / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * 0.25 / 2.0
            right = x + th * 0.25 / 2.0
            
        v_des_left = int(left * 1000)# 
        v_des_right = int(right * 1000)#ticks_per_meter 15915
        self.get_logger().info("Handling twist ommand: " + str(v_des_left) + "," + str(v_des_right))
        message = 'm %.2f %.2f\r' % (v_des_left, v_des_right)
        #self.get_logger().info("Sending speed command message: " + message)
        self._WriteSerial(message)
        
    def AutoDock_callback(self, request, responce):
        responce
        message = 'a \r' 
        self.get_logger().info("Sending auto dock message: " + message)
        self._WriteSerial(message)
        return responce
        

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
