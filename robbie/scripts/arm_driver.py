#!/usr/bin/env python3
'''

Created March, 2017

@author: Peter Heim

  arm_driver.py - gateway to Arduino based arm controller
  Copyright (c) 2011 Peter Heim.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import rclpy
from rclpy.node import Node
import sys
from time import time
import tf2_ros
import math
from math import sin, cos, pi, radians, degrees
from robbie_msg.msg import HeadJoint, ArmJoint

from std_msgs.msg import String
from std_msgs.msg import Float64, Float32

from trajectory_msgs.msg import JointTrajectory

from sensor_msgs.msg import JointState
from SerialDataGateway3 import SerialDataGateway

class ArmDriver(Node):

    def __init__(self):
        super().__init__('arm_driver')
        self.get_logger().info('starting arm driver control')
        self.publisher_ = self.create_publisher(String, 'arm_driver_serial', 10)
        self._arm_JointPublisher = self.create_publisher(JointState, 'joint_states',  5)
        self._SerialDataGateway = SerialDataGateway("/dev/arduino-nano", 115200,  self._HandleReceivedLine)
        self.subscription = self.create_subscription(ArmJoint,'right_arm_controller/command', self._HandleJoint_1_Command,10)
        self.subscription = self.create_subscription(HeadJoint,'head_controller/command', self._HandleJoint_2_Command,10)
        self.subscription = self.create_subscription(Float32,'right_gripper_controller/command', self._HandleJoint_3_Command,10)
        self.subscription
        self.Start()
        
    def _HandleReceivedLine(self,  line):
        msg = String()
        msg.data = line
        self.publisher_.publish(msg)
        #self.get_logger().info(str(msg))
        #self._Counter = self._Counter + 1
        #x = len(line)
        #self.get_logger().info(str(x))
        #if (self._Counter % 50 == 0):
        
        if (len(line) > 0):
            lineParts = line.split('\t')                 
            if (lineParts[0] == 'l'):
                self._Broadcast_servos(lineParts)
                return

    def _Broadcast_servos(self, lineParts):
         P1 = Float32()
         P2 = Float32()
         P3 = Float32()
         P4 = Float32()
         P5 = Float32()
         P6 = Float32()
         P7 = Float32()
         P8 = Float32()
         P9 = Float32()
         P10 = Float32()
         P11 = Float32()
         P12 = Float32()
         P13 = Float32()
         P14 = Float32()
         P15 = Float32()
         P16 = Float32()
         P17 = Float32()
         P18 = Float32()
         P19 = Float32()
         P1.data = (0 + (radians(float(lineParts[1]))))-1.57 # pan
         P2.data = (0 + (radians(float(lineParts[2]))))-1.57 # l1
         P3.data = (0 + (radians(float(lineParts[3]))))-1.57# l2
         P4.data = (0 + (radians(float(lineParts[4]))))-1.57# l3
         P5.data = (0 + (radians(float(lineParts[5]))))-1.57# l4
         P6.data = (0 + (radians(float(lineParts[6]))))-1.57# l5
         P7.data = (0 + (radians(float(lineParts[7]))))-1.57# l gripper
         P8.data = (0 + (radians(float(lineParts[8]))))-1.57#r0
         P9.data = (0 + (radians(float(lineParts[9]))))-1.57#r1
         P10.data = (0 + (radians(float(lineParts[10]))))-1.57#r2
         P11.data = (0 + (radians(float(lineParts[11]))))-1.57#r3
         P12.data = (0 + (radians(float(lineParts[12]))))-2.36#r4
         P13.data = (0 + (radians(float(lineParts[13]))))-1.57#r gripper
         P14.data = (0 + (radians(float(lineParts[14]))))-1.57# head roll
         P15.data = (0 + (radians(float(lineParts[15]))))-1.57#head pitch
         P16.data = (0 + (radians(float(lineParts[16]))))-1.57#head yaw
         P14.data = 0.0 #P14.data/6.03
         P15.data = P15.data/6.03
         
         
         Joint_State = JointState()
         Joint_State.header.frame_id = 'base_footprint'
         Joint_State.header.stamp = Node.get_clock(self).now().to_msg()
         Joint_State.name = ['j0_joint','left_j1_joint', 'left_j2_joint', 'left_j3_joint', 'left_j4_joint', 'left_j5_joint','left_gripper_joint','right_j1_joint', 'right_j2_joint', 'right_j3_joint', 'right_j4_joint', 'right_j5_joint', 'right_gripper_joint','head_roll_joint','head_pitch_joint','head_yaw_joint']
         Joint_State.position = [P1.data , P2.data, P3.data, P4.data, P5.data, P6.data, P7.data,  P8.data, P9.data, P10.data, P11.data, P12.data, P13.data, P14.data, P15.data, P16.data]
         Joint_State.header.stamp = Node.get_clock(self).now().to_msg()
         self._arm_JointPublisher.publish(Joint_State)
         #self.get_logger().info(str(Joint_State))
         
    def _HandleJoint_1_Command(self, Command):
                """ Handle movement requests.
                    for all joints
                    send message in degrees 0 -180
                """
                
                j0 = degrees(Command.j0) +90 # ok
                j1 = degrees(Command.j1) +90
                j2 = degrees(Command.j2) +90
                j3 = degrees(Command.j3) +90
                j4 = degrees(Command.j4) +90
                j5 = degrees(Command.j5) +90
                j6 = degrees(Command.j6) +90
                
                message = 'r %d %d %d %d %d %d %d  \r' % (j0, j1, j2, j3, j4, j5, j6)  
                self.get_logger().info(str(message))
                self._WriteSerial(message)
                
    def _HandleJoint_2_Command(self, Command):
                """ Handle movement requests.
                    head 
                    send message in degrees 0 -180
                """
                j1 = (Command.j1)* 6.03
                
                j0 = j1 * -1.0
                j0 = degrees(j0) +90 # roll
                j1 = degrees(j1) +90 # pitch
                j2 = degrees(Command.j2) +90 # yaw
               
                
                
                
                message = 'h %d %d %d  \r' % (j0, j1, j2)  
                #self.get_logger().info(str(message))
                self._WriteSerial(message)
                  
    def _HandleJoint_3_Command(self, Command):
                """ Handle movement requests.
                    right gripper
                    send message in degrees 0 -180
                """
                #msg =Float32()
                
                j0 = degrees(Command.data) +90
                
                message = 'b %d  \r' % (j0)  
                self.get_logger().info(str(j0))
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
         


def main(args=None):
    rclpy.init(args=args)

    arm_driver = ArmDriver()

    rclpy.spin(arm_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

