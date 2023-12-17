#!/usr/bin/env python
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
from robbie_msg.msg import ArmJoint, HeadJoint, ArmState

from std_msgs.msg import String
from std_msgs.msg import Float64, Float32

from sensor_msgs.msg import JointState

class ArmDriver(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.get_logger().info('starting joint state publisher')
        # Create a timer that will gate the node actions twice a second
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self._Broadcast_joints)
        self.publisher_ = self.create_publisher(String, 'arm_driver_serial', 10)
        self.R_j0 = 0.0
        self.R_j1 = 0.0
        self.R_j2 = 0.0
        self.R_j3 = 0.0
        self.R_j4 = 0.0
        self.R_j5 = 0.0
        self.R_j6 = 0.0
        self.R_G = 0.0
        self.L_j0 = 0.0
        self.L_j1 = 0.0
        self.L_j2 = 0.0
        self.L_j3 = 0.0
        self.L_j4 = 0.0
        self.L_j5 = 0.0
        self.L_j6 = 0.0
        self.L_G = 0.0
        self.pan = 0.0
        self.tilt =0.0
        
        
        self._arm_JointPublisher = self.create_publisher(JointState, 'joint_states',  5)
        
        #self.subscription_R = self.create_subscription(ArmJoint,'R_joints', self._HandleJoint_1_Command,10)
        #self.subscription_L = self.create_subscription(ArmJoint,'L_joints', self._HandleJoint_2_Command,10)
        self.subscription_H = self.create_subscription(HeadJoint,'H_joints', self._HandleJoint_3_Command,10)
        #self.subscription_GR = self.create_subscription(Float32,'R_gripper', self._HandleJoint_4_Command,10)
        #self.subscription_LG = self.create_subscription(Float32,'L_gripper', self._HandleJoint_5_Command,10)
        self.subscription_AS = self.create_subscription(ArmState,'arm_state', self._HandleJoint_6_Command,10)
        #self.subscription
        #self.Start()
        
    

    def _Broadcast_joints(self):
         
        
         #P19 = float(lineParts[19])
         Joint_State = JointState()
         Joint_State.header.frame_id = 'base_footprint'
         Joint_State.name = ['R_j0' ,'R_j1', 'R_j2' ,'R_j3' ,'R_j4' ,'R_j5' ,'R_j6' ,'R_G' ,'L_j0' ,'L_j1', 'L_j2' ,'L_j3' ,'L_j4' ,'L_j5' ,'L_j6' ,'L_G' , 'pan','tilt']
         Joint_State.position = [self.R_j0, self.R_j1, self.R_j2, self.R_j3, self.R_j4, self.R_j5, self.R_j6, self.R_G, self.L_j0, self.L_j1, self.L_j2, self.L_j3, self.L_j4, self.L_j5, self.L_j6, self.L_G, self.pan, self.tilt]
         Joint_State.header.stamp = Node.get_clock(self).now().to_msg()
         self._arm_JointPublisher.publish(Joint_State)
         #self.get_logger().info(str(Joint_State))
         
    def _HandleJoint_1_Command(self, Command):
                """ Handle movement requests.
                    right joints
                    
                """
                
                self.R_j0 = float(Command.j0)
                self.R_j1 = float(Command.j1)
                self.R_j2 = float(Command.j2)
                self.R_j3 = float(Command.j3)
                self.R_j4 = float(Command.j4)
                self.R_j5 = float(Command.j5)
                self.R_j6 = float(Command.j6)
                

    def _HandleJoint_2_Command(self, Command):
                """ Handle movement requests.
                    left joints
                """
                
                self.L_j0 = float(Command.j0)
                self.L_j1 = float(Command.j1)
                self.L_j2 = float(Command.j2)
                self.L_j3 = float(Command.j3)
                self.L_j4 = float(Command.j4)
                self.L_j5 = float(Command.j5)
                self.L_j6 = float(Command.j6)
                
    def _HandleJoint_3_Command(self, Command):
                """ Handle movement requests.
                    head joints
                """
                
                self.pan = float(Command.j0)
                self.tilt = float(Command.j1)
                
    def _HandleJoint_4_Command(self, Command):
                """ Handle movement requests.
                    right gripper joints
                """
                
                self.R_G = Command.data                       
                #self.get_logger().info(str(self.R_G))
                
    def _HandleJoint_5_Command(self, Command):
                """ Handle movement requests.
                    right gripper joints
                """
                
                self.L_G = Command.data                         
                #self.get_logger().info(str(self.L_G))                
                
    def _HandleJoint_6_Command(self, Command):
                """ Handle movement requests.
                    right gripper joints
                """            
                self.L_j0 = float(Command.l0)
                self.L_j1 = float(Command.l1)
                self.L_j2 = float(Command.l2)
                self.L_j3 = float(Command.l3)
                self.L_j4 = float(Command.l4)
                self.L_j5 = float(Command.l5)
                self.L_j6 = float(Command.l6)   
                self.R_G = float(Command.l7)
                self.R_j0 = float(Command.r0)
                self.R_j1 = float(Command.r1)
                self.R_j2 = float(Command.r2)
                self.R_j3 = float(Command.r3)
                self.R_j4 = float(Command.r4)
                self.R_j5 = float(Command.r5)
                self.R_j6 = float(Command.r6)            
                self.R_G = float(Command.r7)
      
                         
        
    
         


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

