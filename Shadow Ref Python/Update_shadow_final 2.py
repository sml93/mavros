# -*- coding: utf-8 -*-
"""
Created on Wed Jul 31 23:27:27 2019

@author: emman
"""

#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
# Updated: Tarek Taha : tarek.taha@kustar.ac.ae, Vladimir Ermakov
#    - Changed topic names after re-factoring : https://github.com/mavlink/mavros/issues/233
#    - Use mavros.setpoint module for topics

import rospy
import thread
import threading
import time
import mavros

from numpy import linalg #linear algebra, can be used to find things like eigenvectors etc.
import numpy as np

from math import *
from mavros.utils import *
from mavros import setpoint as SP
from std_msgs.msg import Header #type 
from std_msgs.msg import Float64, Float32 #type from msg generation
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped #geometry
#from tf.transformations import quaternion_from_euler #transformation
from mavros_msgs.srv import CommandLong #service_class
from sensor_msgs.msg import NavSatFix,Range,LaserScan #type
from mavros_msgs.msg import PositionTarget,Thrust
from tf2_msgs.msg import TFMessage
import time
from teraranger_array.msg import RangeArray



class Straight_Inspection:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.x = 2.80 # starting x pos
        self.y = 1.70 # starting y pos
        self.z = None
        self.armed = False  
        self.timeout = 180
        self.rate = rospy.Rate(50) # 10hz is the original 
        self.local_position = PoseStamped() 
        self.counter = 0
        self.counter_2 = 0
        self.counter_3 = 0
        self.counter_4 = 0
        self.z_increment = 0.40
        self.sensor_0 = 0.0
        self.sensor_45 = 0.0
        self.sensor_90 = 0.0
        self.sensor_135 = 0.0 
        self.sensor_180 = 0.0
        self.sensor_225 = 0.0
        self.sensor_270 = 0.0
        self.sensor_315 = 0.0 
        self.kp = 0.05
        self.kp_z = 0.05
        self.kp_est = 0.80
        self.min_alt = 0.80
        self.wall_boundary = 1.50
        self.left_est = 0.0 
        self.top_est = 0.0
        self.right_est = 0.0
        self.bot_est = 0.0
    
        #rospy.Subscriber("subscribed topic",topic type,callback invoked with the message as the first arg)  

        #rospy.Subscriber('/tf', TFMessage, self.callback_tf)
        #rospy.Subscriber("scan",  LaserScan, self.scan_callback)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.position_callback)
        rospy.Subscriber("hub_1/ranges_raw", RangeArray, self.hub_callback)
        self.pub_spt = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pub_vis = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=10)
      
        while not rospy.is_shutdown(): #subsequent functions below are automatically run using this while loop (inert functions)  
            
            print "0: ", self.sensor_0, "alti: ", abs(self.local_position.pose.position.z), "Counter: ", self.counter, "Counter2: ", self.counter_2     
             
            if self.sensor_0 <= 2.40: # first wall
                self.counter = self.counter + 1
               
            if self.counter < 4:
                self.left() 
            
            elif self.counter >= 4:
                self.top_right_bot() 

            self.estimator()
            self.rate.sleep()
 

    def position_callback(self, data): # callback invoked with the message as the first arg
        self.local_position = data 


    # def callback_tf(self,data):
    #     if data.transforms[0].child_frame_id == "base_footprint":
    #         self.y = -data.transforms[0].transform.translation.x
    #         self.x = data.transforms[0].transform.translation.y
    #         self.z = data.transforms[0].transform.translation.z


    # def scan_callback(self,data):
    #     angle_ls = []
    #     for i in range(0,1440,180):
    #         angle_ls.append(-(i+1))

    #     self.scan = data
    #     self.sensor_0 = self.scan.ranges[angle_ls[0]]
    #     self.sensor_45 = self.scan.ranges[angle_ls[7]]
    #     self.sensor_90 = self.scan.ranges[angle_ls[6]]
    #     self.sensor_135 = self.scan.ranges[angle_ls[5]]
    #     self.sensor_180 = self.scan.ranges[angle_ls[4]]
    #     self.sensor_225 = self.scan.ranges[angle_ls[3]]
    #     self.sensor_270 = self.scan.ranges[angle_ls[2]]
    #     self.sensor_315 = self.scan.ranges[angle_ls[1]]  


    def hub_callback(self,msg): # callback invoked with the message as the first arg
        self.collective = msg
        for i in self.collective.ranges:
            self.sensor_90 = self.collective.ranges[1].range 
            self.sensor_0 = self.collective.ranges[3].range
            self.sensor_270 = self.collective.ranges[5].range 
            self.sensor_180 = self.collective.ranges[7].range  


    def left(self):
        # Initialise necessary headers
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "left"

        # Establish desired setpoints
        pos.pose.position.x = self.local_position.pose.position.x - (self.kp * ( self.sensor_0 - self.wall_boundary ))
        pos.pose.position.y = self.local_position.pose.position.y - (self.kp * ( self.sensor_90 - self.wall_boundary ))
        pos.pose.position.z = self.local_position.pose.position.z - (self.kp_z * ( self.local_position.pose.position.z - (self.min_alt) ))    
            
        roll_degrees = 0.0 
        roll = radians(roll_degrees)
        pitch_degrees = 0.0 
        pitch = radians(pitch_degrees)
        yaw_degrees = 180.0 #+ (self.kp * (self.sensor_45 - self.sensor_135)) 
        yaw = radians(yaw_degrees)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        quaternion = [qx,qy,qz,qw]
        pos.pose.orientation = Quaternion(*quaternion)
        
        self.left_est = 1.0
        self.top_est = 0.0

        
        # Publishing to rostopic 
        pos.header.stamp = rospy.Time.now()  
        self.pub_spt.publish(pos) # updated pose from pos.pose.orientation


    def top_right_bot(self):
        # Initialise necessary headers
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "top_right_bot" 

        if self.sensor_0 <= 2.20: # second wall
            self.counter_2 = self.counter_2 + 1


        if self.counter_2 < 4:    
            # Establish desired setpoints
            pos.pose.position.x = self.local_position.pose.position.x - (self.kp * ( self.sensor_90 - self.wall_boundary ))
            pos.pose.position.y = self.local_position.pose.position.y + (self.kp * ( self.sensor_0 - self.wall_boundary ))
            pos.pose.position.z = self.local_position.pose.position.z - (self.kp_z * ( self.local_position.pose.position.z - (self.min_alt) )) 
            
            roll_degrees = 0.0 
            roll = radians(roll_degrees)
            pitch_degrees = 0.0 
            pitch = radians(pitch_degrees)
            yaw_degrees = 90.0 #+ (self.kp * (self.sensor_45 - self.sensor_135)) 
            yaw = radians(yaw_degrees)

            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            quaternion = [qx,qy,qz,qw]
            pos.pose.orientation = Quaternion(*quaternion)
            
            self.left_est = 0.0
            self.top_est = 1.0
            

        elif self.counter_2 >= 4:
                
            pos.pose.position.x = self.local_position.pose.position.x - (self.kp * ( self.sensor_90 - self.wall_boundary ))
            pos.pose.position.y = self.local_position.pose.position.y - (self.kp * ( self.sensor_180 - self.wall_boundary ))
            pos.pose.position.z = self.local_position.pose.position.z - (self.kp_z * ( self.local_position.pose.position.z - (self.min_alt + self.z_increment) ))
        
            roll_degrees = 0.0 
            roll = radians(roll_degrees)
            pitch_degrees = 0.0 
            pitch = radians(pitch_degrees)
            yaw_degrees = 90.0 #+ (self.kp * (self.sensor_45 - self.sensor_135)) 
            yaw = radians(yaw_degrees)

            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            quaternion = [qx,qy,qz,qw]
            pos.pose.orientation = Quaternion(*quaternion)
            
            self.left_est = 0.0
            self.top_est = 1.0
            

        pos.header.stamp = rospy.Time.now()
        self.pub_spt.publish(pos)
        
            
    def estimator(self):

        # Initialise necessary headers
        pos = PoseStamped()
        pos.header = Header()
        pos.header.frame_id = "vision"

        if self.left_est == 1.0 and self.top_est == 0.0: 
            
            if self.sensor_0 == self.x:
                pos.pose.position.x = 0.0
            else:
                pos.pose.position.x = -(self.x - self.sensor_0) * self.kp_est

            if self.sensor_90 == self.y:
                pos.pose.position.y = 0.0 
            else:
                pos.pose.position.y = -(self.y - self.sensor_90) * self.kp_est 

        elif self.left_est == 0.0 and self.top_est == 1.0: 
            
            if self.sensor_90 == self.x:
                pos.pose.position.x = 0.0
            else:
                pos.pose.position.x = -(self.x - self.sensor_90) * self.kp_est 

            if self.sensor_180 == self.y:
                pos.pose.position.y = 0.0 
            else:
                pos.pose.position.y = -(self.y - self.sensor_180) * self.kp_est
                
            
        pos.header.stamp = rospy.Time.now() # Update timestamp for each published SP
        self.pub_vis.publish(pos)
            

if __name__ == '__main__':
    rospy.init_node('shadow_test_node', anonymous=True)

    node = Straight_Inspection()

    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped
     
