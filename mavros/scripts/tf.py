# -*- coding: utf-8 -*-
"""
Created on Tue Jun 11 15:06:57 2019

@author: Frank
"""

import rospy
#from sensor_msgs.msg import NavSatFix,Range #type
from tf2_msgs.msg import TFMessage
import time
import math


#test

class Straight_Inspection:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):

		self.x = None

		self.y = None

		self.rate = rospy.Rate(50) # 10hz is the original
		rospy.Subscriber('/tf',  TFMessage, self.callback_tf)
        
		while not rospy.is_shutdown(): #subsequent functions below are automatically run using this while loop (inert functions)  
			while not self.x or not self.y:
				print "no tf"
				time.sleep(0.01)
				
			print "X", self.x, "Y", self.y
			
			#drone_tf_input
			#SLAM_tf_output = self.tf.transforms[0].transform.translation
			
			#drone_tf_input.x = SLAM_tf_output.y	
			#drone_tf_input.y = -SLAM_tf_output.x
			
			self.rate.sleep()
			
			
    def callback_tf(self,data):
    	if data.transforms[0].child_frame_id == "base_footprint":
			self.y = -data.transforms[0].transform.translation.x
			self.x = data.transforms[0].transform.translation.y



    
if __name__ == '__main__':
    rospy.init_node('shadow_test_node', anonymous=True)

    node = Straight_Inspection()

    rospy.spin()  # spin() simply keeps python from exiting until this node is stopped
     
