#!/usr/bin/env python

import rospy
import numpy as np
import random as ra
from std_msgs.msg import Float32 

def squaregen():
	rospy.init_node('square_size_gen', anonymous=True)
	pub = rospy.Publisher('square_size', Float32, queue_size=10)

        while not rospy.is_shutdown():  
		square_length = ra.uniform(0.05, 0.20)

		pub.publish(square_length)
        	rospy.loginfo(square_length)
       		rate = rospy.Rate(0.05)  # Publish at 0.05hz or every 20 seconds
        	rate.sleep()


if  __name__ == "__main__" :
        try:
                ne = squaregen()
        except rospy.ROSInterruptException: pass

