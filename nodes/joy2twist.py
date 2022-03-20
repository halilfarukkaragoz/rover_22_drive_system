#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist



class j2t():
    def __init__(self):
        self.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.twist_pub = rospy.Publisher("/drive_system/twist",Twist,queue_size=10)
        while not rospy.is_shutdown():
            rospy.init_node("Joy2Twist",anonymous=True)
            rospy.Subscriber("/joy",Joy,self.joy_callback,queue_size=10)
            self.joy2twist()
            rospy.sleep(0.001)
            self.twist_pub.publish(self.Twist_data2pub)


    def joy_callback(self,data):
        self.axes= np.array(data.axes)
        self.buttons= np.array(data.buttons)   

    def joy2twist(self):
        self.Twist_data2pub = Twist()
        
        self.Twist_data2pub.angular.x = 0.
        self.Twist_data2pub.angular.y = 0.
        self.Twist_data2pub.angular.z = self.axes[0]*2*np.pi
        self.Twist_data2pub.linear.x  = self.axes[4]*20
        self.Twist_data2pub.linear.y  = self.axes[3]*20
        self.Twist_data2pub.linear.z  = 0.
        


if __name__ == "__main__":
    j2t()
    rospy.spin()