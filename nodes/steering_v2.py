#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class steering_ctrl():
    def __init__(self):
        rospy.init_node("Steering_system",anonymous=True)

        self.steering_publisher = rospy.Publisher('/rover_steering_controller/command', JointTrajectory, queue_size=10) 

        self.steering_msg = JointTrajectory()
        self.steering_msg.joint_names = ["steering_leftfront_joint", 
                                         "steering_leftrear_joint" , 
                                         "steering_rightrear_joint", 
                                         "steering_rightfront_joint"]
        

        self.pub_wheel  = (rospy.Publisher("/rover_wheel_leftfront/command",Float64,queue_size=10) ,
                           rospy.Publisher("/rover_wheel_leftrear/command",Float64,queue_size=10)  ,
                           rospy.Publisher("/rover_wheel_rightfront/command",Float64,queue_size=10),
                           rospy.Publisher("/rover_wheel_rightrear/command",Float64,queue_size=10) ) 

        self.leftF, self.leftR, self.rightF, self.rightR = 0,0,0,0
        self.steer_arr=np.array( (0,0,0,0) )

        self.Twist_data =Twist()
        rospy.Subscriber("/drive_system/twist",Twist,self.twist_callback,queue_size=10)
        
        while not rospy.is_shutdown():
            self.action()
            rospy.sleep(0.01)

            
    
    def twist_callback(self,data):
        #print("sublandi")
        self.Twist_data=data

    def tank_rotation(self):
        self.steering_msg = JointTrajectory() 
        self.steering_msg.joint_names = ["steering_leftfront_joint", "steering_leftrear_joint", "steering_rightrear_joint", "steering_rightfront_joint"] 
        self.steering_points = JointTrajectoryPoint()
        self.steer_arr = np.deg2rad((np.array((-45,45,-45,45))))
        self.steering_points.positions = self.steer_arr
        self.steering_points.time_from_start = rospy.Duration.from_sec(0.005)
        self.steering_msg.points.append(self.steering_points) 
        self.steering_publisher.publish(self.steering_msg)
        self.wheel = (self.Twist_data.angular.z*1.7584/-2*np.pi  , self.Twist_data.angular.z*1.7584/-2*np.pi,
                      self.Twist_data.angular.z*1.7584/2*np.pi , self.Twist_data.angular.z*1.7584/2*np.pi)
        self.pub_wheel[0].publish(self.wheel[0])
        self.pub_wheel[1].publish(self.wheel[1])
        self.pub_wheel[2].publish(self.wheel[2])
        self.pub_wheel[3].publish(self.wheel[3])

    def stLim(self,steer_deg,dDeg,is_eye=True):
        return (steer_deg+dDeg)%(np.pi+is_eye*10**-7)-np.pi/2

    def cartesian_wheel_rot(self,deg):

        zeroPos = np.array((-np.pi/2,np.pi/2,-np.pi/2,np.pi/2))

        self.steering_msg = JointTrajectory() 
        self.steering_msg.joint_names = ["steering_leftfront_joint", "steering_leftrear_joint", "steering_rightrear_joint", "steering_rightfront_joint"] 
        self.steering_points = JointTrajectoryPoint()
        
        self.steer_arr=(self.stLim(zeroPos[0],deg,is_eye=True),
                        self.stLim(zeroPos[1],deg,is_eye=True),
                        self.stLim(zeroPos[2],deg,is_eye=True),
                        self.stLim(zeroPos[3],deg,is_eye=True))

        self.steering_points.positions = self.steer_arr
        self.steering_points.time_from_start = rospy.Duration.from_sec(0.005)
        self.steering_msg.points.append(self.steering_points)
        self.steering_publisher.publish(self.steering_msg)
        self.wheel= (np.sqrt(self.Twist_data.linear.y**2+self.Twist_data.linear.x**2),
                     np.sqrt(self.Twist_data.linear.y**2+self.Twist_data.linear.x**2),
                     np.sqrt(self.Twist_data.linear.y**2+self.Twist_data.linear.x**2),
                     np.sqrt(self.Twist_data.linear.y**2+self.Twist_data.linear.x**2))
        
        if self.Twist_data.linear.x>=0 :
            self.pub_wheel[0].publish(self.wheel[0])
            self.pub_wheel[1].publish(self.wheel[1])
            self.pub_wheel[2].publish(self.wheel[2])
            self.pub_wheel[3].publish(self.wheel[3])
        else:
            self.pub_wheel[0].publish(-self.wheel[0])
            self.pub_wheel[1].publish(-self.wheel[1])
            self.pub_wheel[2].publish(-self.wheel[2])
            self.pub_wheel[3].publish(-self.wheel[3])




    def cartesian_motion(self):
        degWheel = np.arctan(self.Twist_data.linear.y/(self.Twist_data.linear.x+10**-7))
        self.cartesian_wheel_rot(degWheel)
        #print self.axes
        #print degWheel

    def empty_state(self): #TODO silinecek bura
        steering_publisher = rospy.Publisher('/rover_steering_controller/command', JointTrajectory, queue_size=10) 
        steering_msg = JointTrajectory() 
        steering_msg.joint_names = ["steering_leftfront_joint", "steering_leftrear_joint", "steering_rightrear_joint", "steering_rightfront_joint"] 
        steering_points = JointTrajectoryPoint() # contain joint positions
        steer_arr = np.deg2rad((np.array((-45,45,-45,45))))
        steering_points.positions = steer_arr# steerinlerin konumları liste olarak eşitleyebilirsin benim yaptığım gibi
        steering_points.time_from_start = rospy.Duration.from_sec(0.005)
        steering_msg.points.append(steering_points) 
        steering_publisher.publish(steering_msg)

    def action(self):
        if abs(self.Twist_data.angular.z)>0:
            self.tank_rotation()
        else:
            self.cartesian_motion()
        


if __name__ == "__main__":
    steering_ctrl()
    rospy.spin()