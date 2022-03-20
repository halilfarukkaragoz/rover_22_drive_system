#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


if __name__ == "__main__":

	rospy.init_node('sim_control')
	steering_publisher = rospy.Publisher('/rover_steering_controller/command', JointTrajectory, queue_size=10) # Arm jointtrajectory publisher publish all joints at the same time

	rate = rospy.Rate(1) # rate value of 150 Hz

	pub_lf= rospy.Publisher("/rover_wheel_leftfront/command",Float64,queue_size=10)
	pub_lr= rospy.Publisher("/rover_wheel_leftrear/command",Float64,queue_size=10)
	pub_rf= rospy.Publisher("/rover_wheel_rightfront/command",Float64,queue_size=10)
	pub_rr= rospy.Publisher("/rover_wheel_rightrear/command",Float64,queue_size=10)



	def joy_callback(data):
		right_axis= data.axes[3:5]

		steering_msg = JointTrajectory() # message for joint publisher
		steering_msg.joint_names = ["steering_leftfront_joint", "steering_leftrear_joint", "steering_rightrear_joint", "steering_rightfront_joint"] # jointlerin isimleri bizde
		steering_points = JointTrajectoryPoint() # contain joint positions
		lf_velocity= 0.
		lr_velocity= 0.
		rf_velocity= 0.
		rr_velocity= 0.
        
		if abs(right_axis[0])>abs(right_axis[1]):
			

			steer_arr = np.deg2rad((np.array((-45,45,-45,45))))
			steering_points.positions = steer_arr# steerinlerin konumları liste olarak eşitleyebilirsin benim yaptığım gibi
			steering_points.time_from_start = rospy.Duration.from_sec(0.005)
			
			lf_velocity= -1.*right_axis[0]*20.0
			lr_velocity= -1.*right_axis[0]*20.0
			rf_velocity= right_axis[0]*20.0
			rr_velocity= right_axis[0]*20.0

		else:
			steer_arr = np.deg2rad((np.array((0,0,0,0))))
			steering_points.positions = steer_arr# steerinlerin konumları liste olarak eşitleyebilirsin benim yaptığım gibi
			steering_points.time_from_start = rospy.Duration.from_sec(0.005)

			lf_velocity= right_axis[1]*50.0
			lr_velocity= right_axis[1]*50.0
			rf_velocity= right_axis[1]*50.0
			rr_velocity= right_axis[1]*50.0

		steering_msg.points.append(steering_points) # listenin appendlenmesi gerek

			
		steering_publisher.publish(steering_msg) # publish arm positions
		pub_lf.publish(lf_velocity)
		pub_lr.publish(lr_velocity)
		pub_rf.publish(rf_velocity)
		pub_rr.publish(rr_velocity)
		

	while not rospy.is_shutdown():
		

        	rospy.Subscriber("/joy",Joy,joy_callback,queue_size=10)

        	rate.sleep()
