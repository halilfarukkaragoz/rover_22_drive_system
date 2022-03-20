#!/usr/bin/env python

"""
Author: Yunus Emre Karaoglan
Date: 26-02-2022

"""

import rospy
from rover_22_drive_system.msg import Steering
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from math import pi, cos, sin
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class SteeringOdometry:
    """ class that calculates odometry depends on wheel angles, wheel velocities and IMU data
    """
    def __init__(self):
        rospy.init_node("steer_odom",anonymous=False)

        rospy.Subscriber("drive_system/gazebo_encoder", Steering, self.encoder_callback)
        rospy.Subscriber("/imu1/data", Imu, self.imu_callback)

        self.odomPub = rospy.Publisher("/odometry/wheel",Odometry,queue_size=10)
        self.encoder_data = Steering()

        self.x = 0
        self.y = 0
        self.vx = 0
        self.vy = 0

        self.isYawInit = False # Flag ---> Init yaw is found or not 
        self.initYaw = 0
        self.yawDiff = 0
        self.yawCounter = 0
        self.roll = 0
        self.pitch = 0

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"

        self.wheel_angles_encoder = [0, 0, 0, 0]
        self.wheel_angles = [0, 0, 0, 0]
        self.wheel_vels = [0.0, 0.0,  0.0, 0.0]

        self.initTime = rospy.Time.now()
        self.currentTime = rospy.Time.now()
        self.lastTime = rospy.Time.now()
        self.RATE = rospy.Rate(30)

        self.main()


    def imu_callback(self, data):
        """ Callback function for IMU data

        Args:
            data (sensor_msgs.Imu)):  Incoming data
        """
	
        self.roll, self.pitch, yaw = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        if not self.isYawInit and abs(yaw) <= 2*pi and self.yawCounter < 5:
            self.yawCounter += 1           
            self.initYaw += yaw
        if self.yawCounter == 5:
            self.initYaw /= 5
        self.isYawInit = True
        
        self.yawDiff = yaw - self.initYaw


    def encoder_callback(self, data): 
        """ Callback function for encoder data coming from wheel encoder and steering encoder

        Args:
            data (Steering.msg): Contains wheel speed and wheel angle for each wheel 
        """
	#print("callback")
        self.wheel_vels = data.wheel_speed
	#print(self.wheel_vels)
        self.wheel_angles = list(data.steering_angle)
        self.find_wheel_angles()
    
    def find_wheel_angles(self):
        """ Function to find angle of each wheel
        """
        for i in range(0,4):
            self.wheel_angles[i] = self.map_the_angle(self.wheel_angles[i])

    def map_the_angle(self, angle):
        """ Function to map the angle between -pi to pi

        Args:
            angle (int): data gathered from steering encoder, 1024 to 3072 

        Returns:
            radian: mapped radian data 
        """
        return (angle - 2048)*pi/2048

    def find_rover_velocity(self):
        """ Sum of all wheel velocities on x and y axes
        """
        for i in range(0,4):
	    #print("asd")
            self.vx = self.wheel_vels[i] * cos(self.wheel_angles[i])
            self.vy = self.wheel_vels[i] * sin(self.wheel_angles[i])

    def publishOdom(self, x, y):
        """ 

        Args:
            x (_type_): pose of the rover on x-axis
            y (_type_): pose of the rover on y-axis
        """
        self.odom_msg.header.stamp = rospy.Time.now()
        
        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y
        
        quaternion = quaternion_from_euler(self.roll, self.pitch, self.yawDiff)
        self.odom_msg.pose.pose.orientation.x = quaternion[0]
        self.odom_msg.pose.pose.orientation.y = quaternion[1]
        self.odom_msg.pose.pose.orientation.z = quaternion[2]
        self.odom_msg.pose.pose.orientation.w = quaternion[3]

        self.odomPub.publish(self.odom_msg)


    def main(self):
        """ Main function to do everything in loop
        """
        while not rospy.is_shutdown():
            self.currentTime = rospy.Time.now()
            dt = (self.currentTime - self.lastTime).to_sec()
            self.lastTime = self.currentTime
	    #print(self.yawDiff)
            print("vx:",self.vx," vy:",self.vy)
            self.find_rover_velocity()
            self.x += dt*(self.vx * cos(self.yawDiff) - self.vy * sin(self.yawDiff))
            self.y += dt*(self.vx * sin(self.yawDiff) + self.vy * cos(self.yawDiff))
	    #print("x:",self.x," y:",self.y)
            self.publishOdom(self.x, self.y)

if __name__=="__main__":
    try:
        SteeringOdometry()
    except KeyboardInterrupt:
        rospy.signal_shutdown()
