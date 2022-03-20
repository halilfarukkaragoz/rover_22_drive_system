#pragma once
#include <iostream>
#include <cmath>
#include <ctime>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
//#include <std_msgs/Float64MultiArray.h>
//#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
//#include "../../msg/Steering.msg"
//#include </home/kutaygallo/old_steering_ws/devel/include/rover_22_control/Steering.h>
#include "rover_22_drive_system/Steering.h"
//#include </home/kutaygallo/old_steering_ws/devel/include/rover_22_control/Steering.h>

using namespace std;

class Steering_v2_odom{
    private:
        ros::NodeHandle NH;
        double x, y, z, wheel_radius, Pi = M_PI, circumference;
        float mean_vel, wheel_angles[4], wheel_vels[4];//
        double yaw_init, yaw_current, yaw_last; //counter issue
        double roll_current, pitch_current;
        int frequency;
        double dt, dx, dy, dth;

        rover_22_drive_system::Steering steering_msg;
        nav_msgs::Odometry odom_msg;
        geometry_msgs::TransformStamped odom_trans;
        geometry_msgs::Quaternion odom_quat;
        tf::TransformBroadcaster odom_broadcaster;
        ros::Subscriber imuSub, steeringSub;
        ros::Publisher odomPub;
        ros::Time current_time, last_time;
    public:
        Steering_v2_odom();
        void imu_callback(const sensor_msgs::Imu& msg);
        void steering_callback(const rover_22_drive_system::Steering& msg);
        void controller();  
};