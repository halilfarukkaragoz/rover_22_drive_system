#include "../include/rover_22_drive_system/steering_v2_odom.h"

using namespace std;

void mySigintHandler(int sig){ros::shutdown();}

Steering_v2_odom::Steering_v2_odom(){
    x = y = z = yaw_init = yaw_current = mean_vel = 0;
    //NH.getParam("/wheel_radius", wheel_radius);
    wheel_radius = 0.14;
    circumference = 2 * Pi * wheel_radius;
    frequency = 20;
    current_time = last_time = ros::Time::now();
    //Question marks has to be filled.
    odomPub = NH.advertise<nav_msgs::Odometry>("drive_system/odometry",10);
    imuSub = NH.subscribe("/imu/data", 10, &Steering_v2_odom::imu_callback, this);
    steeringSub = NH.subscribe("/drive_system/gazebo_encoder", 10, &Steering_v2_odom::steering_callback, this);
    controller();
}
void Steering_v2_odom::imu_callback(const sensor_msgs::Imu& msg){
    //Update last yaw.
    yaw_last = yaw_current;
    //Convert quaternion to euler (m and q are temporary,
    //roll and pitch are not used in this mode.)
    tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll_current, pitch_current, yaw_current);
    //Calculation of initial yaw by taking average of the first 5 message.
    //Add extra 90 degrees due to the imu's reference which is magnetic east.
    yaw_current += Pi/2;
}
void Steering_v2_odom::steering_callback(const rover_22_drive_system::Steering& msg){
    steering_msg = msg;
    for(int i=0; i<steering_msg.steering_angle.size(); i++){//speed and angle size are the same.
        wheel_angles[i] = (steering_msg.steering_angle[i]-2048)*Pi/2048;//Conversion to radian.
        wheel_vels[i] = steering_msg.wheel_speed[i]/6.67;
        mean_vel += steering_msg.wheel_speed[i]/steering_msg.steering_angle.size();
    }
}

void Steering_v2_odom::controller(){
    ros::Rate rate(frequency);
    while(ros::ok){
        ros::spinOnce();
        current_time = ros::Time::now();
        //cartesian case
        dt = (current_time-last_time).toSec();
        //If right front & left rear sheel velocities are in the opposite directions,
        //It means that there is a tank rotation.
        //Else, the vehicle will move in cartesian path.
        if(wheel_angles[0]*wheel_angles[2] < 0){
            dx = dy = 0;
            dth = yaw_current - yaw_last;
        }
        else{
            dx = dt*wheel_vels[0]*cos(wheel_angles[0]+yaw_current);//All the angles are the same.
            dy = dt*wheel_vels[0]*sin(wheel_angles[0]+yaw_current);//All the angles are the same.
            dth = 0;
        }

        x += dx;
        y += dy;

        odom_quat = tf::createQuaternionMsgFromYaw(yaw_current);
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = dx/dt;
        odom_msg.twist.twist.linear.y = dy/dt;
        odom_msg.twist.twist.angular.z = dth/dt;

        odomPub.publish(odom_msg);
        rate.sleep();

        last_time = current_time;
    }
}

int main(int argc, char *argv[]){   
    ros::init(argc, argv, "Steering_system",ros::init_options::NoSigintHandler);
    Steering_v2_odom steering;
    signal(1, mySigintHandler);
}

