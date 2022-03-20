#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rover_22_drive_system/Steering.h"
#include <iostream>
#include <signal.h>
#include <cmath>
#include <valarray>
#include <string>

using namespace std;
#define d1vector valarray<double> 
#define d2vector valarray<valarray<double>>

void mySigintHandler(int sig) {ros::shutdown();}
class gazebo_encoder
{
private:
    ros::NodeHandle nh;
    ros::Subscriber jointStateSub;
    ros::Publisher gazeboEncoder;
    rover_22_drive_system::Steering Steering_msg;
    double Pi = M_PI;
    float wheel_speed[4], steer_angle[4];
    d1vector orientationQ = {0,0,0,1},Dnow={0,0,0,0};

public:
    gazebo_encoder(int argc, char *argv[]){
        ros::Rate rate(150);
        jointStateSub = nh.subscribe("/joint_states",10, &gazebo_encoder::joint_state_cb, this);
        gazeboEncoder = nh.advertise<rover_22_drive_system::Steering>("/drive_system/gazebo_encoder",10);
        cout<< "Pusblishing encoder values of Gazebo!"<<endl;

        while (ros::ok)
        {
            gazeboEncoder.publish(Steering_msg);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void joint_state_cb(const sensor_msgs::JointState& msg){
        for (int i = 0; i < 4; i++)
        {
            Steering_msg.steering_angle[i]=round(msg.position[i]/Pi*2048)+2048;
            Steering_msg.wheel_speed[i]=round(10*msg.velocity[i+4])/10;
        }
        Steering_msg.goal_stage=1;
        Steering_msg.stamp = ros::Time::now();    
    }

};


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "Steering_gazebo_encoder",ros::init_options::NoSigintHandler);
    gazebo_encoder han_gazebo_encoder(argc,argv);
    signal(1, mySigintHandler);

    return 0;
}