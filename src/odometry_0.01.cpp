// odometry for steering 3.1.1
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "rover_22_drive_system/Steering.h"
#include "nav_msgs/Odometry.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"

#include <iostream>
#include <signal.h>
#include <cmath>
#include <valarray>
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"


using namespace std;
#define d1vector valarray<double> 
#define d2vector valarray<valarray<double>>


void mySigintHandler(int sig) {ros::shutdown();}

class odometry
{
private:
    ros::NodeHandle nh;
    ros::Subscriber imuSub,encoderSub;
    rover_22_drive_system::Steering encoder_msg;
    ros :: Publisher odomPub;
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped tf;
    tf::TransformBroadcaster tf_br;
    geometry_msgs::Quaternion quat;
    double Pi = M_PI,length=.824,width=.914;
    float wheel_speed[4], steer_angle[4];
    double dt = 0;
    double x,y,z;
    double x_speed,y_speed,z_speed;
    double angular_z = 0;
    double yaw,yaw_prev,yaw_start=0;
    bool first_yaw=1;
    ros::Time current_time;
    d1vector Vmean={0,0},cartesian_rot_theta={0,0,0,0};
    d2vector R_on,Vn,Vn_global;

public:
    odometry(int argc, char *argv[]){
        ros::Rate rate(200); // 200
        //imuSub = nh.subscribe("/imu1/data",10,&odometry::imu_cb,this);
        imuSub = nh.subscribe("/imu/data",10,&odometry::imu_cb,this);

        encoderSub = nh.subscribe("drive_system/gazebo_encoder",10, &odometry::encoder_cb, this);
        odomPub = nh.advertise<nav_msgs::Odometry>("drive_system/odometry_new_01",10);
        cout<< "Odometry Started!"<<endl;
        x=0,y=0,z=0;
        x_speed = 0,y_speed = 0,z_speed = 0;
        current_time = ros::Time::now();
        init_params();
        yaw_prev = yaw;

        for (int i = 0; i < R_on.size(); i++)
        {
            cartesian_rot_theta[i] = (Pi/2+atan2(R_on[i][0],R_on[i][1]))*-1;
        }
        cout <<endl;
        while (ros::ok)
        {
            for (int i = 0; i < Vn.size(); i++){
            Vn[i]=VectorCalculator(encoder_msg.wheel_speed[i],encoder_msg.steering_angle[i]);}
            //Disp4_2Mat(Vn,'v');
            Vmean=MeanCalculator(Vn);            
            //cout<< Vmean[0] <<"   "<< Vmean[1]<<endl;
            find_global_speeds();

            dt = (ros::Time::now() - current_time).toSec();
            current_time = ros::Time::now();


            x += x_speed*dt;
            y += y_speed*dt;


            // cout<<speed<<" "<< angular_z << endl;
            // cout<<"radius: "<< radius << endl;

            
            quat = tf::createQuaternionMsgFromYaw(yaw);
            publish_odom();
            broadcast_tf();
            //std::cout<<x  << " " << y << std::endl;
	    //cout<< "the yaw is : " << this->yaw<<endl;
            ros::spinOnce();
            rate.sleep();
        }
    }

    
    void broadcast_tf(){ // publishing odom msg
        tf.header.stamp = current_time;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = quat;

        tf_br.sendTransform(tf);

    }



    void publish_odom (){ // broadcasting tf
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = quat;
        
        odom.twist.twist.linear.x = x_speed;
        odom.twist.twist.linear.y = y_speed;
        odom.twist.twist.linear.z = 0.0;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = angular_z;

        odomPub.publish(odom);
    }

    double find_l2_norm(double x ,double y ){
        return sqrt(x*x+y*y);
    }
    
    void find_global_speeds(){
        double relx = Vmean[0];
        double rely = Vmean[1];
        //double norm = find_l2_norm(relx,rely);
        //cout <<  cos(yaw) * rely << " " <<  sin(yaw) * rely << endl;
        x_speed = (cos(yaw) * relx  - sin(yaw) * rely) / 6.7; // constant may change 
        y_speed = (sin(yaw) * relx + cos(yaw) * rely) / 6.7; // constant may change 
        angular_z = (yaw - yaw_prev)/ dt;
        yaw_prev = yaw;

    }


    void init_params(){

        R_on = createMatrix(4,2) ;
        Vn   = createMatrix(4,2) ;
        Vn_global   = createMatrix(4,2) ;

        R_on[0] = {length/ 2.,width/ 2.};
        R_on[1] = {length/-2.,width/ 2.};
        R_on[2] = {length/-2.,width/-2.};
        R_on[3] = {length/ 2.,width/-2.};
        
    }


    void encoder_cb(const rover_22_drive_system::Steering& msg){
        encoder_msg = msg;
        for (int i = 0; i < 4; i++){
            encoder_msg.steering_angle[i] = (encoder_msg.steering_angle[i]-2048)/2048*Pi;      
            cout <<encoder_msg.steering_angle[i]<<endl; 
        }
    }
    void imu_cb(const sensor_msgs::Imu& msg){
        tf::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, this->yaw);
        //this->yaw += M_PI_2;
	if (this->yaw<10 && first_yaw )
    {
	    this->yaw_start=this->yaw;
	    first_yaw=0;
    }
	this->yaw -= this->yaw_start;
    }
    void imu_new_cb(const sensor_msgs::Imu& msg){
        angular_z = msg.angular_velocity.z;
        //this->yaw += M_PI_2;
        if (dt != 0 ){
            this->yaw += angular_z*dt;
        }
    }
    
    d1vector VectorCalculator(double Mag,double theta){
        return { Mag*cos(theta),Mag*sin(theta) };
    }

    d1vector MeanCalculator(d2vector M){
        double x_mean=0,y_mean=0;
            for (int i = 0; i < M.size(); i++)
            {
                    x_mean += M[i][0]/M.size(); 
                    y_mean += M[i][1]/M.size(); 
            }
        return {x_mean,y_mean};
    }


    d2vector createMatrix(int n0,int n1){
        d2vector arr(n0);
        for(int k=0;k<arr.size();k++)arr[k].resize(n1);
        return arr;}

    void Disp4_2Mat(d2vector arr,char text='*'){ 
        cout<<"//////////////////////////////////////////////////"<<endl;
        cout<<"+++++++++++++++++++"<<text<<"+++++++++++++++++++++"<<endl;
        cout<<"x 0----- "<< arr[0][0]<<"  y------ "<<arr[0][1]<<endl;
        cout<<"x 1----- "<< arr[1][0]<<"  y------ "<<arr[1][1]<<endl;
        cout<<"x 2----- "<< arr[2][0]<<"  y------ "<<arr[2][1]<<endl;
        cout<<"x 3----- "<< arr[3][0]<<"  y------ "<<arr[3][1]<<endl;
        cout<<"---------------------------------------------------"<<endl;
    }
};


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "Steering_odometry_new",ros::init_options::NoSigintHandler);
    odometry han_odometry(argc,argv);
    signal(1, mySigintHandler);

    return 0;
}