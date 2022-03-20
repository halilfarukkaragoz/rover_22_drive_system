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

#define INF exp(15)




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
    d1vector Vmean={0,0};
    d1vector Rn={0,0};
    d2vector Vn,wheel_locations,Vn_global;

public:
    odometry(int argc, char *argv[]){
        ros::Rate rate(200); // 200
        //imuSub = nh.subscribe("/imu1/data",10,&odometry::imu_cb,this);
        
        imuSub = nh.subscribe("/imu/data",10,&odometry::imu_cb,this);
        encoderSub = nh.subscribe("drive_system/gazebo_encoder",10, &odometry::encoder_cb, this);
        odomPub = nh.advertise<nav_msgs::Odometry>("drive_system/odometry",10);
        cout<< "Odometry Started!"<<endl;
        x=0,y=0,z=0;
        x_speed = 0,y_speed = 0,z_speed = 0;
        current_time = ros::Time::now();
        init_params();
        yaw_prev = yaw;

        cout <<endl;
        while (ros::ok)
        {
            dt = (ros::Time::now() - current_time).toSec();
            current_time = ros::Time::now();
            
            //callculate odom for all wheel
            for (int i = 0; i < Vn.size(); i++)
            {
                Vn[i]=VectorCalculator(encoder_msg.wheel_speed[i],encoder_msg.steering_angle[i]);
                find_global_speeds(Vn[i][0],Vn[i][1],i);
            }
            find_center();
            double R_len = find_l2_norm(Rn[0],Rn[y]);
            // if(Rn[0] != INF){
            //     double R_len = find_l2_norm(Rn[0],Rn[y]);
            //     double V_scaler = R_len * angular_z;
            //     double angle = atan2(Rn[1],Rn[0]);
            //     x_speed = V_scaler * sin(angle);
            //     y_speed = V_scaler * cos(angle);             
            // }    
            // else{
            //     x_speed = (Vn[0][0] + Vn[1][0] + Vn[2][0] + Vn[3][0]) / 4;
            //     y_speed = (Vn[0][1] + Vn[1][1] + Vn[2][1] + Vn[3][1]) / 4;
            // }
            // x += x_speed *dt;
            // y += y_speed *dt;

            angular_z = (yaw - yaw_prev)/ dt;
            yaw_prev = yaw;

            quat = tf::createQuaternionMsgFromYaw(yaw);
            //publish odom
            publish_odom();
            //boradcast tf
            broadcast_tf();
            std::cout<<"R " << Rn[0] << " " << Rn[1] <<" "<<R_len << std::endl;
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


    void find_center(){
        d1vector func1 = find_funct(0); // function is ax + b and arguments of vector are a and b 
        d1vector func2 = find_funct(1);
        d1vector func3 = find_funct(2);
        d1vector func4 = find_funct(3);


        if( abs(func1[0] - func2[0]) < 0.1 &&  abs(func3[0] == func4[0]) < 0.1 ){
            Rn = {INF,INF};
        }
        else{

            // a1x + b1 = a2x + b2
            // x = (b2-b1) / (a1-a2) 
            //left front and back
            
            double a = func1[0] - func2[0];
            double b = func2[1] - func1[1];
            double x1 = b/a; 
            double y1 = func1[0]*x1 + func1[1];

            // a1x + b1 = a2x + b2
            // x = (b2-b1) / (a1-a2)
            //right front and back

            a = func1[0] - func3[0];
            b = func1[1] - func3[1];
            double x2 = b/a;
            double y2 = a*x2 + b;

            a = func1[0] - func4[0];
            b = func1[1] - func4[1];
            double x3 = b/a;
            double y3 = a*x2 + b;

            a = func4[0] - func3[0];
            b = func4[1] - func3[1];
            double x4 = b/a;
            double y4 = a*x2 + b;

            a = func2[0] - func4[0];
            b = func2[1] - func4[1];
            double x5 = b/a;
            double y5 = a*x2 + b;

            a = func3[0] - func4[0];
            b = func3[1] - func4[1];
            double x6 = b/a;
            double y6 = a*x2 + b;

            // cout << " 1 " << x1 << " " << y1 << endl;

            // cout << " 2 " <<x2 << " " << y2 << endl;

            // cout << " 3 " <<x3 << " " << y3 << endl;

            // cout <<" 4 " << x4 << " " << y4 << endl;

            // cout << " 5 " <<x5 << " " << y5 << endl;

            // cout << " 6 " <<x6 << " " << y6 << endl;


            //Rn = { (x1 + x2 + x4 + x3 + x5 + x6 ) / 6 , (y1 + y2 + y3 + y4 + y5 + y6) / 6 };

        }

    }

    d1vector find_funct(int i){  // return slopes and constants of a wheel
        float len = find_l2_norm(Vn[i][0],Vn[i][1]);
        float tan = atan2(Vn_global[i][0],Vn_global[i][1]);
        tan += M_PI_2;
        double x = sin(tan) * len;
        double y = cos(tan) * len; // check for sin-cos later
        //cout << x << " " << y <<endl;
        double slope = 0;
        if(y != 0 ) slope = y/x;
        //cout <<slope<<endl;
        d1vector vec = {slope ,wheel_locations[i][0]};
        return vec;

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
    
    void find_global_speeds(double x,double y, int k){
        double relx = x;
        double rely = y;
        //double norm = find_l2_norm(relx,rely);
        //cout <<  cos(yaw) * rely << " " <<  sin(yaw) * rely << endl;
        x_speed = (cos(yaw) * relx  - sin(yaw) * rely) / 6.7; // constant may change 
        y_speed = (sin(yaw) * relx + cos(yaw) * rely) / 6.7; // constant may change 
        Vn_global[k][0] = x_speed;
        Vn_global[k][1] = y_speed;

    }


    void init_params(){
        Vn   = createMatrix(4,2) ;
        wheel_locations  = createMatrix(4,2) ;
        Vn_global   = createMatrix(4,2) ;
        
        wheel_locations[0] = {length/ 2.,width/ 2.};
        wheel_locations[1] = {length/-2.,width/ 2.};
        wheel_locations[2] = {length/-2.,width/-2.};
        wheel_locations[3] = {length/ 2.,width/-2.};
    }



    void encoder_cb(const rover_22_drive_system::Steering& msg){
        encoder_msg = msg;
        for (int i = 0; i < 4; i++){
            encoder_msg.steering_angle[i] = (encoder_msg.steering_angle[i]-2048)/2048*Pi;      
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