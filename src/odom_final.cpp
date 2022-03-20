// odometry for steering 3.1.1
//Halil Faruk Karagöz 
// All wheels are tracked independently

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
#define d3vector valarray<valarray<valarray<double>>>



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
    d2vector Vn,wheel_locations;
    d3vector possible_locations;
    double dummyX,dummyY;

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
            double sumX = 0, sumY = 0;
            
            //callculate odom for all wheel
            for (int i = 0; i < Vn.size(); i++)
            {
                Vn[i]=VectorCalculator(encoder_msg.wheel_speed[i],encoder_msg.steering_angle[i]);
                find_global_speeds(Vn[i][0],Vn[i][1]);

                //cout <<x_speed<<endl;
                //cout <<y_speed<<endl;

                wheel_locations[i][0] += x_speed *dt;
                wheel_locations[i][1] += y_speed *dt;
                
                sumX += wheel_locations[i][0];
                sumY += wheel_locations[i][1];

            }
            find_possible_locations();
            //cout <<sumX<< " "<< sumY<<endl;
           //Disp4_2Mat(wheel_locations);
            

            //middle of wheels is the location of the rover
            x = sumX/4; y = sumY/4;

            angular_z = (yaw - yaw_prev)/ dt;
            yaw_prev = yaw;

            

            // cout<<speed<<" "<< angular_z << endl;
            // cout<<"radius: "<< radius << endl;

            // publish_wheel(1,odomPub_wheel1);
            // publish_wheel(2,odomPub_wheel2);
            // publish_wheel(3,odomPub_wheel3);
            // publish_wheel(4,odomPub_wheel4);

            quat = tf::createQuaternionMsgFromYaw(yaw);
            //publish odom
            publish_odom();
            //boradcast tf
            broadcast_tf();
            std::cout<<wheel_locations[0][0]  << " " << wheel_locations[0][1] << std::endl;
	        //cout<< "the yaw is : " << this->yaw<<endl;
            ros::spinOnce();
            rate.sleep();

        }
    }

    void find_possible_locations(){
        d2vector wheel1_possibilities = createMatrix(3,2);
        d2vector wheel2_possibilities = createMatrix(3,2);
        d2vector wheel3_possibilities = createMatrix(3,2);
        d2vector wheel4_possibilities = createMatrix(3,2);
        d2vector rotated = createMatrix(4,2);

        d1vector w1_r = rotate(wheel_locations[0],-yaw);
        d1vector w2_r = rotate(wheel_locations[1],-yaw);
        d1vector w3_r = rotate(wheel_locations[2],-yaw);
        d1vector w4_r = rotate(wheel_locations[3],-yaw);

        w1_r = rotate(w1_r,yaw);
        dummyX = w1_r[0],dummyY = w1_r[1];
        rotated[0] = w1_r, rotated[1] = w2_r, rotated[2] = w3_r, rotated[3] = w4_r;

        wheel1_possibilities[0] = rotate({w2_r[0] + length, w2_r[1]},yaw);
        wheel1_possibilities[1] = rotate({w3_r[0] + length, w3_r[1] + width},yaw);
        wheel1_possibilities[2] = rotate({w4_r[0] , w4_r[1] + width},yaw);
        cout <<yaw<<endl;
        cout <<"rotated:"<<endl;
        Disp4_2Mat(rotated);

        cout << "location"<<endl;
        Disp4_2Mat(wheel_locations);


        cout << "pos:" <<(wheel1_possibilities[0][0] + wheel1_possibilities[1][0] + wheel1_possibilities[2][0])/3 << " " <<(wheel1_possibilities[0][1] + wheel1_possibilities[1][1] + wheel1_possibilities[2][1])/3<<endl;
        //Disp3_2Mat(wheel1_possibilities);
        // wheel2_possibilities[0] = rotate({w1_r[0] - length, w1_r[1]},yaw);
        // wheel2_possibilities[1] = rotate({w3_r[0], w3_r[1] + width},yaw);
        // wheel2_possibilities[2] = rotate({w4_r[0] - length , w4_r[1] + width},yaw);


    }

    d1vector rotate(d1vector location,double yaw){
        double x_in = location[0]-x,y_in = location[1]-y;
        double py = -(-y_in *cos(yaw) - x_in * sin(yaw));
        double px = -y_in * sin(yaw) + x_in * cos(yaw);
        d1vector ret = {px+x,py+y};
        dummyX = px+x ,dummyY = py+y;
        return ret;
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

        odom.pose.pose.position.x = dummyX;
        odom.pose.pose.position.y = dummyY;
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
    
    void find_global_speeds(double x,double y){
        double relx = x;
        double rely = y;
        //double norm = find_l2_norm(relx,rely);
        //cout <<  cos(yaw) * rely << " " <<  sin(yaw) * rely << endl;
        x_speed = (cos(yaw) * relx  - sin(yaw) * rely) / 6.7; // constant may change 
        y_speed = (sin(yaw) * relx + cos(yaw) * rely) / 6.7; // constant may change 
        
    }


    void init_params(){
        Vn   = createMatrix(4,2) ;
        wheel_locations  = createMatrix(4,2) ;
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

void Disp3_2Mat(d2vector arr,char text='*'){ 
        cout<<"//////////////////////////////////////////////////"<<endl;
        cout<<"+++++++++++++++++++"<<text<<"+++++++++++++++++++++"<<endl;
        cout<<"x 0----- "<< arr[0][0]<<"  y------ "<<arr[0][1]<<endl;
        cout<<"x 1----- "<< arr[1][0]<<"  y------ "<<arr[1][1]<<endl;
        cout<<"x 2----- "<< arr[2][0]<<"  y------ "<<arr[2][1]<<endl;
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