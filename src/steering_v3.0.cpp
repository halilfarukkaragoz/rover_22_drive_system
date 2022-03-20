#include <iostream>
#include <signal.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_msgs/Float64.h"
#include <cmath>
#include <valarray>
#include <iomanip>
#include <numeric>
#include <vector>

using namespace std;
#define d1vector valarray<double> 
#define d2vector valarray<valarray<double> >


void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}


class steering
{
private:
    double Pi = M_PI;
    double _eps = pow(10,-11);
    const double length = 0.914, width = 0.824; 
    double w;

    ros::Publisher pub_steer;
    ros::Publisher pub_wheel[4];
    geometry_msgs::Twist twist;
    ros::Subscriber twistSub;
    trajectory_msgs::JointTrajectory steering_msg;
    trajectory_msgs::JointTrajectoryPoint steering_points;
    d1vector R,V,x_axis,D_n,V_n; //Distance vector of rover's location to ICR and speed of the rover
    d2vector R_n,R_on;


public:

    steering(ros::NodeHandle *nh){
        init_params();
        cout << fixed;
        cout << setprecision(7);
        ros::Rate rate(150);
        pub_steer = nh->advertise<trajectory_msgs::JointTrajectory>("/rover_steering_controller/command",10);
        pub_wheel[0] = nh->advertise<std_msgs::Float64>("/rover_wheel_leftfront/command" ,10);
        pub_wheel[1] = nh->advertise<std_msgs::Float64>("/rover_wheel_leftrear/command" ,10);
        pub_wheel[2] = nh->advertise<std_msgs::Float64>("/rover_wheel_rightfront/command",10);
        pub_wheel[3] = nh->advertise<std_msgs::Float64>("/rover_wheel_rightrear/command" ,10);

        steering_msg.joint_names= {"steering_leftfront_joint",
                                    "steering_leftrear_joint" ,
                                    "steering_rightrear_joint",
                                    "steering_rightfront_joint"} ; 
      
        twistSub = nh->subscribe("/drive_system/twist",10,&steering::twist_cb,this);
        while (ros::ok)
        {
            ros::spinOnce();
            rate.sleep();
            w = twist.angular.z*2*Pi/3;
            V = {twist.linear.x,twist.linear.y};

            init_params();

            switch (  (w!=0) + (abs(V).max()!=0)*2 )
            {
            case 1:
                TankRotation(R_on);
                break;
            case 2:
                CartesianMotion();
                break;            
            case 3:
                SteeringMotion();
                break;            
            default:
                IDLE();
                break;
            }




            //cout<< "V1 --- "<< V_n[1]<<"V2 --- "<< V_n[1]<< "V3 --- "<< V_n[2]<< "V4 --- "<< V_n[3] << endl;
            //cout<< "D1 --- "<< delta[0]<< "D2 --- "<< delta[1]<< "D3 --- "<< delta[2]<< "D4 --- "<< delta[3] << endl;
            //Disp4_2Mat(R_n,'R');
        }

    }

    void twist_cb(const geometry_msgs::Twist& msg){
        twist =msg;
    }

    void SteeringMotion(){
            d1vector R(2) , V_n(4),V_empty(4);
            R = Find_ICR(V,w);

            //cout<<R[0] << "    "<< R[1]<<endl; 
            for (int i = 0; i < R_n.size(); i++) R_n[i] = R_on[i]-R;
            d1vector D_n = DeltaCalculator(R_n);
            for (int i = 0; i < V_n.size(); i++) V_n[i] = mag(R_n[i])*-w;
            
            GazeboPublisher(D_n,V_n);
            
    }

    double mag(d1vector arr){ 
        return hypot(arr[1],arr[0]); }

    d2vector createMatrix(int n0,int n1){
        d2vector arr(n0);
        for(int k=0;k<arr.size();k++)arr[k].resize(n1);
        return arr;
    }

    double stLim(double steer_deg, double dDeg) {
        return fmod(  steer_deg+dDeg  ,  Pi+_eps)  -Pi/2. ;}


    void IDLE(){
        d1vector D_n(4),V_n(4);
        GazeboPublisher(D_n,V_n);   
    }

    void TankRotation(d2vector R_on){
        d1vector steer_arr(4),V_n(4);

        for (int i = 0; i < 4; i++) steer_arr[i]= (((i<2)-.5)*2)*fmod(atan2(-1*(R_on[i][0]),R_on[i][1]*(((i<2)-.5)*2)),Pi/2) ;
        for (int i = 0; i < 4; i++) V_n[i] = -1*twist.angular.z*1.7584/2*Pi*(((i<2)-.5)*2);

        GazeboPublisher(steer_arr,V_n);

    }

    void CartesianMotion(){
        double degWheel = atan2((twist.linear.y),(twist.linear.x+_eps));
        if  (isnan(degWheel))  {degWheel=0;} 
        d1vector steer_arr(4),V_n(4);
        for (size_t i = 0; i < steer_arr.size(); i++) steer_arr[i]=0;        
        for (int i = 0; i < 4; i++) steer_arr[i]= degWheel;

        for (int i = 0; i < 4; i++) V_n[i]= hypot(twist.linear.x,twist.linear.y)*(((twist.linear.x>=0*twist.linear.y>=0 )-.5)*2);
        GazeboPublisher(steer_arr,V_n);
    }


    void init_params(){
        d1vector R(2),V(2);
        const d1vector x_axis = {1.,0.}; 
        R_n  = createMatrix(4,2) ;
        R_on = createMatrix(4,2) ;

        R_on[0] = {length/ 2.,width/ 2.};
        R_on[1] = {length/-2.,width/ 2.};
        R_on[2] = {length/-2.,width/-2.};
        R_on[3] = {length/ 2.,width/-2.};

    }

    d1vector Find_ICR(d1vector V, double w){
        double V_mag = hypot(V[0],V[1]);
        double R_mag = V_mag/(w+_eps);
        //cout<< "R_mag    "<<R_mag<<"  V_mag" <<V_mag  << endl;
        return {  R_mag*sin( atan2(-1*V[1],V[0] )) , R_mag*cos( atan2(-1*V[1],V[0]) )  };
    }


    void GazeboPublisher(d1vector D_n,d1vector V_n){
        trajectory_msgs::JointTrajectory steering_msg;
        steering_msg.joint_names= {"steering_leftfront_joint",
                                    "steering_leftrear_joint" ,
                                    "steering_rightrear_joint",
                                    "steering_rightfront_joint"} ; 
        trajectory_msgs::JointTrajectoryPoint steering_points;
        for(double& c: D_n) steering_points.positions.push_back(c);
        steering_points.time_from_start =ros::Duration(0.005); 
        steering_msg.points.push_back(steering_points);
        pub_steer.publish(steering_msg);

        std_msgs::Float64 wheel[4];
        for (int i = 0; i < 4; i++) wheel[i].data = V_n[i];
        for (int i = 0; i < 4; i++) pub_wheel[i].publish(wheel[i]);
        cout<< "V1 --- "<< V_n[1]<<"V2 --- "<< V_n[1]<< "V3 --- "<< V_n[2]<< "V4 --- "<< V_n[3] << endl;
        cout<< "D1 --- "<< 180/Pi*D_n[0]<< "D2 --- "<< 180/Pi*D_n[1]<< "D3 --- "<< 180/Pi*D_n[2]<< "D4 --- "<< 180/Pi*D_n[3] << endl;

    }

    d1vector DeltaCalculator(d2vector R_n){
        d1vector delta(4);
        for (int i = 0; i < R_n.size(); i++) delta[i] = atan2(-1*R_n[i][0],R_n[i][1]); 
        return delta;
    }

    




    void Disp4_2Mat(d2vector arr,char text='*'){ 
            cout<<"//////////////////////////////////////////////////"<<endl;
            cout<<"+++++++++++++++++++"<<text<<"+++++++++++++++++++++"<<endl;
            cout<<"x 0----- "<< arr[0][0]<<"  y------ "<<arr[0][1]<<endl;
            cout<<"x 1----- "<< arr[1][0]<<"  y------ "<<arr[1][1]<<endl;
            cout<<"x 2----- "<< arr[2][0]<<"  y------ "<<arr[2][1]<<endl;
            cout<<"x 3----- "<< arr[3][0]<<"  y------ "<<arr[3][1]<<endl;
            cout<<"---------------------------------------------------"<<endl;
    }

    





    ~steering(){};
};


/*
steering::steering()
{
}

steering::~steering()
{
}*/


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Steering_system",ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    steering Steering(&nh);
    signal(1, mySigintHandler);

    return 0;
}
