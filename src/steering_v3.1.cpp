#include <iostream>
#include <signal.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_msgs/Float64.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "rover_22_drive_system/Steering.h"
#include <cmath>
#include <valarray>
#include <iomanip>
#include <numeric>
#include <vector>

using namespace std;
#define d1vector valarray<double> 
#define d2vector valarray<valarray<double>>


void mySigintHandler(int sig){  ros::shutdown();}


class steering
{
private:
    ros::NodeHandle nh;
    bool twistChange=1;
    double _eps = pow(10,-11),w, inf=pow(10,10);
    double length, width, rpm, Pi = M_PI, angle_limit=Pi;
    vector<double> StateArr={0,0,0,0};
    vector<double> VelArr  ={0,0,0,0};
    geometry_msgs::Twist twist;
    ros::Subscriber twistSub,gazeboPosSub;
    d1vector V_direction={1,1,1,1};
    d1vector R={0,0},V={0,0},x_axis,D_n={0,0,0,0},V_n,Dnow={0,0,0,0},Doptimal,Vsteer={0,0,0,0},D_forward={-Pi,0,Pi}; //Distance vector of rover's location to ICR and speed of the rover
    d2vector R_n,R_on;
    ros::Publisher SteeringPub;


public:
    steering(int argc, char *argv[]){
        nh.getParam("/steering/length",length);
        nh.getParam("/steering/width",width);
        nh.getParam("/steering/rpm",rpm);
        cout<< "STEERING STARTED BABY!! "<<endl;
        cout << fixed;
        cout << setprecision(7);
        ros::Rate rate(150);
        SteeringPub = nh.advertise<rover_22_drive_system::Steering>("/drive_system/steer_sent",10);
        twistSub     = nh.subscribe("/drive_system/twist", 10, &steering::twist_cb, this);
        //gazeboPosSub = nh.subscribe("/rover_steering_controller/state", 10, &steering::Gazebo_state_cb, this);
        init_params();
        while (ros::ok)
        {
            ros::spinOnce();
            rate.sleep();
            //if(!twistChange){continue;}
            w = twist.angular.z*2*Pi/3;
            V = {twist.linear.x,twist.linear.y};
            R = FindICR(V,w);
            SteeringMotion(R,R_on,w);
            //cout<< "Rx ="<<R[0]<<"  Ry ="<<R[1]<<"  w= "<<w<<" Vx ="<<V[0]<<" Vy ="<<V[1]<<endl;
        }
        
    }
    void SteeringMotion(d1vector R, d2vector R_on, double w){
        d1vector V_n(4);
        // R = FindICR(V,w); TODO silincek

        //cout<<R[0] << "    "<< R[1]<<endl; 
        for (int i = 0; i < R_n.size(); i++) R_n[i] = R_on[i]-R;
        d1vector D_n = DeltaCalculator(R_n);
        for (int i = 0; i < V_n.size(); i++) {
            V_n[i] = mag(R_n[i])*-w;
            if(w==0){V_n[i]=mag(V);}
            }
        for (int i = 0; i < D_n.size(); i++) D_n[i]= MVDelta2(Dnow[i],D_n[i],V_direction[i]);//TODO burada kodun ayarlanması lazım
        

        //cout<< "D_1 ="<<D_n[0]<< " D_2 ="<<D_n[1]<< " D_3 ="<<D_n[2]<< " D_4 ="<<D_n[3]<<endl;
        //cout<< "V_1 ="<<V_n[0]<< " V_2 ="<<V_n[1]<< " V_3 ="<<V_n[2]<< " V_4 ="<<V_n[3]<<endl;
        SteerPublisher(D_n,V_n*V_direction);
            
    }
    double MVDelta2(double now,double raw,double &direction){
        if(abs(raw)>Pi/2){
            direction = -1;
            return raw - abs(raw)/raw*Pi;
        }else{
            direction = 1;
            return raw ;
        }
    }

    double MVDelta(double now,double raw,double &direction){
        double abs_del = fmod(raw, Pi );
        d1vector dDelta= {Pi,0,-1*Pi}, possible_del(dDelta.size()),dDistance(dDelta.size());
        possible_del = dDelta - abs_del;
        int indexx;
        for (int i = 0; i < possible_del.size(); i++)
        {   //cout<<"the angle:"<< possible_del[i] <<"    :" <<(angle_limit<abs(possible_del[i])) << endl;
            if(angle_limit<abs(possible_del[i])) {possible_del[i]=inf;}
        }

        dDistance = abs(possible_del - now);
        indexx = findIndex(dDistance,dDistance.min());
        if (indexx%2==0){direction=-1;}
        cout<<"-now :"<<now<<endl;
        cout<< "pos1 ="<<possible_del[0]<< " pos2 ="<<possible_del[1]<< " pos3 ="<<possible_del[2]<< " POS ="<<possible_del[indexx]<<endl;
        return possible_del[indexx];
    }

    void init_params(){

        R_n  = createMatrix(4,2) ;
        R_on = createMatrix(4,2) ;

        R_on[0] = {length/ 2.,width/ 2.};  // left front 
        R_on[1] = {length/-2.,width/ 2.};  // left back 
        R_on[2] = {length/-2.,width/-2.};  // right back 
        R_on[3] = {length/ 2.,width/-2.};  // right front 
    }

    d1vector FindICR(d1vector V, double w ){
        double V_mag = hypot(V[0],V[1]);
        double R_mag = V_mag/w;

        d1vector V_ = {V[0],V[1]};
        if(w==0){
        return {inf*cos( atan2(V[1],V[0])- Pi/2) ,inf*sin( atan2( V[1],V[0])- Pi/2)};          
        }
        return {R_mag*cos( atan2(V[1],V[0])+ Pi/2) ,R_mag*sin( atan2( V[1],V[0])+ Pi/2 )};
    }

    d1vector DeltaCalculator(d2vector R_n){
        d1vector delta(4);
        for (int i = 0; i < R_n.size(); i++) delta[i] = atan2(-1*R_n[i][0],R_n[i][1]); 
        return delta;
    }

    void twist_cb(const geometry_msgs::Twist& msg){
        if(twist == msg){
            twistChange = 0;
        }else{
        twist = msg;
        twistChange = 1;
        }
    }
    double mag(d1vector arr){return hypot(arr[1],arr[0]);}


    void SteerPublisher(d1vector D_n,d1vector V_n){

        std_msgs::Float64 wheel[4];
        std_msgs::Float64 angle[4];
        rover_22_drive_system::Steering msg;
        msg.stamp = ros::Time::now();
        msg.goal_stage = 0;


        for (int i = 0; i < 4; i++)
        {
            msg.wheel_speed[i] = V_n[i]; 
            msg.steering_angle[i] = round(D_n[i]*1800/Pi);
        }

        SteeringPub.publish(msg);
        
        

    }

    double AngleTime(d1vector D_n, d1vector Dnow){
        double dD= abs(D_n-Dnow).max();
        return 10*dD/Pi/rpm+_eps;
    }


    d2vector createMatrix(int n0,int n1){
        d2vector arr(n0);
        for(int k=0;k<arr.size();k++)arr[k].resize(n1);
        return arr;
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

    int findIndex(d1vector arr, double n){
        for (int i = 0; i < arr.size(); i++)
        {
            if(n ==arr[i]){
                return i;
            }
        }    
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
    steering Steering(argc, argv);
    signal(1, mySigintHandler);

    return 0;
}