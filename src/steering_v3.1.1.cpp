#include <iostream>
#include <signal.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_msgs/Float64.h"
#include "rover_22_drive_system/Steering.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include <cmath>
#include <valarray>
#include <iomanip>
#include <numeric>
#include <vector>

using namespace std;
#define d1vector valarray<double>
#define d2vector valarray<valarray<double>>

void mySigintHandler(int sig){ros::shutdown();}

class steering{
    private:
        ros::NodeHandle nh;
        double _eps = pow(10,-11), inf=pow(10,10);
        double length, width, w, rpm, Pi = M_PI;//, angle_limit=Pi
        vector<double> StateArr={0,0,0,0}, VelArr  ={0,0,0,0}; //Vectors to publish via Gazebo_state_cb().
        sensor_msgs::Joy joy;
        ros::Subscriber joySub,gazeboPosSub;
        d1vector R={0,0}, V={0,0}, D_Now={0,0,0,0}, V_direction={1,1,1,1}, V_Steer={0,0,0,0};//Distance vector of rover's location to ICR and speed of the rover
        //d1vector x_axis,D_n={0,0,0,0},V_n={0,0,0,0},Doptimal,D_forward={-Pi,0,Pi};
        d1vector D_Wheels ={0,0,0,0}, V_Wheels ={0,0,0,0}; //Valarrays for single wheel movements.
        d2vector R_n,R_on;
        ros::Publisher pub_steer, pub_wheel[4], steeringPub;
    public:
        steering(){
            nh.getParam("/steering/length",length);
            nh.getParam("/steering/width",width);
            nh.getParam("/steering/rpm",rpm);
            cout<< "STEERING STARTED BABY!! "<<endl;
            cout << fixed;
            cout << setprecision(7);
            ros::Rate rate(150);
            pub_steer    = nh.advertise<trajectory_msgs::JointTrajectory>("/rover_steering_controller/command",10);
            pub_wheel[0] = nh.advertise<std_msgs::Float64>("/rover_wheel_leftfront/command" ,10);
            pub_wheel[1] = nh.advertise<std_msgs::Float64>("/rover_wheel_leftrear/command" ,10);
            pub_wheel[2] = nh.advertise<std_msgs::Float64>("/rover_wheel_rightfront/command",10);
            pub_wheel[3] = nh.advertise<std_msgs::Float64>("/rover_wheel_rightrear/command" ,10);

            joySub       = nh.subscribe("/joy", 10, &steering::joy_cb, this);
            gazeboPosSub = nh.subscribe("/rover_steering_controller/state", 10, &steering::Gazebo_state_cb, this);
            steeringPub = nh.advertise<rover_22_drive_system::Steering>("/drive_system/steer_sent",10);
            init_params();
            
            //Fillers for joy vectors. Since their size is undefined at initialization, it is impossible to index them in the following methods.
            joy.buttons.resize(11,0);
            joy.axes.resize(8,1);

            while(ros::ok){
                V_Wheels ={0,0,0,0};
                ros::spinOnce();
                rate.sleep();
                if (pushedButtonIndex(joy) >= 0) singleWheel();
                else{
                    D_Wheels ={0,0,0,0};
                    FindICR();
                    SteeringMotion();
                }
            }
        }

        void joy_cb(const sensor_msgs::Joy& msg){
            joy = msg;
            w = joy.axes[0]*pow(Pi,2)*10/3;
            V = {joy.axes[4]*20, joy.axes[3]*20};
        }

        void Gazebo_state_cb(const control_msgs::JointTrajectoryControllerState& msg){
            for (int i=0; StateArr.size()!=0; i++) StateArr.pop_back();
            for (int i=0; VelArr.size()!=0; i++) VelArr.pop_back();
            //cout << msg.actual<<endl;
            for (int i=0; i<4; i++) StateArr.push_back(msg.actual.positions[i]);
            for (int i=0; i<4; i++) VelArr.push_back(msg.desired.velocities[i]);
            for (int i=0; i<V_Steer.size(); i++) V_Steer[i] = VelArr[i];
            for (int i=0; i<D_Now.size(); i++) D_Now[i] = StateArr[i];
        }

        void init_params(){
            R_n  = createMatrix(4,2);
            R_on = createMatrix(4,2);

            R_on[0] = {length/ 2,width/ 2};
            R_on[1] = {length/-2,width/ 2};
            R_on[2] = {length/-2,width/-2};
            R_on[3] = {length/ 2,width/-2};
        }

        d2vector createMatrix(int n0,int n1){
            d2vector arr(n0);
            for(int k=0; k<arr.size(); k++) arr[k].resize(n1);
            return arr;
        }

        int pushedButtonIndex(const sensor_msgs::Joy& msg){
            int index = -1;
            for (int i=0; i<4; i++) {
                if (msg.buttons[i] == 1){
                    index = i;
                    break;
                }
            }
            return index;
        }

        void singleWheel(){
            int wheelIndex, velIndex;
            switch(pushedButtonIndex(joy)){
                case 0://A-Left Rear
                    wheelIndex = 1;
                    velIndex = 1;
                    break;
                case 1://B-Right Rear
                    wheelIndex = 2;
                    velIndex = 3;
                    break;
                case 2://X-Left Front
                    wheelIndex = 0;
                    velIndex = 0;
                    break;
                case 3://Y-Right Front
                    wheelIndex = 3;
                    velIndex = 2;
                    break;
            }
            if(joy.axes[5] < 1){
                V_Wheels[velIndex] = 10*(-joy.axes[5]+1);
            }
            if(D_Wheels[wheelIndex] <= Pi/2 && D_Wheels[wheelIndex] >= -Pi/2) D_Wheels[wheelIndex] += joy.axes[0]*Pi/100;
            GazeboPublisher(D_Wheels, V_Wheels);//, 0
            //SteeringPublisher(D_Wheels,V_Wheels);
        }

        void FindICR(){
            if(w==0) R = {inf*cos( atan2(V[1],V[0])-Pi/2 ) ,inf*sin( atan2( V[1],V[0])-Pi/2 )};
            else{
                double V_mag = hypot(V[0],V[1]);
                double R_mag = V_mag/w;
                R = {R_mag*cos( atan2(V[1],V[0])+ Pi/2) ,R_mag*sin( atan2( V[1],V[0])+ Pi/2 )};
            }
        }

        void SteeringMotion(){
            d1vector V_n(4);
            for (int i = 0; i < R_n.size(); i++) R_n[i] = R_on[i]-R;
            d1vector D_n = DeltaCalculator(R_n);
            for (int i = 0; i < V_n.size(); i++){
                V_n[i] = mag(R_n[i])*(-w)*(-joy.axes[5]+1);
                if(w==0) V_n[i] = mag(V)*(-joy.axes[5]+1);
            }
            for (int i = 0; i < D_n.size(); i++) D_n[i]= MVDelta2(D_Now[i],D_n[i],V_direction[i]);//TODO burada kodun ayarlanması lazım
            GazeboPublisher(D_n,V_n*V_direction);//,AngleTime(D_n,D_Now)
            //SteeringPublisher(D_n,V_n*V_direction);
        }

        d1vector DeltaCalculator(d2vector R_n){
            d1vector delta(4);
            for (int i=0; i<R_n.size(); i++) delta[i] = atan2(-1*R_n[i][0],R_n[i][1]); 
            return delta;
        }

        double mag(d1vector arr){return hypot(arr[1],arr[0]);}

        double MVDelta2(double now,double raw,double &direction){
            if(abs(raw)>Pi/2){
                direction = -1;
                return raw - abs(raw)/raw*Pi;
            }else{
                direction = 1;
                return raw;
            }
        }

        void GazeboPublisher(d1vector D_n,d1vector V_n){//,double time
            trajectory_msgs::JointTrajectory steering_msg;
            trajectory_msgs::JointTrajectoryPoint steering_points;

            steering_msg.joint_names= {"steering_leftfront_joint",
                                        "steering_leftrear_joint" ,
                                        "steering_rightrear_joint",
                                        "steering_rightfront_joint"};
            
            for(double& c: D_n) steering_points.positions.push_back(c);
            //cout<< time<<endl;
            steering_points.time_from_start =ros::Duration(0.05); 
            steering_msg.points.push_back(steering_points);


            std_msgs::Float64 wheel[4];

            //if (abs(V_Steer).max()> 0.25) for (int i=0; i<4; i++) wheel[i].data = 0;
            for (int i=0; i<4; i++) wheel[i].data = V_n[i];
            pub_steer.publish(steering_msg);
            for (int i = 0; i < 4; i++) pub_wheel[i].publish(wheel[i]);
        }

        void SteeringPublisher(d1vector D_n,d1vector V_n){
            std_msgs::Float64 wheel[4], angle[4];
            rover_22_drive_system::Steering msg;
            msg.stamp = ros::Time::now();
            msg.goal_stage = 0;

            for (int i = 0; i < 4; i++){
                msg.wheel_speed[i] = V_n[i]; 
                msg.steering_angle[i] = round(D_n[i]*2048/Pi)+2048;
            }
            steeringPub.publish(msg);
        }

        double AngleTime(d1vector D_n, d1vector D_Now){
            double dD = abs(D_n-D_Now).max();
            return 10*dD/Pi/rpm+_eps;
        }
        ~steering(){};
        /*
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
        */
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "Steering_system",ros::init_options::NoSigintHandler);
    steering Steering;
    signal(1, mySigintHandler);
    return 0;
}
