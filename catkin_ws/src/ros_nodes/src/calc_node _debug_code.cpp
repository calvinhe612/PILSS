#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>


std_msgs::Float64 joint[5];
int jointIndx = 0;
float angle1=0;
float angle2=0;
bool firstRunFlag = true;
bool finishedFlag = true;
void chatterCallback1(const control_msgs::JointControllerState::ConstPtr& msg)
{
   angle1=msg->set_point;
   std::cout  << std::endl << "1" << ":   " <<msg->set_point << std::endl;
  // joint[jointIndx] = msg->set_point;
   angle1=msg->set_point;
}

void chatterCallback2(const control_msgs::JointControllerState::ConstPtr& msg)
{
   std::cout  << std::endl << "2" << ":   " <<msg->set_point << std::endl;
  // joint[jointIndx] = msg->set_point;
   angle2=msg->set_point;
}


#define PI 3.14159265
#define l1 1.05
#define l2 1.1




int main(int argc, char** argv)
{
  static float a = 0;
 
  ros::init(argc,argv, "abc");
  ros::NodeHandle nh1;
  ros::NodeHandle nh2;
  ros::NodeHandle nh3;
  ros::NodeHandle nh4;
  ros::NodeHandle nh5;

  ros::Publisher pub0 = nh1.advertise<std_msgs::Float64>("rrbot/joint0_position_controller/command",1000);
  ros::Publisher pub1 = nh2.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",1000);
  ros::Publisher pub2 = nh3.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",1000);
  ros::Publisher pub3 = nh4.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",1000);
  ros::Publisher pub4 = nh5.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",1000);
  
 
  
  std_msgs::Float64 abc;


 // ros::init(argc,argv, "listener");
  ros::NodeHandle nhh0;
  ros::NodeHandle nhh1;
  ros::NodeHandle nhh2;
  ros::NodeHandle nhh3;  
  ros::NodeHandle nhh4;
   jointIndx = 0;
  //ros::Subscriber sub0 = nhh0.subscribe("rrbot/joint0_position_controller/state",1,      chatterCallback);
  jointIndx = 1;
  ros::Subscriber sub1 = nhh1.subscribe("rrbot/joint1_position_controller/state",1,      chatterCallback1);
  jointIndx = 2;  
  ros::Subscriber sub2 = nhh2.subscribe("rrbot/joint2_position_controller/state",1,      chatterCallback2);
  jointIndx = 3;  
  //ros::Subscriber sub3 = nhh3.subscribe("rrbot/joint3_position_controller/state",1,      chatterCallback);
  jointIndx = 4;   
  //ros::Subscriber sub4 = nhh4.subscribe("rrbot/joint4_position_controller/state",1,      chatterCallback);
 
  ros::Rate rate(200);

  double x,y,z,d2, theta1p, theta1n, theta2p, theta2n, theta0p, theta1C, theta2C, theta3C, theta1F, theta2F,theta3F, theta1Step, theta2Step, theta3Step ;
  static double origX=0, origY=0, origZ=0;
  double theta1Diff=0 ,theta2Diff=0, zDiff=0;
  
     int cnt = 0;
  while(ros::ok())
  {
     if(firstRunFlag)
     {
       x=1;
       y=1;
       z=1;

     std::cout << joint[0]  <<"\n"<< joint[1] <<"\n"<<joint[2] <<"\n"<<joint[3] <<"\n" <<joint[4] <<"\n";
     d2 = (pow(x,2)+ pow(y,2) - pow(l1,2)-pow(l2,2))/(2*l1*l2); 
     theta2p = atan2(sqrt(1-pow(d2,2)),d2);
     theta1p = -1*(atan2(y,x) + atan2(l2*sin(theta2p),l1+l2*cos(theta2p)));


    
     //publishi theta1
     abc.data = theta1p;
     pub1.publish(abc);

     //publishi theta2
     abc.data = theta2p;
     pub2.publish(abc);
     abc.data = z;     
     pub3.publish(abc);
     ros::Duration(0.01).sleep();
     ros::spinOnce();   
     
     std::cout <<  "\nTheta 2C: " << theta1p << " " << angle1;//*180/PI;
     std::cout <<  "\nTheta 3C: " << theta2p << " " << angle2;//*180/PI << "\n";

     theta1C = angle1;
     theta2C = angle2;
      firstRunFlag = false;
     }
     else
     {
       if(finishedFlag)
       {
         std::cin >> x;
         std::cin >> y;
         std::cin >> z;
         finishedFlag = false;
         
         d2 = (pow(x,2)+ pow(y,2) - pow(l1,2)-pow(l2,2))/(2*l1*l2); 
         theta2p = atan2(sqrt(1-pow(d2,2)),d2);
         theta1p = -1*(atan2(y,x) + atan2(l2*sin(theta2p),l1+l2*cos(theta2p)));
         theta1F = theta1p;
         theta2F = theta2p;
         theta1Step = (theta1F-theta1C)/10;
         theta2Step = (theta2F - theta2C)/10;
	 std::cout <<"FINISHED\n";
       }

      std::cout <<"NOT\n";

     theta1C =  theta1C + theta1Step;
     theta2C =  theta2C + theta2Step;
 
     //std::cout << joint[0]  <<"\n"<< joint[1] <<"\n"<<joint[2] <<"\n"<<joint[3] <<"\n" <<joint[4] <<"\n";
     
     //publishi theta1
     abc.data = theta1C;
     pub1.publish(abc);

     //publishi theta2
     abc.data = theta2C;
     pub2.publish(abc);
     abc.data = z;     
     pub3.publish(abc);
     
     cnt++;
     std::cout <<  "\nTheta 2: " << theta1C << " " << theta1F;//*180/PI;
     std::cout <<  "\nTheta 3: " << theta2C << " " << theta2F;//*180/PI << "\n";
     if(theta1C >=  theta1F || cnt > 10)
     {
        finishedFlag = true;
     }
     ros::Duration(0.01).sleep();
     
     ros::spinOnce();   
     } 
      
  }
}

