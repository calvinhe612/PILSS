#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>


std_msgs::Float64 joint[5];
int jointIndx = 0;

void chatterCallback1(const control_msgs::JointControllerState::ConstPtr& msg)
{
   std::cout  << std::endl << "1" << ":   " <<msg->set_point << std::endl;
  // joint[jointIndx] = msg->set_point;
}

void chatterCallback2(const control_msgs::JointControllerState::ConstPtr& msg)
{
   std::cout  << std::endl << "2" << ":   " <<msg->set_point << std::endl;
  // joint[jointIndx] = msg->set_point;
}


#define PI 3.14159265
#define l1 1.05
#define l2 1.1




int main(int argc, char** argv)
{
  static float a = 0;
 
  ros::init(argc,argv, "commandNode");
  ros::NodeHandle nh1;
  ros::NodeHandle nh2;
  ros::NodeHandle nh3;
  ros::NodeHandle nh4;
  ros::NodeHandle nh5;

  ros::Publisher pubX = nh1.advertise<std_msgs::Float64>("xPosition",1000);
  ros::Publisher pubY = nh2.advertise<std_msgs::Float64>("yPosition",1000);
  ros::Publisher pubZ = nh3.advertise<std_msgs::Float64>("zPosition",1000);

  
  std_msgs::Float64 X,Y,Z;
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
     std::cin >> X.data;
     std::cin >> Y.data;
     std::cin >> Z.data;

     pubX.publish(X);
     pubY.publish(Y);
     pubZ.publish(Z);

     
     ros::spinOnce();
     loop_rate.sleep();
     
  
  }
}

