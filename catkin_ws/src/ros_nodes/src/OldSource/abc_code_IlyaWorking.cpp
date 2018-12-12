#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>
#include <cmath>

#define PI 3.14159265
#define l1 1
#define l2 1
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
  
  ros::Rate rate(200);

  double x,y,z,d2, theta1p, theta1n, theta2p, theta2n, theta0p;
  while(ros::ok())
  {
     
     std::cin >> x;
     std::cin >> y;
     std::cin >> z;
     

     d2 = (pow(x,2)+ pow(y,2) - pow(l1,2)-pow(l2,2))/(2*l1*l2); 

     theta2p = atan2(sqrt(1-pow(d2,2)),d2);
     theta2n = -atan2(sqrt(1-pow(d2,2)),d2);
     theta1p = -1*(atan2(y,x) + atan2(l2*sin(theta2p),l1+l2*cos(theta2p)));
     theta1n = atan2(y,x) - atan2(l2*sin(theta2p),l1+l2*cos(theta2p));
     //std::cout << "\n" << "Theta 0: "<<theta0p*180/PI;
     std::cout <<  "\nTheta 2: " << theta1p*180/PI;
     std::cout <<  "\nTheta 3: " << theta2p*180/PI << "\n";
     std::cout <<  "\nTheta 2: " << theta1n*180/PI << "\n";
     std::cout <<  "\nTheta 3: " << theta2n*180/PI << "\n";

     //theta1p = 
    // a = 0.6;
      
//     ros_time = ros::time::now();
     theta0p = atan2(y,x);
     abc.data = theta0p;
     abc.data = 0.0;
     //pub0.publish(abc);
     abc.data = theta1p;
     //abc.data = y;
     pub1.publish(abc);
     abc.data = theta2p;
     pub2.publish(abc);
     abc.data = 0.0;
     abc.data = z;     
     pub3.publish(abc);
     
     //ros::spinOnce();
     rate.sleep();
  }
}

