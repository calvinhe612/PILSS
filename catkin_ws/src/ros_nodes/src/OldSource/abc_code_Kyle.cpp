#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>
#include <cmath>

#define PI 3.14159265
#define l1 0.95
#define l2 0.90
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

  double x=0,y=0,z=0,xPos,yPos,zPos,d2, theta1p, theta1n, theta2p, theta2n, theta0p;
  while(ros::ok())
  {
     float thetas[3];
     float pos[3];

     std::cin >> xPos;
     std::cin >> yPos;
     std::cin >> zPos;
     float a[4] = {1,1,0.0,0.0};
     float d[4] = {1,0.0,0.0,0.25};

	//Compute inverse kinematics
	float c2 = (pow(xPos,2)+pow(yPos,2)-pow(a[0],2)-pow(a[1],2))/(2*a[0]*a[1]);
	float t2 = atan2(c2,sqrt(1-c2))-M_PI/2; //for some reason, I needed to subtract pi/2 for t2 to be correct
	float t1 = atan2(xPos,yPos)-atan2(a[0]+a[1]*c2,a[1]*sin(t2)); //value is correct at (1 0 0.4), but -90 instead of 90 for (0 1 0.4)
	float t4 = t1+t2; // - atan2(r11,r12). r11 and r12 must come from desired rotation matrix
	float d3 = d[0] - zPos - d[3]; //had to use d[0] value since I added it in

	//Assign results to input pointers
	thetas[0] = t1;///M_PI*180;
	thetas[1] = t2;///M_PI*180;
	thetas[3] = t4;///M_PI*180;
	//*dist = d3;     
	
     //theta1p = 
    // a = 0.6;
      
//     ros_time = ros::time::now();
     theta0p = atan2(y,x);
     abc.data = theta0p;
     abc.data = 0.0;
     //pub0.publish(abc);
     
     abc.data = thetas[0];
     //abc.data = y;
     pub1.publish(abc);
     
     abc.data = thetas[1];
     pub2.publish(abc);
     
     abc.data = 0.0;
     abc.data = d3;     
     pub3.publish(abc);
     
     abc.data = thetas[3];
     pub4.publish(abc);
     
     std::cout << t1/M_PI*180 << "\n" <<t2/M_PI*180 << "\n" << t4/M_PI*180 << "\n" << d3 << "\n";
     //ros::spinOnce();
     rate.sleep();
  }
}

