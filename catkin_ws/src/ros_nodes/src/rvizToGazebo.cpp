#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>


ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
std_msgs::Float64 abc;

void forwardToGazebo(const sensor_msgs::JointState::ConstPtr &msg)
{
  float offset = 0.19672;
  abc.data = msg->position[0];
  pub1.publish(abc);
  abc.data = msg->position[1];
  pub2.publish(abc);
  abc.data = msg->position[2] - offset;
  pub3.publish(abc);
  abc.data = msg->position[3];
  pub4.publish(abc);
}




int main(int argc, char** argv)
{
  static float a = 0;
 
  ros::init(argc,argv, "abc");
  //ros::NodeHandle nh1;
  ros::NodeHandle nh2;
  ros::NodeHandle nh3;
  ros::NodeHandle nh4;
  ros::NodeHandle nh5;

  //ros::Publisher pub0 = nh1.advertise<std_msgs::Float64>("rrbot/joint0_position_controller/command",1000);
  pub1 = nh2.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",1000);
  pub2 = nh3.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",1000);
  pub3 = nh4.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",1000);
  pub4 = nh5.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",1000);

  ros::NodeHandle nhh1;
 
  ros::Subscriber sub1 = nhh1.subscribe("/joint_states",1,forwardToGazebo);
 
  ros::Rate rate(200);

  ros::spin();
}

