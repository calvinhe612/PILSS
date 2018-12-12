#include <ros/ros.h>
#include <std_msgs/Int32.h>
int main(int argc, char** argv)
{
  ros::init(argc,argv, "abc");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Int32>("abc_topic",1000);
  
  std_msgs::Int32 abc = 0;
  
 
  abc;
  
  while(ros : ok)
  {
     ros_time = ros::time::now();
     pub>publish(abc);
     ros::spinOnce();
  }
}

