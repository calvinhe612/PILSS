#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
void chatterCallback(const control_msgs::JointControllerState::ConstPtr& msg)
{
  // std::cout << msg->set_point << std::endl;

}


int main(int argc, char** argv)
{
  ros::init(argc,argv, "listener");
  ros::NodeHandle nh;
 // ros::Rate loop_rate(1000000);
  ros::Subscriber sub = nh.subscribe("rrbot/joint2_position_controller/state",1,      chatterCallback);
  
 // ros::spin();
  
  std_msgs::Float64 abc;
  //abc.data = 0;
  
  ros::Rate rate(1);
  while(ros::ok())
  {
    // ros_time = ros::time::now();
    // pub.publish(abc);
     ros::spinOnce();
     rate.sleep();
   //ros::spin();
  }
}

