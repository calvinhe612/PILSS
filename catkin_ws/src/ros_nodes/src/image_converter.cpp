#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
using namespace cv;
#include <std_msgs/Float64.h>

std_msgs::Float64 pubData;
Point pt(-1,-1);
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        // Store point coordinates
        pt.x = x;
        pt.y = y;
       // newCoords = true;
	std::cout<<x<<"\n"<<y<<"\n";
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  ros::NodeHandle nh2;
  ros::Publisher pub2=nh2.advertise<std_msgs::Float64>("test/xmouseclick/",1000);
  ros::NodeHandle nh3;
  ros::Publisher pub3=nh3.advertise<std_msgs::Float64>("test/ymouseclick/",1000);
  ros::Rate rate(200);

  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("rrbot/camera3/image_raw", 1, imageCallback);
  setMouseCallback("view", mouse_callback);
 ros::Rate loop_rate(10);
  while(ros::ok())
  {
 
  double xclick, yclick;
  pubData.data = (double)pt.x; 
  pub2.publish(pubData);
  pubData.data = (double)pt.y; 
  pub3.publish(pubData);
  ros::spinOnce();
  loop_rate.sleep();
  }
  cv::destroyWindow("view");
}
