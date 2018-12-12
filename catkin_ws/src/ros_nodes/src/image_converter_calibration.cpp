
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
using namespace cv;
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <iostream>
#include <fstream>

struct cameraViewInfo{

Point lClickPos;
//hold total left click count
int lClickCount = 0;
//hold total right click count
int rClickCount = 0;

};


//hold motion plan designated by right click
std_msgs::Float32MultiArray motionPlan;


//setting flag to 1 indicates calc node should start the motion
std_msgs::Int32 motionFlagROS;

//array publish testing
//std_msgs::Float32MultiArray calibArray;


//calculate calibration matrix
void calcCalibrationMatrix(std::vector<std::vector<float>> camViewPoints)
{
    //print camera view matrix
    std::cout<<"\n\nCamera view points: \n";
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            std::cout<<camViewPoints[i][j] << "\t";
        }
        std::cout << "\n";
    }
    
    //calculate determinant
    float determinant = 0;
    for(int i = 0; i < 3; i++)
    {
        determinant = determinant + (camViewPoints[0][i] * (camViewPoints[1][(i+1)%3] 
          * camViewPoints[2][(i+2)%3] - camViewPoints[1][(i+2)%3] * camViewPoints[2][(i+1)%3]));
    }

    std::cout<<"\n\ndeterminant: "<<determinant;

    //calculate inverse of camera view matrix 
    static std::vector<std::vector<float> > camInverse = {{0, 0, 0},
    					                  {0, 0, 0},
					                  {0, 0, 0}};
    
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            camInverse[i][j] = ((camViewPoints[(j+1)%3][(i+1)%3] * camViewPoints[(j+2)%3][(i+2)%3]) 
              - (camViewPoints[(j+1)%3][(i+2)%3] * camViewPoints[(j+2)%3][(i+1)%3]))/determinant;           
        }
    }
  
    //print camInverse[3][3];
    std::cout<<"\n Inverse matrix: \n";
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            std::cout<<camInverse[i][j] << "\t";
        }
        std::cout << "\n";
    }
    
    //hold robot space locations
    static std::vector<std::vector<float> > matRobot =   {{0.5,  0.5, 1},
    					                  {0.5, -0.5, 1},
					                  {-0.5, 0.5, 1}};
    //print robot space matrix
    std::cout << "\n\nRobot space points: \n";
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++){
            std::cout << matRobot[i][j] << "\t";
        }
        std::cout << "\n";
    }

    //calibration matrix 
    static std::vector<std::vector<float> > calibMat =   {{0, 0, 0},
    					                  {0, 0, 0},
					                  {0, 0, 0}};

    //calculate calibration matrix
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
	   for(int k=0; k < 3; k++)
	   {
               calibMat[i][j]  += camInverse[i][k] * matRobot[k][j];
	   }
        }
    }
    
    //Print calibration matrix
    std::cout<<"\nCalibration Matrix: \n";
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            std::cout << calibMat[i][j] << "\t";
        }
        std::cout << "\n";
    }

    //store calibration matrix to one dim array for publishing to a ROS topic
    /*for(int i = 0; i < 3; i++){
       for(int j = 0; j < 3; j++){
    	    calibArray.data.push_back(calibMat[i][j]);
          }
    }*/

    //store calibration matrix to file
    std::string filePath = "/home/ilya/calMatrix.csv"; 
    std::cout<<"\nSaving Matrix to: " + filePath << "\n";
    std::ofstream myfile;
    
    myfile.open(filePath);
    
    std::string str = "";
    for(int k = 0; k < 3; k++){
       for(int j = 0; j < 3; j++)
       {
         str = str + std::to_string(calibMat[k][j]) + ",";   
       }
       str.pop_back();
       str = str + "\n";
       myfile << str;
       std::cout <<str;
       str = "";
    }
    myfile.close();

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
     // Create black empty images
     Mat imageToEdit;
     imageToEdit = cv_bridge::toCvShare(msg, "bgr8")->image;
     if(motionFlagROS.data == 1)
     {
         line( imageToEdit, Point( motionPlan.data[0], motionPlan.data[1] ), Point( motionPlan.data[2], motionPlan.data[3]), 
           Scalar( 110, 220, 0 ),  2, 8 );
     }
     cv::imshow("view", imageToEdit);
     cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


void printXY(int  x, int  y)
{
  std::cout << "\n" << "x: " << x << " y: " << y ;
}

void mouse_callback(int  event, int  x, int  y, int  flag, void *ptr)
{
    //store position of the first 3 clicks
    static std::vector<std::vector<float> > camMat = {{0, 0, 0},
    					              {0, 0, 0},
					              {0, 0, 0}};
    cameraViewInfo *myCameraInfo = static_cast<cameraViewInfo *>(ptr);
    if (event == EVENT_LBUTTONDOWN)
    {
        myCameraInfo->lClickPos.x = x;
        myCameraInfo->lClickPos.y = y;
        printXY(x,y);

        if(myCameraInfo->lClickCount < 3)
        {
          camMat[myCameraInfo->lClickCount][0] = (float)x;
          camMat[myCameraInfo->lClickCount][1] = (float)y;
          camMat[myCameraInfo->lClickCount][2] = 1.0;
        }
	
        if(myCameraInfo->lClickCount == 3)
        {
            calcCalibrationMatrix(camMat);
        }
        myCameraInfo->lClickCount++;
       
    }if(event == EVENT_RBUTTONDOWN)
    {
        if(myCameraInfo->rClickCount == 0)
	{          
          motionPlan.data.push_back((float)x);
          motionPlan.data.push_back((float)y);
	}else if(myCameraInfo->rClickCount == 1)
        {
          motionPlan.data.push_back((float)x);
          motionPlan.data.push_back((float)y);
        }
        myCameraInfo->rClickCount++;
        
        
    }
}

    
int main(int argc, char **argv)
{
  
  cameraViewInfo myCameraInfo;
  
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  
  //publisher for x location of the mouse click
  ros::Publisher xPublisher=nh.advertise<std_msgs::Float64>("rrbot/xmouseclick/",1000);

  //publisher for y location of the mouse click
  ros::Publisher yPublisher=nh.advertise<std_msgs::Float64>("rrbot/ymouseclick/",1000);

  //publisher for mouse click counter
  ros::Publisher lClickCntPublisher=nh.advertise<std_msgs::Int32>("rrbot/lClickCount/",1000);

  //create view of the camera feed
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber imageSub = it.subscribe("rrbot/camera3/image_raw", 1, imageCallback);
  
  setMouseCallback("view", mouse_callback, &(myCameraInfo));

  //publisher for right click counter
  ros::Publisher rClickCntPublisher=nh.advertise<std_msgs::Int32>("rrbot/rClickCount/",1000);
  //publisher for start and end point array
  ros::Publisher motionPlanPublisher = nh.advertise<std_msgs::Float32MultiArray>("rrbot/motionPlan", 1000);

  ros::Publisher motionFlagPublisher=nh.advertise<std_msgs::Int32>("rrbot/motionFlag",1000);
  
  std_msgs::Float64 xLoacationROS;
  std_msgs::Float64 yLoacationROS;
  
  std_msgs::Int32 lClickCountROS;
  std_msgs::Int32 rClickCountROS;
 
  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    //publish x  
    xLoacationROS.data = (double)myCameraInfo.lClickPos.x; 
    xPublisher.publish(xLoacationROS);
    //publish y  
    yLoacationROS.data = (double)myCameraInfo.lClickPos.y; 
    yPublisher.publish(yLoacationROS);
    //publish mouse click counter  
    lClickCountROS.data = myCameraInfo.lClickCount; 
    lClickCntPublisher.publish(lClickCountROS);

    //publish right click counter  
    rClickCountROS.data = myCameraInfo.rClickCount; 
    rClickCntPublisher.publish(rClickCountROS);

    //publish motion plan once two right clicks happen
    if(myCameraInfo.rClickCount==2)
    {
      motionPlanPublisher.publish(motionPlan);
      motionFlagROS.data = 1;
      motionFlagPublisher.publish(motionFlagROS);
      myCameraInfo.rClickCount = 0;
      motionPlan.data.clear();    
    }
  
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  cv::destroyWindow("view");
}
