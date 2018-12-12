#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Int32.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <iostream>
#include <fstream>



#define PI 3.14159265
#define l1 0.45	//length of the first link
#define l2 0.5 //length of the second link

class Robot
{
  public:
   int motionFlag = 0;
   float xCamPos=0;
   float yCamPos=0;
   int lClickCnt=0;
   std::vector<float> motionPlan;
   std::vector<std::vector<float>> calibrationMatrix{{0, 0, 0},
                                                     {0, 0, 0},
                                                     {0, 0, 0},};    
   
   void motionFlagCountCallback(const std_msgs::Int32::ConstPtr& msg);
   void xCallback(const std_msgs::Float64::ConstPtr& msg);
   void yCallback(const std_msgs::Float64::ConstPtr& msg);
   void lClickCountCallback(const std_msgs::Int32::ConstPtr& msg);
   void setCalibrationMatrix();
   void motionPlanCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
   float getPathSlope();
   std::vector<float> getConvertedMouseClickXY();

};


//store motionFlag
void Robot::motionFlagCountCallback(const std_msgs::Int32::ConstPtr& msg)
{
   motionFlag = msg->data;  
}

//store x mouse click position
void Robot::xCallback(const std_msgs::Float64::ConstPtr& msg)
{
   xCamPos = msg->data;  
}

//store y mouse click position
void Robot::yCallback(const std_msgs::Float64::ConstPtr& msg)
{
   yCamPos = msg->data;
}

//store number of left clicks
void Robot::lClickCountCallback(const std_msgs::Int32::ConstPtr& msg)
{
   lClickCnt = msg->data;
}

float Robot::getPathSlope()
{
  if(motionFlag)
  {
    float slope = (motionPlan[3] - motionPlan[1]) / (motionPlan[0]-motionPlan[2]);
    return slope;
  }else
  {
    return 0;
  }
}

//load calibration matrix from file and store it in calibrationMatrix
void Robot::setCalibrationMatrix()
{
    std::ifstream  data("/home/ilya/calMatrix.csv");
    std::string line;
    for(int i = 0; i < 3; i++)
    {
        std::getline(data,line);
        std::stringstream lineStream(line);
        std::string cell;
        for(int k = 0; k < 3; k++)
        { 
           getline(lineStream,cell,',');
           std::cout<<cell;
           calibrationMatrix[i][k] = std::stof(cell);
        }
        
    }
    //Print calibration matrix
    std::cout<<"\nCalibration Matrix: \n";
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            std::cout << calibrationMatrix[i][j] << "\t";
        }
        std::cout << "\n";
    }

}

//store motion plan set by right clicks in camera space
void Robot::motionPlanCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{  
  motionPlan = array->data;
}


//return cameraXY position converted to robot space 
std::vector<float> Robot::getConvertedMouseClickXY()
{
        
  std::vector<float> xypoint;
  float camSpacePos[1][3];
  float robSpacePos[1][3];
 
  //reset robot space matrix
  robSpacePos[0][0] = 0;
  robSpacePos[0][1] = 0;
  robSpacePos[0][2] = 1;
  //set camera space matrix
  camSpacePos[0][0] = xCamPos;
  camSpacePos[0][1] = yCamPos;
  camSpacePos[0][2] = 1;
  //convert camera space to robot space
  for(int j = 0; j < 3; ++j){
    for(int k = 0; k < 3; ++k)
    {
      robSpacePos[0][j] += camSpacePos[0][k] * calibrationMatrix[k][j];
    }
  }
  xypoint.push_back(robSpacePos[0][0]);
  xypoint.push_back(robSpacePos[0][1]);
  return xypoint;
} 



///////////Testing
//std_msgs::Float64 joint[5];
void joint1_PositionState(const control_msgs::JointControllerState::ConstPtr& msg)
{
   //std::cout  << std::endl << "1" << ":   " <<msg->set_point << std::endl;
  //joint[jointIndx] = msg->set_point;
}

void joint2_PositionState(const control_msgs::JointControllerState::ConstPtr& msg)
{
  // std::cout  << std::endl << "2" << ":   " <<msg->set_point << std::endl;
  // joint[jointIndx] = msg->set_point;
}
//////////////


//return inverse kinematics calculations based on (x,y) end effector position  
std::vector<float> inverseK(float x, float y, float xDirection, float yDirection, float pathSlope)
{
  std::vector<float>  jointangles;
  float theta1p=0, theta2p=0, theta4p=0;
  //height off the floor
  float theta3p = -0.01;
  
  float d2 = (pow(x,2)+ pow(y,2) - pow(l1,2)-pow(l2,2))/(2*l1*l2); 
  theta2p = atan2(sqrt(1-pow(d2,2)),d2);
  theta1p = -1*(atan2(y,x) + atan2(l2*sin(theta2p),l1+l2*cos(theta2p)));
  
  if(pathSlope != 0)
  {
    double normalToPath = -(1/pathSlope);
    xDirection = x+5;
    yDirection = y+5*pathSlope; 

    //calculate blade angle theta4 needed to point towards xDirection yDirection
    float xt2 = (xDirection-x)*cos(theta1p+theta2p)-(yDirection-y)*sin(theta1p+theta2p);
    float yt2 = (xDirection-x)*sin(theta1p+theta2p)+(yDirection-y)*cos(theta1p+theta2p); 
    float theta4p =  -atan2(yt2,xt2);
  }

  jointangles.push_back(theta1p);
  jointangles.push_back(theta2p);
  jointangles.push_back(theta3p);
  jointangles.push_back(theta4p);
  return jointangles;

}


int main(int argc, char** argv)
{
  Robot myRobot;
  ros::init(argc,argv, "calc_node");
  ros::NodeHandle nh;
  
  ros::Publisher joint0pub = nh.advertise<std_msgs::Float64>("rrbot/joint0_position_controller/command",1000);
  ros::Publisher joint1pub = nh.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",1000);
  ros::Publisher joint2pub = nh.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",1000);
  ros::Publisher joint3pub = nh.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",1000);
  ros::Publisher joint4pub = nh.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",1000);
  ros::Publisher motionFlagpub = nh.advertise<std_msgs::Int32>("rrbot/motionFlag",1000);
  

 // ros::init(argc,argv, "listener");
  ros::NodeHandle nhh;
  
  ros::Subscriber joint1sub = nh.subscribe("rrbot/joint1_position_controller/state",1,      joint1_PositionState);
  ros::Subscriber joint2sub = nh.subscribe("rrbot/joint2_position_controller/state",1,      joint2_PositionState);
  ros::Subscriber xMouseClicksub = nh.subscribe("rrbot/xmouseclick",1,      &Robot::xCallback, &myRobot);
  ros::Subscriber yMouseClicksub = nh.subscribe("rrbot/ymouseclick",1,      &Robot::yCallback, &myRobot);
  ros::Subscriber lClickCountsub = nh.subscribe("rrbot/lClickCount",1, &Robot::lClickCountCallback, &myRobot);
  ros::Subscriber motionFlagsub = nh.subscribe("rrbot/motionFlag",1, &Robot::motionFlagCountCallback, &myRobot);
  ros::Subscriber motionPlansub = nh.subscribe("rrbot/motionPlan", 1, &Robot::motionPlanCallback, &myRobot);

  ros::spinOnce();
  ros::Rate rate(200);

  float x,y,z=-0.1,d2, theta1p, theta1n, theta2p, theta2n, theta0p;
  
  //true while performing calibration
  bool isCalibrationDone = false;
  //3 calibration positions
  float calibPos[3][2] = {{0.5,0.5},
                           {0.5,-0.5},
                           {-0.5,0.5}};
  std::vector<float> xyPoint;
  std_msgs::Int32 motionFlagROS;
  int stepCnt = 0;

  while(ros::ok())
  { 
      //perform calibration routine 
      if(!isCalibrationDone)
      { 
        if(myRobot.lClickCnt == -1)
        {
          x = calibPos[0][0];
          y = calibPos[0][1];
         // std::cout<<"\n" <<"Click on calibration point: " << calibrationPointCnt << " at: " << x <<"," << y<< "\n" ;
          
        }else{
	  x = calibPos[myRobot.lClickCnt][0];
          y = calibPos[myRobot.lClickCnt][1];
          //std::cout<<"\n" <<"Click on calibration point: " << calibrationPointCnt << " at: " << x <<"," << y<< "\n" ;
        }
        std::cout<<"\nCalibration matrix: \n";
	if(myRobot.lClickCnt > 2)
        {
           myRobot.setCalibrationMatrix();
          isCalibrationDone = true;
        }
      }else if(myRobot.motionFlag == 1)
      {
          std::vector<float> xyPoint;
          float cameraXStart = myRobot.motionPlan[0];
          float cameraYStart = myRobot.motionPlan[1];
          float cameraXEnd = myRobot.motionPlan[2];
          float cameraYEnd = myRobot.motionPlan[3];
          float xStep = (cameraXEnd-cameraXStart)/1000;
          float yStep = (cameraYEnd-cameraYStart)/1000;
          
          myRobot.xCamPos = cameraXStart + stepCnt*xStep;
          myRobot.yCamPos = cameraYStart + stepCnt*yStep;
          stepCnt++;
          xyPoint = myRobot.getConvertedMouseClickXY();
          x = xyPoint[0];
          y = xyPoint[1];
          std::cout<<"\n" <<"X: " << x <<"\n";
          std::cout<< "Y: " << y <<"\n";
          if(stepCnt > 1000)
          { 
	    stepCnt=0;
            motionFlagROS.data = 0;
            motionFlagpub.publish(motionFlagROS);
          }
          
      }else
      {
          xyPoint = myRobot.getConvertedMouseClickXY();
          x = xyPoint[0];
          y = xyPoint[1];
          
      }
     float pathSlope = myRobot.getPathSlope();
     std::vector<float> jointAngleVector;
     jointAngleVector = inverseK(x,y, 0, 0, pathSlope);
     
     std_msgs::Float64 holdRosFloat;
     //publishi theta1
     holdRosFloat.data = jointAngleVector[0];
     joint1pub.publish(holdRosFloat);

     //publishi theta2
     holdRosFloat.data = jointAngleVector[1];
     joint2pub.publish(holdRosFloat);

     holdRosFloat.data = jointAngleVector[2];     
     joint3pub.publish(holdRosFloat);
      
     //calculate perpendicular to the motion 
       
     holdRosFloat.data = jointAngleVector[3];     
     joint4pub.publish(holdRosFloat);

     rate.sleep();     
     ros::spinOnce();

  }
}

