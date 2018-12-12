#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointControllerState.h>

#define PI 3.14159265
#define l1 0.45
#define l2 0.5
#define l3 0.375
#define l3T 0.5

//Globals for ROS callback
ros::Publisher pub;
std_msgs::Float64 abc;

double pointToolParallel(double fX, double fY, double tX, double tY)
{
  double x2,y2,d2,theta1p,theta2p,theta3p;

  // if(tX-fX == 0)
  // {
  //   tY = fY + 0.01;
  //   tX = fX;
  // }
  // else
  // {
  //   double slope = -(fY-tY)/(fX-tX);

  //   tX = fX + 0.01;
  //   tY = fY + 0.01*slope;
  // }

  d2 = (pow(tX,2)+ pow(tY,2) - pow(l1,2)-pow(l2,2))/(2*l1*l2); 
  theta2p = atan2(sqrt(1-pow(d2,2)),d2);
  theta1p = -1*(atan2(tY,tX) + atan2(l2*sin(theta2p),l1+l2*cos(theta2p)));

  x2 = (tX - fX)*cos(theta1p+theta2p)-(tY - fY)*sin(theta1p+theta2p);
  y2 = (tX - fX)*sin(theta1p+theta2p)+(tY - fY)*cos(theta1p+theta2p);  

  theta3p = atan2(x2,y2);

  theta3p += PI/2;

  double baseTheta3p = theta3p + theta1p + theta2p;

  int baseTheta3Deg = baseTheta3p * 180/PI;

  baseTheta3Deg %= (360);

  baseTheta3p = baseTheta3Deg *PI/180;

  // Reposition link5 to face outward
  // if(baseTheta3p < -PI/2 || baseTheta3p > PI/2)
  if(baseTheta3p < 0 || baseTheta3p > PI)
  {
    theta3p += PI;
  }

  // std::cout << "In pointToolParallel" << std::endl;
  // std::cout <<  "Theta 1: " << theta1p << std::endl;//*180/PI;
  // std::cout <<  "Theta 2: " << theta2p << std::endl;//*180/PI << "\n";
  // std::cout <<  "Theta 3: " << theta3p << std::endl;
  // std::cout <<  "d2: " << d2 << std::endl;
  // std::cout <<  "x2: " << x2 << std::endl;
  // std::cout <<  "y2: " << y2 << std::endl;

  return theta3p;
}

double pointToolPerpendicular(double fX, double fY, double tX, double tY)
{
  double x2,y2,d2,theta1p,theta2p,theta3p;

  // if(tX-fX == 0)
  // {
  //   tY = fY + 0.01;
  //   tX = fX;
  // }
  // else
  // {
  //   double slope = -(fY-tY)/(fX-tX);

  //   tX = fX + 0.01;
  //   tY = fY + 0.01*slope;
  // }
  
  d2 = (pow(tX,2)+ pow(tY,2) - pow(l1,2)-pow(l2,2))/(2*l1*l2); 
  theta2p = atan2(sqrt(1-pow(d2,2)),d2);
  theta1p = -1*(atan2(tY,tX) + atan2(l2*sin(theta2p),l1+l2*cos(theta2p)));

  x2 = (tX - fX)*cos(theta1p+theta2p)-(tY - fY)*sin(theta1p+theta2p);
  y2 = (tX - fX)*sin(theta1p+theta2p)+(tY - fY)*cos(theta1p+theta2p);  

  theta3p = atan2(x2,y2);

  double baseTheta3p = theta3p + theta1p + theta2p;

  int baseTheta3Deg = baseTheta3p * 180/PI;

  baseTheta3Deg %= (360);

  baseTheta3p = baseTheta3Deg *PI/180;

  // Reposition link5 to face outward
  // if(baseTheta3p < -3*PI/4 || baseTheta3p > 3*PI/4)
  if(baseTheta3p < 0 || baseTheta3p > PI)
  {
    theta3p += PI;
  }

  return theta3p;
}

void invKin(moveit::planning_interface::MoveGroupInterface* move_group, double x, double y, double z, double *theta1p, double *theta2p, double *theta3p, double *d3)
{
  double d2,x2,y2,x0,y0;

  d2 = (pow(x,2)+ pow(y,2) - pow(l1,2)-pow(l2,2))/(2*l1*l2); 
  *theta2p = atan2(sqrt(1-pow(d2,2)),d2);
  *theta1p = -1*(atan2(y,x) + atan2(l2*sin(*theta2p),l1+l2*cos(*theta2p)));
  *d3 = l3-z-l3T/2;

  geometry_msgs::PoseStamped current_pose = move_group->getCurrentPose();

  x0 = current_pose.pose.position.x; //need to get current x-position from current_state
  y0 = current_pose.pose.position.y; //need to get current y-position from current_state

  std::cout <<  "Current X: " << x0 << std::endl;
  std::cout <<  "Current Y: " << y0 << std::endl;

  x2 = (x - x0)*cos(*theta1p+*theta2p)-(y - y0)*sin(*theta1p+*theta2p);
  y2 = (x - x0)*sin(*theta1p+*theta2p)+(y - y0)*cos(*theta1p+*theta2p);  

  *theta3p = atan2(x2,y2);
}

void parallelMotionPose(moveit::planning_interface::MoveGroupInterface* move_group, double x, double y, double z, double *theta1p, double *theta2p, double *theta3p, double *d3)
{
  invKin(move_group,x,y,z,theta1p,theta2p,theta3p,d3);

  double baseTheta3p = *theta3p + *theta1p + *theta2p;

  int baseTheta3Deg = baseTheta3p * 180/PI;

  baseTheta3Deg %= (360);

  baseTheta3p = baseTheta3Deg *PI/180;

  // Reposition link5 to face outward
  // if(baseTheta3p < -3*PI/4 || baseTheta3p > 3*PI/4)
  if(baseTheta3p < 0 || baseTheta3p > PI)
  {
    *theta3p += PI;
  }
}

void perpendicularMotionPose(moveit::planning_interface::MoveGroupInterface* move_group, double x, double y, double z, double *theta1p, double *theta2p, double *theta3p, double *d3)
{
  invKin(move_group,x,y,z,theta1p,theta2p,theta3p,d3);

  *theta3p += PI/2;

  double baseTheta3p = *theta3p + *theta1p + *theta2p;

  int baseTheta3Deg = baseTheta3p * 180/PI;

  baseTheta3Deg %= (360);

  baseTheta3p = baseTheta3Deg *PI/180;

  // Reposition link5 to face outward
  // if(baseTheta3p < -3*PI/4 || baseTheta3p > 3*PI/4)
  if(baseTheta3p < 0 || baseTheta3p > PI)
  {
    *theta3p += PI;
  }
}

void move(moveit::planning_interface::MoveGroupInterface* move_group,const robot_state::JointModelGroup* joint_model_group,double theta1p,double theta2p,double theta3p,double d3)
{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  std::cout <<  "Theta 1: " << theta1p << std::endl;//*180/PI;
  std::cout <<  "Theta 2: " << theta2p << std::endl;//*180/PI << "\n";
  std::cout <<  "Dist 4:  " << d3 << std::endl;
  std::cout <<  "Theta 3: " << theta3p << std::endl;
    
  moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
  
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = theta1p;  // radians
  joint_group_positions[1] = theta2p;  // radians
  joint_group_positions[2] = d3;  // meters
  joint_group_positions[3] = theta3p;  //radians
  move_group->setJointValueTarget(joint_group_positions);

  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("PILSS", "Performing Motion %s", success ? "" : "FAILED");

  move_group->move();
}


//ROS callback
void motionPlan(const std_msgs::Float32MultiArray::ConstPtr& array)
{
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group.setGoalTolerance(0.001);
  move_group.setPlanningTime(10);

  
  double x,y,z,d2,d3, theta1p, theta2p, theta3p;

  // double xs[3] = {0.25,0.5,0.75};
  // double ys[3] = {0,0,0.25};
  // double zs[3] = {0.1,0,0};

  // double xs[3] = {array->data[0],array->data[3],array->data[6]};
  // double ys[3] = {array->data[1],array->data[4],array->data[7]};
  // double zs[3] = {array->data[2],array->data[5],array->data[8]};

  int steps = 6;

  double xs[1+steps] = {array->data[0],0,0,0,0,0,array->data[3]};
  double ys[1+steps] = {array->data[1],0,0,0,0,0,array->data[4]};
  double zs[1+steps] = {array->data[2],0,0,0,0,0,array->data[5]};

  float xStep = (xs[steps]-xs[0])/steps;
  float yStep = (ys[steps]-ys[0])/steps;
  //float zStep = (zs[steps]-zs[0])/steps;

  int i;
  for(i=1;i<steps;i++)
  {
    xs[i] = xs[i-1]+xStep;
    ys[i] = ys[i-1]+yStep;
    //zs[i] = zs[i-1]+zStep; //this becomes 0.2, 0.167, 0.133, ... 0.33, 0.0
    zs[i] = 0.0;
    std::cout << "zs" << zs[i] << std::endl;
  }
  

  for(i=0;i<(steps+1);i++)
  {
    x = xs[i];
    y = ys[i];
    z = zs[i];

    // if(i == 1)
    // {
    //   parallelMotionPose(&move_group,x,y,z,&theta1p,&theta2p,&theta3p,&d3);
    //   theta3p = pointToolParallel(xs[i-1],ys[i-1],x,y);
    //   move(&move_group,joint_model_group,theta1p,theta2p,theta3p,d3);
    //   ros::Duration(5).sleep();
    //   d3 = l3-l3T/2; //lower to table
    // }
    // else if(i == 2)
    // {
    //   // perpendicularMotionPose(&move_group,x,y,z,&theta1p,&theta2p,&theta3p,&d3);
    //   // move(&move_group,joint_model_group,theta1p,theta2p,theta3p,d3);
    //   // ros::Duration(5).sleep();
    //   theta3p = pointToolPerpendicular(x,y,xs[i-1],ys[i-1]);
    //   move(&move_group,joint_model_group,theta1p,theta2p,theta3p,d3);
    //   ros::Duration(5).sleep();
    //   parallelMotionPose(&move_group,x,y,z,&theta1p,&theta2p,&theta3p,&d3);
    // }

    if(i == 0)
    {
      perpendicularMotionPose(&move_group,x,y,z,&theta1p,&theta2p,&theta3p,&d3);
      theta3p = pointToolPerpendicular(x,y,xs[i+1],ys[i+1]);
      move(&move_group,joint_model_group,theta1p,theta2p,theta3p,d3);
      ros::Duration(1).sleep();
      d3 = l3-l3T/2; //lower to table
    }
    else
    {
      parallelMotionPose(&move_group,x,y,z,&theta1p,&theta2p,&theta3p,&d3);
    }
    
    move(&move_group,joint_model_group,theta1p,theta2p,theta3p,d3);

    if(i == 0 || i == steps)
    {
      ros::Duration(1).sleep();
    }
  }

  //Move off table
  float zDist = 0.1;
  d3 = l3-zDist-l3T/2;
  move(&move_group,joint_model_group,theta1p,theta2p,theta3p,d3);
  ros::Duration(1).sleep();

  //Move to home position
  perpendicularMotionPose(&move_group,0.95,0,0.2,&theta1p,&theta2p,&theta3p,&d3);
  move(&move_group,joint_model_group,theta1p,theta2p,theta3p,d3);
  ros::Duration(5).sleep();

  abc.data = 1;
  pub.publish(abc);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_demo");
  ros::NodeHandle node_handle;
  ros::NodeHandle pub_handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Subscriber sub = node_handle.subscribe("/rrbot/moveLocation",1,motionPlan);
  pub = pub_handle.advertise<std_msgs::Float64>("/rrbot/motionComplete",1000);
 
  ros::Rate rate(200);

  while(ros::ok())
  {

  }

  ros::shutdown();
}

