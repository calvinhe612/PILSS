# PILSS: Pharmaceutical Industrial Limb Sorting System 

By Calvin He, Ilya Lifshits, and Kyle Young 

Worcester Polytechnic Institute 

RBE 594 Fall 2018 - Capstone Project Experience in Robotics Engineering 

Advisor: Riad Hammoud 

 

## Install following software in Ubuntu 16.04 before running PILSS: 

ROS Kinetic 

Gazebo 7.X 

RViz 

MoveIt! 

 

## Install libraries for Python 2.7: 

Numpy 

Rospy 

Sklearn 

Scipy 

Shapely 

 

## The Python implementations uses the following additional functionality and is already included in the repository: 

https://github.com/ndanielsen/Same-Size-K-Means 

https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/ 

Acknowledgement to Nathan Danielsen and Adrian Rosebrock for their open sourced implementations. 

 

# Cloning and catkin_make the PILSS repository: 

    git clone https://github.com/calvinhe612/PILSS.git 

    cp –r PILSS/catkin_ws ~/catkin_ws 

    cd ~/catkin_ws 

    catkin_make 

 

## Source catkin_ws: 

The following commands need to be executed in their own terminal. Before executing them, first need to source the catkin_ws. This can be done by first running: 

    source ~/catkin_ws/devel/setup.bash  

in every terminal before running the command, or you can add the above line to the ~/.bashrc file. 

 

## Use MoveIt! Setup Assistant and load in the robot model: 

    roslaunch moveit_setup_assistant setup_assistant.launch 

In the popped up MoveIt! Setup Assistant window, click on “Edit Existing MoveIt Configuration Package”. Then click on “Browse” and select directory: ~/catkin_ws/src/rrbot_moveit_config. Then press “Load Files”. 

 

## Run each separate command in order in their own new terminal: 

    roslaunch rrbot_moveit_config demo.launch 

    rosrun ros_nodes moveit_pills 

    rosrun ros_nodes rviz_to_gazebo_node 

    roslaunch rrbot_gazebo rrbot_world.launch 

    roslaunch rrbot_control rrbot_control.launch 

    python ~/catkin_ws/python_cv/pilss_detection.py 
