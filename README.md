PILSS: Pharmaceutical Industrial Limb Sorting System


cd ~/catkin_ws

git clone https://github.com/calvinhe612/PILSS.git

catkin_make

source ~/catkin_ws/devel/setup.bash

# Run following in seperate terminal
roslaunch rrbot_moveit_config demo.launch
rosrun abc_qa moveit_demo
rosrun abc_qa rviz_to_gazebo_node

roslaunch rrbot_gazebo rrbot_world.launch
roslaunch rrbot_control rrbot_control.launch

python pill_tracker/pill_tracker.py

TODO:
- clean up code
- rename ros packages
