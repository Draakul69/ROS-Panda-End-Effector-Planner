Code delveloped by Abubakar Boyi
Student ID 200871774

To run the package

1) Unzip into catkin workspace


2) Make sure relevent dependancies are pre-installed
Downlaod the following packages:
-- git clone -b ROS_DISTRO-devel https://github.com/ros-planning/moveit_tutorials.git;
-- git clone -b ROS_DISTRO-devel https://github.com/ros-planning/panda_moveit_config.git;
Where ROS_DISTRO is your Ros distribution (e.g Kinetic, Melodic)

3) Run catkin_make to ensure dependencies are built

4) source your workspace

5) run the launch file for Rviz with the command ""roslaunch 
ar_week10_test demo.launch

6) Run the node for generating random square lengths with the command ""rosrun ar_week10_test square_size_generator.py""

7) Run the node for commanding the panda arm with the command ""rosrun ar_week10_test move_panda_square.py""


