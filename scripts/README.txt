Instructions for Matlab/Python talking to one another in ROS. This is a derivative of the ROS Tutorials, so only do what is required.

Author: Steve Crews, 10 Feb 2022


ASSUMES ROS IS ALREADY INSTALLED ON COMPUTER WITH NUMPY PACKAGE IN PYTHON


-------------------------------------
1. CREATING CATKIN WORKSPACE

*Skip this step if you want to work in an existing catkin workspace.

You can integrate into an existing catkin workspace (i.e. "catkin_ws") or start a new workspace for this tutorial. I recommend starting a new workspace just for practice. Thus, we use "catkin_ws3" instead of "catkin_ws" below.  

Extracts from 
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

$ source /opt/ros/noetic/setup.bash 
$ mkdir -p ~/catkin_ws3/src
$ cd ~/catkin_ws3/
$ catkin_make
$ source devel/setup.bash


-------------------------------------
2. CREATING A ROS PACKAGE

*Skip this step if you want to work in an existing ROS package.

You can integrate into an existing ROS package (i.e. "beginner_tutorials") or start a new ROS package for this tutorial. I recommend starting a new package just for practice. Thus, we use "ROSpgk_python_matlab" instead of "beginner_tutorials" below.  

Extracts from
http://wiki.ros.org/catkin/Tutorials/CreatingPackage

$ cd ~/catkin_ws3/src
$ catkin_create_pkg ROSpgk_yourname std_msgs rospy roscpp
$ cd ~/catkin_ws3
$ catkin_make
$ . ~/catkin_ws3/devel/setup.bash


-------------------------------------
3. ADDING SOME SCRIPTS

We will use Python and Matlab scripts to talk to one another via ROS.

Python extracts from
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
and
http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber




TERMINAL 1
$ roscore

For each subsequent terminal
$ source /opt/ros/noetic/setup.bash
$ source devel/setup.bash

$ rostopic list
/rosout
/rosout_agg


TERMINAL 2: Publish /Aflat_py
$ rosrun ROSpgk_python_matlab python_pub.py

$ rostopic list (new items marked with *)
/Aflat_py*
/rosout
/rosout_agg



TERMINAL 3: Publish /Bflat_mat
Start matlab instance for publishing
$ matlab
>> rosinit
-run matlab_pub.m


$ rostopic list
/Aflat_py
/Bflat_mat*
/rosout
/rosout_agg


TERMINAL 4: Subscribe /Aflat_py /Bflat_mat Publish Cflat_py Dflat_py
Subscribe to python- and matlab-originated ROS messages, do matrix manipulation, and then publish
$ rosrun ROSpgk_python_matlab python_sub_ops_pub.py

$ rostopic list
/Aflat_py
/Bflat_mat
/Cflat_py*
/Dflat_py*
/rosout
/rosout_agg


TERMINAL 5: Subscribe /Aflat_py /Bflat_mat Publish Cflat_mat Dflat_mat
Subscribe to python- and matlab-originated ROS messages, do matrix manipulation, and then publish
Start matlab instance for publishing
$ matlab
>> rosinit
-run matlab_sub_pub.m

$ rostopic list 
/Aflat_py
/Bflat_mat
/Cflat_mat*
/Cflat_py
/Dflat_mat*
/Dflat_py
/rosout
/rosout_agg




At this point, both Python and Matlab are pushing out the same data
/Cflat_py.data = /Cflat_mat.data
/Dflat_py.data = /Dflat_mat.data




