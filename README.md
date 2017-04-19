[![Build Status](https://travis-ci.org/banuprathap/turtlebot_walker.svg?branch=master)](https://travis-ci.org/banuprathap/turtlebot_walker)

ROS Turtlebot Random Waker
============================
- ROS package containing a node to make a Turtlebot wander around with obstacle
avoidance by simply moving forward and turning when an obstacle is detected


## Dependencies

- ROS Indigo running on Ubuntu 14.04. 
- Gazebo 
- If you do not have ROS, download the bash script [here](https://gist.github.com/banuprathap/b2dab970df1f89573203b546c5eb3a5c) and run it as **sudo**. Note: This script assumes you're running Ubuntu 14.04.


### Package Dependency
- roscpp
- rospy
- sensor_msgs
- geometry_msgs
- turtlebot_gazebo

## Build steps
- Open a terminal
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src && catkin_init_workspace
git clone https://github.com/banuprathap/turtlebot_walker.git
cd ..
catkin_make
source ./devel/setup.bash
```

## Running the demo
- In your terminal

```bash
roslaunch turtlebot_walker navigate.launch
```

## Enable rosbag recording

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch turtlebot_walker navigate.launch enable_record:=true
```
## Inspect rosbag 

```bash
cd ~/.ros/
rosbag info recording.bag
```

## Create Documentation

You will require **rosdoc_lite** package to create documents in ROS standards. You can get that by running
```bash
sudo apt-get install ros-indigo-rosdoc-lite
```
Once installed, run the following command replacing ***path*** with absolute path to the package in your file system. Assuming the *catkin_ws* is in your home directory, ***path*** would be *~/catkin_ws/src/turtlebot_walker/*

```bash
rosdoc_lite path
```