/*
 *MIT License
 *
 *  Copyright (c) 2017 Banuprathap Anandan
 *
 *  AUTHOR : BANUPRATHAP ANANDAN
 *  AFFILIATION : UNIVERSITY OF MARYLAND, MARYLAND ROBOTICS CENTER
 *  EMAIL : BPRATHAP@UMD.EDU
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 *
 *
 *  Program: Simulator for a rectangular robot
 *
 *
 */

/**
 * @file walker.cpp
 * @brief A simple program that drives turtlebot
 *        straight until it senses an obstacle and
 *        back to the start location in a loop
 * @author Banuprathap Anandan
 * @date   04/18/2017
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


class Walker {
  public:
    Walker(ros::NodeHandle& n);
    void callback(const sensor_msgs::LaserScan::ConstPtr& input);
  private:
    float obsDist;
    bool forward;
    ros::Subscriber subLaser;
    ros::Publisher pub;
    geometry_msgs::Twist twist;
};  //  End of class

void Walker::callback(const sensor_msgs::LaserScan::ConstPtr& input) {
  float min = 0;
  for (int i = 0; i < input->ranges.size(); i++) {
    if (input->ranges[i] > min)
      min = input->ranges[i];
  }
  obsDist = min;
  ROS_INFO("Distance Ahead %0.1f", obsDist);
}




Walker::Walker(ros::NodeHandle& n) {
  subLaser = n.subscribe("/scan", 1000, &Walker::callback, this);
  pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  ros::Rate loop_rate(2);
  while (n.ok()) {
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    if (obsDist > 0.8) {
      twist.linear.x = 0.1;
      ROS_INFO("Distance: %0.1f Moving Forward", obsDist);
    } else {
      twist.angular.z = 1.5;
      ROS_INFO("Distance: %0.1f Turning", obsDist);
    }
    pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");
  ros::NodeHandle n;
  Walker walk(n);
  return 0;
}
