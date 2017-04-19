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

class Walker {
  public:
    Walker() {
      sub = n.subscribe("/scan", 1000, &Walker::callback, this);
    }
    void callback(const sensor_msgs::LaserScan::ConstPtr& input) {
      ROS_INFO("RANGE SIZE %ld", input->ranges.size());
    }
  private:
    ros::NodeHandle n;
    ros::Subscriber sub;
};  //  End of class

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("I heard: %s", msg->data.c_str());
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");
  Walker walk;
  ros::spin();
  return 0;
}

