/*
======================================================================
Project Name    : test subscribe data
File Name       : test_sub_data
Encoding        : UTF-8
Creation Date   : 2017/02/27

Copyright © 2017 Alice.
======================================================================
*/

#include "ros/ros.h"
#include "detect_circle/MBinput.h"

void SubCallback(const detect_circle::MBinput& msg)
{
  ROS_INFO("MBdata::MB_pole1:%d,MB_pole2:%d,color:%c,x:%f,y:%f,theta:%f,x_sigma:%f,y_sigma:%f,theta_sigma:%f", (int)msg.MB_pole1, (int)msg.MB_pole2, (char)msg.color, msg.x, msg.y, msg.theta, msg.x_sigma, msg.y_sigma, msg.theta_sigma);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_sub_data");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("MBdata", 1, SubCallback);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
