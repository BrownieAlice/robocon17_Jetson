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
#include "detect_cercle/MBinput.h"

void SubCallback(const detect_cercle::MBinput& msg)
{
  ROS_INFO("MBdata::MB_pole:%d,color:%c,x:%f,y:%f,theta:%f", (int)msg.MB_pole, (char)msg.color, msg.x, msg.y, msg.theta);
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
