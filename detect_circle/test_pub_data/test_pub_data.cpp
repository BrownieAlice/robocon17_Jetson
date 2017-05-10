/*
======================================================================
Project Name    : test pubulish data
File Name       : test_pub_data
Encoding        : UTF-8
Creation Date   : 2017/02/27

Copyright © 2017 Alice.
======================================================================
*/

#include "ros/ros.h"
#include "detect_circle/Joutput.h"
#include "detect_circle/Jline.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pub_data");
  ros::NodeHandle n;
  ros::Publisher Jdata_pub = n.advertise<detect_circle::Joutput>("Jdata", 1);
  ros::Publisher Jline_pub = n.advertise<detect_circle::Jline>("Jline", 1);
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    static float x = 0, theta = 0;

    detect_circle::Joutput msg1;
    detect_circle::Jline msg2;

    ROS_INFO("pub test data");

    msg1.MB_pole = 1;
    msg1.x = ++x;
    msg1.y = -30;
    msg1.stamp = ros::Time::now();
    ROS_INFO("Jdata::MB_pole:%d,x:%f,y:%f", msg1.MB_pole, msg1.x, msg1.y);

    Jdata_pub.publish(msg1);

    msg2.theta = ++theta;
    msg2.stamp = ros::Time::now();
    ROS_INFO("Jline::theta:%f", msg2.theta);

    Jline_pub.publish(msg2);


    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
