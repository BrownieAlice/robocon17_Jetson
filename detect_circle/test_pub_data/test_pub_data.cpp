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
#include "detect_circle/Jcircle.h"
#include "detect_circle/Jline.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pub_data");
  ros::NodeHandle n;
  ros::Publisher Jcircle_pub = n.advertise<detect_circle::Jcircle>("Jcircle", 1);
  ros::Publisher Jline_pub = n.advertise<detect_circle::Jline>("Jline", 1);
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    static double x = 0, theta = 0;

    x++;
    theta++;
    if (x < 1000)
    {
      x = -1000;
    }
    if (theta < 1000)
    {
      theta = -1000;
    }

    detect_circle::Jcircle msg1;
    detect_circle::Jline msg2;

    ROS_INFO("pub test data");

    msg1.MB_pole = 1;
    msg1.x = x;
    msg1.y = -30;
    msg1.x_sigma = 0.01;
    msg1.y_sigma = 0.01;
    msg1.stamp = ros::Time::now();
    ROS_INFO("Jcircle::MB_pole:%d,x:%f,y:%f,x_sigma:%f,y_sigma:%f", msg1.MB_pole, msg1.x, msg1.y, msg1.x_sigma, msg1.y_sigma);

    Jcircle_pub.publish(msg1);

    msg2.theta = theta;
    msg2.line_distance = 1.0;
    msg2.sigma = 0.001;
    msg2.stamp = ros::Time::now();
    ROS_INFO("Jline::theta:%f,line_distance:%f,sigma:%f", msg2.theta, msg2.line_distance, msg2.sigma);

    Jline_pub.publish(msg2);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
