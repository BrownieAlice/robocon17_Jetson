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
#include "detect_cercle/Joutput.h"
#include "detect_cercle/Jtheta.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pub_data");
  ros::NodeHandle n;
  ros::Publisher Jdata_pub = n.advertise<detect_cercle::Joutput>("Jdata", 1);
  ros::Publisher Jtheta_pub = n.advertise<detect_cercle::Jtheta>("Jtheta", 1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    detect_cercle::Joutput msg1;
    detect_cercle::Jtheta msg2;

    ROS_INFO("pub test data");

    msg1.MB_pole = 1;
    msg1.x = 30;
    msg1.y = -30;
    msg1.stamp = ros::Time::now();
    ROS_INFO("Jdata::MB_pole:%d,x:%f,y:%f", msg1.MB_pole, msg1.x, msg1.y);

    Jdata_pub.publish(msg1);

    msg2.theta = 90.0f / 180 * 3.1415;
    msg2.stamp = ros::Time::now();
    ROS_INFO("Jtheta::theta:%f", msg2.theta);

    Jtheta_pub.publish(msg2);


    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
