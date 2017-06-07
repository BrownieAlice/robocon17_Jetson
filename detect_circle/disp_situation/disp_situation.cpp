/*
======================================================================
Project Name    : disp situatuon
File Name       : disp_situation
Encoding        : UTF-8
Creation Date   : 2017/06/07

Copyright © 2017 Alice.
======================================================================
*/

#include <boost/format.hpp>
#include "ros/ros.h"
#include "detect_circle/Jcircle.h"
#include "detect_circle/Jline.h"
#include "detect_circle/MBinput.h"
#include "sensor_msgs/LaserScan.h"
#include "disp_situation_p.hpp"

namespace
{
  namespace param
  {
    constexpr int loop_hz = 5;
  }
  namespace var
  {
    bool ethLRF_flag = false;
    bool usbLRF_flag = false;
    bool MBdata_flag = false;
    bool Jcircle_flag = false;
    bool Jline_flag = false;
  }
} // namespace.

int main(int argc, char **argv)
{
  ros::init(argc, argv, "disp_situation");
  ros::NodeHandle n;

  ros::Subscriber sub_ethLRF = n.subscribe("scan_ethLRF", 1, ethLRF_callback);
  ros::Subscriber sub_usbLRF = n.subscribe("scan_usbLRF", 1, usbLRF_callback);
  ros::Subscriber sub_MBdata = n.subscribe("MBdata", 1, MBdata_callback);
  ros::Subscriber sub_Jcircle = n.subscribe("Jcircle", 1, Jcircle_callback);
  ros::Subscriber sub_Jdata = n.subscribe("Jline", 1, Jline_callback);

  ros::Rate loop_rate(param::loop_hz);

  while (ros::ok())
  {
    std::cout << boost::format("\x1b[33m" "********\n" "\x1b[39m");
    write_situation("connect ethLRF", var::ethLRF_flag);
    write_situation("connect usbLRF", var::usbLRF_flag);
    write_situation("connect MB", var::MBdata_flag);
    write_situation("find circle", var::Jcircle_flag);
    write_situation("find line", var::Jline_flag);
    std::cout << boost::format("\x1b[33m" "********\n" "\x1b[39m");

    var::ethLRF_flag = var::usbLRF_flag = var::MBdata_flag = var::Jcircle_flag = var::Jline_flag = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

static void ethLRF_callback(const sensor_msgs::LaserScan& msg)
{
  var::ethLRF_flag = true;
  return;
}

static void usbLRF_callback(const sensor_msgs::LaserScan& msg)
{
  var::usbLRF_flag = true;
  return;
}

static void MBdata_callback(const detect_circle::MBinput& msg)
{
  var::MBdata_flag = false;
  return;
}

static void Jcircle_callback(const detect_circle::Jcircle& Jcircle)
{
  var::Jcircle_flag = true;
  return;
}

static void Jline_callback(const detect_circle::Jline& Jline)
{
  var::Jline_flag = true;
  return;
}

static void write_situation(const std::string name, const bool flag)
{
  if(flag)
  {
    std::cout << boost::format("\x1b[36m" "success to %s\n" "\x1b[39m") % name;
  }
  else
  {
    std::cout << boost::format("\x1b[31m" "fail to %s\n" "\x1b[39m") % name;
  }
  return;
}
