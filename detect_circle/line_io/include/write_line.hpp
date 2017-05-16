#ifndef INCLUDED_WRITE_LINE
#define INCLUDED_WRITE_LINE

#include "ros/ros.h"

void write_line(const double x1, const double y1, const double x2, const double y2, const int n, const ros::Publisher &marker_pub, const ros::Duration &lifetime);

#endif
