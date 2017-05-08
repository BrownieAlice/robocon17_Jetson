#ifndef INCLUDED_WRITE_CIRCLE
#define INCLUDED_WRITE_CIRCLE

#include "ros/ros.h"

void write_circle(const double x, const double y, const double rad, ros::Publisher &marker_pub);

#endif
