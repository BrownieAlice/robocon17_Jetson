#ifndef INCLUDED_WRITE_CERCLE
#define INCLUDED_WRITE_CERCLE

#include "ros/ros.h"

void write_cercle(const double x, const double y, const double rad, ros::Publisher &marker_pub);

#endif
