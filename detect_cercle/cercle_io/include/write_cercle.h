#ifndef WRITE_CERCLE_TO_RVIZ
#define WRITE_CERCLE_TO_RVIZ

#include "ros/ros.h"

void write_cercle(const double x, const double y, const double rad, ros::Publisher &marker_pub);

#endif
