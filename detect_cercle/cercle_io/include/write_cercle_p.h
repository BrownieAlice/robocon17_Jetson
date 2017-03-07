#ifndef WRITE_CERCLE_TO_RVIZ_PRIVATE
#define WRITE_CERCLE_TO_RVIZ_PRIVATE

#include "ros/ros.h"

void write_cercle(const double x, const double y, const double rad, ros::Publisher &marker_pub);
static void init_publish();

#endif
