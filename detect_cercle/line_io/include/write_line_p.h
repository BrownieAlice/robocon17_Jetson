#ifndef WRITE_LINE_TO_RVIZ_PRIVATE
#define WRITE_LINE_TO_RVIZ_PRIVATE

#include "ros/ros.h"

void write_line(const double x1, const double y1, const double x2, const double y2, int n, ros::Publisher &marker_pub,const ros::Duration &lifetime);
static void init_publish();

#endif
