#ifndef LINE_IO_DETECT_CERCLE
#define LINE_IO_DETECT_CERCLE

#include "sensor_msgs/LaserScan.h"
static void Laser_Callback(const sensor_msgs::LaserScan& msg);
void lsd_detect(const sensor_msgs::LaserScan& msg);
void CV_hough(const sensor_msgs::LaserScan& msg);

#ifndef M_PI
#define M_PI 3.1415
#endif

#endif
