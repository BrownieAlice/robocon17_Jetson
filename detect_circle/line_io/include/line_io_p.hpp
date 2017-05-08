#ifndef INCLUDED_LINE_IO_P
#define INCLUDED_LINE_IO_P

#include "sensor_msgs/LaserScan.h"
#include "detect_circle/MBinput.h"

static void MB_Callback(const detect_circle::MBinput& msg);
static void Laser_Callback(const sensor_msgs::LaserScan& msg);
static int lsd_detect(const sensor_msgs::LaserScan& msg, float *robot_theta_return, float *line_distance, double origin_x, double origin_y);

#ifndef M_PI
#define M_PI 3.1415
#endif

#endif
