#ifndef INCLUDED_CIRCLE_IO_P
#define INCLUDED_CIRCLE_IO_P

#include <Eigen/Dense>
#include "detect_circle/MBinput.h"
#include "sensor_msgs/LaserScan.h"

static Eigen::Matrix4d RotMatrix(double theta);
static Eigen::Matrix4d TransMatrix(double x, double y);
static void calc_matrix(Eigen::Matrix4d *AbsToLRF, const float x, const float y, const float theta, const float LRF_diff_x, const float LRF_diff_y);
static void Sub_Callback(const detect_circle::MBinput& msg);
static void Laser_Callback(const sensor_msgs::LaserScan& msg);
int isInSigma(double calc_x, double calc_y, double x, double y, double x_sigma, double y_sigma);

#ifndef M_PI
#define M_PI 3.1415
#endif

#endif
