#ifndef INCLUDED_CIRCLE_IO_P
#define INCLUDED_CIRCLE_IO_P

#include <Eigen/Dense>
#include "detect_circle/MBinput.h"
#include "sensor_msgs/LaserScan.h"

static Eigen::Matrix4d RotMatrix(double theta);
static Eigen::Matrix4d TransMatrix(double x, double y);
static void calc_matrix(Eigen::Matrix4d *AbsToLRF, const float x, const float y, const float theta, const float LRF_diff_x, const float LRF_diff_y, const float LRF_diff_float);
static void Sub_Callback(const detect_circle::MBinput& msg);
static void Laser_Callback(const sensor_msgs::LaserScan& msg);
static int isInSigma(const double calc_x, const double calc_y, const double x, const double y, const double x_sigma, const double y_sigma);
static int SearchPole(const sensor_msgs::LaserScan& msg, const int watch_pole_num);
double calc_thr2(double pole_dis);

#ifndef M_PI
#define M_PI 3.1415
#endif

#endif
