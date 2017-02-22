#ifndef CERCLE_IO_DETECT_CERCLE
#define CERCLE_IO_DETECT_CERCLE

static Eigen::Matrix4d RotMatrix(double theta);
static Eigen::Matrix4d TransMatrix(double x, double y);
static void calc_matrix(Eigen::Matrix4d &AbsToLRF);
static void Sub_Callback(const detect_cercle::MBinput& msg);
static void Laser_Callback(const sensor_msgs::LaserScan& msg);

#ifndef M_PI
#define M_PI 3.1415
#endif

#endif