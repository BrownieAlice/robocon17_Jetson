#ifndef INCLUDED_LINE_IO_P
#define INCLUDED_LINE_IO_P

#include <boost/optional.hpp>
#include "sensor_msgs/LaserScan.h"
#include "detect_circle/MBinput.h"
#include "ros/ros.h"
extern "C"
{
  #include "../lib/lsd.h"
}

typedef struct {
  double theta;
  double line_distance;
} line_detect;
// ロボットの姿勢角と中心と直線との距離.

typedef struct{
  double MB_theta;
  double MB_sigma;
  char field_color;
  ros::Time stamp;
} MB_info;
// MBからの姿勢角とその標準偏差, フィールド色, タイムスタンプの情報.

typedef struct {
  int height;
  int width;
  // 画像の縦横の画素.
  double x_min;
  double x_wid;
  double x_max;
  // 現実の距離を画像に変換した時のxの最小値, 1pixelあたりの距離, 最大値.
  double y_min;
  double y_wid;
  double y_max;
  // 現実の距離を画像に変換した時のyの最小値, 1pixelあたりの距離, 最大値.
} LRF_image;

typedef struct{
  double x0;
  double y0;
  double x1;
  double y1;
  double length;
  bool inside_line;
} line_position;
// 線分の2点と長さの情報.

static void MB_Callback(const detect_circle::MBinput &msg);
static void Laser_Callback(const sensor_msgs::LaserScan &msg);
static boost::optional<line_position> lsd_detect(const sensor_msgs::LaserScan& msg, const LRF_image &LRF_image_data, const int dim, const ros::Publisher &marker_pub, const ros::Duration &lifetime);
static void convert_position_data(const sensor_msgs::LaserScan &msg, image_double lsdImage, const LRF_image &LRF_image_data);
static boost::optional<line_position> search_lonngest_line(const ntuple_list lineSeg, const LRF_image &LRF_image_data, const int dim, const ros::Publisher &marker_pub, const ros::Duration &lifetime);
static boost::optional<line_position> search_2nd_lonngest_line(const ntuple_list lineSeg, const LRF_image &LRF_image_data, const int dim);
static boost::optional<line_detect> convert_line_data(const boost::optional<line_position> &line_position_data, const double origin_x, const double origin_y, const double min_len, const double offset);
static boost::optional<line_position> check_inside_line(const boost::optional<line_position> &line_position_data, const sensor_msgs::LaserScan &msg, const size_t count);
static int judge_intersected(const double ax, const double ay, const double bx, const double by, const double cx, const double cy, const double dx, const double dy);
static boost::optional<line_detect> modify_for_field(boost::optional<line_detect> line_detect_data, const char field_color);
static boost::optional<line_detect> sigma_check(const boost::optional<line_detect> &line_detect_data, const double MB_theta, const double MB_sigma);
static void publish_line_detect(const boost::optional<line_detect> &line_detect_data, const double J_sigma, detect_circle::Jline *msg, const ros::Publisher &Jline_pub);

#ifndef M_PI
#define M_PI 3.1415
#endif

#endif
