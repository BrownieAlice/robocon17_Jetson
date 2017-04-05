#include <math.h>
#include <iostream>
#include "cv.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "detect_cercle/Jtheta.h"
#include "visualization_msgs/Marker.h"
#include "../include/write_line.h"
#include "../include/line_io_p.h"
#include "../lib/lsd.h"

namespace
{
  namespace param
  {
    constexpr int height = 256;
    constexpr int width = 512;
    // 距離データを画像化する際の縦と横のピクセル数.

    constexpr double x_min = 0;
    constexpr double x_wid = 1.5f / height;
    constexpr double x_max = x_min + x_wid * height;
    // x方向(height)の1ピクセルあたりの幅と,画像全体での最大値/最小値.

    constexpr double y_min = -3;
    constexpr double y_wid = 6.0f / width;
    constexpr double y_max = y_min + y_wid * width;
    // y方向(width)の1ピクセルあたりの幅と,画像全体での最大値/最小値.

    constexpr int pix_reso = 1;
    constexpr double angle_reso = M_PI / 180 / 2;
    // ピクセル解像度と角度解像度[deg].

    constexpr int thr = 70;
    // 閾値.

    constexpr int thr_len = 350;
    // 線分と検出する際の線分の最小長.

    constexpr int thr_inter = 90;
    // 同一線分とみなす際の線分の間隔.

    constexpr double err_theta = 10.0f / 180 * M_PI;
    // この範囲の検出角度のみを姿勢各候補としてカウントする.

    constexpr int dim = 5;
    // 検出する直線数.

    constexpr double min_len = 5;
    // 最小の直線の長さ.
  }

  namespace var
  {
    IplImage *img;
    CvMemStorage *storage;
    CvSeq *lines = 0;
    ros::Publisher marker_pub;
    ros::Publisher Jtheta_pub;
    detect_cercle::Jtheta msg;
  }

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_io");
  ros::NodeHandle n;
  ros::Subscriber laser = n.subscribe("scan_usbLRF",1,Laser_Callback);
  var::Jtheta_pub = n.advertise<detect_cercle::Jtheta>("Jtheta", 1);
  var::marker_pub = n.advertise<visualization_msgs::Marker>("write_line", 1);
  ros::Rate loop_rate(4);

  var::img = cvCreateImage(cvSize(param::width, param::height), 8, 1 );
  var::storage = cvCreateMemStorage (0);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  cvReleaseImage(&var::img);
  cvReleaseMemStorage (&var::storage);
  return 0;
}

void Laser_Callback(const sensor_msgs::LaserScan& msg)
{
  ros::Time begin = ros::Time::now();

  lsd_detect(msg);

  ros::Time end = ros::Time::now();

  std::cout <<  "time:" << (end - begin) * 1000 << "[ms]" << std::endl;
  return;
}

void lsd_detect(const sensor_msgs::LaserScan& msg)
{
  image_double lsdImage = new_image_double_ini(param::width, param::height, 0);
  ntuple_list lineSeg = new_ntuple_list(param::dim);

  int step = param::width;
  double range_min = msg.range_min;
  double angle_min = msg.angle_min;
  double angle_increment = msg.angle_increment;
  size_t size = msg.ranges.size();
  for (size_t i = 0; i < size ; i++)
  {
    double range = msg.ranges[i];
    if(range < range_min)
    {
      continue;
      // 最低距離より短かったら無視
    }
    double angle = angle_min + angle_increment * i;

    double x = range * cos(angle);
    double y = range * sin(angle);

    if (x < param::x_min || param::x_max < x || y < param::y_min || param::y_max < y)
    {
      continue;
      // 範囲外だった.
    }

    int height = static_cast<int>((x - param::x_min) / param::x_wid);
    int width = static_cast<int>((y - param::y_min) / param::y_wid);

    if (param::height <= height || param::width < width || height < 0 || width < 0)
    {
      continue;
      // 配列外参照.
    }
    lsdImage->data[step * height + width] += static_cast<double>(msg.intensities[i]);
  }

  lineSeg = LineSegmentDetection(lsdImage, 0.4, 1.0, 1.0, 22.5, 0.0, 0.7, 1024, 255.0, NULL);

  int theta_count = 0;
  double len = 0;
  double robot_theta;

  for(unsigned int i = 0; i < lineSeg->size; i++)
  {
    double x0 = lineSeg->values[1 + param::dim * i] * param::x_wid + param::x_min;
    double y0 = lineSeg->values[0 + param::dim * i] * param::y_wid + param::y_min;
    double x1 = lineSeg->values[3 + param::dim * i] * param::x_wid + param::x_min;
    double y1 = lineSeg->values[2 + param::dim * i] * param::y_wid + param::y_min;
    write_line(x0, y0, x1, y1, i, var::marker_pub);
    std::cout << "x0:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << x0 << "[m]" << ", ";
    std::cout << "y0:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << y0 << "[m]" << ", ";
    std::cout << "x1:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << x1 << "[m]" << ", ";
    std::cout << "y1:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << y1 << "[m]" << ", ";

    double theta = atan2(y1 - y0, x1 - x0)- M_PI / 2;
    if( theta < -M_PI/2 )
    {
      theta += M_PI;
    }

    double this_len = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0);

    std::cout << "theta:" << std::fixed << std::setprecision(1) << std::setfill(' ') << std::setw(4) << std::right << theta * 180 / M_PI << "[deg]" << std::endl;

    if ( len < this_len )
    {
      len = this_len;
      robot_theta = theta;
      theta_count++;
    }
  }

  if (theta_count != 0)
  {
    if ( len < param::min_len )
    {
      // 直線が短すぎる.
      return;
    }
    std::cout << "len:" << len << "[m^2]" << std::endl;

    std::cout << "robot_theta:" << std::fixed << std::setprecision(1) << std::setfill(' ') << std::setw(4) << std::right << robot_theta * 180 / M_PI << "[deg]" << std::endl;

    var::msg.theta = static_cast<float>(robot_theta);
    var::msg.stamp = ros::Time::now();
    var::Jtheta_pub.publish(var::msg);
  }
}

void CV_hough(const sensor_msgs::LaserScan& msg)
{
  cvZero(var::img);

  int step = var::img->widthStep;
  double range_min = msg.range_min;
  double angle_min = msg.angle_min;
  double angle_increment = msg.angle_increment;
  size_t size = msg.ranges.size();
  for (size_t i = 0; i < size ; i++)
  {
    double range = msg.ranges[i];
    if(range < range_min)
    {
      continue;
      // 最低距離より短かったら無視
    }
    double angle = angle_min + angle_increment * i;

    double x = range * cos(angle);
    double y = range * sin(angle);

    if (x < param::x_min || param::x_max < x || y < param::y_min || param::y_max < y)
    {
      continue;
      // 範囲外だった.
    }

    int height = cvRound((x - param::x_min) / param::x_wid);
    int width = cvRound((y - param::y_min) / param::y_wid);

    if (param::height <= height || param::width < width || height < 0 || width < 0)
    {
      continue;
      // 配列外参照.
    }
    var::img->imageData[step * height + width] = 128;
  }

  var::lines = cvHoughLines2 (var::img, var::storage, CV_HOUGH_PROBABILISTIC, param::pix_reso, param::angle_reso, param::thr, param::thr_len, param::thr_inter);
  // ハフ変換

  int max = var::lines->total;
  std::cout << "num:" << max << std::endl;;

  double robot_theta_sum = 0;
  int theta_count = 0;

  for (int i = 0; i < max; i++)
  {
    CvPoint* line = (CvPoint*)cvGetSeqElem(var::lines,i);
    double x0 = line[0].y * param::x_wid + param::x_min;
    double y0 = line[0].x * param::y_wid + param::y_min;
    double x1 = line[1].y * param::x_wid + param::x_min;
    double y1 = line[1].x * param::y_wid + param::y_min;
    write_line(x0, y0, x1, y1, i, var::marker_pub);
    std::cout << "x0:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << x0 << "[m]" << ", ";
    std::cout << "y0:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << y0 << "[m]" << ", ";
    std::cout << "x1:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << x1 << "[m]" << ", ";
    std::cout << "y1:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << y1 << "[m]" << ", ";

    double theta = atan2(y1 - y0, x1 - x0)- M_PI / 2;

    std::cout << "theta:" << std::fixed << std::setprecision(1) << std::setfill(' ') << std::setw(4) << std::right << theta * 180 / M_PI << "[deg]" << std::endl;

    if (-param::err_theta < theta && theta < param::err_theta)
    {
      // 角度が範囲内.
      robot_theta_sum += -theta;
      theta_count++;
    }

  }

  if (theta_count != 0)
  {
    float robot_theta = robot_theta_sum / theta_count;
    std::cout << "robot_theta:" << std::fixed << std::setprecision(1) << std::setfill(' ') << std::setw(4) << std::right << robot_theta * 180 / M_PI << "[deg]" << std::endl;

    var::msg.theta = static_cast<float>(robot_theta);
    var::msg.stamp = ros::Time::now();
    var::Jtheta_pub.publish(var::msg);
  }
}
