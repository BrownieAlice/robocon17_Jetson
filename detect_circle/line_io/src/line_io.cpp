 #define LINE_IO_DEBUG_MODE

#include <math.h>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "detect_circle/Jline.h"
#include "detect_circle/MBinput.h"
#include "visualization_msgs/Marker.h"
#include "../include/write_line.hpp"
#include "../include/line_io_p.hpp"

extern "C"
{
  #include "../lib/lsd.h"
}

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

    constexpr int dim = 5;
    // 検出する直線数.

    constexpr double min_len = 1.5;
    constexpr double min_len_sq = min_len * min_len;
    // 最小の直線の長さ.

    constexpr int loop_hz = 4;
    // 実行する周波数.

    const ros::Duration MBdata_late(500e-3);
    // 許容する遅れ時間.

    const ros::Duration lifetime(1/static_cast<double>(loop_hz));
    // 許容する遅れ時間.

    constexpr double lrf_diff_x = 0.4425;
    constexpr double lrf_diff_y = 0.4698;
    // LRF座標系での, 中心からみたLRFの位置.

    constexpr double sigma = (double)1.0/3*180/M_PI;
  }

  namespace var
  {
    ros::Publisher marker_pub;
    ros::Publisher Jline_pub;
    detect_circle::Jline msg;
    // ROSノード用の変数.

    bool write_position = false;
    double theta, MB_sigma;
    char color;
    ros::Time stamp;
  }

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_io");
  ros::NodeHandle n;
  ros::Subscriber MB_input = n.subscribe("MBdata", 1, MB_Callback);
  ros::Subscriber laser = n.subscribe("scan_usbLRF",1,Laser_Callback);
  var::Jline_pub = n.advertise<detect_circle::Jline>("Jline", 1);
  var::marker_pub = n.advertise<visualization_msgs::Marker>("write_line", 1);
  ros::Rate loop_rate(param::loop_hz);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

static void MB_Callback(const detect_circle::MBinput& msg)
{
  // uartにより発行されたトピックの購読.
  var::write_position = true;
  var::theta = msg.theta;
  var::stamp = msg.stamp;
  var::MB_sigma = msg.theta_sigma;
  var::color = (char)msg.color;
}

static void Laser_Callback(const sensor_msgs::LaserScan& msg)
{
  ros::Time begin = ros::Time::now();

  if (false == var::write_position)
  {
    return;
  }
  var::write_position = false;

  #ifndef LINE_IO_DEBUG_MODE
  const ros::Duration diff = ros::Time::now() - var::stamp;
  if (param::MBdata_late < diff)
  {
    // 位置情報が指定時間よりも古い情報なら処理しない.
    return;
  }
  #endif

  double robot_theta;
  // ロボットの姿勢角.
  double line_distance;
  // 木枠との距離.
  int success = lsd_detect(msg, &robot_theta, &line_distance, -param::lrf_diff_x, -param::lrf_diff_y);

  ros::Time end = ros::Time::now();

  if ( 0 == success )
  {
    // 直線検出に成功してる.
    if (var::color == 'B')
    {
      // 青フィールド時.
    }
    else if (var::color == 'R')
    {
      // 赤フィールド時.
      robot_theta += M_PI;
    }
    else
    {
      // それ以外の文字.
      return;
    }

    if (robot_theta < var::theta - var::MB_sigma * 3 || var::theta + var::MB_sigma * 3 < robot_theta)
    {
      // 送られた姿勢角よりあきらかにずれが大きい.
      return;
    }

    std::cout << "robot_theta:" << std::fixed << std::setprecision(1) << std::setfill(' ') << std::setw(4) << std::right << robot_theta * 180 / M_PI << "[deg]" << std::endl;

    var::msg.theta = robot_theta;
    var::msg.line_distance = line_distance;
    var::msg.sigma = param::sigma;
    var::msg.stamp = ros::Time::now();
    var::Jline_pub.publish(var::msg);
    // 発行.
  }

  std::cout <<  "time:" << (end - begin) * 1000 << "[ms]" << std::endl;
  return;
}

static int lsd_detect(const sensor_msgs::LaserScan& msg, double *robot_theta_return, double *line_distance, const double origin_x, const double origin_y)
{
  // LSDを用いた直線検出.

  int success = -1;
  // 成功したかどうかの戻り値.

  image_double lsdImage = new_image_double_ini(param::width, param::height, 0);
  ntuple_list lineSeg = new_ntuple_list(param::dim);
  // lsd用の構造体を作る.

  int step = param::width;
  double range_min = msg.range_min;
  double angle_min = msg.angle_min;
  double angle_increment = msg.angle_increment;
  size_t size = msg.ranges.size();
  // センサの各種情報を取得.

  for (size_t i = 0; i < size ; i++)
  {
    double range = msg.ranges[i];
    // 距離情報.

    if(range < range_min)
    {
      continue;
      // 最低距離より短かったら無視.
    }
    double angle = angle_min + angle_increment * i;
    // 角度情報.

    double x = range * cos(angle);
    double y = range * sin(angle);
    // x,y平面での値.

    if (x < param::x_min || param::x_max < x || y < param::y_min || param::y_max < y)
    {
      continue;
      // 範囲外だった.
    }

    int height = static_cast<int>((x - param::x_min) / param::x_wid);
    int width = static_cast<int>((y - param::y_min) / param::y_wid);
    // 離散化したときのheight,widthの値.

    if (param::height <= height || param::width < width || height < 0 || width < 0)
    {
      continue;
      // 配列外参照.
    }

    lsdImage->data[step * height + width] += static_cast<double>(msg.intensities[i]);
    // lsd用の構造体に情報を与える.
  }

  lineSeg = LineSegmentDetection(lsdImage, 0.4, 1.0, 1.0, 22.5, 0.0, 0.7, 1024, 255.0, NULL);
  // 線分検出.

  int theta_count = 0;
  // 何回角度情報があったか.
  double len = 0;
  // 今まで最も長かった線分の長さ.
  double robot_theta = 0;
  // 推定したもっともらしい姿勢角情報.
  double line_x0, line_y0, line_x1, line_y1;
  // もっとも長い線分の2点の情報を格納.

  for(unsigned int i = 0; i < lineSeg->size; i++)
  {
    double x0 = lineSeg->values[1 + param::dim * i] * param::x_wid + param::x_min;
    double y0 = lineSeg->values[0 + param::dim * i] * param::y_wid + param::y_min;
    double x1 = lineSeg->values[3 + param::dim * i] * param::x_wid + param::x_min;
    double y1 = lineSeg->values[2 + param::dim * i] * param::y_wid + param::y_min;
    // 線分情報(ホントの座標系に変換してもいる).

    write_line(x0, y0, x1, y1, i, var::marker_pub, param::lifetime);
    // rviz上に線分を表示.

    std::cout << "x0:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << x0 << "[m]" << ", ";
    std::cout << "y0:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << y0 << "[m]" << ", ";
    std::cout << "x1:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << x1 << "[m]" << ", ";
    std::cout << "y1:" << std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(6) << std::right << y1 << "[m]" << ", ";
    // 線分情報を標準入出力に出力.

    double theta = atan2(y1 - y0, x1 - x0)- M_PI / 2;
    // 角度を計算.
    if( theta < -M_PI/2 )
    {
      theta += M_PI;
    }
    // 角度を -PI/2以上PI/2以下にする.

    double this_len = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0);
    // この線分の長さ(2乗値).

    std::cout << "theta:" << std::fixed << std::setprecision(1) << std::setfill(' ') << std::setw(5) << std::right << theta * 180 / M_PI << "[deg]" << std::endl;

    if ( len < this_len )
    {
      // もっとも長い線分ならロボットの姿勢角情報とみなす.
      len = this_len;
      robot_theta = -theta;
      theta_count++;
      line_x0 = x0;
      line_y0 = y0;
      line_x1 = x1;
      line_y1 = y1;
    }
  }

  if (theta_count != 0)
  {
    // 角度を計算できた.
    if ( param::min_len_sq < len )
    {
      // 直線が指定以上の長さがある.

      std::cout << "len:" << len << "[m^2]" << std::endl;

      double rel_line_x0 = line_x0 - origin_x;
      double rel_line_y0 = line_y0 - origin_y;
      double rel_line_x1 = line_x1 - origin_x;
      double rel_line_y1 = line_y1 - origin_y;
      // 機体中心から見た直線の点の位置.

      *line_distance = fabs(rel_line_x0 * rel_line_y1 - rel_line_x1 * rel_line_y0) / sqrt(len);
      std::cout << "distance to line:" << *line_distance << "[m]" << std::endl;
      *robot_theta_return = robot_theta;
      success = 0;
    }
  }

  free_image_double(lsdImage);
  free_ntuple_list(lineSeg);
  // メモリ解放.

  return success;
}
