#include <math.h>
#include <iostream>
#include <boost/optional.hpp>
#include <boost/format.hpp>
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


    constexpr LRF_image LRF_image_data = {
      height, width, x_min, x_wid, x_max, y_min, y_wid, y_max
    };
    // LRFを画像データの情報.

    constexpr int dim = 5;
    // 検出する直線数.

    constexpr double min_len = 1.5;
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

    constexpr double sigma = (double)1.0 / 3 / 180 * M_PI;
    // 線分検出による姿勢角の標準偏差.

    constexpr double offset = (double)1.2 / 180 * M_PI;
    // LRFの設置誤差によるオフセット.
  }

  namespace var
  {
    ros::Publisher marker_pub;
    ros::Publisher Jline_pub;
    detect_circle::Jline msg;
    // ROSノード用の変数.

    boost::optional<MB_info> MB_info_datas;
    // MBからの情報を格納.
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

static void MB_Callback(const detect_circle::MBinput &msg)
{
  // uartにより発行されたトピックの購読.
  MB_info tmp_data = {
    msg.theta, msg.theta_sigma, (char)msg.color, msg.stamp
  };
  var::MB_info_datas = tmp_data;
}

static void Laser_Callback(const sensor_msgs::LaserScan &msg)
{
  ros::Time begin = ros::Time::now();

  if (!var::MB_info_datas)
  {
    // MBからの情報がなかったら計算しない.
    return;
  }

  #ifndef LINE_IO_DEBUG_MODE
  const ros::Duration diff = ros::Time::now() - var::MB_info_datas->stamp;
  if (param::MBdata_late < diff)
  {
    // 位置情報が指定時間よりも古い情報なら処理しない.
    return;
  }
  #endif

  const boost::optional<line_position> line_position_data = lsd_detect(msg, param::LRF_image_data, param::dim, var::marker_pub, param::lifetime);
  // 線分情報の取得.

  boost::optional<line_detect> line_detect_data = convert_line_data(line_position_data, -param::lrf_diff_x, -param::lrf_diff_y, param::min_len, param::offset);
  // ロボットの姿勢角と中心と直線との距離を計算.

  line_detect_data = modify_for_field(line_detect_data, var::MB_info_datas->field_color);
  // フィールドに合わせて修正.

  line_detect_data = sigma_check(line_detect_data, var::MB_info_datas->MB_theta, var::MB_info_datas->MB_sigma);
  // 3sigmaの範囲内にあるかを確認.

  ros::Time end = ros::Time::now();

  publish_line_detect(line_detect_data, param::sigma, &var::msg, var::Jline_pub);
  // 発行.

  std::cout << boost::format("time:%5.2f[ms]") %  ((end - begin).toSec() * 1000) << std::endl;

  var::MB_info_datas = boost::none;
  return;
}

static boost::optional<line_position> lsd_detect(const sensor_msgs::LaserScan &msg, const LRF_image &LRF_image_data, const int dim, const ros::Publisher &marker_pub, const ros::Duration &lifetime)
{
  // LSDを用いた1個の線分検出.

  image_double lsdImage = new_image_double_ini(LRF_image_data.width, LRF_image_data.height, 0);
  ntuple_list lineSeg = new_ntuple_list(dim);
  // lsd用の構造体を作る.

  convert_position_data(msg, lsdImage, LRF_image_data);
  // LRFデータをx-yデータに変換.

  lineSeg = LineSegmentDetection(lsdImage, 0.5, 1.2, 1.0, 22.5, 0.0, 0.7, 1024, 255.0, NULL);
  // 線分検出.

  boost::optional<line_position> line_position_data = search_lonngest_line(lineSeg, LRF_image_data, dim, marker_pub, lifetime);
  // 最大の長さの線分を検出.

  free_image_double(lsdImage);
  free_ntuple_list(lineSeg);
  // メモリ解放.

  return line_position_data;
}

static void convert_position_data(const sensor_msgs::LaserScan& msg, image_double lsdImage, const LRF_image &LRF_image_data)
{
  // LRFの位置情報からx-yの情報に変換.

  const int step = LRF_image_data.width;
  const double range_min = msg.range_min;
  const double angle_min = msg.angle_min;
  const double angle_increment = msg.angle_increment;
  const size_t size = msg.ranges.size();
  // センサの各種情報を取得.

  for (size_t i = 0; i < size ; i++)
  {
    const double range = msg.ranges[i];
    // 距離情報.

    if(range < range_min)
    {
      continue;
      // 最低距離より短かったら無視.
    }
    const double angle = angle_min + angle_increment * i;
    // 角度情報.

    const double x = range * cos(angle);
    const double y = range * sin(angle);
    // x,y平面での値.

    if (x < LRF_image_data.x_min || LRF_image_data.x_max < x || y < LRF_image_data.y_min || LRF_image_data.y_max < y)
    {
      continue;
      // 範囲外だった.
    }

    const int height = static_cast<int>((x - LRF_image_data.x_min) / LRF_image_data.x_wid);
    const int width = static_cast<int>((y - LRF_image_data.y_min) / LRF_image_data.y_wid);
    // 離散化したときのheight,widthの値.

    if (LRF_image_data.height <= height || LRF_image_data.width <= width || height < 0 || width < 0)
    {
      continue;
      // 配列外参照.
    }

    lsdImage->data[step * height + width] += static_cast<double>(msg.intensities[i]);
    // lsd用の構造体に情報を与える.
  }

  return;
}

static boost::optional<line_position> search_lonngest_line(const ntuple_list lineSeg, const LRF_image &LRF_image_data, const int dim, const ros::Publisher &marker_pub, const ros::Duration &lifetime)
{
  // 線分リストから最大の長さの線分を取得する.

  boost::optional<line_position> line_position_data;
  // 線分情報の格納.

  for (unsigned int i = 0; i < lineSeg->size; i++)
  {
    const double x0 = lineSeg->values[1 + dim * i] * LRF_image_data.x_wid + LRF_image_data.x_min;
    const double y0 = lineSeg->values[0 + dim * i] * LRF_image_data.y_wid + LRF_image_data.y_min;
    const double x1 = lineSeg->values[3 + dim * i] * LRF_image_data.x_wid + LRF_image_data.x_min;
    const double y1 = lineSeg->values[2 + dim * i] * LRF_image_data.y_wid + LRF_image_data.y_min;
    // 線分情報(ホントの座標系に変換してもいる).

    write_line(x0, y0, x1, y1, i, marker_pub, lifetime);
    // rviz上に線分を表示.

    const double len = sqrt( (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) );
    // この線分の長さ.

    std::cout << boost::format("x0:%6.3f[m], y0:%6.3f[m], x1:%6.3f[m], y1:%6.3f[m], len:%5.4f[m]") % x0 % y0 % x1 % y1 % len << std::endl;
    // 線分情報を標準入出力に出力.

    if ( !line_position_data || line_position_data->length < len )
    {
      // 直線情報の更新.

      const line_position tmp_data = {x0, y0, x1, y1, len};
      line_position_data = tmp_data;
    }
  }

  return line_position_data;
}

static boost::optional<line_detect> convert_line_data(const boost::optional<line_position> &line_position_data, const double origin_x, const double origin_y, const double min_len, const double offset)
{
  // 線分情報を角度と距離の情報に変換.

  boost::optional<line_detect> line_detect_data;
  // 直線情報の格納.

  if ( line_position_data )
  {
    // 線分情報が存在する.

    if ( min_len < line_position_data->length )
    {
      // 直線が指定以上の長さがある.

      double theta = -(atan2(line_position_data->y1 - line_position_data->y0, line_position_data->x1 - line_position_data->x0)- M_PI / 2) + offset;
      // 角度を計算.
      if( theta < -M_PI/2 )
      {
        theta += M_PI;
      }
      else if ( M_PI/2 < theta)
      {
        theta -= M_PI;
      }
      // 角度を -PI/2以上PI/2以下にする.
      std::cout << boost::format("theta:%+5.2f") % (theta * 180 / M_PI) << "[deg]" << std::endl;
      // 標準入出力に出力.

      const double rel_line_x0 = line_position_data->x0 - origin_x;
      const double rel_line_y0 = line_position_data->y0 - origin_y;
      const double rel_line_x1 = line_position_data->x1 - origin_x;
      const double rel_line_y1 = line_position_data->y1 - origin_y;
      // 機体中心から見た直線の点の位置.

      const double line_distance = fabs(rel_line_x0 * rel_line_y1 - rel_line_x1 * rel_line_y0) / line_position_data->length;
      // ロボット中心から直線までの距離.

      std::cout << boost::format("distance to line:%4.2f[m]") % line_distance << std::endl;
      // 標準入出力に出力.

      line_detect tmp_data = {theta, line_distance};
      line_detect_data = tmp_data;
    }
  }

  return line_detect_data;
}

static boost::optional<line_detect> modify_for_field(boost::optional<line_detect> line_detect_data, const char field_color)
{
  if(!line_detect_data)
  {
    // 検出できてない.
    return line_detect_data;
  }

  if (field_color == 'B')
  {
    // 青フィールド時.
  }
  else if (field_color == 'R')
  {
    // 赤フィールド時.
    line_detect_data->theta += M_PI;
  }
  else
  {
    // 他の値.
    line_detect_data = boost::none;
  }

  return line_detect_data;
}

static boost::optional<line_detect> sigma_check(const boost::optional<line_detect> &line_detect_data, const double MB_theta, const double MB_sigma)
{
  // 計算した値が3simaの範囲内にあるかを推定する.

  if(line_detect_data)
  {
    // 検出できている.

    if(line_detect_data->theta < MB_theta - 3 * MB_sigma || MB_theta + 3 * MB_sigma < line_detect_data->theta)
    {
      // 3sigmaの範囲内になかった.

      return boost::none;
    }

  }

  return line_detect_data;
}

static void publish_line_detect(const boost::optional<line_detect> &line_detect_data, const double J_sigma, detect_circle::Jline *msg, const ros::Publisher &Jline_pub)
{
  // ロボットの自己位置と木枠までの距離を発行.

  if(line_detect_data)
  {
    // 情報があった.
    msg->theta = line_detect_data->theta;
    msg->line_distance = line_detect_data->line_distance;
    msg->sigma = J_sigma;
    msg->stamp = ros::Time::now();
    Jline_pub.publish(*msg);

    std::cout << boost::format("robot theta:%+4.1f[deg]") % (line_detect_data->theta * 180 / M_PI) << std::endl;
    std::cout << boost::format("\x1b[36m" "success to detect line." "\x1b[39m") << std::endl;
  }
  else
  {
    std::cout << boost::format("\x1b[31m" "fail to detect line." "\x1b[39m") << std::endl;
  }
}
