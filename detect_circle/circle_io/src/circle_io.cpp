#define DEIGEN_NO_DEBUG
// #define CIRCLE_IO_DEBUG_MODE

#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include "ros/ros.h"
#include "detect_circle/MBinput.h"
#include "detect_circle/Jcircle.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "../include/circle_io_p.hpp"
#include "../include/pole.hpp"
#include "../include/detect_circle_cuda.hpp"
#include "../include/write_circle.hpp"

namespace
{
  namespace param
  {
    constexpr double LRF_diff_x = 0.4425, LRF_diff_y = 0.4698;
    // LRF座標系での話

    constexpr int pole_num = 7;
    // ポールの個数

    const ros::Duration MBdata_late(400e-3);
    // 許容する遅れ時間.

    const Pole pole[pole_num] =
    {
      Pole( 3.5,  7.075, 0),
      Pole( 5.5,  7.075, 1),
      Pole( 7.5,  7.075, 2),
      Pole( 9.5,  7.075, 3),
      Pole(11.5,  7.075, 4),
      Pole( 7.5,  4.075, 5),
      Pole( 7.5, 10.075, 6)
    };
    // ポールの配列

    constexpr float hough_thr_coe = 0.6;
    // ハフ返還時の閾値. 1[m]あたりの係数.

    hough_param_str hough_param = {
      0.03f, // x_wid.   ハフ変換する際のxの間隔.
      0.03f, // y_wid.   ハフ変換する際のyの間隔.
      64,   // x_num.   ハフ変換する際のxの個数.
      64,   // y_num.   ハフ変換する際のyの個数.
      8,    // near_x.  ハフ変換する際のxの近傍とみなす個数.
      8,    // near_y.  ハフ変換する際のyの近傍とみなす個数.
      7.0f,  // weight.  重み付き平均を求める際の範囲.
      0     // thr.     ハフ変換する際の閾値.
    };

    constexpr int lrf_num = 1024;
    // LRFの見る点の数.

    constexpr float rad = 0.28f / 2;
    // ポールの半径.

    constexpr double x_sigma = 0.02, y_sigma = 0.02;
    // x,yの分散.
  }

  namespace var
  {
    ros::Publisher Jcircle_pub;
    ros::Publisher marker_pub;
    detect_circle::Jcircle msg;

    bool write_position = false;
    int MB_pole1, MB_pole2;
    char color;
    double x, y, theta, x_sigma, y_sigma, theta_sigma;
    ros::Time stamp;
  }

  const float rad_err1=0.1,rad_err2=0.05,rad_err3=0.03,allow_err1=0.01,allow_err2=0.005;
  const int thr2=8;
  const int warp=32,limit_count=50;
  float pole_rel_x=0,pole_rel_y=0;
  /*
   rad…ポールの半径
   rad_err1…1回目のハフ変換で求めた円の座標からの許容するズレ
   rad_err2…1回目の最尤推定法で求めた園の座標から許容するズレ
   allow_err1…1回目の最尤推定法で許容する誤差
   allow_err2…2回目の最尤推定法で許容する誤差
   x,y…今推定しているポールの座標
   x_wid,y_wid…ハフ変換する際の間隔
   x_num,y_num…ハフ変換する際の個数
   lrf_begin/end…lrfのデータを見始める/見終わる個数
   near_x,near_y…近傍とみなす範囲
   thr…閾値
   weight…重み付き平均を求める際の範囲
  */

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "circle_io");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("MBdata", 1, Sub_Callback);
  ros::Subscriber laser = n.subscribe("scan_ethLRF",1,Laser_Callback);
  var::Jcircle_pub = n.advertise<detect_circle::Jcircle>("Jcircle", 1);
  var::marker_pub = n.advertise<visualization_msgs::Marker>("write_circle", 1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

static Eigen::Matrix4d RotMatrix(double theta)
{
  // 同時座標変換における(Z軸中心の)回転行列

  Eigen::Matrix4d mat;
  mat <<  cos(theta), -sin(theta),  0,  0,
          sin(theta), cos(theta),   0,  0,
          0,          0,            1,  0,
          0,          0,            0,  1;

  return mat;
}

static Eigen::Matrix4d TransMatrix(double x, double y)
{
  // 同時座標変換における並進移動の変換行列.

  Eigen::Matrix4d mat;
  mat <<  1, 0, 0, x,
          0, 1, 0, y,
          0, 0, 1, 0,
          0, 0, 0, 1;

  return mat;
}

static void calc_matrix(Eigen::Matrix4d *AbsToLRF, const float x, const float y, const float theta, const float LRF_diff_x, const float LRF_diff_y)
{
  // 絶対座標系からLRF座標系への同時変換行列の計算
  *AbsToLRF =  Eigen::MatrixXd::Identity(4,4);
  *AbsToLRF = (*AbsToLRF) * TransMatrix((double)x, (double)y) * RotMatrix((double)theta);
  // 機体の今いる自己位置へ移動.
  *AbsToLRF = (*AbsToLRF) * RotMatrix(M_PI/2);
  // LRFの座標系のとり方に変換.
  *AbsToLRF = (*AbsToLRF) * TransMatrix(LRF_diff_x,LRF_diff_y);
  // 機体中心からLRF中心に移動.
}

static void Sub_Callback(const detect_circle::MBinput& msg)
{
  // uartにより発行されたトピックの購読
  var::write_position = true;
  var::MB_pole1 = (int)msg.MB_pole1;
  var::MB_pole2 = (int)msg.MB_pole2;
  var::color = (char)msg.color;
  var::x = msg.x;
  var::y = msg.y;
  var::theta = msg.theta;
  var::x_sigma = msg.x_sigma;
  var::y_sigma = msg.y_sigma;
  var::theta_sigma = msg.theta_sigma;
  var::stamp = msg.stamp;
  // ROS_INFO("MBdata::MB_pole:%d,color:%c,x:%f,y:%f,theta:%f",(int)msg.MB_pole,(char)msg.color,msg.x,msg.y,msg.theta);
}

void Laser_Callback(const sensor_msgs::LaserScan& msg)
{
  // LRFデータの購読.
  if (false == var::write_position)
  {
    // なにも書き込まれていない時.
    return;
  }

  var::write_position = false;

  #ifndef CIRCLE_IO_DEBUG_MODE

  const ros::Duration diff = ros::Time::now() - var::stamp;

  if (param::MBdata_late < diff)
  {
    // 位置情報が指定時間よりも古い情報なら処理しない.
    return;
  }

  #endif

  int search_success = -1;
  if (0 <= var::MB_pole1 || var::MB_pole1 < param::pole_num)
  { 
    search_success = SearchPole(msg, var::MB_pole1);
  }
  if (-1 == search_success)
  {
    if (0 <= var::MB_pole2 || var::MB_pole2 < param::pole_num)
    {
    search_success = SearchPole(msg, var::MB_pole2);
    }
  }
  if (-1 == search_success)
  {
    std::cout << " fail to calc." << std::endl;
  }  
}

int isInSigma(double calc_x, double calc_y, double x, double y, double x_sigma, double y_sigma)
{
  int success = 0;
  if (calc_x < x - x_sigma || x + x_sigma < calc_x || calc_y < y - y_sigma || y + y_sigma < calc_y)
  {
    success = -1;
  }
  return success;
}

int SearchPole(const sensor_msgs::LaserScan& msg, const int watch_pole_num)
{
  Eigen::Matrix4d AbsToLRF;
  calc_matrix(&AbsToLRF, var::x, var::y, var::theta, param::LRF_diff_x, param::LRF_diff_y);
  // 絶対座標系からLRF座標系への同時変換行列

  const Eigen::Vector4d pole_abs = param::pole[watch_pole_num].getVector();
  // ポールの絶対位置

  const Eigen::Vector4d pole_rel = AbsToLRF.inverse() * pole_abs;
  // LRFからみたポールの相対座標

  // printf("x:%f,y:%f\n", pole_rel(0), pole_rel(1));

  pole_rel_x = pole_rel(0);
  pole_rel_y = pole_rel(1);
  //printf("%f,%f\n", pole_rel_x, pole_rel_y);

  const float pole_dis = sqrt(pole_rel_x * pole_rel_x + pole_rel_y * pole_rel_y);
  // ポールまでの距離.

  param::hough_param.thr = static_cast<int>(6 - param::hough_thr_coe * pole_dis);

  const float thr2 = 16 - pole_dis * 0.8;

  const int calc_success = detect_circle_cuda(msg.ranges, param::lrf_num, msg.angle_min, msg.angle_increment, &param::hough_param, thr2, &pole_rel_x, &pole_rel_y, param::rad, rad_err1, rad_err2, rad_err3, allow_err1, allow_err2, warp, limit_count);

  if (-1 == calc_success)
  {
    // 計算できなかった時
    return -1;
  }

  write_circle(pole_rel_x, pole_rel_y, param::rad, var::marker_pub);

  //printf("p:%f,q:%f\n", p, q);
  Eigen::Vector4d pole_rel_modify;
  pole_rel_modify << pole_rel_x, pole_rel_y, 0, 1;
  // 検出後のLRFから見たポールの位置.

  const Eigen::Vector4d pole_abs_modify = AbsToLRF * pole_rel_modify;
  // 検出後のポールの絶対位置.

  Eigen::Matrix4d AbsToModify = TransMatrix(pole_abs(0) - pole_abs_modify(0), pole_abs(1) - pole_abs_modify(1));

  Eigen::Vector4d machine_abs;
  machine_abs << var::x, var::y, 0, 1;
  // 機体の自己位置

  Eigen::Vector4d machine_abs_modify = AbsToModify * machine_abs;
  // 機体の修正後の自己位置

  int in_sigma = isInSigma(machine_abs_modify(0), machine_abs_modify(1), var::x, var::y, var::x_sigma, var::y_sigma);

  if (0 == in_sigma){
    var::msg.MB_pole = watch_pole_num;
    var::msg.x = machine_abs_modify(0);
    var::msg.y = machine_abs_modify(1);
    var::msg.x_sigma = param::x_sigma;
    var::msg.y_sigma = param::y_sigma;
    var::msg.stamp = ros::Time::now();

    var::Jcircle_pub.publish(var::msg);


    std::cout << "x:" << machine_abs_modify(0) << " y:" << machine_abs_modify(1) << std::endl;
    return 0;
  }
  else
  {
    std::cout << "not in sigma" << std::endl;
    return -1;
  }
}
