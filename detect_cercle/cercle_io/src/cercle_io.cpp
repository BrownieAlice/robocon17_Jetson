#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include "ros/ros.h"
#include "detect_cercle/MBinput.h"
#include "detect_cercle/Joutput.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "../include/cercle_io_p.h"
#include "../include/pole.h"
#include "../include/detect_cercle_cuda.h"
#include "../include/write_cercle.h"
#define DEIGEN_NO_DEBUG
#define CERCLE_IO_DEBUG_MODE

namespace
{
  namespace param
  {
    constexpr double LRF_diff_x = 0.435, LRF_diff_y = 0;
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
  }

  namespace var
  {
    ros::Publisher Jdata_pub;
    ros::Publisher marker_pub;
    detect_cercle::Joutput msg;

    bool write_position=false;
    int MB_pole;
    char color;
    float x,y,theta;
    ros::Time stamp;
  }

  constexpr int lrf_data = 1081;
  // LRFの総データ数

  const float rad=0.28/2,rad_err1=0.1,rad_err2=0.05,rad_err3=0.03,allow_err1=0.01,allow_err2=0.005;
  const float x_wid=0.01,y_wid=0.01;
  const int x_num=256,y_num=256;
  const int lrf_begin=29,lrf_end=lrf_data-29,lrf_num=lrf_end-lrf_begin+1;
  const int near_x=6,near_y=6,thr=1,thr2=8;
  const float weight=5;
  const int warp=32,limit_count=50;
  float pole_rel_x=0,pole_rel_y=0;
  bool calc_flag=false;
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
  ros::init(argc, argv, "cercle_io");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("MBdata", 1, Sub_Callback);
  ros::Subscriber laser = n.subscribe("scan_ethLRF",1,Laser_Callback);
  var::Jdata_pub = n.advertise<detect_cercle::Joutput>("Jdata", 1);
  var::marker_pub = n.advertise<visualization_msgs::Marker>("write_cercle", 1);
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

static void Sub_Callback(const detect_cercle::MBinput& msg)
{
  // uartにより発行されたトピックの購読
  var::write_position = true;
  var::MB_pole = (int)msg.MB_pole;
  var::color = (char)msg.color;
  var::x = msg.x;
  var::y = msg.y;
  var::theta = msg.theta;
  var::stamp = msg.stamp;
  // ROS_INFO("MBdata::MB_pole:%d,color:%c,x:%f,y:%f,theta:%f",(int)msg.MB_pole,(char)msg.color,msg.x,msg.y,msg.theta);
}

void Laser_Callback(const sensor_msgs::LaserScan& msg)
{
  // LRFデータの購読
  if (false == var::write_position)
  {
    // なにも書き込まれていない時.
    return;
  }

  var::write_position = false;

  if (var::MB_pole < 0)
  {
    return;
  }
  if (param::pole_num <= var::MB_pole)
  {
    std::cout << "invalid pole_number" << std::endl;
    return;
  }

  #ifndef CERCLE_IO_DEBUG_MODE

  const ros::Duration diff = ros::Time::now() - var::stamp;

  if (param::MBdata_late < diff)
  {
    // 位置情報がlatemsよりも古い情報なら処理しない.
    return;
  }

  #endif

  Eigen::Matrix4d AbsToLRF;
  calc_matrix(&AbsToLRF, var::x, var::y, var::theta, param::LRF_diff_x, param::LRF_diff_y);
  // 絶対座標系からLRF座標系への同時変換行列

  const Eigen::Vector4d pole_abs = param::pole[var::MB_pole].getVector();
  // ポールの絶対位置

  const Eigen::Vector4d pole_rel = AbsToLRF.inverse() * pole_abs;
  // LRFからみたポールの相対座標

  // printf("x:%f,y:%f\n", pole_rel(0), pole_rel(1));

  pole_rel_x=pole_rel(0);
  pole_rel_y=pole_rel(1);

  detect_cercle_cuda(msg.ranges, lrf_num, msg.angle_min, msg.angle_increment, x_wid, y_wid, x_num, y_num, near_x, near_y, thr, thr2, &pole_rel_x, &pole_rel_y, &calc_flag, rad, rad_err1, rad_err2, rad_err3, allow_err1, allow_err2, weight, warp, limit_count);

  if (false==calc_flag)
  {
    // 計算できなかった時
    return;
  }

  write_cercle(pole_rel_x, pole_rel_y, rad, var::marker_pub);

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

  std::cout << "x:" << machine_abs_modify(0) << " y:" << machine_abs_modify(1) << std::endl;

  var::msg.MB_pole = var::MB_pole;
  var::msg.x = machine_abs_modify(0);
  var::msg.y = machine_abs_modify(1);
  var::msg.stamp = ros::Time::now();

  var::Jdata_pub.publish(var::msg);
}
