/*
======================================================================
Project Name    : MB uart communication
File Name       : MB_uart_communication
Encoding        : UTF-8
Creation Date   : 2017/02/22

Copyright © 2017 Alice.
======================================================================
*/

#include "ros/ros.h"
#include "detect_circle/MBinput.h"
#include "detect_circle/Jcircle.h"
#include "detect_circle/Jline.h"
#include "./lib/uart.hpp"
#include "./MB_uart_communication_p.hpp"
#include <stdint.h>

namespace
{
  bool uart_flag = false;
  // uart接続ができているかどうか.
  constexpr unsigned int main_loop_hz = 100;
  // mainループの周期[Hz].
  constexpr long connect_loop_hz = 1000;
  // 再接続周期[Hz].
  constexpr long uart_wait_ns = 5000000;
  // uartが何も文字を取得しなかった時に待つ秒数[ns].
  constexpr long timeout_us = 100000;
  // read/writeでタイムアウトとみなす秒数[us].
  constexpr int timeout_lim = 5;
  // 連続タイムアウトを切断されたとみなす回数.
  constexpr int zero_lim = 10;
  // uartが何も文字を連続で取得しなかった時に切断されたとみなす回数.
  constexpr char serial_dev[] = "/dev/ttyUSB_MB";
  // シリアルポート名.
  const ros::Duration data_late(500e-3);
  // 許容する遅れ時間.

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MB_uart_communication");
  ros::NodeHandle n;
  ros::Publisher MBdata_pub = n.advertise<detect_circle::MBinput>("MBdata", 1);
  // MBから受け取ったデータをメイン処理プロセスに引き渡す.
  ros::Subscriber Jcircle_sub = n.subscribe("Jcircle", 1, subscribe_Jcircle);
  // 処理されたデータをMBに引き渡すためにこちらに渡す.
  ros::Subscriber Jline_sub = n.subscribe("Jline", 1, subscribe_Jline);
  // 処理された姿勢角データをMBに引き渡すためにこちらに渡す.

  while (ros::ok())
  {
    // メインループ

    if (false == uart_flag)
    {
      // uart接続が切れていたら接続し直す.

      ROS_INFO("MB is disconnected.");
      const int open_result = open_serial_port(serial_dev);
      // ポートを開ける

      if (0 == open_result)
      {
        uart_flag = true;
      }
      else
      {
        uart_flag = false;
      }

      ros::Rate loop_rate(connect_loop_hz);
      // ループ周期を規定
       loop_rate.sleep();
    }
    else
    {
      // すでにuart接続ができている.
      ros::Rate loop_rate(main_loop_hz);
      // ループ周期を規定

      int8_t MB_pole1 = 0, MB_pole2 = 0, color = 0;
      double x = 0, y = 0, theta = 0, x_sigma = 0, y_sigma = 0, theta_sigma = 0;
      int success;
      // MBからデータを受け取るための変数.

      success = get_uart_input(&MB_pole1, &MB_pole2, &color, &x, &y, &theta, &x_sigma, &y_sigma, &theta_sigma);
      // MBからデータを受け取る.

      switch (success)
      {
        case 1:
          // MBからのデータ受け取り成功時.

          publish_MBdata(MB_pole1, MB_pole2, color, x, y, theta, x_sigma, y_sigma, theta_sigma, MBdata_pub);
          // MBからのデータをメイン処理プロセスに引き渡す.
          ros::spinOnce();
          // 処理されたデータを受け取るコールバック関数の呼び出しが可能なら行われる.
          break;
        case 0:
          // MBからデータを受け取れなかった時.
          break;
        case -1:
          // 受け取り時にエラー発生.
          uart_flag = false;
          break;
        default :
          break;
      }

      loop_rate.sleep();
    }
  }
  close_serial_port();
}

static int16_t ucharToint16(unsigned char data1, unsigned char data2)
{
  // 2つのunsigned char型を1つのint16_t型にする.
  return (int16_t)((uint16_t)data1)|((uint16_t)data2<<8);
  /*
  処理系依存
  */
}

static int get_uart_input(int8_t *MB_pole1, int8_t *MB_pole2, int8_t *color, double *x, double *y, double *theta, double *x_sigma, double *y_sigma, double theta_sigma)
{
  //  MBからのデータを受け取る.
  unsigned char data[15];
  int flag = get_MB_data('X', data, sizeof(data), uart_wait_ns, timeout_us, timeout_lim, zero_lim);
  // MBからのデータ受け取り.

  if (1 == flag)
  {
    *MB_pole1 = static_cast<int8_t>(data[0]);
    *MB_pole2 = static_cast<int8_t>(data[1]);
    *color = static_cast<int8_t>(data[2]);
    // 処理系依存動作
    *x = static_cast<double>(ucharToint16(data[3], data[4]))/1000;
    *y = static_cast<double>(ucharToint16(data[5], data[6]))/1000;
    *theta = static_cast<double>(ucharToint16(data[7], data[8]))/1000;
    *x_sigma = static_cast<double>(ucharToint16(data[9], data[10]))/10000;
    *y_sigma = static_cast<double>(ucharToint16(data[11], data[12]))/10000;
    *theta_sigma = static_cast<double>(ucharToint16(data[13], data[14]))/1000;
    return 1;
  }
  else
  {
    return flag;
  }
}

static void publish_MBdata(const int8_t MB_pole1, const int8_t MB_pole2, const int8_t color, const double x, const double y, const double theta, const double x_simga, const double y_sigma, const double theta_sigma, const ros::Publisher MBdata_pub)
{
  // MBからのデータを発行する.
  detect_circle::MBinput MBinfo;

  MBinfo.MB_pole1 = MB_pole1;
  MBinfo.MB_pole2 = MB_pole2;
  MBinfo.color = color;
  MBinfo.x = x;
  MBinfo.y = y;
  MBinfo.theta = theta;
  MBinfo.x_sigma = x_sigma;
  MBinfo.y_sigma = y_sigma;
  MBinfo.theta_sigma = theta_sigma;
  MBinfo.stamp = ros::Time::now();

  MBdata_pub.publish(MBinfo);
  ROS_INFO("MBdata::MB_pole1:%d,MB_pole2:%d,color:%c,x:%f,y:%f,theta:%f,x_sigma:%f,y_sigma:%f,theta_sigma:%f", (int)MB_pole1, (int)MB_pole2, (char)color, x, y, theta, x_sigma, y_sigma, theta_sigma);
}

static void subscribe_Jcircle(const detect_circle::Jcircle& Jcircle)
{
  int success;
  success = uart_output(Jcircle.MB_pole, Jcircle.x, Jcircle.y, Jcircle.x_sigma, Jcircle.y_sigma, Jcircle.stamp);
  uart_flag = 1 == success ? true : false;
}

static void subscribe_Jline(const detect_circle::Jline& Jline)
{
  int success = uart_output(Jline.theta, Jline.sigma, Jline.line_distance, Jline.stamp);
  uart_flag = 1 == success ? true : false;
}

static int uart_output(const int8_t MB_pole, const double x, const double y, const double x_sigma, const double y_sigma, const ros::Time stamp)
{
  // 円検出より推定した事故位置情報をMBに送信する.
  const int16_t send_x = static_cast<int>(x * 1000);
  const unsigned char X_L = static_cast<unsigned char>(send_x);
  const unsigned char X_H = static_cast<unsigned char>((uint16_t)send_x>>8);
  // 推定したx方向の値.
  const int16_t send_y = static_cast<int>(y * 1000);
  const unsigned char Y_L = static_cast<unsigned char>(send_y);
  const unsigned char Y_H = static_cast<unsigned char>((uint16_t)send_y>>8);
  // 推定したy方向の値.
  const int16_t send_x_sigma = static_cast<int>(x_sigma * 10000);
  const unsigned char SX_L = static_cast<unsigned char>(send_x_sigma);
  const unsigned char SX_H = static_cast<unsigned char>((uint16_t)send_x_sigma>>8);
  // 推定したx方向の値.
  const int16_t send_y_sigma = static_cast<int>(y_sigma * 10000);
  const unsigned char SY_L = static_cast<unsigned char>(send_y_sigma);
  const unsigned char SY_H = static_cast<unsigned char>((uint16_t)send_y_sigma>>8);
  // 推定したy方向の値.

  /*
  処理系依存動作
  */

  ros::Duration diff = ros::Time::now()-stamp;
  if (data_late < diff)
  {
    // あまりにも前のデータなら送信しない.
    ROS_INFO("too late data.");
    return(0);
  }

  const size_t data_num = 9;
  // 送るデータ数
  unsigned char *data = (unsigned char *)malloc(data_num);
  if (NULL == data)
  {
    return(-1);
  }

  data[0] = static_cast<unsigned char>(MB_pole);
  data[1] = X_H;
  data[2] = X_L;
  data[3] = Y_H;
  data[4] = Y_L;
  data[5] = SX_H;
  data[6] = SX_L;
  data[7] = SY_H;
  data[8] = SY_L;

  const int success = put_J_data('X', data, data_num, timeout_us, timeout_lim);

  if (1 == success)
  {
    ROS_INFO("Jcircle::MB_pole:%d,x:%f,y:%f,x_sigma:%f,y_sigma:%f", static_cast<int>(MB_pole), x, y, x_sigma, y_sigma);
  }

  free(data);
  return success;
}

static int uart_output(const double theta, const double sigma, const double line_distance, const ros::Time stamp)
{
  // 木枠検出の際のMBへのデータの送信.
  const int16_t send_theta = static_cast<int>(theta * 1000);
  const unsigned char T_L = static_cast<unsigned char>(send_theta);
  const unsigned char T_H = static_cast<unsigned char>((uint16_t)send_theta>>8);
  // 姿勢角情報.
  const int16_t send_sigma = static_cast<int>(sigma * 1000);
  const unsigned char S_L = static_cast<unsigned char>(send_sigma);
  const unsigned char S_H = static_cast<unsigned char>((uint16_t)send_sigma>>8);
  // 姿勢角の標準偏差.
  const int16_t send_line_distance = static_cast<int>(line_distance * 1000);
  const unsigned char LD_L = static_cast<unsigned char>(send_line_distance);
  const unsigned char LD_H = static_cast<unsigned char>((uint16_t)send_line_distance>>8);
  // 木枠との距離.
  /*
  処理系依存動作
  */

  ros::Duration diff = ros::Time::now() - stamp;
  if (data_late < diff)
  {
    // あまりにも前のデータなら送信しない.
    ROS_INFO("too late data.");
    return(0);
  }

  const size_t data_num = 6;
  // 送るデータ数
  unsigned char *data = (unsigned char *)malloc(data_num);
  if (NULL == data)
  {
    return(-1);
  }

  data[0] = T_H;
  data[1] = T_L;
  data[2] = S_H;
  data[3] = S_L;
  data[4] = LD_H;
  data[5] = LD_L;

  const int success = put_J_data('Y', data, data_num, timeout_us, timeout_lim);

  if (1 == success)
  {
    ROS_INFO("Jline::theta:%f,line_distance:%f",theta,line_distance);
  }

  free(data);
  return success;
}
