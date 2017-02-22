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
#include "detect_cercle/MBinput.h"
#include "detect_cercle/Joutput.h"
#include "./lib/uart.h"
#include "./MB_uart_communication_p.h"
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
  constexpr char *serial_dev = "/dev/ttyUSB_MB";
  // シリアルポート名.
  constexpr int late_ms = 500;
  
}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MB_uart_communication");
  ros::NodeHandle n;
  ros::Publisher MBdata_pub = n.advertise<detect_cercle::MBinput>("MBdata", 1);
  // MBから受け取ったデータをメイン処理プロセスに引き渡す.
  ros::Subscriber Jdata_sub = n.subscribe("Jdata", 1, subscribe_Jdata);
  // 処理されたデータをMBに引き渡すためにこちらに渡す.

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

      int8_t MB_pole = 0, color = 0;
      float x = 0, y = 0, theta = 0;
      int success;
      // MBからデータを受け取るための変数.

      success = get_uart_input(&MB_pole, &color, &x, &y, &theta);
      // MBからデータを受け取る.

      switch (success)
      {
        case 1:
          // MBからのデータ受け取り成功時.

          publish_MBdata(MB_pole, color, x, y, theta, MBdata_pub);
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

static int get_uart_input(int8_t *MB_pole, int8_t *color, float *x, float *y, float *theta)
{
  //  MBからのデータを受け取る.
  unsigned char data[8];
  int flag;
  flag = get_MB_data('X', data, sizeof(data), uart_wait_ns, timeout_us, timeout_lim, zero_lim);
  // MBからのデータ受け取り.

  if (1 == flag)
  {
    *MB_pole = static_cast<int8_t>(data[0]);
    *color = static_cast<int8_t>(data[1]);
    // 処理系依存動作
    *x = static_cast<float>(ucharToint16(data[2], data[3]))/1000;
    *y = static_cast<float>(ucharToint16(data[4], data[5]))/1000;
    *theta = static_cast<float>(ucharToint16(data[6], data[7]))/1000;
    return 1;
  }
  else
  {
    return flag;
  }
}

static void publish_MBdata(const int8_t MB_pole, const int8_t color, const float x, const float y, const float theta, const ros::Publisher MBdata_pub)
{
  // MBからのデータを発行する.
  detect_cercle::MBinput MBinfo;

  MBinfo.MB_pole = MB_pole;
  MBinfo.color = color;
  MBinfo.x = x;
  MBinfo.y = y;
  MBinfo.theta = theta;
  MBinfo.stamp = ros::Time::now();

  MBdata_pub.publish(MBinfo);
  ROS_INFO("MBdata::MB_pole:%d,color:%c,x:%f,y:%f,theta:%f", (int)MB_pole, (char)color, x, y, theta);
}

static void subscribe_Jdata(const detect_cercle::Joutput& Jdata)
{
  int success;
  success = uart_output(Jdata.MB_pole, Jdata.x, Jdata.y, Jdata.stamp);
  uart_flag = 1 == success ? true : false;
}

static int uart_output(const int8_t MB_pole, const float x, const float y, const ros::Time stamp)
{
    int16_t send_x = static_cast<int>(x*1000);
    int16_t send_y = static_cast<int>(y*1000);
    unsigned char X_L = static_cast<unsigned char>(send_x);
    unsigned char X_H = static_cast<unsigned char>((uint16_t)send_x>>8);
    unsigned char Y_L = static_cast<unsigned char>(send_y);
    unsigned char Y_H = static_cast<unsigned char>((uint16_t)send_y>>8);
    /*
    処理系依存動作
    */

    unsigned char *data = (unsigned char *)malloc(5);
    if (NULL == data)
    {
      return(-1);
    }

    ros::Duration diff = ros::Time::now()-stamp;

    if (late_ms < diff.sec*1000+diff.nsec/1000000)
    {
      ROS_INFO("too late data.");
      return(0);
    }

    data[0] = static_cast<unsigned char>(MB_pole);
    data[1] = X_H;
    data[2] = X_L;
    data[3] = Y_H;
    data[4] = Y_L;

    int success;

    success = put_J_data('X', data, 5, timeout_us, timeout_lim);

    if (1 == success)
    {
      ROS_INFO("Jdata::MB_pole:%d,x:%f,y:%f", static_cast<int>(MB_pole), x, y);
    }

    free(data);
    return success;
}
