/*
======================================================================
Project Name    : connect usbLRF
File Name       : connect_usbLRF
Encoding        : UTF-8
Creation Date   : 2017/03/07

Copyright © 2017 Alice.
======================================================================
*/

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <urg_c/urg_sensor.h>
#include <urg_c/urg_utils.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "../include/connect_usbLRF_p.h"

namespace
{
  namespace param
  {
    // パラメータ

    constexpr char connect_device[] = "COM1";
    // デバイス名.
    constexpr long int connect_baudrate = 115200;
    // ボーレート.
    constexpr int LRF_recconect_hz = 10;
    // LRFと接続できなかった時再接続する周期[Hz].
    constexpr int timeout_ms = 100;
    // LRFのタイムアウト時間[ms].

    constexpr int memory_reensure_hz = 10;
    // メモリを確保できなかった時に再確保する周期[Hz].
    constexpr int main_loop_hz = 50;
    // メイン関数の実行周期[Hz].
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "connect_usbLRF");
  ros::NodeHandle n;
  // ROS設定

  urg_t urg;
  int length_data_size;
  // 取得したLRFのデータ数を格納.
  long int *length_data = nullptr;
  // LRFの距離データを格納.
  unsigned short *intensity_data = nullptr;

  loop_to_connect_LRF(&urg, param::connect_device, param::connect_baudrate, param::LRF_recconect_hz, param::timeout_ms);
  // LRFに接続.

  loop_to_ensure_memory(&urg, length_data, param::memory_reensure_hz);
  // 距離データ用のメモリを確保.
  loop_to_ensure_memory(&urg, intensity_data, param::memory_reensure_hz);
  // 強度データ用のメモリを確保.

  while (ros::ok()) {
    ros::Rate loop_rate(param::main_loop_hz);
    // ループ周期設定.

    length_data_size = urg_get_distance_intensity(&urg, length_data, intensity_data, NULL);
    // LRFのデータを入手.

    if(length_data_size < 0)
    {
      // エラーでLRFのデータを入手できなかった時.
      ROS_INFO("urg error:%s", urg_error(&urg));
      ROS_INFO("fail to get LRF data.");
      urg_close(&urg);
      ROS_INFO("close LRF connection.");
      //LRFと切断.

      loop_to_connect_LRF(&urg, param::connect_device, param::connect_baudrate, param::LRF_recconect_hz, param::timeout_ms);
    }
    else
    {
      // 正常にデータ取得

      ROS_INFO("success to get LRF data. %d data got.",length_data_size);

    }
  }

  std::free(length_data);
  std::free(intensity_data);
  // メモリ解放

  urg_stop_measurement(&urg);
  ROS_INFO("stop LRF measure.");
  // LRFを停止.

  urg_close(&urg);
  ROS_INFO("close LRF connection.");
  //LRFと切断.

  return 0;
}

static void loop_to_connect_LRF(urg_t *urg_p, const char *connect_device,const long int connect_baudrate, const int LRF_recconect_hz, const int timeout_ms)
{
  // LRFと接続,通信開始.

  ros::Rate loop_rate(LRF_recconect_hz);
  // ループ周期設定.

  int err_val;
  // エラーの戻り値を格納.

  do
  {
    err_val = urg_open(urg_p, URG_SERIAL, connect_device, connect_baudrate);
    // LRFと接続.

    if (0 == err_val)
    {
      // 正常に接続.
      ROS_INFO("success to conect LRF.");
      break;
    }
    else
    {
      // 接続できず.
      ROS_INFO("error:[%s]%s",connect_device,strerror(errno));
      ROS_INFO("urg error:%s", urg_error(urg_p));
      ROS_INFO("fail to conect LRF. try to recconect.");
    }

    loop_rate.sleep();
  }
  while (ros::ok());

  if(!ros::ok())
  {
    return;
  }

  urg_set_timeout_msec(urg_p, timeout_ms);
  // タイムアウト値を設定.

  err_val = urg_start_measurement(urg_p, URG_DISTANCE_INTENSITY, URG_SCAN_INFINITY, 0);
  // LRFで計測を開始.

  if (0 == err_val)
  {
    // 正常に計測開始
    ROS_INFO("success to start measure.");
  }
  else
  {
    // 計測開始できず.
    ROS_INFO("urg error:%s", urg_error(urg_p));
    ROS_INFO("fail to start measure.");

    urg_close(urg_p);
    ROS_INFO("close LRF connection.");
    // LRFと切断.

    loop_to_connect_LRF(urg_p, connect_device, connect_baudrate, LRF_recconect_hz, timeout_ms);
    // 再接続(再起).
  }
}

static void loop_to_ensure_memory(const urg_t *urg_p, long int *length_data, const int memory_reensure_hz)
{
  // LRFの距離データ格納用のメモリを確保

  ros::Rate loop_rate(memory_reensure_hz);
  // ループ周期設定.

  do
  {
    length_data = (long *)std::malloc(sizeof(long) * urg_max_data_size(urg_p));

    if(nullptr == length_data)
    {
      // メモリ確保できず
      ROS_INFO("error:%s",std::strerror(errno));
      ROS_INFO("fail to ensure memory.");
    }
    else
    {
      break;
    }

    loop_rate.sleep();
  }
  while(ros::ok());
}

static void loop_to_ensure_memory(const urg_t *urg_p, unsigned short *intensity_data, const int memory_reensure_hz)
{
  // LRFの強度データ格納用のメモリを確保

  ros::Rate loop_rate(memory_reensure_hz);
  // ループ周期設定.

  do
  {
    intensity_data = (unsigned short *)std::malloc(sizeof(unsigned short) * urg_max_data_size(urg_p));

    if(nullptr == intensity_data)
    {
      // メモリ確保できず
      ROS_INFO("error:%s",std::strerror(errno));
      ROS_INFO("fail to ensure memory.");
    }
    else
    {
      break;
    }

    loop_rate.sleep();
  }
  while(ros::ok());
}
