/*
======================================================================
Project Name    : connect ethLRF
File Name       : connect_ethLRF
Encoding        : UTF-8
Creation Date   : 2017/03/08

Copyright © 2017 Alice.
======================================================================
*/

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <vector>
#include "../lib/urg_utils.h"
#include "../lib/urg_sensor.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "../include/connect_ethLRF_p.h"
#include "../include/connect_LRF_lib.h"

namespace
{
  namespace param
  {
    // パラメータ

    constexpr urg_connection_type_t connection_type = URG_ETHERNET;
    // 接続方式.
    constexpr char connect_address[] = "192.168.0.10";
    // デバイス名.
    constexpr long int connect_port = 10940;
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
  ros::init(argc, argv, "connect_ethLRF");
  ros::NodeHandle n;
  ros::Publisher lrf_data = n.advertise<sensor_msgs::LaserScan>("scan_ethLRF", 2);
  sensor_msgs::LaserScan msg;
  // ROS設定

  urg_t urg;
  int length_data_size;
  // 取得したLRFのデータ数を格納.
  long int *length_data = nullptr;
  // LRFの距離データを格納.
  unsigned short *intensity_data = nullptr;
  // LRFの強度データを格納.

  loop_to_connect_LRF(&urg, param::connection_type, param::connect_address, param::connect_port, param::LRF_recconect_hz, param::timeout_ms);
  // LRFに接続.

  loop_to_ensure_memory(&urg, &length_data, param::memory_reensure_hz);
  // 距離データ用のメモリを確保.
  loop_to_ensure_memory(&urg, &intensity_data, param::memory_reensure_hz);
  // 強度データ用のメモリを確保.

  while (ros::ok()) {
    ros::Rate loop_rate(param::main_loop_hz);
    // ループ周期設定.

    length_data_size = urg_get_distance_intensity(&urg, length_data, intensity_data, NULL);
    // LRFのデータを入手.

    if (length_data_size < 0)
    {
      // エラーでLRFのデータを入手できなかった時.
      ROS_INFO("urg error:%s", urg_error(&urg));
      ROS_INFO("fail to get LRF data.");
      urg_close(&urg);
      ROS_INFO("close LRF connection.");
      //LRFと切断.

      loop_to_connect_LRF(&urg, param::connection_type, param::connect_address, param::connect_port, param::LRF_recconect_hz, param::timeout_ms);
    }
    else if (length_data_size == 0)
    {
      ROS_INFO("got no data.");
    }
    else
    {
      // 正常にデータ取得

      convert_ros_data(&urg, length_data, intensity_data, &msg.ranges, &msg.intensities);
      // データの変換.

      set_msg_data(&msg, &urg, length_data_size);
      // メッセージの設定

      lrf_data.publish(msg);
      // 発行.
    }

    loop_rate.sleep();
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
