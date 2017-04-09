/*
======================================================================
Project Name    : connect LRF lib
File Name       : connect_LRF_lib
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
#include "../include/connect_xxxLRF.hpp"
#include "../include/connect_LRF_lib_p.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, param::node_name);
  ros::NodeHandle n;
  ros::Publisher lrf_data = n.advertise<sensor_msgs::LaserScan>(param::topic_name, 2);
  sensor_msgs::LaserScan msg;
  // ROS設定

  urg_t urg;
  // urg_t構造体.
  int length_data_size;
  // 取得したLRFのデータ数を格納.
  long int *length_data;
  // LRFの距離データを格納.
  unsigned short *intensity_data;
  // LRFの強度データを格納.

  loop_to_connect_LRF(&urg, param::connection_type, param::connect_address_device, param::connect_port_baudrate, param::LRF_recconect_hz, param::timeout_ms);
  // LRFに接続.

  int size = urg_max_data_size(&urg);

  loop_to_ensure_memory(size, &length_data, param::memory_reensure_hz);
  // 距離データ用のメモリを確保.
  loop_to_ensure_memory(size, &intensity_data, param::memory_reensure_hz);
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

      loop_to_connect_LRF(&urg, param::connection_type, param::connect_address_device, param::connect_port_baudrate, param::LRF_recconect_hz, param::timeout_ms);

      if(size != urg_max_data_size(&urg))
      {
        // サイズが変わった.

        std::free(length_data);
        std::free(intensity_data);
        // メモリ解放

        size = urg_max_data_size(&urg);

        loop_to_ensure_memory(size, &length_data, param::memory_reensure_hz);
        // 距離データ用のメモリを確保.
        loop_to_ensure_memory(size, &intensity_data, param::memory_reensure_hz);
        // 強度データ用のメモリを確保.
      }
    }
    else if (length_data_size == 0)
    {
      ROS_INFO("got no data.");
    }
    else
    {
      // 正常にデータ取得

      convert_ros_data(size, length_data, intensity_data, &msg.ranges, &msg.intensities);
      // データの変換.

      set_msg_data(&msg, size, urg, length_data_size, param::frame_name);
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

static void loop_to_connect_LRF(urg_t *urg_p,  urg_connection_type_t connection_type, const char *connect_address_device,const long int connect_port_baudrate, const int LRF_recconect_hz, const int timeout_ms)
{
  // LRFと接続,通信開始.

  ros::Rate loop_rate(LRF_recconect_hz);
  // ループ周期設定.

  int err_val;
  // エラーの戻り値を格納.

  do
  {
    err_val = urg_open(urg_p, connection_type, connect_address_device, connect_port_baudrate);
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
      ROS_INFO("error:[%s]%s",connect_address_device,strerror(errno));
      ROS_INFO("urg error:%s", urg_error(urg_p));
      ROS_INFO("fail to conect LRF. try to recconect.");

      std::system("dmesg | grep URG");
      // おまじない.
    }

    urg_close(urg_p);
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

    loop_to_connect_LRF(urg_p, connection_type, connect_address_device, connect_port_baudrate, LRF_recconect_hz, timeout_ms);
    // 再接続(再起).
  }
}

static void convert_ros_data(const int size,const long int *length_data,const unsigned short *intensity_data, std::vector<float> *ros_ranges, std::vector<float> *ros_intensities)
{
  // LRFのデータをLaserScan型のデータに変更.

  if(nullptr == length_data || nullptr == intensity_data)
  {
    // ヌルポインタがある.
    ROS_INFO("include nullptr");
    return;
  }

  ros_ranges->resize(size);
  ros_intensities->resize(size);

  for(int i = 0; i < size; i++)
  {
    (*ros_ranges)[i] = static_cast<float>(length_data[i]) / 1000;
    // [mm]->[m]
    (*ros_intensities)[i] = static_cast<float>(intensity_data[i]);
  }

}

static void set_msg_data(sensor_msgs::LaserScan *msg_p,const int size, const urg_t &urg, const int real_data_size, const char frame_name[])
{
  msg_p->header.stamp = ros::Time::now();
  msg_p->header.frame_id = frame_name;

  long int min_distance = 0,max_distance = 0;
  urg_distance_min_max(&urg, &min_distance, &max_distance);
  msg_p->range_max = static_cast<float>(max_distance) / 1000;
  msg_p->range_min = static_cast<float>(min_distance) / 1000;

  msg_p->scan_time = static_cast<float>(urg_scan_usec(&urg))/1000000;
  msg_p->time_increment = msg_p->scan_time / size;

  msg_p->angle_min = static_cast<float>(urg_index2rad(&urg, 0));
  msg_p->angle_max = static_cast<float>(urg_index2rad(&urg, real_data_size - 1));
  msg_p->angle_increment = (msg_p->angle_max - msg_p->angle_min) / (real_data_size - 1);
}

template <typename T>
static void loop_to_ensure_memory(const int size, T **data, const int memory_reensure_hz)
{
  // LRFのデータ格納用のメモリを確保

  ros::Rate loop_rate(memory_reensure_hz);
  // ループ周期設定.

  do
  {
    *data = (T *)std::malloc(sizeof(T) * size);

    if(nullptr == *data)
    {
      // メモリ確保できず
      ROS_INFO("error:%s",std::strerror(errno));
      ROS_INFO("fail to ensure memory.");
    }
    else
    {
      // 確保成功
      break;
    }

    loop_rate.sleep();
  }
  while(ros::ok());
}
