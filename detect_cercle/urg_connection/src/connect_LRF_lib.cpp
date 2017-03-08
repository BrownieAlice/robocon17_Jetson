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

void loop_to_connect_LRF(urg_t *urg_p,  urg_connection_type_t connection_type, const char *connect_address,const long int connect_port, const int LRF_recconect_hz, const int timeout_ms)
{
  // LRFと接続,通信開始.

  ros::Rate loop_rate(LRF_recconect_hz);
  // ループ周期設定.

  int err_val;
  // エラーの戻り値を格納.

  do
  {
    err_val = urg_open(urg_p, connection_type, connect_address, connect_port);
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
      ROS_INFO("error:[%s]%s",connect_address,strerror(errno));
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

    loop_to_connect_LRF(urg_p, connection_type, connect_address, connect_port, LRF_recconect_hz, timeout_ms);
    // 再接続(再起).
  }
}


void loop_to_ensure_memory(const urg_t *urg_p, long int **length_data, const int memory_reensure_hz)
{
  // LRFの距離データ格納用のメモリを確保

  ros::Rate loop_rate(memory_reensure_hz);
  // ループ周期設定.

  do
  {
    *length_data = (long *)std::malloc(sizeof(long) * urg_max_data_size(urg_p));

    if(nullptr == *length_data)
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

void loop_to_ensure_memory(const urg_t *urg_p, unsigned short **intensity_data, const int memory_reensure_hz)
{
  // LRFの強度データ格納用のメモリを確保

  ros::Rate loop_rate(memory_reensure_hz);
  // ループ周期設定.

  do
  {
    *intensity_data = (unsigned short *)std::malloc(sizeof(unsigned short) * urg_max_data_size(urg_p));

    if(nullptr == *intensity_data)
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

void convert_ros_data(const urg_t *urg_p,const long int *length_data,const unsigned short *intensity_data, std::vector<float> *ros_ranges, std::vector<float> *ros_intensities)
{
  // LRFのデータをLaserScan型のデータに変更.

  if(nullptr == length_data || nullptr == intensity_data)
  {
    // ヌルポインタがある.
    ROS_INFO("include nullptr");
    return;
  }

  const int size = urg_max_data_size(urg_p);

  ros_ranges->clear();
  ros_intensities->clear();

  for(int i = 0; i < size; i++)
  {
    ros_ranges->push_back((float)length_data[i] / 1000);
    // [mm]->[m]
    ros_intensities->push_back((float)intensity_data[i]);
  }

}

void set_msg_data(sensor_msgs::LaserScan *msg_p, const urg_t *urg_p, const int real_data_size)
{
  const int size = urg_max_data_size(urg_p);
  msg_p->header.stamp = ros::Time::now();
  msg_p->header.frame_id = "/laser";

  long int min_distance = 0,max_distance = 0;
  urg_distance_min_max(urg_p, &min_distance, &max_distance);
  msg_p->range_max = (float)max_distance / 1000;
  msg_p->range_min = (float)min_distance / 1000;

  msg_p->scan_time = (float)urg_scan_usec(urg_p)/1000000;
  msg_p->time_increment = msg_p->scan_time / size;

  msg_p->angle_min = (float)urg_index2rad(urg_p, 0);
  msg_p->angle_max = (float)urg_index2rad(urg_p, real_data_size - 1);
  msg_p->angle_increment = (msg_p->angle_max - msg_p->angle_min) / (real_data_size - 1);
}
