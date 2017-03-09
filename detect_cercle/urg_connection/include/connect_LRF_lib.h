#ifndef connect_LRF_lib
#define connect_LRF_lib

#include <vector>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include "ros/ros.h"
#include "../lib/urg_utils.h"
#include "../lib/urg_sensor.h"
#include "sensor_msgs/LaserScan.h"

void loop_to_connect_LRF(urg_t *urg_p,  urg_connection_type_t connection_type, const char *connect_address_device,const long int connect_port_baurate, const int LRF_recconect_hz, const int timeout_ms);
void convert_ros_data(const int size,const long int *length_data,const unsigned short *intensity_data, std::vector<float> *ros_ranges, std::vector<float> *ros_intensities);
void set_msg_data(sensor_msgs::LaserScan *msg_p,const int size, const urg_t &urg, const int real_data_size, const char frame_name[]);


template <typename T>
void loop_to_ensure_memory(const int size, T **data, const int memory_reensure_hz)
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

#endif
