#ifndef INCLUDED_CONNCET_LRF_LIB
#define INCLUDED_CONNCET_LRF_LIB

#include <vector>
#include "../lib/urg_utils.h"
#include "../lib/urg_sensor.h"
#include "sensor_msgs/LaserScan.h"

static void loop_to_connect_LRF(urg_t *urg_p,  urg_connection_type_t connection_type, const char *connect_address_device,const long int connect_port_baurate, const int LRF_recconect_hz, const int timeout_ms);
static void convert_ros_data(const int size,const long int *length_data,const unsigned short *intensity_data, std::vector<float> *ros_ranges, std::vector<float> *ros_intensities);
static void set_msg_data(sensor_msgs::LaserScan *msg_p,const int size, const urg_t &urg, const int real_data_size, const char frame_name[]);
template <typename T>
static void loop_to_ensure_memory(const int size, T **data, const int memory_reensure_hz);

#endif
