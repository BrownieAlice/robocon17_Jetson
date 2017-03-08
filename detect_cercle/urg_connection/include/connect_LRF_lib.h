#ifndef connect_LRF_lib
#define connect_LRF_lib

#include <vector>
#include "../lib/urg_utils.h"
#include "../lib/urg_sensor.h"
#include "sensor_msgs/LaserScan.h"

void loop_to_connect_LRF(urg_t *urg_p,  urg_connection_type_t connection_type, const char *connect_address,const long int connect_port, const int LRF_recconect_hz, const int timeout_ms);
void loop_to_ensure_memory(const urg_t *urg_p, long int **length_data, const int memory_reensure_hz);
void loop_to_ensure_memory(const urg_t *urg_p, unsigned short **intensity_data, const int memory_reensure_hz);
void convert_ros_data(const urg_t *urg_p,const long int *length_data,const unsigned short *intensity_data, std::vector<float> *ros_ranges, std::vector<float> *ros_intensities);
void set_msg_data(sensor_msgs::LaserScan *msg_p, const urg_t *urg_p, const int real_data_size);

#endif
