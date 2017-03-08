#ifndef connect_usbLRF_p
#define connect_usbLRF_p

#include <urg_c/urg_sensor.h>
#include <urg_c/urg_utils.h>

static void loop_to_connect_LRF(urg_t *urg_p, const char *connect_device,const long int connect_baudrate, const int LRF_recconect_hz, const int timeout_ms);
static void loop_to_ensure_memory(const urg_t *urg_p, long int *length_data, const int memory_reensure_hz);
static void loop_to_ensure_memory(const urg_t *urg_p, unsigned short *intensity_data, const int memory_reensure_hz);

#endif
