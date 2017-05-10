#ifndef INCLUDED_MB_UART_COMMUNICATION_P
#define INCLUDED_MB_UART_COMMUNICATION_P

#include "ros/ros.h"
#include "detect_circle/Jcircle.h"
#include <stdint.h>

static int16_t ucharToint16(unsigned char data1, unsigned char data2);
static int get_uart_input(int8_t *MB_pole1, int8_t *MB_pole2, int8_t *color, double *x, double *y, double *theta, double *x_sigma, double *y_sigma, double *theta_sigma);
static void publish_MBdata(const int8_t MB_pole1, const int8_t MB_pole2, const int8_t color, const double x, const double y, const double theta, const double x_simga, const double y_sigma, const double theta_sigma, const ros::Publisher MBdata_pub);
static void subscribe_Jcircle(const detect_circle::Jcircle&);
static void subscribe_Jline(const detect_circle::Jline& Jline);
static int uart_output(const int8_t MB_pole, const double x, const double y, const double x_sigma, const double y_sigma, const ros::Time stamp);
static int uart_output(const double theta, const double sigma, const double line_distance, const ros::Time stamp);

#ifndef M_PI
#define M_PI 3.1415

#endif
#endif
