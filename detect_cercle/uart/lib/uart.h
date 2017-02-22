#ifndef CONNECT_BY_UART
#define CONNECT_BY_UART

int open_serial_port(const char *modem_dev);
void close_serial_port(void);
int get_MB_data(const char init, unsigned char *data, const size_t num, const long int once_wait_ns, const long int timeout_us, const int timeout_lim, const int zero_lim);
int put_J_data(const char init, const unsigned char *s, const size_t size, const long int timeout_us, const long int timeout_lim);

#endif
