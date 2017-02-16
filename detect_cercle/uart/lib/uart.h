#ifndef CONNECT_BY_UART
#define CONNECT_BY_UART

int open_serial_port(const char *modem_dev);
void close_serial_port(void);
int put_serial_char(const unsigned char c,const long int timeout_us,const long timeout_lim);
int put_serial_string(const unsigned char *s,const size_t size,const long int timeout_us,const long timeout_lim);
int get_serial_char(unsigned char *s,const long int timeout_us,const int timeout_lim);
int get_MB_data(const char init,unsigned char *data,const size_t num,const long int loop,const long int timeout_us,const int timeout_lim,const int zero_lim);
int put_Jdata(const char init,const unsigned char *s,const size_t size,const long int timeout_us,const long int timeout_lim);

#define BAUDRATE B115200

#endif
