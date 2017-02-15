#ifndef CONNECT_BY_UART
#define CONNECT_BY_UART

int open_serial_port(const char *modem_dev);
void close_serial_port(void);
int put_serial_char(unsigned char c);
int put_serial_string(char *s);
unsigned char get_serial_char(ssize_t *result,long int timeout_us,int timeout_lim);
int get_MB_data(char init,unsigned char *data,size_t num,long int loop,long int timeout_us,int timeout_lim,int zero_lim);
void continue_connect_uart(long int loop,const char *modem_dev);

#define BAUDRATE B115200

#endif
