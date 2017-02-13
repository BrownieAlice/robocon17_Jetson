#ifndef CONNECT_BY_UART
#define CONNECT_BY_UART

int open_serial_port(const char *);
void close_serial_port(void);
int put_serial_char(unsigned char);
int put_serial_string(char *);
unsigned char get_serial_char(ssize_t *,long int,int);
int get_MB_data(char,unsigned char *,size_t,long int,long int);
void continue_connect_uart(long int,const char *);

#define BAUDRATE B115200

#endif
