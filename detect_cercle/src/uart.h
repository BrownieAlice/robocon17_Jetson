#ifndef _UART_
#define _UART_

int open_serial_port(const char *modem_dev);
void close_serial_port(void);
int put_serial_char(unsigned char c);
int put_serial_string(char *s);
unsigned char get_serial_char();

#endif
