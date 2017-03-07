#ifndef CONNECT_BY_UART_P
#define CONNECT_BY_UART_P

#include <stddef.h>
#include <termios.h>

static void init_newtio(void);
static int compare_termious(const struct termios *settio_p, const struct termios *nowtio_p);
static int get_and_wait_char(unsigned char *s, const struct timespec wait, const long int timeout_us, const int timeout_lim);
static int timeout_check(long int timeout_us);
static int timeout_coutinuous_check(const int timeout_result, long int *timeout_count, const long int timeout_lim, const char *s);
int open_serial_port(const char *modem_dev);
void close_serial_port(void);
// static int put_serial_char(const unsigned char c, const long int timeout_us, const long int timeout_lim);
static int put_serial_string(const unsigned char *s, const size_t size, const long int timeout_us, const long timeout_lim);
int put_J_data(const char init, const unsigned char *s, const size_t size, const long int timeout_us, const long int timeout_lim);
static int get_serial_char(unsigned char *s, const long int timeout_us, const int timeout_lim);
int get_MB_data(const char init, unsigned char *data, const size_t num, const long int once_wait_ns, const long int timeout_us, const int timeout_lim, const int zero_lim);

#define BAUDRATE B115200

#endif
