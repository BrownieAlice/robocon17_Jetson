#ifndef _MB_UART_
#define _MB_UART_

bool try_connect_MB(void);
void get_uart_input(int8_t *,int8_t *,float *,float *,float *,bool *);
void publish_MBdata(int8_t,int8_t,float,float,float,bool,ros::Publisher);
void subscribe_Jdata(const detect_cercle::Joutput&);
bool uart_output(int8_t,float,float);
const unsigned int main_loop_hz=25;
const unsigned int connect_loop_hz=50;
#ifndef M_PI
#define M_PI 3.1415
#endif
#endif
