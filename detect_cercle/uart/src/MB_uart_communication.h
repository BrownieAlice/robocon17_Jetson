#ifndef MB_UART
#define MB_UART

int try_connect_MB(void);
int get_uart_input(int8_t *,int8_t *,float *,float *,float *);
void publish_MBdata(int8_t,int8_t,float,float,float,bool,ros::Publisher);
void subscribe_Jdata(const detect_cercle::Joutput&);
bool uart_output(int8_t,float,float);

#ifndef M_PI
#define M_PI 3.1415

#endif
#endif
