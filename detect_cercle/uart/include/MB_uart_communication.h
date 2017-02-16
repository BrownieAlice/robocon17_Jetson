#ifndef MB_UART
#define MB_UART

int try_connect_MB(void);
int get_uart_input(int8_t *MB_pole,int8_t *color,float *x,float *y,float *theta);
void publish_MBdata(const int8_t MB_pole,const int8_t color,const float x,const float y,const float theta,const ros::Publisher MBdata_pub);
void subscribe_Jdata(const detect_cercle::Joutput&);
int uart_output(const int8_t MB_pole,const float x,const float y,const ros::Time stamp);

#ifndef M_PI
#define M_PI 3.1415

#endif
#endif
