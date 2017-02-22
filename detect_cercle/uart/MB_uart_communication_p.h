#ifndef MB_UART
#define MB_UART
static int16_t ucharToint16(unsigned char data1, unsigned char data2);
static int get_uart_input(int8_t *MB_pole,int8_t *color,float *x,float *y,float *theta);
static void publish_MBdata(const int8_t MB_pole,const int8_t color,const float x,const float y,const float theta,const ros::Publisher MBdata_pub);
static void subscribe_Jdata(const detect_cercle::Joutput&);
static int uart_output(const int8_t MB_pole,const float x,const float y,const ros::Time stamp);

#ifndef M_PI
#define M_PI 3.1415

#endif
#endif
