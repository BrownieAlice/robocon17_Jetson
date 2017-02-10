#include "ros/ros.h"
#include "detect_cercle/MBinput.h"
#include "../include/uart_setting.h"
#include "../include/uart.cpp"
#include "MB_uart_communication.h"
#include <stdint.h>

int main(int argc,char **argv){
  ros::init(argc,argv,"MB_uart_communication");
  ros::NodeHandle n;

  try_connect_MB();
  
  
}

void try_connect_MB(void){
  /*
    シリアルポートをオープンできなければ1sおきに再試行し続ける.
   */
  int open_success;
  open_success=open_serial_port(serial_dev);
  ros::Rate loop_rate(1);
  while(-1==open_success){
    ROS_INFO("fail to open serial port.");
    ROS_INFO("try every 1s to open serial port.\n");
    open_success=open_serial_port(serial_dev);
    loop_rate.sleep();
  }
}

void get_uart_input(int8_t *MB_pole,int8_t *color,float *x,float *y,float *theta,bool *success){
  if('X'==get_serial_char()){
    const unsigned char n=get_serial_char();
    *MB_pole=(int8_t)n;
    const unsigned char c=get_serial_char();
    *color=(int8_t)c;
    const unsigned char xl=get_serial_char();
    const unsigned char xh=get_serial_char();
    *x=(float)((int16_t)(xl|xh<<8))/1000;
    const unsigned char yl=get_serial_char();
    const unsigned char yh=get_serial_char();
    *y=(float)((int16_t)(yl|yh<<8))/1000;
    const unsigned char tl=get_serial_char();
    const unsigned char th=get_serial_char();
    const int theta_i=(int)((int16_t)(tl|th<<8));
    *theta=(float)theta_i/100.0/180*PI;
    const unsigned char calc_sum=n+c+xl+xh+yl+yh+tl+th;
    const unsigned char calc_val=get_serial_char();

    if(calc_val!=calc_sum){
      ROS_INFO("checksum is invalid\n");
      *success=false;
    }
    else{
      *success=true;
    }
  }
}
