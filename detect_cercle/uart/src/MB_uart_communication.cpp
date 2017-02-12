#include "ros/ros.h"
#include "detect_cercle/MBinput.h"
#include "detect_cercle/Joutput.h"
#include "../include/uart_setting.h"
#include "../include/uart.cpp"
#include "MB_uart_communication.h"
#include <stdint.h>

namespace{
  bool uart_flag=false;
}

int main(int argc,char **argv){
  ros::init(argc,argv,"MB_uart_communication");
  ros::NodeHandle n;
  ros::Publisher MBdata_pub = n.advertise<detect_cercle::MBinput>("MBdata", 5);
  // MBから受け取ったデータをメイン処理プロセスに引き渡す.
  ros::Subscriber Jdata_sub = n.subscribe("Jdata",5,subscribe_Jdata);
  // 処理されたデータをMBに引き渡すためにこちらに渡す.

  uart_flag=try_connect_MB();
  // uart接続が正しく出来ているかどうかを格納する.

  while(ros::ok()){
    // メインループ

    if(false==uart_flag){
      // uart接続が切れていたら接続し直す.
      ROS_INFO("MB is disconnected.\n");
      uart_flag=try_connect_MB();
    }
    ros::Rate loop_rate(main_loop_hz);
    // ループ周期を規定

    int8_t MB_pole=0,color=0;
    float x=0,y=0,theta=0;
    bool success=false;
    // MBからデータを受け取るための変数.

    get_uart_input(&MB_pole,&color,&x,&y,&theta,&success);
    // MBからデータを受け取る.
    publish_MBdata(MB_pole,color,x,y,theta,success,MBdata_pub);
    // MBからのデータをメイン処理プロセスに引き渡す.

    ros::spinOnce();
    // 処理されたデータを受け取るコールバック関数の呼び出しが可能なら行われる.
    //loop_rate.sleep();
    }
}

bool try_connect_MB(void){
  /*
    シリアルポートをオープンできなければ周波数 connect_loop_hz で再試行し続ける.
   */
  int open_success;
  // シリアルポートが開けたかどうかを格納する.

  open_success=open_serial_port(serial_dev);
  // シリアルポートオープン

  ros::Rate loop_rate(connect_loop_hz);
  // ループ周期を規定

  while(-1==open_success){
    // シリアルポートが開けなかった時のループ.

    ROS_INFO("fail to open serial port.");
    ROS_INFO("try %dHz to open serial port.\n",connect_loop_hz);
    open_success=open_serial_port(serial_dev);
    loop_rate.sleep();
    // 一定周期で再接続を試み続ける.

  }
  return true;
  // ここに来れていたら必ずポートをオープンで来てる.
}

void get_uart_input(int8_t *MB_pole,int8_t *color,float *x,float *y,float *theta,bool *success){
  //  MBからのデータを受け取る.

  if('X'==get_serial_char()){
    // 最初の通信文字を検知したら処理を開始.

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
    *theta=(float)theta_i/100.0/180*M_PI;

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

void publish_MBdata(int8_t MB_pole,int8_t color,float x,float y,float theta,bool success,ros::Publisher MBdata_pub){
  if(success){
    detect_cercle::MBinput MBinfo;

    MBinfo.MB_pole=MB_pole;
    MBinfo.color=color;
    MBinfo.x=x;
    MBinfo.y=y;
    MBinfo.theta=theta;
    MBinfo.get_time=ros::Time::now();

    MBdata_pub.publish(MBinfo);
    ROS_INFO("MBdata::MB_pole:%d,color:%d,x:%f,y:%f,theta:%f\n",(int)MB_pole,(int)color,x,y,theta);
  }
}

void subscribe_Jdata(const detect_cercle::Joutput& Jdata){
  uart_flag=uart_output(Jdata.MB_pole,Jdata.x,Jdata.y);
}

bool uart_output(int8_t MB_pole,float x,float y){
    int16_t send_x=(int)(x*1000);
    int16_t send_y=(int)(y*1000);
    char X_L=(char)send_x;
    char X_H=(char)(send_x>>8);
    char Y_L=(char)send_y;
    char Y_H=(char)(send_y>>8);

    int i=0;

    i+=put_serial_char('X');
    i+=put_serial_char(X_H);
    i+=put_serial_char(X_L);
    i+=put_serial_char(Y_H);
    i+=put_serial_char(Y_L);
    i+=put_serial_char((char)MB_pole+X_H+X_L+Y_H+Y_L);
    ROS_INFO("Jdata::MB_pole:%d,x:%f,y:%f",(int)MB_pole,x,y);
    return i==0?true:false;
}
