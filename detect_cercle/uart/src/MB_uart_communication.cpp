#include "ros/ros.h"
#include "detect_cercle/MBinput.h"
#include "detect_cercle/Joutput.h"
#include "../include/uart.cpp"
#include "MB_uart_communication.h"
#include <stdint.h>

namespace{
  bool uart_flag=false;
  // uart接続ができているかどうか.
}

int main(int argc,char **argv){
  ros::init(argc,argv,"MB_uart_communication");
  ros::NodeHandle n;
  ros::Publisher MBdata_pub = n.advertise<detect_cercle::MBinput>("MBdata", 5);
  // MBから受け取ったデータをメイン処理プロセスに引き渡す.
  ros::Subscriber Jdata_sub = n.subscribe("Jdata",5,subscribe_Jdata);
  // 処理されたデータをMBに引き渡すためにこちらに渡す.

  try_connect_MB();
  uart_flag=true;
  // uart接続が正しく出来ているかどうかを格納する.

  while(ros::ok()){
    // メインループ

    if(false==uart_flag){
      // uart接続が切れていたら接続し直す.
      ROS_INFO("MB is disconnected.\n");
      try_connect_MB();
      uart_flag=true;
    }
    ros::Rate loop_rate(main_loop_hz);
    // ループ周期を規定

    int8_t MB_pole=0,color=0;
    float x=0,y=0,theta=0;
    int success;
    // MBからデータを受け取るための変数.

    success=get_uart_input(&MB_pole,&color,&x,&y,&theta);
    // MBからデータを受け取る.
    if(1==success){
      publish_MBdata(MB_pole,color,x,y,theta,success,MBdata_pub);
      // MBからのデータをメイン処理プロセスに引き渡す.
    }else if(-1==success){
      uart_flag=false;
    }

    ros::spinOnce();
    // 処理されたデータを受け取るコールバック関数の呼び出しが可能なら行われる.
    //loop_rate.sleep();
    }
}

int try_connect_MB(void){
  /*
    シリアルポートをオープンできなければ connect_loop_ns [ns]で再試行し続ける.
   */

   continue_connect_uart(connect_loop_ns,serial_dev);
   return(0);
  // ここに来れていたら必ずポートをオープンで来てる.
}

int get_uart_input(int8_t *MB_pole,int8_t *color,float *x,float *y,float *theta){
  //  MBからのデータを受け取る.
  int flag;
  unsigned char data[8];
  flag=get_MB_data('X',data,sizeof(data),5000);

  if(1==flag){
    *MB_pole=(int8_t)data[0];
    *color=(int8_t)data[1];
    *x=(float)((int16_t)(data[2]|data[3]<<8))/1000;
    *y=(float)((int16_t)(data[4]|data[5]<<8))/1000;
    const int theta_i=(int)((int16_t)(data[6]|data[7]<<8));
    *theta=(float)theta_i/100.0/180*M_PI;
    return 1;
  }else{
    return flag;
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
