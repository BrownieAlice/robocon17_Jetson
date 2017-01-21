#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <cstdlib>
#include "my_def.h"
#include "detect_cercle_cuda.h"
#include "pole.cpp"
#include "uart.cpp"

namespace poles{
 pole polelist[8];
}

namespace{
  const float rad=0.26/2,rad_err1=0.1,rad_err2=0.05,rad_err3=0.01,allow_err1=0.01,allow_err2=0.005;
  const float x_wid=0.01,y_wid=0.01;
  const int x_num=256,y_num=256;
  const int lrf_begin=29,lrf_end=LRF_DATA-29,lrf_num=lrf_end-lrf_begin+1;
  const int near_x=6,near_y=6,thr=2,thr2=6;
  const float weight=5;
  const int warp=32,limit_count=50;
  float p=0,q=0;
  int pole_number=0;
  bool calc_flag=false;
  /*
   rad…ポールの半径
   rad_err1…1回目のハフ変換で求めた円の座標からの許容するズレ
   rad_err2…1回目の最尤推定法で求めた園の座標から許容するズレ
   allow_err1…1回目の最尤推定法で許容する誤差
   allow_err2…2回目の最尤推定法で許容する誤差
   x,y…今推定しているポールの座標
   x_wid,y_wid…ハフ変換する際の間隔
   x_num,y_num…ハフ変換する際の個数
   lrf_begin/end…lrfのデータを見始める/見終わる個数
   near_x,near_y…近傍とみなす範囲
   thr…閾値
   weight…重み付き平均を求める際の範囲
  */
}

void Laser_Callback(const sensor_msgs::LaserScan& msg);
void init_poles(pole *polelist);
void uart_calc();
void get_uart_input(int *MB_pole,char *color,float *x,float *y,float *theta,bool *success);
void put_uart_output(int MB_pole,float x,float y);

int main( int argc, char** argv )
{
  init_poles(poles::polelist);

  ros::init(argc, argv, "detect_cercle");
  ros::NodeHandle node;
  ros::Subscriber laser;
  laser = node.subscribe("scan",2,Laser_Callback);

  const char *serial_dev=DEV;
  open_serial_port(serial_dev);
  while(ros::ok()){
   uart_calc();
  }
  close_serial_port();
  return 0;
}


void Laser_Callback(const sensor_msgs::LaserScan& msg){
  const int start=ros::Time::now().nsec;

  float *ranges;
  ranges = (float*)malloc(lrf_num*sizeof(float));
  if (ranges == NULL)return;

  for(int i=lrf_begin;i<=lrf_end;i++)ranges[i-lrf_begin]=(float)msg.ranges[i];

  detect_cercle_cuda(ranges,lrf_num,lrf_begin,msg.angle_min,msg.angle_increment,x_wid,y_wid,x_num,y_num,near_x,near_y,thr,thr2,&p,&q,&calc_flag,rad,rad_err1,rad_err2,rad_err3,allow_err1,allow_err2,weight,warp,limit_count);

  if(calc_flag){
   //poles::polelist[pole_number].set_esti_posi(p,q);
  }

  const int end=ros::Time::now().nsec;
  //printf("time:%d\n",(end-start)/1000000);
}


void init_poles(pole *polelist){
 polelist[0].init_pole(7.075,3.5,0);
 polelist[1].init_pole(7.075,5.5,1);
 polelist[2].init_pole(7.075,7.5,2);
 polelist[3].init_pole(7.075,9.5,3);
 polelist[4].init_pole(7.075,11.5,4);
 polelist[5].init_pole(4.075,7.5,5);
 polelist[6].init_pole(10.075,7.5,6);
}

void uart_calc(){

    int MB_pole;
    char color;
    float x,y,theta;
    bool success;
    get_uart_input(&MB_pole,&color,&x,&y,&theta,&success);

    const float MB_esti_x=x,MB_esti_y=y;

    if(!success){
	printf("checksum invalid\n");
  	return;
    }
    printf("pole:%d,color:%c,x:%f,y:%f,theta:%f\n",MB_pole,color,x,y,theta);

    poles::polelist[MB_pole].get_esti_posi(&p,&q);
    if(color=='R'){
     p-=y;
     q-=x;
     const float pole_rad=atan2(q,p);
     const float pole_range=sqrt(p*p+q*q);
     p=pole_range*cos(pole_rad-theta);
     q=pole_range*sin(pole_rad-theta);
    }else if(color=='B'){
     p=14.15-p-y;
     q=-(q-x);
     const float pole_rad=atan2(q,p);

	//printf("p:%f,q:%f,pole_rad:%f\n",p,q,pole_rad/PI*180);
     const float pole_range=sqrt(p*p+q*q);
     p=pole_range*cos(pole_rad-theta);
     q=pole_range*sin(pole_rad-theta);
    }
    pole_number=MB_pole;

    printf("esti_p:%f,esti_q:%f\n",p,q);
    ros::spinOnce();
    printf("calc_p:%f,calc_q:%f\n",p,q);

    const float pole_rad=atan2(q,p);
    const float pole_range=sqrt(p*p+q*q);
    float p=pole_range*cos(pole_rad+theta);
    float q=pole_range*sin(pole_rad+theta);
    printf("calc2_x:%f,calc2_y:%f\n",p,q);
    if(color=='R'){
     p+=y;
     q+=x;
    }else{
     p=14.15-p-y;
     q=-(q-x);
    }
    printf("poleset_p:%f,poleset_q:%f\n",p,q);

    float pole_abs_x,pole_abs_y;

    if(calc_flag){
     //poles::polelist[MB_pole].set_esti_posi(p,q);
     poles::polelist[MB_pole].get_esti_posi(&pole_abs_x,&pole_abs_y);

     float jetson_x,jetson_y;
     if(color=='R'){
      jetson_x=q-pole_abs_y+MB_esti_x;
      jetson_y=p-pole_abs_x+MB_esti_y;
     }else{
      jetson_x=pole_abs_y-q+MB_esti_x;
      jetson_y=pole_abs_x-p+MB_esti_y;
     }

     put_uart_output(MB_pole,jetson_y,jetson_x);
    }
    return;
}

void get_uart_input(int *MB_pole,char *color,float *x,float *y,float *theta,bool *success){
    char s;
    do{
     s=get_serial_char();
    }while(s!='X');
    unsigned char n=get_serial_char();
    *MB_pole=(int)n;
    *color=get_serial_char();
    unsigned char xl=get_serial_char();
    unsigned char xh=get_serial_char();
    *x=(float)((short)(xl|xh<<8))/1000;
    unsigned char yl=get_serial_char();
    unsigned char yh=get_serial_char();
    *y=(float)((short)(yl|yh<<8))/1000;
    unsigned char tl=get_serial_char();
    unsigned char th=get_serial_char();
    int theta_i=(int)((short)(tl|th<<8));
    *theta=(float)theta_i/100.0/180*PI;
    unsigned char calc_sum=n+(*color)+xl+xh+yl+yh+tl+th;
    unsigned char calc_val=get_serial_char();

    if(calc_val!=calc_sum)*success=false;
    else *success=true;
    return;
}

void put_uart_output(int MB_pole,float p,float q){
    put_serial_char('X');
    put_serial_char((char)MB_pole);
    int send_x=(int)(q*1000);
    int send_y=(int)(p*1000);
    char X_L=(char)send_x;
    char X_H=(char)(send_x>>8);
    char Y_L=(char)send_y;
    char Y_H=(char)(send_y>>8);
    put_serial_char(X_H);
    put_serial_char(X_L);
    put_serial_char(Y_H);
    put_serial_char(Y_L);
    put_serial_char((char)MB_pole+X_H+X_L+Y_H+Y_L); 

    printf("send_x:%f,send_y:%f\n",(float)(short)(X_H<<8|X_L)/1000,(float)(short)(Y_H<<8|Y_L)/1000);
}
