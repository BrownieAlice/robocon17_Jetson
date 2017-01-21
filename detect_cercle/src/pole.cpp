#include "pole.h"
pole::pole(){
 define_posi[0]=0;
 define_posi[1]=0;
 num=0;
 esti_start=false;
}

pole::pole(float x,float y,int n){
 define_posi[0]=x;
 define_posi[1]=y;
 num=n;
 esti_start=true;
}

void pole::init_pole(float x,float y,int n){
 define_posi[0]=x;
 define_posi[1]=y;
 num=n;
}

void pole::set_esti_posi(float x,float y){
 if(esti_start){
  esti_posi[0]=0.9*esti_posi[0]+0.1*x;
  esti_posi[1]=0.9*esti_posi[1]+0.1*y;
 }
 else{
  esti_posi[0]=x;
  esti_posi[1]=y;
  esti_start=true;
 }
}

void pole::get_esti_posi(float *x,float *y){
 if(esti_start){
  (*x)=esti_posi[0];
  (*y)=esti_posi[1];
 }
 else{
  (*x)=define_posi[0];
  (*y)=define_posi[1];
 }
}
