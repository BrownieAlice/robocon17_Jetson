#include "my_def.h"
#include "detect_cercle_cuda.h"
#include <cuda.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <math_functions.h>
#include <time.h>
#include <sys/time.h>
__global__ void make_xy_data(float *ranges_d,float *xy_data_d,const int lrf_begin,const int lrf_num,const float angle_min,const float angle_increment){
  const int num = blockIdx.x*blockDim.x+threadIdx.x;
  //printf("%d\n",num);
  if(num<lrf_num){
   const float rad = (lrf_begin + num)*angle_increment + angle_min;
   //rad…LRFが取得した距離情報の角度情報
 
   float sinx, cosx;
   __sincosf(rad, &sinx, &cosx);
   //sin(rad)とcos(rad)を同時取得
 
   const float laser_x = ranges_d[num] * cosx;
   const float laser_y = ranges_d[num] * sinx;
   //LRFの距離情報をx-y変換

   xy_data_d[2*num]=laser_x;
   xy_data_d[2*num+1]=laser_y;
  }
 return;
}
		
__global__ void make_graph(float *xy_data_d, int *list_d, const int lrf_begin, const float angle_min, const float angle_increment,const float x,const float y,const int x_num,const int y_num,const float x_wid,const float y_wid,const float radius){
  const int i = blockIdx.x, j = threadIdx.x;
  //blockIdx.x…LRFの何個目の距離情報か threadIdx.x…何個目のxか

  const float rad = (lrf_begin + i)*angle_increment + angle_min;
  //rad…LRFが取得した距離情報の角度情報
  
  const float laser_x = xy_data_d[2*i];
  const float laser_y = xy_data_d[2*i+1];
  //LRFの距離情報をx-y変換
  
  const float x_range=x_num/2*x_wid+rad;
  const float y_range=y_num/2*y_wid+rad;
  const bool flag=laser_x>x-x_range&&laser_x<x+x_range&&laser_y>y-y_range&&laser_y<y+y_range;
  //LRFの距離情報が注目範囲内に収まっているか確認
  
  if(flag){
    //ブロックごとにある点の距離データを割り当て
    const float hough_x=x-x_num/2*x_wid+j*x_wid;
    //ハフ変換する際のx
    
    const float eq=radius*radius-(hough_x-laser_x)*(hough_x-laser_x);
    const float root=eq>=0?sqrt(eq):0;
    const int hough_y_1=(int)(((root+laser_y)-y+y_num/2*y_wid)/y_wid);
    const int hough_y_2=(int)(((-root+laser_y)-y+y_num/2*y_wid)/y_wid);
    //円の方程式に基づいてハフ変換する際のyを計算
    //平方根を取るときに中身が負でないことの確認をしている
    
    bool flag1=hough_y_1>=0&&hough_y_1<y_num&&eq>=0;
    bool flag2=hough_y_2>=0&&hough_y_2<y_num&&eq>=0;
    //ハフ変換で求めたyが範囲内にあるかどうか
    if(flag1)list_d[j*y_num+hough_y_1]+=1;
    if(flag2)list_d[j*y_num+hough_y_2]+=1;
    //p-q分布の完成
  }
  return;
}

__global__ void find_cercle(int *list_d, float *cercle_d, int *head_d, const int near_x,const int near_y,const int x_num,const int y_num, const int thr, const float weight){
  const int i = blockIdx.x, j = threadIdx.x;
  //blockIdx.x…何個目のxか threadIdx.x…何個目のyか
  
  const int pos = i*y_num + j;
  //p-qグラフの位置に該当する配列の番号
  
  const int val = list_d[pos];
  //注目する箇所の値
  
  const int x_start  = i - near_x <0 ? 0 : i - near_x;
  const int x_end =  i + near_x >= x_num ? x_num - 1 : i + near_x;
  const int y_start = j - near_y<0 ? 0 : j - near_y;
  const int y_end = j + near_y >= y_num ? y_num - 1 : j + near_y;
  //範囲情報を元に探索するxの範囲とyの範囲を決定
  float x_ave=0,x_count=0,y_ave=0,y_count=0;
  //xとyの重み付き平均を求めようとしている
  //_aveは重みを付けた総和、_countは重みの総和
  //weighが線形的に見る範囲の数

  int count = 0;
  for (int k = x_start; k <= x_end; k++){
    for (int l = y_start; l <= y_end; l++){
      const int com=list_d[k*y_num+l];
      const bool count_flag=(val < com)||(val==com&&pos<k*y_num+l);
      count +=(int)count_flag;
      float z=sqrt((float)((k-i)*(k-i)+(l-j)*(l-j)));
      z=z<weight?weight-z:0;
      //重みを計算
      x_ave+=com*z*k;
      x_count+=com*z;
      y_ave+=com*z*l;
      y_count+=com*z;
    }
  }
  
  bool flag = count==0;
  //極大の時true、それ以外flase
  
  flag = flag&&list_d[pos]>thr;
  //閾値を考慮
  
  if (flag){
    int my_head = head_d[0]++;
    cercle_d[my_head * 2] = x_ave/x_count;
    cercle_d[my_head * 2 + 1] = y_ave/y_count;
  }
  //flag==trueなら書き込み

}

__global__ void cuda_select_data(float *xy_data_d,bool *data_list_d,const int lrf_num,const float p,const float q,const float radius,const float rad_err){
  const int num = blockIdx.x*blockDim.x+threadIdx.x;
  if(num<lrf_num){
   const float laser_x=xy_data_d[2*num],laser_y=xy_data_d[2*num+1];
   const bool flag=abs(sqrt((laser_x-p)*(laser_x-p)+(laser_y-q)*(laser_y-q))-radius)<rad_err;
   data_list_d[num]=flag;
 }
 return;
}

void select_data(float *xy_data,float *xy_data_d, float *select_data,int *select_data_num,const int lrf_num,const float p,const float q,const float radius,const float rad_err,const int warp){
  bool *data_list,*data_list_d;
  data_list=(bool*)malloc(lrf_num*sizeof(bool));
  cudaMalloc((void**)&data_list_d,lrf_num*sizeof(bool));
  dim3 block_0((int)floor((lrf_num+warp-1)/warp),1,1);
  dim3 thread_0(warp,1,1);
  cuda_select_data<<<block_0,thread_0>>>(xy_data_d,data_list_d,lrf_num,p,q,radius,rad_err);
  cudaMemcpy(data_list,data_list_d,lrf_num*sizeof(bool),cudaMemcpyDeviceToHost);
  for(int i=0;i<lrf_num;i++){
   if(data_list[i]){
     int my_head = (*select_data_num)++;
     select_data[my_head*2]=xy_data[2*i];
     select_data[my_head*2+1]=xy_data[2*i+1];
     //printf("selectdata:%d\n",select_data_num_d[0]);
   }
 }
}

void calc_mle(float *select_datas,int select_data_num,const float rad,float *x_esti,float *y_esti,const float allow_err,const int limit_count,float *j_ml){
  float err=0,x_posi[2],y_posi[2],dif;
  x_posi[0]=*x_esti;
  y_posi[0]=*y_esti;
  int count=0;
  do{
    //printf("num:%d\n",select_data_num);
    float j_a=0,j_b=0;
    *j_ml=0;
    for(int i=0;i<select_data_num;i++){
      const float x_a=select_datas[2*i],y_a=select_datas[2*i+1];
      const float val=x_a*x_a+y_a*y_a-2*(*x_esti)*x_a-2*(*y_esti)*y_a+(*x_esti)*(*x_esti)+(*y_esti)*(*y_esti);
      *j_ml+=(val-rad*rad)*(val-rad*rad)/val;
      j_a+=(2*(val-rad*rad)*(2*(*x_esti)-2*x_a)-(val-rad*rad)*(val-rad*rad)*(2*(*x_esti)-2*x_a))/(val*val);
      j_b+=(2*(val-rad*rad)*(2*(*y_esti)-2*y_a)-(val-rad*rad)*(val-rad*rad)*(2*(*y_esti)-2*y_a))/(val*val);
    }
    err=j_a*j_a+j_b*j_b;
    if(err>allow_err){
      j_a=j_a/sqrt(err)*allow_err;
      j_b=j_b/sqrt(err)*allow_err;
    }
    x_posi[1]=x_posi[0];
    y_posi[1]=y_posi[0];
    x_posi[0]=*x_esti;
    y_posi[0]=*y_esti;
    *x_esti-=j_a;
    *y_esti-=j_b;
    //printf("mle x:%f,y:%f,j_a:%f,j_b:%f\n",*x_esti,*y_esti,j_a,j_b);
    dif=(x_posi[1]-*x_esti)*(x_posi[1]-*x_esti)+(y_posi[1]-*y_esti)*(y_posi[1]-*y_esti);
  }while(allow_err*allow_err<err&&count++<limit_count&&!(dif<allow_err*allow_err&&count>1));
}

void mle(float *select_datas,int select_data_num,const float rad,float *x_esti,float *y_esti,const float allow_err,const int limit_count){
  //最尤推定法を実行
  /*
    select_datas…点列情報
    select_data_num…点列の個数
    rad…円の半径
    x_esti/y_esti…求めた座標を格納する先
    allow_err…許容誤差
   */

  float j_ml;
  calc_mle(select_datas,select_data_num,rad,x_esti,y_esti,allow_err,limit_count,&j_ml);
  float len=sqrt((*x_esti)*(*x_esti)+(*y_esti)*(*y_esti));
  float pro_x=(*x_esti)+(*x_esti)/len*rad,pro_y=(*y_esti)+(*y_esti)/len*rad;
  
  float j_ml2=0;
  for(int i=0;i<select_data_num;i++){
    const float x_a=select_datas[2*i],y_a=select_datas[2*i+1];
    const float val=x_a*x_a+y_a*y_a-2*(*x_esti)*x_a-2*(*y_esti)*y_a+(*x_esti)*(*x_esti)+(*y_esti)*(*y_esti);
    j_ml2+=(val-rad*rad)*(val-rad*rad)/val;
  }
  if(j_ml>j_ml2){
   *x_esti=pro_x;
   *y_esti=pro_y;
   calc_mle(select_datas,select_data_num,rad,x_esti,y_esti,allow_err,limit_count,&j_ml);
  }
}

void detect_cercle_cuda(float *ranges,const int lrf_num,const int lrf_begin,const float angle_min,const float angle_increment,const float x_wid,const float y_wid,const int x_num,const int y_num,const int near_x,const int near_y,const int thr,const int thr2,float *p,float *q,bool *calc_flag,const float rad,const float rad_err1,const float rad_err2,const float rad_err3,const float allow_err1,const float allow_err2,const float weight, const int warp,const int limit_count){
  /*
    struct timeval start,end;
    時間計測用
    以下使い方
    gettimeofday(&start,NULL);
    hogehoge;
    gettimeofday(&end,NULL);
    printf("time:%d[ns]\n", end.tv_usec - start.tv_usec);

    float elapsed_time_ms=0.0f;
    cudaEvent_t start,stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start,0);
    hoge<<<,>>>()
    cudaEventRecord(stop,0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsed_time_ms,start,stop);
    printf("time:%8.2f ms\n",elapsed_time_ms);
    cudaEventDestory(start);
    cudaEventDestory(end);
  */
  //printf("num:%d,theta_num:%d,rho_num:%d\n", num, theta_num, rho_num);
  *calc_flag=false;

  if (lrf_num>1024){
   printf("%dreturn\n",lrf_num);
   return;
  }
  //gpuの可能並列化数を超えていたらreturn

  float *ranges_d;
  cudaMalloc((void**)&ranges_d, lrf_num*sizeof(float));
  cudaMemcpy(ranges_d, ranges, lrf_num*sizeof(float), cudaMemcpyHostToDevice);
  //device用のLRF距離情報

  float *xy_data,*xy_data_d;
  xy_data=(float*)malloc(2*lrf_num*sizeof(float));
  cudaMalloc((void**)&xy_data_d,2*lrf_num*sizeof(float));
  
  dim3 block_0((int)floor((lrf_num+warp-1)/warp),1,1);
  dim3 thread_0(warp,1,1);

  make_xy_data<<<block_0,thread_0>>>(ranges_d,xy_data_d,lrf_begin,lrf_num,angle_min,angle_increment);

  cudaMemcpy(xy_data,xy_data_d,2*lrf_num*sizeof(float),cudaMemcpyDeviceToHost);
  int *list;
  list = (int*)malloc(x_num*y_num*sizeof(int));
  if (list == NULL)return;
  //p-qグラフ

  for (int i = 0; i < x_num*y_num; i++)list[i] = 0;
  //p-qグラフ初期化


  int *list_d;
  cudaMalloc((void**)&list_d, x_num*y_num*sizeof(int));
  cudaMemcpy(list_d, list, x_num*y_num * sizeof(int), cudaMemcpyHostToDevice);

  //デバイス用p-qグラフ
  
  dim3 blocks_1(lrf_num, 1, 1);
  dim3 threads_1(x_num,1,1);
  //並列化

  make_graph<<<blocks_1,threads_1>>>(xy_data_d,list_d,lrf_begin,angle_min,angle_increment,*p,*q,x_num,y_num,x_wid,y_wid,rad);

  //p-qグラフ作成
  /*
    //p-qグラフを表示
    cudaMemcpy(list, list_d, x_num*y_num * sizeof(int), cudaMemcpyDeviceToHost);
    //list情報をホストに

    for (int i = 0; i < x_num; i++){
    printf("x=%d:", i);
    for (int j = 0; j < y_num; j++){
    if (list[i*y_num + j] >= 2)printf("y=%d,%d ",j, list[i*y_num + j]);
    }
    printf("\n");
    }
  */  


  float *cercle;
  cercle = (float*)malloc(x_num*y_num* 2* sizeof(float));
  //円情報

  /*
  for (int i = 0; i < theta_num*rho_num*2; i++)line[i] = 0;
  //直線情報初期化
  */

  float *cercle_d;
  cudaMalloc((void**)&cercle_d, x_num*y_num*2 * sizeof(float));
  cudaMemcpy(cercle_d, cercle, x_num*y_num *2* sizeof(float), cudaMemcpyHostToDevice);
  //デバイス用直線情報

  int head[1] = { 0 };
  int *head_d;
  cudaMalloc((void**)&head_d, sizeof(int));
  cudaMemcpy(head_d, head, sizeof(int), cudaMemcpyHostToDevice);
  //配列書き込み位置取得用グローバルメモリ上変数

  dim3 blocks_2(x_num, 1, 1);
  dim3 threads_2(y_num, 1,1);

  find_cercle<<<blocks_2,threads_2>>>(list_d,cercle_d,head_d,near_x,near_y,x_num,y_num,thr,weight);

  cudaMemcpy(cercle, cercle_d, x_num*y_num*2 * sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(head, head_d, sizeof(int), cudaMemcpyDeviceToHost);
  
  float *find_cercles;
  find_cercles = (float*)malloc(head[0]*2*sizeof(float));
  memcpy(find_cercles, cercle, head[0]*2*sizeof(float));
  //不必要にでかいcercleを使わず必要数だけ保管するfind_cercle配列を作る

  printf("head:%d\n", head[0]);
  float x_esti,y_esti;
  if(head[0]!=0){
    const float x_b=*p,y_b=*q;
    x_esti=(float)(find_cercles[0]-x_num/2)* x_wid+x_b;
    y_esti=(float)(find_cercles[1]-y_num/2)* y_wid+y_b;
    float diff=(x_b-x_esti)*(x_b-x_esti)+(y_b-y_esti)*(y_b-y_esti);
    for (int i = 1; i < head[0]; i++){
      const float now_x=(float)(find_cercles[2 * i]-x_num/2)* x_wid+x_b;
      const float now_y=(float)(find_cercles[2 * i+1]-y_num/2)* y_wid+y_b;
      const float now_diff=(x_b-now_x)*(x_b-now_x)+(y_b-now_y)*(y_b-now_y);
      if(diff>now_diff){
	x_esti=now_x;
	y_esti=now_y;
	diff=now_diff;
      }
    }
  }
  //予想のp-qに一番近いものを選んでいる

  //for(int i=0;i<head[0];i++)printf("x:%f\ny:%f\n", (float)(find_cercles[2 * i]-x_num/2)* x_wid+x, (float)(find_cercles[2 * i+1]-y_num/2)* y_wid+y);
  //printf("hough-x:%f\nhough-y:%f\n",x_esti,y_esti); 
  //printf("before_esti_x:%f\nbefore_esti_y:%f\n",*p,*q);

  if(head[0]!=0){
    //ここから2回目の処理
    float *select_datas;
    select_datas=(float*)malloc(lrf_num*2*sizeof(float));
    //選別したlrfのデータを保管

    int select_data_num=0;

    select_data(xy_data,xy_data_d,select_datas,&select_data_num,lrf_num,x_esti,y_esti,rad,rad_err1,warp);
    //lrfデータを選別

    printf("select:%d\n",select_data_num);
    mle(select_datas,select_data_num,rad,&x_esti,&y_esti,allow_err1,limit_count);
    //最尤推定を実行
    select_data_num=0;
    select_data(xy_data,xy_data_d,select_datas,&select_data_num,lrf_num,x_esti,y_esti,rad,rad_err2,warp);

    //lrfデータを選別
    mle(select_datas,select_data_num,rad,&x_esti,&y_esti,allow_err2,limit_count);
    //二回目の最尤推定を実行

    select_data_num=0;
    select_data(xy_data,xy_data_d,select_datas,&select_data_num,lrf_num,x_esti,y_esti,rad,rad_err3,warp);
    printf("select:%d\n",select_data_num);
     if(select_data_num>thr2){
      *p=x_esti;
      *q=y_esti;
      *calc_flag=true;
      printf("p=%f\nq=%f\n",*p,*q);
     }
    free(select_datas);
    }

  free(list);
  free(cercle);
  cudaFree(ranges_d);
  cudaFree(list_d);
  cudaFree(cercle_d);
  cudaFree(head_d);
  free(find_cercles);
  
  //printf("time:%f[ms]\n", (float)(end - start)/CLOCKS_PER_SEC*1000);
  //printf("\n");
  return;

}
