#include <cuda.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <math_functions.h>
#include <time.h>
#include <sys/time.h>
#include <vector>
#include <iostream>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/sort.h>
#include "../include/detect_cercle_cuda_p.hpp"

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

int detect_cercle_cuda(const std::vector<float>& host_ranges, const int lrf_num, const float angle_min, const float angle_increment, const hough_param_str *hough_param, const int thr2, float *p, float *q, const float rad, const float rad_err1, const float rad_err2, const float rad_err3, const float allow_err1, const float allow_err2, const int warp, const int limit_count){
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

  int calc_flag = -1;

  if (lrf_num > 1024)
  {
    std::cout << "too many number to see." << std::endl;
    return calc_flag;
  }
  //gpuの可能並列化数を超えていたらreturn

  const size_t ranges_num = host_ranges.size();
  if (ranges_num < lrf_num)
  {
    // LRFの距離データの数が少なすぎた時.
    std::cout << "few number of lrf data." << std::endl;
    return calc_flag;
  }

  thrust::device_vector<float> device_ranges(host_ranges.size());
  // デバイス用の距離データ格納用のvector.
  thrust::device_vector<float> device_xy_data(2 * lrf_num);
  // デバイス用のx-yデータ格納用のvector.
  thrust::host_vector<float> host_xy_data(2 * lrf_num);
  // ホスト用のx-yデータ格納用のvector.
  thrust::copy(host_ranges.begin(), host_ranges.end(), device_ranges.begin());
  // デバイスに距離情報をコピー.

  dim3 block_0((int)floor((lrf_num+warp-1)/warp),1,1);
  dim3 thread_0(warp,1,1);
  // 並列化の大きさを設定.

  const int lrf_offset = (ranges_num - lrf_num) / 2;
  // lrfデータの中央部分のみを見るため,そのオフセットを指定.

  make_xy_data<<<block_0,thread_0>>>(
    (float *)thrust::raw_pointer_cast(device_ranges.data()),
    (float *) thrust::raw_pointer_cast(device_xy_data.data()),
    lrf_offset, lrf_num, angle_min, angle_increment);
  // x-yデータを作成.

  thrust::copy(device_xy_data.begin(), device_xy_data.end(), host_xy_data.begin());
  // ホストにx-yデータをコピー.

  float *xy_data, *xy_data_d;
  xy_data = (float *) thrust::raw_pointer_cast(host_xy_data.data());
  xy_data_d = (float *) thrust::raw_pointer_cast(device_xy_data.data());

  const float x_range = hough_param->x_num / 2 * hough_param->x_wid;
  const float y_range = hough_param->y_num / 2 * hough_param->y_wid;
  // ハフ変換時に見る範囲.

  thrust::device_vector<int> device_hough_list(hough_param->x_num * hough_param->y_num, 0);
  // デバイス用のハフ変換データ格納用のvector.

  dim3 blocks_1(lrf_num, 1, 1);
  dim3 threads_1(hough_param->x_num,1,1);
  // 並列化.

  make_hough_graph<<<blocks_1,threads_1>>>((float *)thrust::raw_pointer_cast(device_xy_data.data()),
  (int *)thrust::raw_pointer_cast(device_hough_list.data()),
  *p, *q,
  hough_param->x_num, hough_param->y_num,
  hough_param->x_wid, hough_param->y_wid,
  x_range, y_range, rad);

  // print_hough_gragh(device_hough_list, x_num, y_num);
  // ハフ変換グラフを表示.

  thrust::device_vector<float> device_cercle(2 * hough_param->x_num * hough_param->y_num);
  // デバイス用の検出円格納用のvector.
  thrust::host_vector<float> host_cercle(2 * hough_param->x_num * hough_param->y_num);
  // ホスト用の検出円格納用のvector.

  thrust::device_vector<int> device_head(1);
  // デバイス用の検出円数格納用のvector.
  thrust::host_vector<int> host_head(1);
  // ホスト用の検出円数格納用のvector.

  host_head[0] = 0;

  thrust::copy(host_head.begin(), host_head.end(), device_head.begin());
  // デバイスに検出円数をコピー.

  dim3 blocks_2(hough_param->x_num, 1, 1);
  dim3 threads_2(hough_param->y_num, 1,1);

  find_cercle<<<blocks_2,threads_2>>>((int *)thrust::raw_pointer_cast(device_hough_list.data()),
  (float *)thrust::raw_pointer_cast(device_cercle.data()),
  (int *)thrust::raw_pointer_cast(device_head.data()),
  hough_param->near_x, hough_param->near_y,
  hough_param->x_num, hough_param->y_num,
  hough_param->thr, hough_param->weight);
  // 円検出.

  thrust::copy(device_cercle.begin(), device_cercle.end(), host_cercle.begin());
  // ホストにx-yデータをコピー.
  thrust::copy(device_head.begin(), device_head.end(), host_head.begin());
  // ホストにx-yデータをコピー.


  float x_esti, y_esti;
  if(host_head[0] != 0){
    const float x_b = *p, y_b = *q;
    // x-yの中心位置(オフセット).
    x_esti = (float)(host_cercle[0] - hough_param->x_num / 2) * hough_param->x_wid + x_b;
    y_esti = (float)(host_cercle[1] - hough_param->y_num / 2) * hough_param->y_wid + y_b;
    // 検出したx-yデータ.

    float diff = (x_b-x_esti) * (x_b-x_esti) + (y_b-y_esti) * (y_b-y_esti);
    // 中心位置からのズレ.

    for (int i = 1; i < host_head[0]; i++){
      const float now_x = (float)(host_cercle[2 * i] - hough_param->x_num / 2) * hough_param->x_wid + x_b;
      const float now_y = (float)(host_cercle[2 * i + 1] - hough_param->y_num / 2) * hough_param->y_wid + y_b;
      const float now_diff = (x_b-now_x) * (x_b-now_x) + (y_b-now_y) * (y_b-now_y);
      if (diff > now_diff)
      {
        x_esti = now_x;
	      y_esti = now_y;
	      diff = now_diff;
      }
    }
  }
  // 予想の位置に一番近いものを選んでいる.

  /*
  if (host_head[0] != 0){
    printf("%d\n",host_head[0]);
    *p = x_esti;
    *q = y_esti;
    calc_flag = 0;
    return calc_flag;
  }
  */

  if (host_head[0] != 0){
    //ここから2回目の処理
    float *select_datas;
    select_datas=(float*)malloc(lrf_num*2*sizeof(float));
    //選別したlrfのデータを保管

    int select_data_num=0;

    select_data(xy_data,xy_data_d,select_datas,&select_data_num,lrf_num,x_esti,y_esti,rad,rad_err1,warp);
    //lrfデータを選別

    //printf("select:%d\n",select_data_num);
    mle(select_datas,select_data_num,rad,&x_esti,&y_esti,allow_err1,limit_count);
    //最尤推定を実行
    select_data_num=0;
    select_data(xy_data,xy_data_d,select_datas,&select_data_num,lrf_num,x_esti,y_esti,rad,rad_err2,warp);

    //lrfデータを選別
    mle(select_datas,select_data_num,rad,&x_esti,&y_esti,allow_err2,limit_count);
    //二回目の最尤推定を実行

    select_data_num=0;
    select_data(xy_data,xy_data_d,select_datas,&select_data_num,lrf_num,x_esti,y_esti,rad,rad_err3,warp);
    //printf("select:%d\n",select_data_num);
     if(select_data_num>thr2){
      *p = x_esti;
      *q = y_esti;
      calc_flag = 0;
      // printf("p=%f\nq=%f\n",*p,*q);
     }
    free(select_datas);
    }

  //printf("time:%f[ms]\n", (float)(end - start)/CLOCKS_PER_SEC*1000);
  //printf("\n");
  return calc_flag;

}

__global__ void make_xy_data(const float *device_ranges, float *device_xy_data, const int lrf_offset, const int lrf_num, const float angle_min, const float angle_increment)
{
  // 距離データをx-yデータに変換する.

  const int num = blockIdx.x * blockDim.x + threadIdx.x;
  // 見るべき配列の値.

  if (lrf_num < num)
  {
    // 配列外参照.
    return;
  }

  const float rad = (lrf_offset + num) * angle_increment + angle_min;
  // rad…LRFが取得した距離情報の角度情報.

  float sinx, cosx;
  __sincosf(rad, &sinx, &cosx);
  // sin(rad)とcos(rad)を同時取得.

  const float range =  device_ranges[lrf_offset + num];

  device_xy_data[2 * num] = range * cosx;
  device_xy_data[2 * num + 1] = range * sinx;
  // LRFの距離情報をx-y変換.
}

__global__ void make_hough_graph(float *device_xy_data, int *device_hough_list, const float center_x, const float center_y, const int x_num, const int y_num, const float x_wid, const float y_wid, const float x_range, const float y_range, const float radius){
  // x-yデータからハフ変換したデータ列を作る.

  const int i = blockIdx.x, j = threadIdx.x;
  // blockIdx.x…LRFの何個目の距離情報か threadIdx.x…何個目のxか.

  const float laser_x = device_xy_data[2 * i];
  const float laser_y = device_xy_data[2 * i + 1];
  // LRFの距離情報をx-y変換した情報.

  const bool flag = laser_x > center_x - x_range && laser_x < center_x + x_range && laser_y > center_y - y_range && laser_y < center_y + y_range;
  // LRFの距離情報が注目範囲内に収まっているか確認.
  if (false == flag)
  {
    // 注目範囲外の点だった.
    return;
  }

  const float hough_x = center_x - x_range + j * x_wid;
  // ハフ変換する際のx.

  const float diff_x = (hough_x - laser_x);
  // ハフ変換のxと座標点との差.

  const float eq = radius * radius - diff_x * diff_x;
  // 円としてみた時のyの座標の2乗.


  if (eq < 0)
  {
    // 円として見れない状況.
    return;
  }


  const float root = sqrt(eq);
  const int hough_y_1=(int)(((root+laser_y) - (center_y - y_range)) / y_wid);
  const int hough_y_2=(int)(((-root+laser_y) - (center_y - y_range)) / y_wid);
  // 円の方程式に基づいてハフ変換する際のyを計算.
  // 平方根を取るときに中身が負でないことの確認をしている.

  bool flag1 = hough_y_1 >= 0 && hough_y_1 < y_num;
  bool flag2 = hough_y_2 >= 0 && hough_y_2 < y_num;
  // ハフ変換で求めたyが範囲内にあるかどうか.

  if (flag1)
  {
    device_hough_list[j*y_num+hough_y_1]++;
  }
  if (flag2)
  {
    device_hough_list[j*y_num+hough_y_2]++;
  }
  // ハフ変換のグラフ完成.
}

static void print_hough_gragh(thrust::device_vector<int> &device_hough_list, int x_num, int y_num)
{
  // ハフ変換のグラフを表示.

  thrust::host_vector<int> host_hough_list(x_num * y_num);
  // デバイス用のハフ変換データ格納用のvector.

  thrust::copy(device_hough_list.begin(), device_hough_list.end(), host_hough_list.begin());
  // ホストにハフ変換データをコピー.

  for (int i = 0; i < x_num; i++)
  {
    printf("x=%d:", i);
      for (int j = 0; j < y_num; j++)
      {
        if (device_hough_list[i * y_num + j] >= 2)
        {
          printf("y=%d,%d ", j,  (int)device_hough_list[i*y_num + j]);
        }
      }
    printf("\n");
  }
}

__global__ void find_cercle(int *device_hough_list, float *device_cercle, int *device_head, const int near_x,const int near_y,const int x_num,const int y_num, const int thr, const float weight){
  // ハフ変換のデータから円を検出する.

  const int i = blockIdx.x, j = threadIdx.x;
  // blockIdx.x…何個目のxか threadIdx.x…何個目のyか.

  const int pos = i * y_num + j;
  // ハフ変換グラフの位置に該当する配列の番号.

  const int val = device_hough_list[pos];
  // 注目する箇所の値.

  if(val < thr)
  {
    // 閾値未満.
    return;
  }

  const int x_start  = i - near_x <0 ? 0 : i - near_x;
  const int x_end =  i + near_x >= x_num ? x_num - 1 : i + near_x;
  const int y_start = j - near_y<0 ? 0 : j - near_y;
  const int y_end = j + near_y >= y_num ? y_num - 1 : j + near_y;
  // 範囲情報を元に探索するxの範囲とyの範囲を決定.

  float x_ave = 0, y_ave = 0, count = 0;
  // xとyの重み付き平均を求めようとしている.
  // _aveは重みを付けた総和、countは重みの総和.
  // weighが線形的に見る範囲の数.

  for (int k = x_start; k <= x_end; k++){
    for (int l = y_start; l <= y_end; l++){
      const int com = device_hough_list[k * y_num + l];
      // 比較する場所の値.

      const bool count_flag = (val < com);

      if (true == count_flag)
      {
        return;
      }

      float z = (k-i)*(k-i)+(l-j)*(l-j);
      z = z < weight * weight ? weight * weight - z : 0;
      // 重みを計算.
      x_ave += com * z * k;
      y_ave += com * z * l;
      count += com * z;
    }
  }

  if (0 != count){
    int my_head = device_head[0]++;
    device_cercle[my_head * 2] = x_ave/count;
    device_cercle[my_head * 2 + 1] = y_ave/count;
  }
  // flag==trueなら書き込み.

}
