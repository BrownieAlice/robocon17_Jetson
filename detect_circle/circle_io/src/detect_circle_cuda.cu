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
#include <boost/optional.hpp>
#include <iostream>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/sort.h>
#include "../include/detect_circle_cuda_p.hpp"

boost::optional<position> detect_circle_cuda(const std::vector<float>& host_ranges, const int lrf_num, const float angle_min, const float angle_increment, const hough_param_str *hough_param, const int thr2, const float p, const float q, const float rad, const float rad_err1, const float rad_err2, const float rad_err3, const float allow_err1, const float allow_err2, const int warp, const int limit_count){
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

  boost::optional<position> position_data;

  if (lrf_num > 1024)
  {
    std::cout << "too many number to see." << std::endl;
    return position_data;
  }
  //gpuの可能並列化数を超えていたらreturn

  const size_t ranges_num = host_ranges.size();
  if (ranges_num < lrf_num)
  {
    // LRFの距離データの数が少なすぎた時.
    std::cout << "few number of lrf data." << std::endl;
    return position_data;
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
    (float *)thrust::raw_pointer_cast(device_xy_data.data()),
    lrf_offset, lrf_num, angle_min, angle_increment);
  // x-yデータを作成.

  thrust::copy(device_xy_data.begin(), device_xy_data.end(), host_xy_data.begin());
  // ホストにx-yデータをコピー.

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
  p, q,
  hough_param->x_num, hough_param->y_num,
  hough_param->x_wid, hough_param->y_wid,
  x_range, y_range, rad);

  // print_hough_gragh(device_hough_list, x_num, y_num);
  // ハフ変換グラフを表示.

  thrust::device_vector<float> device_circle(2 * hough_param->x_num * hough_param->y_num);
  // デバイス用の検出円格納用のvector.
  thrust::host_vector<float> host_circle(2 * hough_param->x_num * hough_param->y_num);
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

  find_circle<<<blocks_2,threads_2>>>((int *)thrust::raw_pointer_cast(device_hough_list.data()),
  (float *)thrust::raw_pointer_cast(device_circle.data()),
  (int *)thrust::raw_pointer_cast(device_head.data()),
  hough_param->near_x, hough_param->near_y,
  hough_param->x_num, hough_param->y_num,
  hough_param->thr, hough_param->weight);
  // 円検出.

  thrust::copy(device_circle.begin(), device_circle.end(), host_circle.begin());
  // ホストにx-yデータをコピー.
  thrust::copy(device_head.begin(), device_head.end(), host_head.begin());
  // ホストにx-yデータをコピー.


  float x_esti, y_esti;
  if(host_head[0] != 0){
    const float x_b = p, y_b = q;
    // x-yの中心位置(オフセット).
    x_esti = (float)(host_circle[0] - hough_param->x_num / 2) * hough_param->x_wid + x_b;
    y_esti = (float)(host_circle[1] - hough_param->y_num / 2) * hough_param->y_wid + y_b;
    // 検出したx-yデータ.

    float diff = (x_b-x_esti) * (x_b-x_esti) + (y_b-y_esti) * (y_b-y_esti);
    // 中心位置からのズレ.

    for (int i = 1; i < host_head[0]; i++){
      const float now_x = (float)(host_circle[2 * i] - hough_param->x_num / 2) * hough_param->x_wid + x_b;
      const float now_y = (float)(host_circle[2 * i + 1] - hough_param->y_num / 2) * hough_param->y_wid + y_b;
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
    return position_data;
  }
  */

  if (host_head[0] != 0){
    // ここから2回目の処理.
    std::vector<float> select_datas;
    // 選別したlrfのデータを保管.
    boost::optional<position> calc_position;
    // 計算した位置の格納.

    select_data(host_xy_data, device_xy_data, &select_datas, lrf_num, x_esti, y_esti, rad, rad_err1, warp);
    //lrfデータを選別

    calc_position = mle(select_datas, rad, x_esti, y_esti, allow_err1, limit_count);
    //最尤推定を実行
    if(calc_position)
    {
      x_esti = calc_position->x;
      y_esti = calc_position->y;
    }
    else
    {
      return position_data;
    }

    select_data(host_xy_data, device_xy_data, &select_datas, lrf_num, x_esti, y_esti, rad, rad_err2, warp);
    //lrfデータを選別

    calc_position = mle(select_datas, rad, x_esti, y_esti, allow_err2, limit_count);
    //二回目の最尤推定を実行
    if(calc_position)
    {
      x_esti = calc_position->x;
      y_esti = calc_position->y;
    }
    else
    {
      return position_data;
    }

    select_data(host_xy_data, device_xy_data, &select_datas, lrf_num, x_esti, y_esti, rad, rad_err3, warp);
    //printf("select:%d\n",select_data_num);

    if(select_datas.size() > (size_t)thr2)
    {
      position tmp_position = {x_esti, y_esti};
      position_data = tmp_position;
    }
  }

  return position_data;
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

__global__ void find_circle(int *device_hough_list, float *device_circle, int *device_head, const int near_x, const int near_y,const int x_num,const int y_num, const int thr, const float weight){
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
    device_circle[my_head * 2] = x_ave/count;
    device_circle[my_head * 2 + 1] = y_ave/count;
  }
  // flag==trueなら書き込み.

}

__global__ void cuda_select_data(const float *device_xy_data, bool *device_IsOnCircleBound, const int lrf_num, const float p, const float q, const float radius, const float rad_err)
{
  // 点が中心(p,q)半径radiusの円から半径誤差rad_err内にあるかどうかを計算する.
  const int num = blockIdx.x * blockDim.x + threadIdx.x;
  if (num < lrf_num)
  {
   const float laser_x = device_xy_data[2 * num], laser_y = device_xy_data[2 * num + 1];
   const bool flag = abs(sqrt((laser_x - p) * (laser_x - p) + (laser_y - q) * (laser_y - q)) - radius) < rad_err;
   device_IsOnCircleBound[num] = flag;
 }
 return;
}

void select_data(const thrust::host_vector<float>& host_xy_data, const thrust::device_vector<float>& device_xy_data, std::vector<float> *select_datas, const int lrf_num, const float p, const float q, const float radius, const float rad_err, const int warp)
{
  // 中心(p,q)半径radiusの円の半径誤差rad_err内に点が存在するかどうかを計算する. 存在した場合の天データをselect_dataに,その個数をselect_data_numに格納する.

  select_datas->clear();

  thrust::device_vector<bool> device_IsOnCircleBound(lrf_num);
  // デバイス用の円上かのデータ格納用のvector.
  thrust::host_vector<bool> host_IsOnCircleBound(lrf_num);
  // ホスト用の円上かのデータ格納用のvector.

  dim3 block_0((int)floor((lrf_num + warp - 1) / warp), 1, 1);
  dim3 thread_0(warp, 1, 1);

  cuda_select_data<<<block_0,thread_0>>>(
    (float*)thrust::raw_pointer_cast(device_xy_data.data()),
    (bool*)thrust::raw_pointer_cast(device_IsOnCircleBound.data()),
    lrf_num, p, q, radius, rad_err);
  // 円上にあるかどうかを計算.

  thrust::copy(device_IsOnCircleBound.begin(), device_IsOnCircleBound.end(), host_IsOnCircleBound.begin());
  // ホストにデータをコピー.

  for (int i = 0; i < lrf_num; i++){
    if (host_IsOnCircleBound[i]){
      select_datas->push_back(host_xy_data[2 * i]);
      select_datas->push_back(host_xy_data[2 * i + 1]);
    }
  }
}

double eva_deno(const double x_a, const double y_a, const double a, const double b)
{
  // 1点に関する評価関数の分母を求める.
  return x_a * x_a + y_a * y_a - 2 * a * x_a - 2 * b * y_a + a * a + b * b;
}

double eva_val(const double deno, const double rad)
{
  // 1点に関する評価関数の値を求める.
  return (deno - rad * rad) * (deno - rad * rad) / deno;
}

double eva_d(const double deno, const double rad, const double x_ab, const double ab)
{
  // 1点に関する評価関数のa,bの偏微分地を求める.
  return (2 * (deno - rad * rad ) * (2 * ab - 2 * x_ab) - (deno - rad * rad) * (deno - rad * rad) * (2 * ab - 2 * x_ab)) / (deno * deno);
}

boost::optional<mle_data> calc_mle(const std::vector<float>& select_datas, const float rad, float x, float y, const float allow_err, const int limit_count)
{
  boost::optional<mle_data> data;
  // 最尤推定のデータ型.
  int count = 0;
  // ニュートン法を行った回数.
  double sq_j;
  // 誤差値の二乗.

  do{
    double j_a = 0, j_b = 0, j_ml = 0;
    // a,bdでの偏微分値と評価関数.
    const int select_data_num = static_cast<int>(select_datas.size()) / 2;
    // 点列の数.

    for(int i = 0; i < select_data_num; i++){
      const double x_a = select_datas[2 * i], y_a = select_datas[2 * i + 1];
      const double deno = eva_deno(x_a, y_a, x, y);
      j_ml += eva_val(deno, rad) / select_data_num;
      j_a += eva_d(deno, rad, x_a, x) / select_data_num;
      j_b += eva_d(deno, rad, y_a, y) / select_data_num;
    }
    // 評価関数の値と偏微分値の計算.

    sq_j = j_a * j_a + j_b * j_b;
    if (sq_j > allow_err * allow_err){
      j_a = j_a / sqrt(sq_j) * allow_err;
      j_b = j_b / sqrt(sq_j) * allow_err;
    }
    // 移動距離を算出.

    if(data)
    {
      // 最尤推定のデータ型に書き込みがあった.
      if(data->j_ml < j_ml)
      {
        // 前の評価関数値が今の計算値より低かった.
        return data;
      }
    }

    const mle_data tmp_mle_data = {x, y, j_ml};
    data = tmp_mle_data;
    // 値の退避.

    x -= j_a;
    y -= j_b;
  } while (allow_err * allow_err < sq_j && count++ < limit_count);

  return data;
}

boost::optional<position> mle(const std::vector<float>& select_datas, const float rad, const float x, const float y, const float allow_err, const int limit_count)
{
  //最尤推定法を実行
  /*
    select_datas…点列情報
    rad…円の半径
    x_esti/y_esti…求めた座標を格納する先
    allow_err…許容誤差
  */

  boost::optional<position> position_data;

  boost::optional<mle_data> data = calc_mle(select_datas, rad, x, y, allow_err, limit_count);
  // 最尤推定により計算.
  if(!data)
  {
    // 計算できず.
    return position_data;
  }

  /*
  const double len = sqrt(x * x + y * y);
  // 円までの長さ.
  const double pro_x = x + x / len * rad, pro_y  = y + y / len * rad;
  // 今回の円検出特有の円の手前の谷にハマる現象対策.

  double j_ml = 0;
  const int select_data_num = static_cast<int>(select_datas.size()) / 2;
  for (int i = 0; i < select_data_num; i++){
    const double x_a = select_datas[2 * i], y_a = select_datas[2 * i + 1];
    const double deno = eva_deno(x_a, y_a, pro_x, pro_y) / select_data_num;
    j_ml += eva_val(deno, rad) / select_data_num;
  }
  if (data->j_ml > j_ml){
   data = calc_mle(select_datas, rad, pro_x, pro_y, allow_err, limit_count);
   // 最尤推定により計算.

   if(!data)
   {
     // 計算できず.
     return position_data;
   }
  }
  */
  const int select_data_num = static_cast<int>(select_datas.size()) / 2;
  for (int i = 0; i < 100; i++)
  {
    const double random_x = x + random_val(0.15), random_y = y + random_val(0.15);
    double j_ml = 0;
    for (int j = 0; j < select_data_num; j++){
      const double x_a = select_datas[2 * j], y_a = select_datas[2 * j + 1];
      const double deno = eva_deno(x_a, y_a, random_x, random_y) / select_data_num;
      j_ml += eva_val(deno, rad) / select_data_num;
    }
    //printf("j_ml:%f\n",j_ml);
    //printf("dj_ml:%f\n",data->j_ml);
    if (data->j_ml > j_ml){
      data = calc_mle(select_datas, rad, random_x, random_y, allow_err, limit_count);
      // 最尤推定により計算.
      //printf("iizo\n");
      if(!data)
      {
        // 計算できず.
        return position_data;
      }
    }
  }

  const position tmp_position = {data->x, data->y};
  position_data = tmp_position;
  // データの移動

  return position_data;
}

double random_val(double wid){
    return ((double)rand() + 1.0) / ((double)RAND_MAX + 2.0) * 2 * wid - wid;
}
