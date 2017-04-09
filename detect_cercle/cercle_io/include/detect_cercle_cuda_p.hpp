#ifndef INCLUDED_DETECT_CERCLE_CUDA_P
#define INCLUDED_DETECT_CERCLE_CUDA_P

#include "./detect_cercle_cuda.hpp"

__global__ void make_xy_data(const float *device_ranges, float *device_xy_data, const int lrf_offset, const int lrf_num, const float angle_min, const float angle_increment);

__global__ void make_hough_graph(float *device_xy_data, int *device_hough_list, const float center_x, const float center_y, const int x_num, const int y_num, const float x_wid, const float y_wid, const float x_range, const float y_range, const float radius);

static void print_hough_gragh(thrust::device_vector<int> &device_hough_list, int x_num, int y_num);

__global__ void find_cercle(int *device_hough_list, float *device_cercle, int *device_head, const int near_x,const int near_y,const int x_num,const int y_num, const int thr, const float weight);

#endif
