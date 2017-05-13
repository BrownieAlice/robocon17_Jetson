#ifndef INCLUDED_DETECT_CIRCLE_CUDA_P
#define INCLUDED_DETECT_CIRCLE_CUDA_P

#include "./detect_circle_cuda.hpp"
#include <boost/optional.hpp>

struct mle_data_str
{
  double x;
  double y;
  double j_ml;
};
typedef mle_data_str mle_data;

__global__ void make_xy_data(const float *device_ranges, float *device_xy_data, const int lrf_offset, const int lrf_num, const float angle_min, const float angle_increment);

__global__ void make_hough_graph(float *device_xy_data, int *device_hough_list, const float center_x, const float center_y, const int x_num, const int y_num, const float x_wid, const float y_wid, const float x_range, const float y_range, const float radius);

static void print_hough_gragh(thrust::device_vector<int> &device_hough_list, int x_num, int y_num);

__global__ void find_circle(int *device_hough_list, float *device_circle, int *device_head, const int near_x, const int near_y,const int x_num,const int y_num, const int thr, const float weight);

__global__ void cuda_select_data(const float *xy_data_d, bool *data_list_d, const int lrf_num, const float p, const float q, const float radius, const float rad_err);

void select_data(const thrust::host_vector<float>& host_xy_data, const thrust::device_vector<float>& device_xy_data, std::vector<float> *select_datas, const int lrf_num, const float p, const float q, const float radius, const float rad_err, const int warp);

double eva_deno(const double x_a, const double y_a, const double a, const double b);

double eva_val(const double deno, const double rad);

double eva_d(const double deno, const double rad, const double x_ab, const double ab);

boost::optional<mle_data>  calc_mle(const std::vector<float>& select_datas, const float rad, float x, float y, const float allow_err, const int limit_count);

boost::optional<position> mle(const std::vector<float>& select_datas, const float rad, const float x, const float y, const float allow_err, const int limit_count);

double random_val(double wid);
#endif
