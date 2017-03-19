#ifndef DETECT_CERCLE_CUDA_PRIVATE
#define DETECT_CERCLE_CUDA_PRIVATE

__global__ void make_xy_data(const float *device_ranges, float *device_xy_data, const int lrf_offset, const int lrf_num, const float angle_min, const float angle_increment);

__global__ void make_hough_graph(float *device_xy_data, int *device_hough_list, const float center_x, const float center_y, const int x_num, const int y_num, const float x_wid, const float y_wid, const float x_range, const float y_range, const float radius);

void detect_cercle_cuda(const std::vector<float>& host_ranges, const int lrf_num,const float angle_min,const float angle_increment,const float x_wid,const float y_wid,const int x_num,const int y_num,const int x_near,const int y_near,const int thr,const int thr2,float *p,float *q,bool *calc_flag,const float rad,const float rad_err1,const float rad_err2,const float rad_err3,const float allow_err1,const float allow_err2,const float weight, const int warp,const int limit_count);

#endif
