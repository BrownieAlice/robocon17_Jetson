#ifndef DETECT_CERCLE_CUDA
#define DETECT_CERCLE_CUDA
void detect_cercle_cuda(float *ranges,const int lrf_num,const int lrf_begin,const float angle_min,const float angle_increment,const float x_wid,const float y_wid,const int x_num,const int y_num,const int x_near,const int y_near,const int thr,const int thr2,float *p,float *q,bool *calc_flag,const float rad,const float rad_err1,const float rad_err2,const float rad_err3,const float allow_err1,const float allow_err2,const float weight, const int warp,const int limit_count);
#endif
