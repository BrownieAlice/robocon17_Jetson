#ifndef INCLUDED_DETECT_CIRCLE_CUDA
#define INCLUDED_DETECT_CIRCLE_CUDA
#include <boost/optional.hpp>

struct hough_param_str_def {
  float x_wid; // ハフ変換する際のxの間隔.
  float y_wid; // ハフ変換する際のyの間隔.
  int x_num;    // ハフ変換する際のxの個数.
  int y_num;    // ハフ変換する際のyの個数.
  int near_x;   // ハフ変換する際のxの近傍とみなす個数.
  int near_y;   // ハフ変換する際のyの近傍とみなす個数.
  float weight; // 重み付き平均を求める際の範囲.
  int thr;      // ハフ変換する際の閾値.
} ;
typedef struct hough_param_str_def hough_param_str;

struct position_str
{
  double x;
  double y;
};
typedef position_str position;

boost::optional<position> detect_circle_cuda(const std::vector<float>& host_ranges, const int lrf_num, const float angle_min, const float angle_increment, const hough_param_str *hough_param, const int thr2, const float p, const float q, const float rad, const float rad_err1, const float rad_err2, const float rad_err3, const float allow_err1, const float allow_err2, const int warp, const int limit_count);

#endif
