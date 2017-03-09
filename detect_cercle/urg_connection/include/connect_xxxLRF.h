#ifndef connect_xxxLRF
#define connect_xxxLRF

#include "../lib/urg_utils.h"

namespace param
{
  // パラメータ

  extern const urg_connection_type_t connection_type;
  // 接続方式.
  extern const char connect_address_device[];
  // デバイス名.
  extern const long int connect_port_baudrate;
  // ボーレート.
  extern const int LRF_recconect_hz;
  // LRFと接続できなかった時再接続する周期[Hz].
  extern const int timeout_ms;
  // LRFのタイムアウト時間[ms].

  extern const int memory_reensure_hz;
  // メモリを確保できなかった時に再確保する周期[Hz].
  extern const int main_loop_hz;
  // メイン関数の実行周期[Hz].

  extern const char node_name[];
  // ノード名.
  extern const char topic_name[];
  // トピック名.
  extern const char frame_name[];
  // フレーム名.
}


#endif
