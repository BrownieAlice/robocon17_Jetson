/*
======================================================================
Project Name    : connect ethLRF
File Name       : connect_ethLRF
Encoding        : UTF-8
Creation Date   : 2017/03/08

Copyright © 2017 Alice.
======================================================================
*/
#include "../lib/urg_utils.h"
#include "../include/connect_xxxLRF.h"

namespace param
{
  // パラメータ

  const urg_connection_type_t connection_type = URG_ETHERNET;
  // 接続方式.
  const char connect_address_device[] = "192.168.0.10";
  // デバイス名.
  const long int connect_port_baudrate = 10940;
  // ボーレート.
  const int LRF_recconect_hz = 10;
  // LRFと接続できなかった時再接続する周期[Hz].
  const int timeout_ms = 100;
  // LRFのタイムアウト時間[ms].

  const int memory_reensure_hz = 10;
  // メモリを確保できなかった時に再確保する周期[Hz].
  const int main_loop_hz = 50;
  // メイン関数の実行周期[Hz].

  const char node_name[] = "connect_ethLRF";
  // ノード名.
  const char topic_name[] = "scan_ethLRF";
  // トピック名.
  const char frame_name[] = "laser_eth";
  // フレーム名.
}
