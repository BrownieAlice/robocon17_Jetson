/*
======================================================================
Project Name    : connect usbLRF
File Name       : connect_usbLRF
Encoding        : UTF-8
Creation Date   : 2017/03/08

Copyright © 2017 Alice.
======================================================================
*/

#include "../lib/urg_utils.h"
#include "../include/connect_xxxLRF.hpp"

namespace param
{
  // パラメータ

  const urg_connection_type_t connection_type = URG_SERIAL;
  // 接続方式.
  const char connect_address_device[] = "/dev/ttyACM_TOPURG1";
  // デバイス名.
  const long int connect_port_baudrate = 115200;
  // ボーレート.
  const int LRF_recconect_hz = 10;
  // LRFと接続できなかった時再接続する周期[Hz].
  const int timeout_ms = 100;
  // LRFのタイムアウト時間[ms].

  const int memory_reensure_hz = 10;
  // メモリを確保できなかった時に再確保する周期[Hz].
  const int main_loop_hz = 50;
  // メイン関数の実行周期[Hz].

  const char node_name[] = "connect_usbLRF";
  // ノード名.
  const char topic_name[] = "scan_usbLRF";
  // トピック名.
  const char frame_name[] = "laser_usb";
  // フレーム名.
}
