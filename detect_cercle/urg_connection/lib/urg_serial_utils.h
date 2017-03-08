#ifndef URG_SERIAL_UTILS_H
#define URG_SERIAL_UTILS_H

/*!
  \file
  \brief シリアル用の補助関数
  \author Satofumi KAMIMURA

  $Id: urg_serial_utils.h,v c5747add6615 2015/05/07 03:18:34 alexandr $
*/


//! Finds the serial port
extern int urg_serial_find_port(void);


//! Returns the name of the serial port found
extern const char *urg_serial_port_name(int index);


/*!
  \brief ポートが URG かどうか

  \retval 1 URG のポート
  \retval 0 不明
  \retval <0 エラー

*/
extern int urg_serial_is_urg_port(int index);

#endif /* !URG_SERIAL_UTILS_H */
