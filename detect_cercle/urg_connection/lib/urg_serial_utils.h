#ifndef URG_SERIAL_UTILS_H
#define URG_SERIAL_UTILS_H

/*!
  \file
  \brief �V���A���p�̕⏕�֐�
  \author Satofumi KAMIMURA

  $Id: urg_serial_utils.h,v c5747add6615 2015/05/07 03:18:34 alexandr $
*/


//! Finds the serial port
extern int urg_serial_find_port(void);


//! Returns the name of the serial port found
extern const char *urg_serial_port_name(int index);


/*!
  \brief �|�[�g�� URG ���ǂ���

  \retval 1 URG �̃|�[�g
  \retval 0 �s��
  \retval <0 �G���[

*/
extern int urg_serial_is_urg_port(int index);

#endif /* !URG_SERIAL_UTILS_H */
