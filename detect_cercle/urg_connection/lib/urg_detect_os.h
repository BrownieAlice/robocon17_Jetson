#ifndef URG_DETECT_OS_H
#define URG_DETECT_OS_H

/*!
  \file
  \brief OS �̌��o
  \author Satofumi KAMIMURA

  $Id: urg_detect_os.h,v c5747add6615 2015/05/07 03:18:34 alexandr $
*/

#if defined(_WIN32)
#define URG_WINDOWS_OS

#if defined(_MSC_VER) || defined(__BORLANDC__)
#define URG_MSC
#endif

#elif defined(__linux__)
#define URG_LINUX_OS

#else
// ���o�ł��Ȃ��Ƃ����AMac �����ɂ��Ă��܂�
#define URG_MAC_OS
#endif

#endif /* !URG_DETECT_OS_H */
