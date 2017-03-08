#ifndef URG_DEBUG_H
#define URG_DEBUG_H

/*!
  \file
  \brief URG debugging functions

  \author Satofumi KAMIMURA

  \attention �g���K�v�͂���܂���B

*/

#ifdef __cplusplus
extern "C" {
#endif

#include "urg_sensor.h"


    /*!
     \brief �Z���T�Ƀf�[�^�𒼐ڑ��M����
    */
    extern int urg_raw_write(urg_t *urg, const char *data, int data_size);


    /*!
     \brief �Z���T����f�[�^�𒼐ڎ�M����
    */
    extern int urg_raw_read(urg_t *urg, char *data, int max_data_size,
                            int timeout);

    /*!
     \brief �Z���T������s�܂ł̃f�[�^�𒼐ڎ�M����
     */
    extern int urg_raw_readline(urg_t *urg,char *data, int max_data_size,
                                int timeout);

#ifdef __cplusplus
}
#endif

#endif /* !URG_DEBUG_H */
