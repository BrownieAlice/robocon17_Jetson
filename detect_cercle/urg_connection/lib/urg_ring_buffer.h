#ifndef URG_RING_BUFFER_H
#define URG_RING_BUFFER_H

/*!
  \file
  \brief リングバッファ
  \author Satofumi KAMIMURA

  $Id: urg_ring_buffer.h,v c5747add6615 2015/05/07 03:18:34 alexandr $
*/


//! Control structure of the ring buffer
typedef struct
{
    char *buffer;                 //!< Pointer to the data buffer
    int buffer_size;              //!< Buffer size
    int first;                    //!< Index of the first entry of the buffer
    int last;                     //!< Index of the last entry of the buffer
} ring_buffer_t;


/*!
  \brief 初期化

  \param[in] ring リングバッファの構造体
  \param[in] buffer 割り当てるバッファ
  \param[in] shift_length バッファサイズの 2 の乗数

*/
extern void ring_initialize(ring_buffer_t *ring,
                            char *buffer, const int shift_length);


/*!
  \brief リングバッファのクリア

  \param[in] ring リングバッファの構造体

*/
extern void ring_clear(ring_buffer_t *ring);


/*!
  \brief 格納データ数を返す

  \param[in] ring リングバッファの構造体

*/
extern int ring_size(const ring_buffer_t *ring);


/*!
  \brief 最大の格納データ数を返す

  \param[in] ring リングバッファの構造体

*/
extern int ring_capacity(const ring_buffer_t *ring);


/*!
  \brief データの格納

  \param[in] ring リングバッファの構造体
  \param[in] data データ
  \param[in] size データサイズ

  \return 格納したデータ数

*/
extern int ring_write(ring_buffer_t *ring, const char *data, int size);


/*!
  \brief データの取り出し

  \param[in] ring リングバッファの構造体
  \param[out] buffer データ
  \param[in] size 最大のデータサイズ

  \return 取り出したデータ数

*/
extern int ring_read(ring_buffer_t *ring, char *buffer, int size);

#endif /* ! RING_BUFFER_H */
