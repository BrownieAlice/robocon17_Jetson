#ifndef URG_RING_BUFFER_H
#define URG_RING_BUFFER_H

/*!
  \file
  \brief �����O�o�b�t�@
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
  \brief ������

  \param[in] ring �����O�o�b�t�@�̍\����
  \param[in] buffer ���蓖�Ă�o�b�t�@
  \param[in] shift_length �o�b�t�@�T�C�Y�� 2 �̏搔

*/
extern void ring_initialize(ring_buffer_t *ring,
                            char *buffer, const int shift_length);


/*!
  \brief �����O�o�b�t�@�̃N���A

  \param[in] ring �����O�o�b�t�@�̍\����

*/
extern void ring_clear(ring_buffer_t *ring);


/*!
  \brief �i�[�f�[�^����Ԃ�

  \param[in] ring �����O�o�b�t�@�̍\����

*/
extern int ring_size(const ring_buffer_t *ring);


/*!
  \brief �ő�̊i�[�f�[�^����Ԃ�

  \param[in] ring �����O�o�b�t�@�̍\����

*/
extern int ring_capacity(const ring_buffer_t *ring);


/*!
  \brief �f�[�^�̊i�[

  \param[in] ring �����O�o�b�t�@�̍\����
  \param[in] data �f�[�^
  \param[in] size �f�[�^�T�C�Y

  \return �i�[�����f�[�^��

*/
extern int ring_write(ring_buffer_t *ring, const char *data, int size);


/*!
  \brief �f�[�^�̎��o��

  \param[in] ring �����O�o�b�t�@�̍\����
  \param[out] buffer �f�[�^
  \param[in] size �ő�̃f�[�^�T�C�Y

  \return ���o�����f�[�^��

*/
extern int ring_read(ring_buffer_t *ring, char *buffer, int size);

#endif /* ! RING_BUFFER_H */
