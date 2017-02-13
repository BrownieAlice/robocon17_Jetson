#include <stdio.h>
#include <termios.h>
#include <sys/signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "uart.h"

namespace{
  int fd;
  // filediscripter

  bool open_fd=false;

  struct termios oldtio;
  // 前のシリアルポート設定の保持用

  bool get_old_tio=false;

  struct termios newtio;
  // 新しいシリアルポート設定用

}





int open_serial_port(const char *modem_dev){
  /*
  シリアルポートを開く.
  正常に開けたら0を,開けなければ-1を返す.
  */

  /*
    Settings for new port
    CS8:8n1(8bit,no parity,1 stopbit)
    CLOCAL:local connection,no modem control
    CREAD:enable receiving characters
  */
  newtio.c_cflag=BAUDRATE|CS8|CLOCAL|CREAD;
  //IGNPAR:ignore bytes with parity errors
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_line=0;
  memset(newtio.c_cc,0,sizeof(newtio.c_cc));
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 1;
  //Now clean the modem line and activate the settings for the port
  // 新しいシリアルポート情報


  int success;
  // 戻り地を格納

  if(false==open_fd){
    //filediscripterを入手していなかった時

    fd=open(modem_dev,O_RDWR|O_NOCTTY);
    /*
    指定したシリアルポートを開く.読み書き用かつttyとして開く.
    */
  }

  if(fd==-1){
    // オープンできなかった時の処理

    perror("[uart]can't open serial port.");
    close_serial_port();
    return(-1);
  }else{
    // オープンできた時の処理

    success=tcgetattr(fd,&oldtio);
    // 前のシリアルポート設定を退避
    if(-1==success){
      // 前のシリアルポート設定を入手できなかった時
      get_old_tio=false;

      perror("[uart]can't get oldtio.");
      close_serial_port();
      return(-1);
    }else{
      // 前のシリアルポート設定を入手出来た.
      get_old_tio=true;

      success=tcflush(fd,TCIFLUSH);
      // 前の入出力を終わらせる.
      if(-1==success){
        // 前の入出力が終わらなかった時.

        perror("[uart]can't end old flush.");
        close_serial_port();
        return(-1);
      }


      success=tcsetattr(fd,TCSANOW,&newtio);
      // 新しいシリアルポート設定
      if(-1==success){
        // 設定書き込みができなかった時.

        perror("[uart]can't set newtio.");
        close_serial_port();
        return(-1);
      }else{
        // 設定できた時

        struct termios tmptio;
        // 一時出来なデータ用
        success=tcgetattr(fd,&tmptio);
        // 今の設定を入手
        if(-1==success){
          // 入手できなかった時

          perror("[uart]can't get newtio.");
          close_serial_port();
          return(-1);
        }else{
          // 入手出来た時.

          success=compare_termious(&newtio,&tmptio);
          if(-1==success){
            printf("[uart]can't set newtio correctly.\n");
            close_serial_port();
            return(-1);
          }
        }
      }
    }

    open_fd=true;
    printf("[uart]success to open serial port.\n");
    return(0);
  }
}

int compare_termious(const struct termios *settio_p,const struct termios *nowtio_p){
  /*
  2つのtermious型を比べる.
  等しければ0を,等しくなければ-1を返す.
  */

  bool equal=(settio_p->c_cflag==nowtio_p->c_cflag)&&(settio_p->c_iflag==nowtio_p->c_iflag)&&(settio_p->c_oflag==nowtio_p->c_oflag)&&(settio_p->c_lflag==nowtio_p->c_lflag)&&(settio_p->c_line==nowtio_p->c_line);

  int arr_equal;
  arr_equal=memcmp(settio_p->c_cc,nowtio_p->c_cc,sizeof(settio_p->c_cc));
  if(0!=arr_equal){
    equal=false;
  }

  return(equal==true?0:-1);
}

void close_serial_port(void){
  /*
  シリアルポートを閉じる.
  前のシリアルポート情報を入手出来ていたならそれを設定する.
  ポートを開けていたなら閉じる.
  */

  printf("[uart]close serial port.\n");
  if(get_old_tio&&open_fd){
    tcsetattr(fd,TCSANOW,&oldtio);
  }
  if(open_fd){
    close(fd);
  }
  get_old_tio=false;
  open_fd=false;
}


int put_serial_char(unsigned char c){
  /*
  1文字送る.
  失敗したら-1を成功したら0を返す.
  */

  if(false==open_fd){
    printf("[uart]filediscripter is invalid.\n");
    return(-1);
  }
  if(write(fd,&c,1) != 1){
    printf("[uart]fail to put serial.\n");
    return(-1);
  }
  return(0);
}


int put_serial_string(char *s){
  /*
  文字列を送る.
  失敗したら-1を成功したら0を返す.
  */
  if(false==open_fd){
    printf("[uart]filediscripter is invalid.\n");
    return(-1);
  }
  if(write(fd,s,strlen(s)) != (int)strlen(s)){
    printf("[uart]fail to put serial.\n");
    return(-1);
  }
  return(0);
}


unsigned char get_serial_char(ssize_t *result){
  /*
  1文字取得する.
  resultに失敗/成功が格納される.
  失敗したら-1を成功したら1を何も入手できなかったら0を入れる.
  戻り値は取得した値.取得してないなら0を返す.
  */

  unsigned char c=0;

  if(false==open_fd){
    printf("[uart]filediscripter is invalid.\n");
    *result=-1;
    return(c);
  }

  *result=read(fd,(char *)&c,1);
  /*
  resultには1,0,-1のどれかが格納される.
  1なら1文字ゲット出来たことを表す.
  0ならなにもゲットできなかったことを表す.
  -1ならエラーがおきたことを表す.
  */
  return(c);
}

int get_MB_data(char init,unsigned char *data,size_t num,long int loop){
  /*
  MBからシリアル通信をしてデータを取得する.
  成功したら1,エラーは-1,新規文字列なしまたはチェックサムエラーは0.
  numには初期文字もチェックサムも含めない文字数.
  */

  unsigned char tmp_char;
  ssize_t result;
  bool continue_loop=true;
  struct timespec wait;
  wait.tv_sec=0;
  wait.tv_nsec=loop;

  if(0==num){
    return(0);
  }

  while(continue_loop){
    // 初期文字を検出するループ

    tmp_char=get_serial_char(&result);
    if(0==result){
      // 新規文字列なし

      return(0);
    }else if(-1==result){
      // エラー

      close_serial_port();
      return(-1);
    }else{
      if(init==tmp_char){
        // 初期文字を検出

        continue_loop=false;
      }
    }
  }

  for(size_t i=0;i<num;i++){
    tmp_char=get_serial_char(&result);

    if(0==result){
      // 新規文字列なし

      nanosleep(&wait,NULL);
      tmp_char=get_serial_char(&result);
    }

    if(0==result){
      // 新規文字列なし

      return(0);
    }else if(-1==result){
      // エラー

      close_serial_port();
      return(-1);
    }else{
      // 正常取得
      data[i]=tmp_char;
    }
  }

  unsigned char checksum=0;
  for(size_t i=0;i<num;i++){
    // チェックサムを計算
    checksum+=data[i];
  }

  // チェックサムを取得
  unsigned char get_checksum;
  tmp_char=get_serial_char(&result);

  if(0==result){
    // 新規文字列なし

    nanosleep(&wait,NULL);
    tmp_char=get_serial_char(&result);
  }

  if(0==result){
    // 新規文字列なし

    return(0);
  }else if(-1==result){
    // エラー

    close_serial_port();
    return(-1);
  }
  // 正常取得
  get_checksum=tmp_char;

  if(checksum!=get_checksum){
    // チェックサムが不正
    printf("[uart]checksum is invalid\n");
    return(0);
  }else{
    return(1);
  }

}

void continue_connect_uart(long int loop,const char *modem_dev){
  bool continue_loop=true;
  int success;
  struct timespec wait;
  wait.tv_sec=0;
  wait.tv_nsec=loop;

  close_serial_port();

  while(continue_loop){
    success=open_serial_port(modem_dev);
    if(1==success){
      continue_loop=false;
      break;
    }
    nanosleep(&wait,NULL);
  }

}
