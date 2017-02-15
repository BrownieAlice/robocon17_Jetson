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
  // ファイルディスクリプタ.

  bool open_fd=false;

  struct termios oldtio;
  // 前のシリアルポート設定の保持用.

  bool get_old_tio=false;
  // 前のシリアルポート設定を保持できているか.

  struct termios newtio;
  // 新しいシリアルポート設定用.

  fd_set readfs;
  // ディスクリプション集合体の宣言.

  struct timeval timeout;
  // タイムアウト値の格納.

  void init_newtio(){
    // 新しいシリアルポートのtermiousの初期設定.

    static bool flag=false;
    // 1度だけ実行するため.
    if(true==flag){
      return;
    }
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
    // 新しいシリアルポート情報.

    flag=true;
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

  int get_and_wait_char(unsigned char *s,struct timespec wait,long int timeout_us,int timeout_lim){
    unsigned char tmp_char;
    ssize_t result;

    tmp_char=get_serial_char(&result,timeout_us,timeout_lim);

    if(0==result){
      // 新規文字列なし.

      nanosleep(&wait,NULL);
      tmp_char=get_serial_char(&result,timeout_us,timeout_lim);
    }

    if(0==result){
      // 新規文字列なし.

      return(0);
    }else if(-1==result){
      // エラー.

      return(-1);
    }else{
      // 正常取得.

      *s=tmp_char;
      return(1);
    }
  }

  // 無名名前区間終わり.
}

int open_serial_port(const char *modem_dev){
  /*
  シリアルポートを開く.
  正常に開けたら0を,開けなければ-1を返す.
  */

  init_newtio();
  // 新しいシリアルポート情報を初期化.

  int success;
  // 戻り地を格納.

  if(false==open_fd){
    // filediscripterを入手していなかった時

    fd=open(modem_dev,O_RDWR|O_NOCTTY);
    /*
    指定したシリアルポートを開く.読み書き用かつttyとして開く.
    */
  }

  if(fd==-1){
    // オープンできなかった時の処理.

    perror("[uart]can't open serial port.");
    close_serial_port();
    return(-1);
  }else{
    // オープンできた時の処理.

    success=tcgetattr(fd,&oldtio);
    // 前のシリアルポート設定を退避
    if(-1==success){
      // 前のシリアルポート設定を入手できなかった時.
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
        // 設定できた時.

        struct termios tmptio;
        // 一時出来なデータ用.
        success=tcgetattr(fd,&tmptio);
        // 今の設定を入手.
        if(-1==success){
          // 入手できなかった時.

          perror("[uart]can't get newtio.");
          close_serial_port();
          return(-1);
        }else{
          // 入手出来た時.
          success=compare_termious(&newtio,&tmptio);
          if(-1==success){
            // 望み通りに書き込めてないポート情報が存在した時.
            printf("[uart]can't set newtio correctly.\n");
            close_serial_port();
            return(-1);
          }
        }
      }
    }
    // 正常にポートを開くことができた時.
    open_fd=true;
    printf("[uart]success to open serial port.\n");
    return(0);
  }
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
  }else{
    return(0);
  }
}


unsigned char get_serial_char(ssize_t *result,long int timeout_us,int timeout_lim){
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

  FD_ZERO(&readfs);
  // ディスクリプション集合初期化.

  FD_SET(fd, &readfs);
  // ファイルディスクリプタを登録.

  timeout.tv_sec = 0;
  timeout.tv_usec = timeout_us;
  // タイムアウト値を設定.

  int time_select;
  time_select=select(fd + 1, &readfs, NULL, NULL, &timeout);
  // ファイルディスクリプタを監視し,エラー時は-1を,タイムアウトなら0を,読み書きできる状態ならそれ以外の値を返す.

  static int timeout_count=0;
  // タイムアウトが何回連続で生じたかを計測する.
  if(-1==time_select){
    // エラー時.
    timeout_count=0;

    perror("[uart]select error.\n");
    *result=-1;
    return(c);
  }else if(0==time_select){
    // タイムアウト時
    timeout_count++;

    printf("[uart]read timeout.\n");
    *result=0;
    if(timeout_lim<timeout_count){
      // 指定回数以上タイムアウトした時.エラーを返す用にする.
      printf("[uart]timeouted too many times.");
      *result=-1;
      timeout_count=0;
    }
    return(c);
  }else{
    timeout_count=0;
  }
  *result=read(fd,(char *)&c,1);

  /*
  resultには1,0,-1のどれかが格納される.
  1なら1文字ゲット出来たことを表す.
  0ならなにもゲットできなかった(タイムアウト,ポート消失を含む)ことを表す.
  -1ならエラー(規定回数以上のタイムアウトを含む)がおきたことを表す.
  */
  return(c);
}

int get_MB_data(char init,unsigned char *data,size_t num,long int loop,long int timeout_us,int timeout_lim,int zero_lim){
  /*
  MBからシリアル通信をしてデータを取得する.
  成功したら1,エラーは-1,新規文字列なしまたはチェックサムエラーは0.
  numには初期文字もチェックサムも含めない文字数.
  timeout_usはreadする際にタイムアウトする時間[us].
  timeout_limは連続で何回タイムアウトしたら切断されているとみなすかどうかの回数.
  (MBの電源が切れるとタイムアウトを繰り返す.)
  zero_limは連続で何回何も文字を取得できなかったら切断されているとみなすかどうかの回数.
  (途中でケーブルが抜かれると何も取得できないを繰り返す.)
  */

  unsigned char tmp_char;
  ssize_t result;
  bool continue_loop=true;
  struct timespec wait;
  wait.tv_sec=0;
  wait.tv_nsec=loop;
  static int zero_counter=0;
  // uartが何回連続で何の値も取得できなかったか.

  if(0==num){
    zero_counter++;
    return(zero_lim<zero_counter?zero_counter=0,-1:0);
  }

  while(continue_loop){
    tmp_char=get_serial_char(&result,timeout_us,timeout_lim);
    if(0==result){
      // 新規文字列なし.

      zero_counter++;
      return(zero_lim<zero_counter?zero_counter=0,-1:0);
    }else if(-1==result){
      // エラー.

      close_serial_port();
      return(-1);
    }else{
      if(init==tmp_char){
        // 初期文字を検出.

        continue_loop=false;
      }else{
      // 他の文字を検出.

        continue_loop=true;
      }
    }
  }

  unsigned char checksum=0;
  for(size_t i=0;i<num;i++){
    int success;
    success=get_and_wait_char(&data[i],wait,timeout_us,timeout_lim);

    if(-1==success){
      // エラー.

      close_serial_port();
      return(-1);
    }else if(0==success){
      // 新規文字なし.

      zero_counter++;
      return(zero_lim<zero_counter?zero_counter=0,-1:0);
    }else{
      // チェックサムを計算.
      checksum+=data[i];
    }
  }

  // チェックサムを取得.
  unsigned char get_checksum;
  int success;
  success=get_and_wait_char(&get_checksum,wait,timeout_us,timeout_lim);

  if(0==success){
    // 新規文字列なし.

    zero_counter++;
    return(zero_lim<zero_counter?-1:0);
  }else if(-1==success){
    // エラー.

    close_serial_port();
    return(-1);
  }

  zero_counter=0;
  // ゼロカウンタを0に.

  // 正常取得
  if(checksum!=get_checksum){
    // チェックサムが不正.
    printf("[uart]checksum is invalid\n");
    return(0);
  }else{
    return(1);
  }

}

void continue_connect_uart(long int loop,const char *modem_dev){
  // uart接続が成功するまでループし続ける.

  bool continue_loop=true;
  // ループの可否.

  int success;
  struct timespec wait;
  wait.tv_sec=0;
  wait.tv_nsec=loop;
  // ループ周期.

  close_serial_port();
  // 一度ポートを閉じる.

  while(continue_loop){
    success=open_serial_port(modem_dev);
    if(0==success){
      continue_loop=false;
      break;
    }
    nanosleep(&wait,NULL);
    // スリープ
  }

}
