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
  // ファイルディスクリプタを設定できているかどうか.

  struct termios oldtio;
  // 前のシリアルポート設定の保持用.

  bool get_old_tio=false;
  // 前のシリアルポート設定を保持できているか.

  struct termios newtio;
  // 新しいシリアルポート設定用.

  fd_set readfs;
  // ディスクリプション集合体の宣言.

  void init_newtio(void){
    /*
    新しいシリアルポートのtermiousの初期設定.
    uart用の設定になっている.
    */

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
    2つのtermious構造体を比べる.引数は2つの比べるtemious構造体.
    等しければ0を,等しくなければ-1を返す.
    */

    bool equal=(settio_p->c_cflag==nowtio_p->c_cflag)&&(settio_p->c_iflag==nowtio_p->c_iflag)&&(settio_p->c_oflag==nowtio_p->c_oflag)&&(settio_p->c_lflag==nowtio_p->c_lflag)&&(settio_p->c_line==nowtio_p->c_line);
    // 2つのtemious構造体の排列以外の要素が等しいかを調べている.

    int arr_equal;
    arr_equal=memcmp(settio_p->c_cc,nowtio_p->c_cc,sizeof(settio_p->c_cc));
    // 配列の要素が全て等しいか調べている.

    if(0!=arr_equal){
      equal=false;
    }

    return(equal==true?0:-1);
  }

  int get_and_wait_char(unsigned char *s,const struct timespec wait,const long int timeout_us,const int timeout_lim){
    /*
    1文字取得する.ただし,最初1文字も取得できなかったら指定秒数だけ待機する.
    指定秒数後も取得できなかったら諦める.
    エラーが生じた時は-1を,何も取得できなかった時,タイムアウトは0を,きちんと取得できた時は1を返す.
    引数は,1つめが取得した文字を格納するchar型のポインタ,2つめは待機する時間(timespec構造体),3つめはタイムアウトとみなす時間,4つめはタイムアウトを何回したら切断されるとみなすかの回数.
    */

    unsigned char tmp_char;
    // 一時的に文字を保存するため.
    int result;
    // 結果を格納する.

    result=get_serial_char(&tmp_char,timeout_us,timeout_lim);

    if(0==result){
      // 新規文字列なし.

      nanosleep(&wait,NULL);
      // 指定時間だけ待機.
      result=get_serial_char(&tmp_char,timeout_us,timeout_lim);
    }

    switch(result){
      case -1:
        // エラー
          return(-1);
          break;
      case 0:
        // 新規文字列なし.
        return(0);
        break;
      case 1:
        // 正常取得
        *s=tmp_char;
        return(1);
        break;
      default :
        printf("invalid argument get_and_wait_char()\n");
        return(-1);
    }
  }

  int timeout_check(long int timeout_us){
    /*
    ポートが読み書き可能な常態化チェックする.
    読み書き可能なら1を,タイムアウトしたなら0を,エラーなら1を返す.
    引数はタイムアウトとみなす秒数[us]
    */

    FD_ZERO(&readfs);
    // ディスクリプション集合初期化.

    FD_SET(fd, &readfs);
    // ファイルディスクリプタを登録.

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = timeout_us;
    // タイムアウト値を設定.

    const int time_select=select(fd+1,&readfs,NULL,NULL,&timeout);
    // ファイルディスクリプタを監視し,エラー時は-1を,タイムアウトなら0を,読み書きできる状態ならそれ以外の値を返す.

    const int return_val=time_select>=1?1:time_select;
    // 読み書きできる状態の時に1を返すようにする.

    return(return_val);
  }

  int timeout_coutinuous_check(const int timeout_result,long int *timeout_count,const long int timeout_lim,const char *s){
    /*
    何回連続でタイムアウトをしたかカウントする.
    指定の回数以上タイムアウト,エラー,間違った引数の場合-1を,ただのタイムアウトの場合は0を,正常ならば1を返す.
    引数は,タイムアウトしたかどうかの結果,何回連続でタイムアウトをしたかを格納するint型のポインタ,何回のタイムアウトで切断されたとみなすかの回数,タイムアウト時に出力する文字.
    */
    switch (timeout_result) {
      case -1:
        // エラー時.
        *timeout_count=0;
        perror("[uart]select error.\n");
        return(-1);
        break;
      case 0:
        // タイムアウト時
        (*timeout_count)++;
        printf("[uart]%s timeout.\n",s);
        if(timeout_lim<(*timeout_count)){
          // 指定回数以上タイムアウトした時.エラーを返す用にする.
          printf("[uart]timeouted too many times.\n");
          *timeout_count=0;
          return(-1);
        }
        return(0);
        break;
      case 1:
        // 正常時
        *timeout_count=0;
        return(1);
        break;
      default :
        printf("[uart]invalid argument in timeout_coutinuous_check()\n");
        return(-1);
        break;
    }
  }

  // 無名名前区間終わり.
}

int open_serial_port(const char *modem_dev){
  /*
  シリアルポートを開く.
  正常に開けたら0を,開けなければ-1を返す.
  引数は開くシリアルポート名.
  正常に開けた時に1,開けなかった時-1にを返す.
  */

  init_newtio();
  // 新しいシリアルポート情報を初期化.

  int success;
  // 戻り地を格納.

  if(false==open_fd){
    // ファイルディスクリプタを設定していなかった時

    fd=open(modem_dev,O_RDWR|O_NOCTTY);
    // 指定したシリアルポートを開く.読み書き用かつttyとして開く.
  }

  if(fd==-1){
    // オープンできなかった時の処理.

    perror("[uart]can't open serial port.\n");
    close_serial_port();
    return(-1);
  }

  success=tcgetattr(fd,&oldtio);
  // 前のシリアルポート設定を退避
  if(-1==success){
  // 前のシリアルポート設定を入手できなかった時.
    get_old_tio=false;

    perror("[uart]can't get oldtio.\n");
    close_serial_port();
    return(-1);
  }
  get_old_tio=true;

  success=tcflush(fd,TCIFLUSH);
  // 前の入出力を終わらせる.
  if(-1==success){
    // 前の入出力が終わらなかった時.

    perror("[uart]can't end old flush.\n");
    close_serial_port();
    return(-1);
  }

  success=tcsetattr(fd,TCSANOW,&newtio);
  // 新しいシリアルポート設定
  if(-1==success){
    // 設定書き込みができなかった時.

    perror("[uart]can't set newtio.\n");
    close_serial_port();
    return(-1);
  }

  struct termios tmptio;
  // 一時出来なデータ用.
  success=tcgetattr(fd,&tmptio);
  // 今の設定を入手.
  if(-1==success){
    // 入手できなかった時.

    perror("[uart]can't get newtio.\n");
    close_serial_port();
    return(-1);
  }

  success=compare_termious(&newtio,&tmptio);
  if(-1==success){
    // 望み通りに書き込めてないポート情報が存在した時.

    printf("[uart]can't set newtio correctly.\n");
    close_serial_port();
    return(-1);
  }

  // 正常にポートを開くことができた時.
  open_fd=true;
  printf("[uart]success to open serial port.\n");
  return(0);
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


int put_serial_char(const unsigned char c,const long int timeout_us,const long int timeout_lim){
  /*
  1文字送る.
  失敗したら-1を,タイムアウトしたら0を,成功したら1を返す.
  引数は,送る文字,タイムアウトとみなす時間,連続タイムアウト上限数
  */

  if(false==open_fd){
    printf("[uart]filediscripter is invalid.\n");
    return(-1);
  }

  const int timeout_result=timeout_check(timeout_us);
  // シリアルポートに書き込みが可能か確認.

  static long int timeout_count=0;
  // タイムアウトが何回連続で生じたかを格納する変数.

  const int timeout_continuous_result=timeout_coutinuous_check(timeout_result,&timeout_count,timeout_lim,"write");
  // タイムアウトが何回連続で生じたかの計測.

  if(1!=timeout_continuous_result){
    // 正常に文字が書き込みできないなら終了.
    return(timeout_continuous_result);
  }
  const ssize_t write_result=write(fd,&c,1);
  // 文字を書き込み

  if(1!=write_result){
    // 正常に文字を書き込めていなかった時.
    printf("[uart]fail to put serial.\n");
    return(-1);
  }

  return(1);
}

int put_serial_string(const unsigned char *s,const size_t size,const long int timeout_us,const long timeout_lim){
  /*
  文字列を送る.
  失敗したら-1を,タイムアウトしたら0を,成功したら1を返す.
  引数は,送る文字列,送る文字列の長さ(ヌル文字含まず),タイムアウトとみなす時間,連続タイムアウト上限数
  */
  if(false==open_fd){
    printf("[uart]filediscripter is invalid.\n");
    return(-1);
  }

  const int timeout_result=timeout_check(timeout_us);
  // シリアルポートに書き込みが可能か確認.

  static long int timeout_count=0;
  // タイムアウトが何回連続で生じたかを格納する変数.

  const int timeout_continuous_result=timeout_coutinuous_check(timeout_result,&timeout_count,timeout_lim,"write");
  // タイムアウトが何回連続で生じたかの計測.

  if(1!=timeout_continuous_result){
    // 正常に文字が書き込みできないなら終了.
    return(timeout_continuous_result);
  }

  const ssize_t write_result=write(fd,s,size);
  // 文字列を書き込み

  if(write_result!=(ssize_t)size){
    // 正常に文字を書き込めていなかった時.
    // 処理系依存動作
    printf("[uart]fail to put serial.\n");
    return(-1);
  }

  return(1);
}

int put_Jdata(const char init,const unsigned char *s,const size_t size,const long int timeout_us,const long int timeout_lim){
  /*
  JetsonからMBにデータを送信する.
  initは初期通信文字.
  sは送る文字の中身(初期文字,チェックサム含まず).
  sizeはsの長さ.
  timeout_usはタイムアウトとみなす時間,timeout_limはタイムアウト上限回数.
  失敗時に-1,タイムアウト時に0,成功時に1を返す.
  */

  unsigned char *data=(unsigned char *)malloc(size+2);
  // 送信用文字列確保

  if(NULL==data){
    // メモリ確保失敗時
    printf("[uart]fail to get memory.\n");
    return(-1);
  }

  data[0]=init;
  // 初期文字追加

  unsigned char checksum=0;
  for(size_t i=0;i<size;i++){
    data[i+1]=s[i];
    checksum+=s[i];
  }

  data[size+1]=checksum;

  int success;
  success=put_serial_string(data,size+2,timeout_us,timeout_lim);
  free(data);
  return(success);
}

int get_serial_char(unsigned char *s,const long int timeout_us,const int timeout_lim){
  /*
  1文字取得する.
  失敗,指定回数以上タイムアウトしたら-1,タイムアウトしたら0,成功したら1を返す.
  引数は,入手した値を格納するchar型のポインタ,タイムアウトとみなす時間,連続タイムアウト上限数.
  */

  if(false==open_fd){
    printf("[uart]filediscripter is invalid.\n");
    return(-1);
  }

  const int timeout_result=timeout_check(timeout_us);
  // シリアルポートに書き込みが可能か確認.

  static long int timeout_count=0;
  // タイムアウトが何回連続で生じたかを格納する変数.

  const int timeout_continuous_result=timeout_coutinuous_check(timeout_result,&timeout_count,timeout_lim,"read");
  // タイムアウトが何回連続で生じたかの計測.

  if(1!=timeout_continuous_result){
    // 正常に文字が取得できないなら終了.
    return(timeout_continuous_result);
  }

  const int read_result=read(fd,s,1);
  // 文字を取得

  if(-1==read_result){
    printf("[uart]read error\n");
  }

  return(read_result);
}

int get_MB_data(const char init,unsigned char *data,const size_t num,const long int loop,const long int timeout_us,const int timeout_lim,const int zero_lim){
  /*
  MBからシリアル通信をしてデータを取得する.
  成功したら1,エラーは-1,新規文字列なし,タイムアウト,チェックサムエラーは0.
  numには初期文字もチェックサムも含めない文字数.
  timeout_usはreadする際にタイムアウトする時間[us].
  timeout_limは連続で何回タイムアウトしたら切断されているとみなすかどうかの回数.
  (MBの電源が切れるとタイムアウトを繰り返す.)
  zero_limは連続で何回何も文字を取得できなかったら切断されているとみなすかどうかの回数.
  (途中でケーブルが抜かれると何も取得できないを繰り返す.)
  */

  unsigned char tmp_char;
  int result;
  bool continue_loop=true;
  struct timespec wait;
  wait.tv_sec=0;
  wait.tv_nsec=loop;
  static int zero_counter=0;
  // uartが何回連続で何の値も取得できなかったか.

  while(continue_loop){
    result=get_serial_char(&tmp_char,timeout_us,timeout_lim);
    if(0==result){
      // 新規文字列なし.

      zero_counter++;
      return(zero_lim<zero_counter?zero_counter=0,-1:0);
    }else if(-1==result){
      // エラー.

      zero_counter=0;
      close_serial_port();
      return(-1);
    }else{
      // 何か文字を取得

      zero_counter=0;

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

    switch (success) {
      case -1:
        // エラー.
        zero_counter=0;
        close_serial_port();
        return(-1);
        break;
      case 0:
        // 新規文字なし.
        zero_counter++;
        return(zero_lim<zero_counter?zero_counter=0,-1:0);
        break;
      case 1:
        // チェックサムを計算.
        zero_counter=0;
        checksum+=data[i];
        break;
      default :
        return(-1);
        break;
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

    zero_counter=0;
    close_serial_port();
    return(-1);
  }

  // 正常取得

  if(checksum!=get_checksum){
    // チェックサムが不正.
    printf("[uart]checksum is invalid\n");
    return(0);
  }else{
    return(1);
  }

}
