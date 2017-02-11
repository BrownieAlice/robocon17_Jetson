#include <termios.h>
#include <sys/signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include "uart.h"

//file descriptor
static int fd;

//serial port setting
static struct termios oldtio;
static unsigned char *receive_buffer; 

int open_serial_port(const char *modem_dev){
  struct termios newtio;
  fd=open(modem_dev,O_RDWR|O_NOCTTY);
  if(fd<0){
    perror(modem_dev);
    return(-1);
  }else{
    printf("success to open serial port.\n");
    tcgetattr(fd,&oldtio);
    /* Clear the struct for new port settings */ 
    newtio.c_iflag=0;
    newtio.c_oflag=0;
    newtio.c_cflag=0;
    newtio.c_lflag=0;
    newtio.c_line=0;
    bzero(newtio.c_cc,sizeof(newtio.c_cc));
    
    /* Settings for new port
       CS8:8n1(8bit,no parity,1 stopbit)
       CLOCAL:local connection,no modem control
       CREAD:enable receiving characters
    */
    newtio.c_cflag=BAUDRATE|CS8|CLOCAL|CREAD;
    
    //IGNPAR:ignore bytes with parity errors
    newtio.c_iflag = IGNPAR;
    
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 1;
    
    //Now clean the modem line and activate the settings for the port
    tcflush(fd,TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
    
    return 0;
  }
}

void close_serial_port(void)
{
  printf("close serial port.\n");
  tcsetattr(fd,TCSANOW,&oldtio);
  close(fd);
}


int put_serial_char(unsigned char c){
  if(write(fd,&c,1) != 1){
    printf("fail to put serial.\n");
    return -1;
  }
  return 0;
}


int put_serial_string(char *s){
  if(write(fd,s,strlen(s)) != 1){
    printf("fail to put serial.\n");
    return -1;
  }
  return 0;
}


unsigned char get_serial_char(){
  unsigned char c;
  read(fd,(char *)&c,1);
  return c;
}

