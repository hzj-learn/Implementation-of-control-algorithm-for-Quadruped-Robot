/*!
 * @file rt_serial.cpp
 * @brief Serial port
 */

//#ifdef linux

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#define termios asmtermios
#include <asm/termios.h>
#undef termios
#include <termios.h>
#include <math.h>
#include <pthread.h>
#include <stropts.h>
#include <endian.h>
#include <stdint.h>
#include "rt/rt_serial.h"


/*!
 * 功能：sbus串口初始化
 * baud：串口波特率
 * fd 串行端口FD
 */
void init_serial_for_sbus(int fd, int baud) 
{
  printf("\t[RT SERIAL] Configuring serial device...\n");
  struct termios2 tty;
  ioctl(fd, TCGETS2, &tty);
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= BOTHER;
  tty.c_ispeed = baud;
  tty.c_ospeed = baud;
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
                  | INLCR | IGNCR | ICRNL | IXON);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_cflag &= ~(CSIZE | PARENB);
  tty.c_cflag |= PARENB;
  tty.c_cflag &= ~PARODD;
  tty.c_cflag |=
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 1;       // 
  tty.c_cc[VTIME] = 1;      // 0.5秒读取超时
  ioctl(fd, TCSETS2, &tty);
}



/**
 * @brief 功能：配置串行端口
 * @param fd 串行端口FD
 * @param speed 波特率
 * @param parity 对等
 * @param port 端口号
 */
int set_interface_attribs_custom_baud(int fd, int speed, int parity, int port) 
{
  (void)parity;
  (void)port;

  printf("\t[RT SERIAL] Configuring serial device...\n");
  struct termios2 tty;

  ioctl(fd, TCGETS2, &tty);
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= BOTHER;
  tty.c_ispeed = speed;
  tty.c_ospeed = speed;

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8位字符
  //对不匹配的速度测试禁用IGNBRK；否则接收中断 
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;  //禁用中断处理
  tty.c_lflag = 0;         //没有信号字符，没有回应
  // 无规范处理
  tty.c_oflag = 0;      // 没有重新映射，没有延迟
  tty.c_cc[VMIN] = 0;   // 阅读不会阻碍
  tty.c_cc[VTIME] = 1;  // 0.5秒读取超时

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // 关闭xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);  // 忽略调制解调器控件，
  // enable reading
  // tty.c_cflag &= ~(PARENB | PARODD);      // 关闭奇偶校验
  tty.c_cflag |= PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  // cfmakeraw(&tty);

  ioctl(fd, TCSETS2, &tty);
  return 0;
}

//#endif
