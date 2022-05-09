/*!
 * @file rt_serial.h
 * @brief Serial port
 */

#ifndef _rt_serial
#define _rt_serial

int set_interface_attribs_custom_baud(int fd, int speed, int parity, int port);   //配置串行端口
void init_serial_for_sbus(int fd, int baud);                                      //串口初始化

#endif
