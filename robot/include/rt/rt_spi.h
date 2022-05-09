/*!
 * @file rt_spi.h
 * @brief SPI与通讯板的通信
 */

#ifndef _rt_spi
#define _rt_spi

//#ifdef linux

#include <fcntl.h>      //SPI端口需要的库文件
#include <sys/ioctl.h>  //SPI端口需要的库文件


#ifdef __cplusplus /* 如果这是C++编译器，使用C链接 */
extern "C" 
{
#endif
#include <linux/spi/spidev.h>
#ifdef __cplusplus /* 如果这是C++编译器，使用C++链接 */
}
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>            //SPI端口需要的库文件
#include <spi_command_t.hpp>
#include <spi_data_t.hpp>
#include <spi_torque_t.hpp>

#define K_EXPECTED_COMMAND_SIZE 256
#define K_WORDS_PER_MESSAGE 66
#define K_EXPECTED_DATA_SIZE 116
#define K_KNEE_OFFSET_POS 4.35f

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                \
  (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'),     \
      (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'), \
      (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'), \
      (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')



void init_spi();                                                  //（1）初始化SPI函数
void spi_driver_run();                                            //（2）运行SPI驱动函数
void spi_send_receive(spi_command_t* command, spi_data_t* data);  //（3）从通讯板发送接收数据命令函数
spi_data_t* get_spi_data();                                       //（4）获取spi数据函数
spi_command_t* get_spi_command();                                 //（5）获取spi命令函数


//SPI 命令类信息
typedef struct 
{
//电机期望位置
  float q_des_abad[2];
  float q_des_hip[2];
  float q_des_knee[2];
//电机期望速度
  float qd_des_abad[2];
  float qd_des_hip[2];
  float qd_des_knee[2];
//电机kp位置反馈
  float kp_abad[2];
  float kp_hip[2];
  float kp_knee[2];
//电机kd位置反馈
  float kd_abad[2];
  float kd_hip[2];
  float kd_knee[2];
//电机估计力矩
  float tau_abad_ff[2];
  float tau_hip_ff[2];
  float tau_knee_ff[2];
//标志位
  int32_t flags[2];
//校验位
  int32_t checksum;
} spine_cmd_t;


//SPI 数据类信息
typedef struct 
{
  //电机期望位置
  float q_abad[2];
  float q_hip[2];
  float q_knee[2];
  //电机期望速度
  float qd_abad[2];
  float qd_hip[2];
  float qd_knee[2];
  //标志位
  int32_t flags[2];
  //校验位
  int32_t checksum;

} spine_data_t;

#endif // END of #ifdef linux

#endif

