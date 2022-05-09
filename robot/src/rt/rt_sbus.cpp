/*!
 * @file rt_sbus.cpp
 * @brief 与手柄控制器接收器通信
 */

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <string>

#ifdef linux
#define termios asmtermios
#include <asm/termios.h>
#undef termios
#endif

#include <termios.h>
#include "rt/rt_sbus.h"
#include "rt/rt_serial.h"

pthread_mutex_t sbus_data_m;

uint16_t channels[18];
uint16_t channel_data[18];


#define K_SBUS_PORT_SIM "/dev/ttyUSB0"   //模拟器中SBUS串口名称
#define K_SBUS_PORT_MC "/dev/ttyS4"      //mini cheetah上的SBUS串行端口名称

/*!
 * 功能：将sbus数据消息从硬件数据解压到通道中，一共18个数据
 */
void unpack_sbus_data(uint8_t sbus_data[], uint16_t *channels_) 
{
  if ((sbus_data[0] == 0xF) && (sbus_data[24] == 0x0)) 
  {
    channels_[0] = ((sbus_data[1]) | ((sbus_data[2] & 0x7) << 8));
    channels_[1] = (sbus_data[2] >> 3) | ((sbus_data[3] & 0x3F) << 5);
    channels_[2] = ((sbus_data[3] & 0xC0) >> 6) | (sbus_data[4] << 2) |
                   ((sbus_data[5] & 0x1) << 10);
    channels_[3] = ((sbus_data[5] & 0xFE) >> 1) | ((sbus_data[6] & 0xF) << 7);
    channels_[4] = ((sbus_data[6] & 0xF0) >> 4) | ((sbus_data[7] & 0x7F) << 4);
    channels_[5] = ((sbus_data[7] & 0x80) >> 7) | (sbus_data[8] << 1) |
                   ((sbus_data[9] & 0x3) << 9);
    channels_[6] = ((sbus_data[9] & 0xFC) >> 2) | ((sbus_data[10] & 0x1F) << 6);
    channels_[7] = ((sbus_data[10] & 0xE0) >> 5) | (sbus_data[11] << 3);

    channels_[8] = ((sbus_data[12]) | ((sbus_data[13] & 0x7) << 8));
    channels_[9] = (sbus_data[13] >> 3) | ((sbus_data[14] & 0x3F) << 5);
    channels_[10] = ((sbus_data[14] & 0xC0) >> 6) | (sbus_data[15] << 2) |
                    ((sbus_data[16] & 0x1) << 10);
    channels_[11] = ((sbus_data[16] & 0xFE) >> 1) | ((sbus_data[17] & 0xF) << 7);
    channels_[12] =((sbus_data[17] & 0xF0) >> 4) | ((sbus_data[18] & 0x7F) << 4);
    channels_[13] = ((sbus_data[18] & 0x80) >> 7) | (sbus_data[19] << 1) |
                    ((sbus_data[20] & 0x3) << 9);
    channels_[14] =
        ((sbus_data[20] & 0xFC) >> 2) | ((sbus_data[21] & 0x1F) << 6);
    channels_[15] = ((sbus_data[21] & 0xE0) >> 5) | (sbus_data[22] << 3);
    channels_[16] = (sbus_data[23] & 0x80) >> 7;
    channels_[17] = (sbus_data[23] & 0x40) >> 6;

    pthread_mutex_lock(&sbus_data_m);    //线程上锁 
    for (int i = 0; i < 18; i++)         //把数据搬走
    {
      channel_data[i] = channels_[i];
    }
    pthread_mutex_unlock(&sbus_data_m);  //线程解锁 
  } 
}



/*!
 * 功能：       从串行端口读取数据
 * port：       串口输入端口
 * sbus_data：  存储数据寄存器
 * packet_full：存满一个数据包置1
 */
int read_sbus_data(int port, uint8_t *sbus_data) 
{
  uint8_t packet_full = 0;
  uint8_t read_byte[1] = {0};
  int timeout_counter = 0;
  while ((!packet_full) && (timeout_counter < 50)) //数据包还没有满，或者时间还没到
  {
    timeout_counter++;//超时计时
    while(read(port, read_byte, sizeof(read_byte)) != 1){} //读取一个字节
    for (int i = 0; i < 24; i++)                           //同时一位移动缓冲器
    {
      sbus_data[i] = sbus_data[i + 1];
    }
    sbus_data[24] = read_byte[0];                          //最后一位（停止位）数据清0
    if ((sbus_data[0] == 15) && (sbus_data[24] == 0))      //检查起始和停止字节是否正确
    {
      packet_full = 1;
    }
  }
  return packet_full;
}


/*!
 * 功能：获取sbus频道数据
 */
int read_sbus_channel(int channel) 
{
  pthread_mutex_lock(&sbus_data_m);   //线程上锁
  int value = channel_data[channel];  //从频道上获取数据
  pthread_mutex_unlock(&sbus_data_m); //线程解锁
  return value;
}


/*!
 * 功能：从port接收手柄的数据，接收串行底层数据包
 */
int receive_sbus(int port) 
{
  uint16_t read_buff[25] = {0};                         //（1）定义读取缓存区
  int x = read_sbus_data(port, (uint8_t *)read_buff);   //（2）通过串口底层驱动读串口数据，并存在read_buff中
  //（3）判断有没有读到数据
  if (x) //如果读到数据
  {
    unpack_sbus_data((uint8_t *)read_buff, channels);   //数据解包，将sbus数据消息从硬件数据解压到通道中，，一共18个数据
  } 
  else //如果读不到数据
  {
    printf("SBUS tried read 50 bytes without seeing a packet\n");//提示没有读到数据包
  }
  return x;
}


/*!
 * 功能：初始化SBUS串行端口
 */
int init_sbus(int is_simulator) 
{
  std::string port1;
  //（1）判断是仿真模式还是真实模式,定义扩展端口
  if (is_simulator)     //1）仿真模式的串口端口设置，模拟器扩展到:"/dev/ttyUSB0"
  {
    port1 = K_SBUS_PORT_SIM;
  } 
  else                 //2)真实模式串口端口设置，硬件端口扩展到:"/dev/ttyS4"
  {
    port1 = K_SBUS_PORT_MC;
  }

  //（2）如果串口线程没有打开，打印错误
  if (pthread_mutex_init(&sbus_data_m, NULL) != 0)
  {
    printf("Failed to initialize sbus data mutex.\n");
  }

 //（3）用C++内置库函数打开串口端口的
  int fd1 = open(port1.c_str(), O_RDWR | O_NOCTTY | O_SYNC);    
  if (fd1 < 0)    //1)若没有打开串口成功，打印错误提示
  {
    printf("Error opening %s: %s\n", port1.c_str(), strerror(errno));
  } 
  else            //2)若打开串口成功，就初始化该串口
  {
    init_serial_for_sbus(fd1, 100000);
  }
  return fd1;
}


/*!
 * 功能：手柄行程操纵杆，数据换算，公式：手柄行程操纵杆数字量=（手柄行程操纵杆模拟量-172）*2/（1811-172）-1，这个公式是根据手柄结构来的
 */
static float scale_joystick(uint16_t in) 
{
  return (in - 172) * 2.f / (1811.f - 172.f) - 1.f;
}


/*!
 * 功能：手柄按键状态检测
 */
static TaranisSwitchState map_switch(uint16_t in) 
{
  switch(in) 
  {
    case 1811:   //按下键
      return TaranisSwitchState::SWITCH_DOWN;
    case 992:    //按中间键
      return TaranisSwitchState::SWITCH_MIDDLE;
    case 172:    //按上键 
      return TaranisSwitchState::SWITCH_UP;
    default:
      printf("[SBUS] switch returned bad value %d\n", in);
      return TaranisSwitchState::SWITCH_UP;
  }
}



/*!
 * 功能：手柄按键及摇杆的数据换算
 */
void update_taranis_x7(Taranis_X7_data* data) 
{
  pthread_mutex_lock(&sbus_data_m);                           //（1）线程上锁
  //（2）两个摇杆，包括四个方向的模拟量行程操纵杆，数据换算
  data->left_stick[0] = scale_joystick(channel_data[3]);                    //1）左前后手柄  行程操纵杆，数据换算
  data->left_stick[1] = scale_joystick(channel_data[0]);                    //2）左左右手柄  行程操纵杆，数据换算
  data->right_stick[0] = scale_joystick(channel_data[1]);                   //3）右前后手柄  行程操纵杆，数据换算
  data->right_stick[1] = scale_joystick(channel_data[2]);                   //4）右左右手柄  行程操纵杆，数据换算
 
  //（3）六个按键 
  data->left_lower_left_switch = map_switch(channel_data[4]);               //1）左左键  手柄按键状态检测
  data->left_lower_right_switch = map_switch(channel_data[5]);              //2）左右键  手柄按键状态检测
  data->left_upper_switch = map_switch(channel_data[6]);                    //3）上键    手柄按键状态检测
  data->right_lower_left_switch = map_switch(channel_data[7]);              //4）右左键  手柄按键状态检测
  data->right_lower_right_switch = map_switch(channel_data[8]);             //5）右右键  手柄按键状态检测
  data->right_upper_switch = map_switch(channel_data[9]);                   //6）下键    手柄按键状态检测
  
  //（4）前面的两个单程摇杆
  data->knobs[0] = scale_joystick(channel_data[10]);                        //1）行程操纵杆，数据换算
  data->knobs[1] = scale_joystick(channel_data[11]);                        //2）行程操纵杆，数据换算
  pthread_mutex_unlock(&sbus_data_m);                        //（5）线程解锁
}
