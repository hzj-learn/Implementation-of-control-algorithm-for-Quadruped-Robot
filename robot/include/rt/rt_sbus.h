/*!
 * @file rt_sbus.h
 * @brief 与手柄控制器接收器通信
 */

#ifndef _rt_sbus
#define _rt_sbus

#include <stdint.h>
int init_sbus(int is_simulator);                                //（1）初始化SBUS串行端口
int receive_sbus(int port);                                     //（2）从port接收手柄的数据，接收串行底层数据包
int read_sbus_data(int port, uint8_t *sbus_data);               //（3）从串行端口读取数据
void unpack_sbus_data(uint8_t sbus_data[], uint16_t *channels); //（4）将sbus数据消息从硬件数据解压到通道中，一共18个数据
int read_sbus_channel(int channel);                             //（5）从sbus通道读取数据
void update_taranis_x7(Taranis_X7_data* data);                  //（6）手柄按键及摇杆的数据换算


//手柄按键状态
enum TaranisSwitchState 
{
  SWITCH_UP = 0,      //上按键
  SWITCH_MIDDLE = 1,  //中按键
  SWITCH_DOWN = 2,    //下按键
};



//手柄输入类型
struct Taranis_X7_data 
{
  TaranisSwitchState 
  /*按键类型*/
  left_upper_switch,          //（1）左上键
  left_lower_left_switch,     //（2）左下左键
  left_lower_right_switch,    //（3）左下右键
  right_upper_switch,         //（4）右上键
  right_lower_left_switch,    //（5）右下左键
  right_lower_right_switch;   //（6）右下右键

  /*摇杆类型*/
  float left_stick[2];        //（1）左摇杆，水平竖直两个方向
  float right_stick[2];       //（2）右摇杆，水平竖直两个方向

  /*旋钮类型*/
  float knobs[2];             //（1）两个旋钮
};


#endif
