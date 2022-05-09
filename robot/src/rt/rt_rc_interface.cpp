#include <pthread.h>
#include <rt/rt_rc_interface.h>
#include "Utilities/EdgeTrigger.h"
#include <string.h> // memcpy
#include <stdio.h>
#include <rt/rt_sbus.h>
static pthread_mutex_t lcm_get_set_mutex =
PTHREAD_MUTEX_INITIALIZER;      /**< mutex 用于保护LCM上的gui设置 */

rc_control_settings rc_control;

/* ------------------------- 处理程序 ------------------------- */


/*!
 * 功能：获取遥控手柄控制器设置的函数
 */
void get_rc_control_settings(void *settings) 
{
  pthread_mutex_lock(&lcm_get_set_mutex);                         //（1）线程上锁，进入临界区
  v_memcpy(settings, &rc_control, sizeof(rc_control_settings));   //（2）运行内存复制函数，搬运手柄要设置的内容
  pthread_mutex_unlock(&lcm_get_set_mutex);                       //（3）线程解锁，退出临界区
}



/*!
 * 功能：手柄串口数据包解压成运动控制指令的函数
 * 步骤：
 * （1）定义数据变量，并更新数据
 * （2）把手柄按键和控制机器人变量连接起来
 * （3）手柄按键对应的指令
 * （4）提示要模式选择
 */
EdgeTrigger<int> mode_edge_trigger(0);
EdgeTrigger<TaranisSwitchState> backflip_prep_edge_trigger(SWITCH_UP);
EdgeTrigger<TaranisSwitchState> experiment_prep_edge_trigger(SWITCH_UP);
TaranisSwitchState initial_mode_go_switch = SWITCH_DOWN;
void sbus_packet_complete() 
{
  /*（1）定义操纵杆数据变量，并从内存获取数据*/
  Taranis_X7_data data;                       //1）定义数据变量
  update_taranis_x7(&data);                   //2）从手柄硬件中获取数据,手柄按键及摇杆的数据换算
  float v_scale = data.knobs[0]*1.5f + 2.0f;  //3）设置v摇杆阈值，数值在 0.5 到 3.5
  float w_scale = 2.*v_scale;                 //4）设置w摇杆阈值，数值在 1.0 到 7.0

  /*（2）把手柄硬件按键和控制机器人选择变量映射连接起来*/
  auto estop_switch          = data.right_lower_right_switch;  //1）停止键
  auto mode_selection_switch = data.left_lower_left_switch;    //2）模式选择键
  auto mode_go_switch        = data.left_upper_switch;         //3）开始模式按键
  auto left_select           = data.left_lower_right_switch;   //4）左按键
  auto right_select          = data.right_lower_left_switch;   //5）右按键
  int selected_mode = 0;

 /*（3）运行手柄按键对应的指令*/
  switch(estop_switch) 
   {
    case SWITCH_UP:     //1）停止指令：站起来不动即是停止指令
      selected_mode = RC_mode::OFF;            //设置停止模式标志位
      break;

    case SWITCH_MIDDLE: //2）恢复站立指令
       selected_mode = RC_mode::RECOVERY_STAND;//设置恢复站立模式标志位
      break;

    case SWITCH_DOWN:   //3）运动指令，默认是运动指令
      selected_mode = RC_mode::LOCOMOTION;     //（1）设置运动模式标志位

      if(left_select == SWITCH_UP && right_select == SWITCH_UP)    //（2）若是再选择待机模式，则设置站立平衡模式
      {
        selected_mode = RC_mode::QP_STAND;//设置站立平衡
      }

      if(backflip_prep_edge_trigger.trigger(mode_selection_switch)&& mode_selection_switch == SWITCH_MIDDLE) //（3）若是再选择选择模式，则设置选择指令
      {
        initial_mode_go_switch = mode_go_switch;    //设置选择指令
      }
 
      if(experiment_prep_edge_trigger.trigger(mode_selection_switch)&& mode_selection_switch == SWITCH_DOWN) //（4）若是再选择实验模式（双腿站立、视觉等），则设置选择指令
      {
        initial_mode_go_switch = mode_go_switch;     //设置选择指令
      }

      if(mode_selection_switch == SWITCH_MIDDLE) //（5）若是再选择后空翻模式，先运行后空翻准备指令
      {
        selected_mode = RC_mode::BACKFLIP_PRE;    //默认是后空翻准备指令

        if(mode_go_switch == SWITCH_DOWN && initial_mode_go_switch != SWITCH_DOWN) //1）若是再一次选择后空翻模式，运行后空翻
        {
          selected_mode = RC_mode::BACKFLIP;
        } 
        else if(mode_go_switch == SWITCH_UP) //2）若是没有再一次选择后空翻模式，运行平衡站立
        {
          initial_mode_go_switch = SWITCH_UP;
        }
      } 

      else if(mode_selection_switch == SWITCH_DOWN)//（6）若是再选择实验模式，则进行双足站立
      {
        int mode_id = left_select * 3 + right_select;
        if(mode_id == 0)
        {
          selected_mode = RC_mode::TWO_LEG_STANCE_PRE; //双足站立
          if(mode_go_switch == SWITCH_DOWN && initial_mode_go_switch != SWITCH_DOWN) 
          {
            selected_mode = RC_mode::TWO_LEG_STANCE; //双足站立
          } 
          else if(mode_go_switch == SWITCH_UP) 
          {
            initial_mode_go_switch = SWITCH_UP;//平衡站立
          }
        }
        else if(mode_id == 1)
        { // 可视化
          selected_mode = RC_mode::VISION;
        }
      }

      //（7）步态选择 
      int mode_id = left_select * 3 + right_select;
      constexpr int gait_table[9] = 
      { 0, //（1）stand
        0, //（2）stand
        1, //（3）bounding
        2, //（4）pronking
        3, //（5）gallop
        5, //（6）trot run
        6, //（7）walk1
        7, //（8）walk2
        8, //（9）pace
      };
  // （8）摇杆死区限制
  for(int i(0); i<2; ++i)
  {
    data.left_stick[i]  = deadband(data.left_stick[i], 0.1, -1., 1.);
    data.right_stick[i] = deadband(data.right_stick[i], 0.1, -1., 1.);
  }
  // （9）各种模式下参数设置
  if(selected_mode == RC_mode::LOCOMOTION || selected_mode == RC_mode::VISION)            //1）运动模式或视觉模式下
  {
    rc_control.variable[0] = gait_table[mode_id];            //（1）把步态类型传给rc_control
    rc_control.v_des[0] = v_scale * data.left_stick[1];      //（2）通过摇杆计算x方向的速度
    rc_control.v_des[1] = -v_scale * data.left_stick[0];     //（3）通过摇杆计算y方向的速度
    rc_control.v_des[2] = 0;                                 //（4）默认z方向的速度为0
    rc_control.height_variation = data.knobs[1];             //（5）机身高度
    rc_control.omega_des[0] = 0;                             //（6）欧拉角默认为0 
    rc_control.omega_des[1] = 0;                             //（7）欧拉角默认为0 
    rc_control.omega_des[2] = w_scale * data.right_stick[0]; //（8）通过摇杆计算偏航角
  } 
  else if(selected_mode == RC_mode::QP_STAND || selected_mode == RC_mode::TWO_LEG_STANCE) //2）四脚站立平衡模式或者四脚站立平衡模式下
  {
    rc_control.rpy_des[0] = data.left_stick[0];       //（1）横滚角默认为0 
    rc_control.rpy_des[1] = data.right_stick[1];      //（2）俯仰角默认为0
    rc_control.rpy_des[2] = data.right_stick[0];      //（3）通过摇杆计算偏航角
    rc_control.height_variation = data.left_stick[1]; //（4）机身高度
    rc_control.omega_des[0] = 0;                      //（5）欧拉角默认为0 
    rc_control.omega_des[1] = 0;                      //（6）欧拉角默认为0 
    rc_control.omega_des[2] = 0;                      //（7）欧拉角默认为0 
  } 
  break;
}


/*（10）提示要进行模式选择*/
bool trigger = mode_edge_trigger.trigger(selected_mode);                                 //1）查看现在模式有没有选择
if(trigger || selected_mode == RC_mode::OFF || selected_mode == RC_mode::RECOVERY_STAND) //2）若是无模式或者恢复站立模式下，进行模式选择
{
  if(trigger) 
  {
    printf("MODE TRIGGER!\n");
  }
  rc_control.mode = selected_mode;
}
}




/*!
 * 功能：内存复制函数
 */
void *v_memcpy(void *dest, volatile void *src, size_t n) 
{
  void *src_2 = (void *)src;
  return memcpy(dest, src_2, n);
}



/*!
 * 功能：死区判断并限制函数
 */
float deadband(float command, float deadbandRegion, float minVal, float maxVal)
{
  if (command < deadbandRegion && command > -deadbandRegion) 
  {
    return 0.0;
  } 
  else 
  {
    return (command / (2)) * (maxVal - minVal);
  }
}


