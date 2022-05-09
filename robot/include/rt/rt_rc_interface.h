/**
 * @file rt_rc_interface.h
 *
 */
#ifndef _RT_RC_INTERFACE
#define _RT_RC_INTERFACE


//遥控器控制参数变量
class rc_control_settings 
{
  public:
    double     mode;                //手柄输入的控制模式
    double     p_des[2];            //手柄输入的(x, y)的期望位置：范围 (-1 ~ 1) 
    double     height_variation;    //手柄输入的高度：范围 -1 ~ 1
    double     v_des[3];            //手柄输入的期望速度 -1 ~ 1 * (scale 0.5 ~ 1.5)
    double     rpy_des[3];          //手柄输入的期望rpy -1 ~ 1
    double     omega_des[3];        //手柄输入的期望方向 -1 ~ 1
    double     variable[3];
};

//遥控器模式选择
namespace RC_mode
{
  constexpr int OFF          = 0;           //停止
  constexpr int QP_STAND     = 3;           //QP站立平衡
  constexpr int BACKFLIP_PRE = 4;           //后空翻预备
  constexpr int BACKFLIP     = 5;           //后空翻
  constexpr int VISION       = 6;           //可视化
  constexpr int LOCOMOTION   = 11;          //对角小跑运动
  constexpr int RECOVERY_STAND = 12;        //恢复站立
  // 扩展的模式
  constexpr int TWO_LEG_STANCE_PRE = 20;    //双腿站立预备
  constexpr int TWO_LEG_STANCE = 21;        //双腿站立
};

void sbus_packet_complete();                                                        //手柄串口数据包解压成运动控制指令函数
void get_rc_control_settings(void* settings);                                       //获取控制器的设置函数
void* v_memcpy(void* dest, volatile void* src, size_t n);                           //内存复制函数（库函数）
float deadband(float command, float deadbandRegion, float minVal, float maxVal);    //死区判断函数

#endif
