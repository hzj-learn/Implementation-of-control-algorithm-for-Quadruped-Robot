#ifndef TI_BOARDCONTROL_H
#define TI_BOARDCONTROL_H

#include "cTypes.h"

// 用于upboard的模拟的类

//替换旧的TI板控制代码，该代码要求跟踪命令/数据状态数据。
struct TiBoardCommand 
{
  float position_des[3];    //（1）每条腿三个关节 期望位置
  float velocity_des[3];    //（2）每条腿三个关节 期望速度
  float kp[3];              //（3）每条腿三个关节电机 kp反馈增益
  float kd[3];              //（4）每条腿三个关节电机 kd反馈增益
  float force_ff[3];        //（5）每条腿三个关节电机 前馈力矩
  float tau_ff[3];          //（6）每条腿三个关节电机 反馈转矩
  s32 enable;               //（7）使能信号
  float max_torque;         //（8）每条腿三个关节电机 最大力矩
};

struct TiBoardData 
{
  float position[3];        //（1）每条腿三个关节 位置
  float velocity[3];        //（2）每条腿三个关节 速度
  float force[3];           //（3）每条腿三个关节电机 力矩
  float q[3];               //（4）每条腿三个关节电机 欧拉角度
  float dq[3];              //（5）每条腿三个关节电机 角速度
  float tau[3];             //（6）每条腿三个关节电机 转矩
  float tau_des[3];         //（7）每条腿三个关节电机 期望转矩
  u32 loop_count_ti;        //（8）闭环周期
  u32 ethercat_count_ti;    //（9）以太网计数周期
  u32 microtime_ti;         //（10）upboard系统时间
};

class TI_BoardControl
 {
 public:
  TI_BoardControl() = default;
  void init(float side_sign);                             //（1）upboard初始化函数
  void run_ti_board_iteration();                          //（2）upboard运行函数
  void reset_ti_board_data();                             //（3）upboard复位数据函数
  void reset_ti_board_command();                          //（4）upboard复位指令函数
  void set_link_lengths(float l1, float l2, float l3);    //（5）upboard设置腿的连杆长度函数
  TiBoardCommand command;                      
  TiBoardData data_structure;

  // for Ben's TI board code that uses pointers
  TiBoardData* data;

 private:
  void kinematics(const float side_sign, const float q[3], const float dq[3],   //（6）upboard运动学控制函数
                  float* p, float* v, float J[][3]);
  void impedanceControl(const float side_sign, const float q[3],                //（7）upboard阻抗控制函数
                        const float dq[3], const float position_des[3],
                        const float velocity_des[3], const float kp[3],
                        const float kd[3], const float force_bias[3],
                        const float torque_bias[3], float* position,
                        float* velocity, float* force, float* torque);

  float _side_sign;
  float _l1, _l2, _l3;
  bool link_lengths_set = false;
};

#endif  // TI_BOARDCONTROL_H
