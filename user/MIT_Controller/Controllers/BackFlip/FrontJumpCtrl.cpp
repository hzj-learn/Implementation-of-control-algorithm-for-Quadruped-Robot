#include "FrontJumpCtrl.hpp"

/**
 * 功能：前跳控制指令构造函数
 */
template <typename T>
FrontJumpCtrl<T>::FrontJumpCtrl(DataReader* data_reader,float _dt) : DataReadCtrl<T>(data_reader, _dt) 
{}

/**
 * 功能：前跳控制指令析构函数
 */
template <typename T>
FrontJumpCtrl<T>::~FrontJumpCtrl() 
{}


/**
 * 功能：后空翻动作每一步的迭代处理函数
 */
template <typename T>
void FrontJumpCtrl<T>::OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command) 
{
  DataCtrl::_state_machine_time = _curr_time - DataCtrl::_ctrl_start_time;
  DataCtrl::_b_Preparation = b_preparation;
  _update_joint_command();

  for (int leg = 0; leg < 4; ++leg)  //设置每个电机的5个参数
  {
    for (int jidx = 0; jidx < 3; ++jidx) 
    {
      command[leg].tauFeedForward[jidx] = DataCtrl::_jtorque[3 * leg + jidx];           //电机前馈力矩命令
      command[leg].qDes[jidx] = DataCtrl::_des_jpos[3 * leg + jidx] + 0 * _curr_time;   //电机期望位置
      command[leg].qdDes[jidx] = DataCtrl::_des_jvel[3 * leg + jidx];                   //电机期望速度
      command[leg].kpJoint(jidx, jidx) = DataCtrl::_Kp_joint[jidx];                     //电机KP反馈增益
      command[leg].kdJoint(jidx, jidx) = DataCtrl::_Kd_joint[jidx];                     //电机Kd反馈增益 
    }
  }
}


/**
 * 功能：后空翻更新关节指令函数
 */
template <typename T>
void FrontJumpCtrl<T>::_update_joint_command() 
{
  int pre_mode_duration(700);
  int leg_clearance_iteration_front(240) ; 
  int leg_clearance_iteration(600);
  int leg_ramp_iteration(610);
  int tuck_iteration(610);
  int ramp_end_iteration(700);
  float tau_mult;
  DataCtrl::_des_jpos.setZero();
  DataCtrl::_des_jvel.setZero();
  DataCtrl::_jtorque.setZero();
  DataCtrl::_Kp_joint={10.0, 10.0, 10.0};
  DataCtrl::_Kd_joint={1.0, 1.0, 1.0};

  if ( (DataCtrl::pre_mode_count <  pre_mode_duration) || DataCtrl::_b_Preparation) //如果是移到初始配置状态，以准备前跳
  {  
    if (DataCtrl::pre_mode_count == 0) 
    {
      printf("plan_timesteps: %d \n", DataCtrl::_data_reader->plan_timesteps);
    }
    
    DataCtrl::pre_mode_count += DataCtrl::_key_pt_step;
    DataCtrl::current_iteration = 0;
    tau_mult = 0;
  } 
  else                                                                              //如果不是移到初始配置状态  
  {
    tau_mult = 1.2;
  }

  if (DataCtrl::current_iteration > DataCtrl::_data_reader->plan_timesteps - 1) //从数据文件中获取TIMSTEP数据
  {
    DataCtrl::current_iteration = DataCtrl::_data_reader->plan_timesteps - 1;
  }

  //从MATLAB生成的JUMP_数据文件中获取数据
  float* current_step = DataCtrl::_data_reader->get_plan_at_time(DataCtrl::current_iteration);
  float* tau = current_step + tau_offset;

  //初始化关节参数和扭矩
  Vec3<float> q_des_front;
  Vec3<float> q_des_rear;
  Vec3<float> qd_des_front;
  Vec3<float> qd_des_rear;
  Vec3<float> tau_front;
  Vec3<float> tau_rear;

  //设置关节位置和速度，并将从数据文件中获得的关节扭矩输入
  q_des_front << 0.0, current_step[3], current_step[4];
  q_des_rear << 0.0, current_step[5], current_step[6];
  qd_des_front << 0.0, current_step[10], current_step[11];
  qd_des_rear << 0.0, current_step[12], current_step[13];
  tau_front << 0.0, tau_mult * tau[0] / 2.0, tau_mult * tau[1] / 2.0;
  tau_rear << 0.0, tau_mult * tau[2] / 2.0, tau_mult * tau[3] / 2.0;

  //设置限制，使手臂不会向后摆动太远，也不会撞到腿部 
  if(q_des_front[1] < -M_PI/2.2)
  {
    q_des_front[1] = -M_PI/2.2;
    qd_des_front[1] = 0.;
    tau_front[1] = 0.;
  }
  float s(0.);

  //控制支腿间隙
  if (DataCtrl::current_iteration >= leg_clearance_iteration_front &&
      DataCtrl::current_iteration <=leg_clearance_iteration)
  {
    q_des_front << 0.0, current_step[3], current_step[4];
    current_step = DataCtrl::_data_reader->get_plan_at_time(leg_clearance_iteration_front);
    q_des_front << 0.0, -2.3, 2.5;
    qd_des_front << 0.0, 0.0, 0.0;
    tau_front << 0.0, 0.0, 0.0;
  // 执行“脱节”的联合状态，以保持手在
  }
  
  // 实施另一个控制器
  
  //控制支腿CLEARNACE迭代 
  if (DataCtrl::current_iteration >= leg_clearance_iteration //障碍物坡道至支腿间隙
      && DataCtrl::current_iteration < tuck_iteration) 
  {  
    qd_des_front << 0.0, 0.0, 0.0;
    qd_des_rear << 0.0, 0.0, 0.0;
    tau_front << 0.0, 0.0, 0.0;
    tau_rear << 0.0, 0.0, 0.0;
    s = (float)(DataCtrl::current_iteration - leg_clearance_iteration) /
        (leg_ramp_iteration - leg_clearance_iteration);
    if (s > 1) 
    {
      s = 1;
    }
    Vec3<float> q_des_front_0;
    Vec3<float> q_des_rear_0;
    Vec3<float> q_des_front_f;
    Vec3<float> q_des_rear_f;
    current_step = DataCtrl::_data_reader->get_plan_at_time(tuck_iteration);

    q_des_front_0 << 0.0, -2.3, 2.5;
    q_des_rear_0 << 0.0, current_step[5], current_step[6];
    
    //为LEG_CLEARANCE_迭代设置所需的关节状态 
    current_step = DataCtrl::_data_reader->get_plan_at_time(0);
    q_des_front_f << 0.0, -2.3, 2.5;
    q_des_rear_f << 0.0, -1.25, 2.5;

    //坡道的线性插值
    q_des_front = (1 - s) * q_des_front_0 + s * q_des_front_f;
    q_des_rear = (1 - s) * q_des_rear_0 + s * q_des_rear_f;
  } 

  else if (DataCtrl::current_iteration >= tuck_iteration)   //坡道到着陆配置 
  { 
    qd_des_front << 0.0, 0.0, 0.0;
    qd_des_rear << 0.0, 0.0, 0.0;
    tau_front << 0.0, 0.0, 0.0;
    tau_rear << 0.0, 0.0, 0.0;

    s = (float)(DataCtrl::current_iteration - tuck_iteration) /
        (ramp_end_iteration - tuck_iteration);
    if (s > 1) 
    {
      s = 1;
    }
    Vec3<float> q_des_front_0;
    Vec3<float> q_des_rear_0;
    Vec3<float> q_des_front_f;
    Vec3<float> q_des_rear_f;

    current_step = DataCtrl::_data_reader->get_plan_at_time(tuck_iteration);
    q_des_front_0 << 0.0, -2.3, 2.5;
    q_des_rear_0 << 0.0, -1.25, 2.5;
    //为塔克迭代控制器（着陆控制器）设置所需的关节状态 
    current_step = DataCtrl::_data_reader->get_plan_at_time(0);
    q_des_front_f << 0.0, -0.45, 1.3;
    q_des_rear_f << 0.0, -0.45, 1.3;
    //塔克迭代的线性插值斜坡
    q_des_front = (1 - s) * q_des_front_0 + s * q_des_front_f;
    q_des_rear = (1 - s) * q_des_rear_0 + s * q_des_rear_f;

    DataCtrl::_Kp_joint={20.0, 20.0, 20.0};
    DataCtrl::_Kd_joint={2.0, 2.0, 2.0};
  }

  for (int i = 0; i < 12; i += 3) //外展
  {
    DataCtrl::_des_jpos[i] = 0.0;
    DataCtrl::_des_jvel[i] = 0.0;
    DataCtrl::_jtorque[i] = 0.0;
  }
  DataCtrl::_des_jpos[0] = s * (-0.2);
  DataCtrl::_des_jpos[3] = s * (0.2);
  DataCtrl::_des_jpos[6] = s * (-0.2);
  DataCtrl::_des_jpos[9] = s * (0.2);

  if(DataCtrl::current_iteration >= tuck_iteration)
  {
  DataCtrl::_des_jpos[0] = (-0.2);
  DataCtrl::_des_jpos[3] = (0.2);
  DataCtrl::_des_jpos[6] = (-0.2);
  DataCtrl::_des_jpos[9] = (0.2);
  }

  for (int i = 1; i < 6; i += 3)   //前臀
  {
    DataCtrl::_des_jpos[i] = q_des_front[1];
    DataCtrl::_des_jvel[i] = qd_des_front[1];
    DataCtrl::_jtorque[i] = tau_front[1];
  }

  for (int i = 2; i < 6; i += 3)   //前膝
  {
    DataCtrl::_des_jpos[i] = q_des_front[2];
    DataCtrl::_des_jvel[i] = qd_des_front[2];
    DataCtrl::_jtorque[i] = tau_front[2];
  }

  for (int i = 7; i < 12; i += 3) //后臀
  {
    DataCtrl::_des_jpos[i] = q_des_rear[1];
    DataCtrl::_des_jvel[i] = qd_des_rear[1];
    DataCtrl::_jtorque[i] = tau_rear[1];
  }

  for (int i = 8; i < 12; i += 3) //后膝
  {
    DataCtrl::_des_jpos[i] = q_des_rear[2];
    DataCtrl::_des_jvel[i] = qd_des_rear[2];
    DataCtrl::_jtorque[i] = tau_rear[2];
  }

  DataCtrl::current_iteration += DataCtrl::_key_pt_step;//更新频率0.5kHz
}


template class FrontJumpCtrl<double>;
template class FrontJumpCtrl<float>;
