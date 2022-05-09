/*============================= Joint PD ==============================*/
/**
 * 允许PD控制关节的FSM状态。
 */

#include "FSM_State_JointPD.h"
#include <Configuration.h>

/**
 *功能：向传递状态特定信息的FSM状态的构造函数
 *备注:通用FSM状态构造函数。
 * @param _controlFSMData 保存所有相关的控制数据
 */
template <typename T>
FSM_State_JointPD<T>::FSM_State_JointPD(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::JOINT_PD, "JOINT_PD"),
_ini_jpos(cheetah::num_act_joint)
{
  //现在什么也不做
}


/**
 *功能：进入状态时要执行的行为
 */
template <typename T>
void FSM_State_JointPD<T>::onEnter() 
{
  this->nextStateName = this->stateName;//默认为不过渡
  this->transitionData.zero();          //重置转换数据
  iter = 0;                             //重置计数器
  for(size_t leg(0); leg<4; ++leg)//每个关节位置信息
  {
    for(size_t jidx(0); jidx <3; ++jidx)
    {
      _ini_jpos[3*leg + jidx] = FSM_State<T>::_data->_legController->datas[leg].q[jidx];
    }
  }
}



/**
 * 功能：在每个控制循环迭代中调用要执行的函数。
 */
template <typename T>
void FSM_State_JointPD<T>::run() 
{
  // 这只是一个测试，应该运行您想要的任何其他代码  
  Vec3<T> qDes;
  qDes << 0, -1.052, 2.63;
  Vec3<T> qdDes;
  qdDes << 0, 0, 0;

  static double progress(0.);
  progress += this->_data->controlParameters->controller_dt;
  double movement_duration(3.0);
  double ratio = progress/movement_duration;
  if(ratio > 1.) ratio = 1.;

  this->jointPDControl(0, ratio*qDes + (1. - ratio)*_ini_jpos.head(3), qdDes);
  this->jointPDControl(1, ratio*qDes + (1. - ratio)*_ini_jpos.segment(3, 3), qdDes);
  this->jointPDControl(2, ratio*qDes + (1. - ratio)*_ini_jpos.segment(6, 3), qdDes);
  this->jointPDControl(3, ratio*qDes + (1. - ratio)*_ini_jpos.segment(9, 3), qdDes);
}

/**
 *功能：管理用户可以转换为哪种状态
 *命令或状态事件触发器。
 * @return  要转换为的枚举FSM状态名
 */
template <typename T>
FSM_StateName FSM_State_JointPD<T>::checkTransition() 
{
  this->nextStateName = this->stateName;
  iter++;
  switch ((int)this->_data->controlParameters->control_mode)  //切换FSM控制模式
  {
    case K_JOINT_PD:                                          //基于状态的转换的正常操作
      break;

    case K_IMPEDANCE_CONTROL:                                 //要求更改阻抗控制
      this->nextStateName = FSM_StateName::IMPEDANCE_CONTROL;
      this->transitionDuration = 1.0;                         //转换时间为1秒
      break;

    case K_STAND_UP:                                          //要求更改阻抗控制
      this->nextStateName = FSM_StateName::STAND_UP;
      this->transitionDuration = 0.0;                         //过渡时间很快
      break;

    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;     //要求更改平衡架
      break;

    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;           //要求更改平衡表
      this->transitionDuration = 0.0;                         //过渡时间很快

      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_JOINT_PD << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
  return this->nextStateName;                                 //获取下一个状态
}



/**
 *功能：处理robot在状态之间的实际转换。
 *转换完成时返回true。
 * @return  如果转换完成，则为true
 */
template <typename T>
TransitionData<T> FSM_State_JointPD<T>::transition() 
{
  switch (this->nextStateName)                    //切换FSM控制模式
   {
    case FSM_StateName::IMPEDANCE_CONTROL:          //阻抗控制
      iter++;
      if (iter >= this->transitionDuration * 1000) 
      {
        this->transitionData.done = true;
      } 
      else 
      {
        this->transitionData.done = false;
      }
      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_JOINT_PD << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
  this->transitionData.done = true;               //完成过渡
  return this->transitionData;                    //将转换数据返回到FSM
}



/**
 * 在退出状态时清除状态信息。
 */
template <typename T>
void FSM_State_JointPD<T>::onExit()
{
  //退出时无需清理
}

// template class FSM_State_JointPD<double>;
template class FSM_State_JointPD<float>;
