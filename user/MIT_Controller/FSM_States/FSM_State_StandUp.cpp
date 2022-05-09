/*============================= Stand Up ==============================*/
/**
 *要求机器人站起来的过渡状态
 *平衡控制模式。
 */

#include "FSM_State_StandUp.h"

/**
  *功能：向传递状态特定信息的FSM状态的构造函数
  *通用FSM状态构造函数。
  * @param _controlFSMData  保存所有相关的控制数据
 */
template <typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
_ini_foot_pos(4)
{
  this->checkSafeOrientation = false;           //设置预控安全检查
  this->checkPDesFoot = false;                  //位置控制安全检查
  this->checkForceFeedForward = false;
}


/**
 *功能：进入状态时要执行的行为
 */
template <typename T>
void FSM_State_StandUp<T>::onEnter() 
{
  this->nextStateName = this->stateName;    //默认为不转换
  this->transitionData.zero();              //重置转换数据
  iter = 0;                                 //重置迭代计数器
  for(size_t leg(0); leg<4; ++leg)          //设置4个腿的位置
  {
    _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
  }
}


/**
 * 调用要在每个控制循环迭代中执行的函数。
 * 原理
 * 给定站立的高度，根据运动控制周期把站立过程的占空比不断增大，直到完成站立状态（占空比为1）
 * 给定增益Kp/Kd参数，根据占空比计算脚的z位置，按照控制周期，把脚的控制参数发送给腿部控制器执行
 */
template <typename T>
void FSM_State_StandUp<T>::run() 
{
  if(this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH)    //仅仅只有MINI_CHEETAH
  {
    T hMax = 0.25;//站立高度
    T progress = 2 * iter * this->_data->controlParameters->controller_dt;
    if (progress > 1.)            //站立高度过程占空比限幅
    { progress = 1.; }

    for(int i = 0; i < 4; i++)    //更新腿的信息
    {
      this->_data->_legController->commands[i].kpCartesian = Vec3<T>(500, 500, 500).asDiagonal();   //Kp返回参数
      this->_data->_legController->commands[i].kdCartesian = Vec3<T>(8, 8, 8).asDiagonal();         //Kd返回参数
      this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];                             //脚的位置x不变
      this->_data->_legController->commands[i].pDes[2] =                                            //脚的位置z高度，按照周期不断变高
        progress*(-hMax) + (1. - progress) * _ini_foot_pos[i][2];
    }
  }
}



/**
 * 功能：状态转移检查：管理哪些状态可以由用户命令或状态事件触发器切换
 * @return 要转换为的枚举FSM状态名
 */
template <typename T>
FSM_StateName FSM_State_StandUp<T>::checkTransition() 
{
  this->nextStateName = this->stateName;//把这一次的状态名称赋值给下一次的状态名称
  iter++;
  switch ((int)this->_data->controlParameters->control_mode) //切换FSM控制模式
  {
    case K_STAND_UP:
      break;
    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      break;

    case K_PASSIVE:  
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_PASSIVE << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
  return this->nextStateName; //获取下一个状态
}


/**
 *功能：状态转移， 处理机器人在状态之间的实际转换
 * @return 如果转换完成，则为true
 */
template <typename T>
TransitionData<T> FSM_State_StandUp<T>::transition() 
{
  switch (this->nextStateName)    //完成转换
  {
    case FSM_StateName::PASSIVE: 
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }
  return this->transitionData;//将转换数据返回到FSM
}



/**
 * 功能：在退出状态时清除状态信息。
 */
template <typename T>
void FSM_State_StandUp<T>::onExit()
{
  // 退出时无需清理
}
// template class FSM_State_StandUp<double>;
template class FSM_State_StandUp<float>;
