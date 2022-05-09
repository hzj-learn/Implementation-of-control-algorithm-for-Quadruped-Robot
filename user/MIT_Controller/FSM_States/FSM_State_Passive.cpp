/*============================== Passive ==============================*/
/**
 * FSM声明不调用任何控件。意味着一个安全的状态
 * 机器人不应该做任何事情，因为所有的命令将被设置为0。
 */

#include "FSM_State_Passive.h"

/**
  *功能：向传递状态特定信息的FSM状态的构造函数
  *通用FSM状态构造函数。
  * @param _controlFSMData  保存所有相关的控制数据
 */
template <typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE")
{
  // 啥也不做
  this->checkSafeOrientation = false;          //关闭设置预控制安全检查
  this->checkPDesFoot = false;                 //关闭控制后安全检查
  this->checkForceFeedForward = false;
}



/**
 *功能：进入状态时要执行的行为
 */
template <typename T>
void FSM_State_Passive<T>::onEnter()
{
  this->nextStateName = this->stateName;      //默认是不过渡
  this->transitionData.zero();                //重置转换数据
}



/**
 * 功能：调用要在每个控制循环迭代中执行的函数
 */
template <typename T>
void FSM_State_Passive<T>::run()
{
  testTransition();           //什么都不做，所有的命令都应该以0开头
}



/**
 * 功能：处理机器人在状态之间的实际转换。在转换完成时返回true。
 * @return  如果转换完成，则为true
 */
template <typename T>
TransitionData<T> FSM_State_Passive<T>::testTransition()
{
  this->transitionData.done = true;
  return this->transitionData;
}



/**
 * 功能：状态转移检查：管理哪些状态可以由用户命令或状态事件触发器切换
 * @return 要转换为的枚举FSM状态名
 */
template <typename T>
FSM_StateName FSM_State_Passive<T>::checkTransition()
{
  this->nextStateName = this->stateName;                        //把这一次的状态名称赋值给下一次的状态名称
  iter++;
  switch ((int)this->_data->controlParameters->control_mode)    //切换FSM控制模式
  {
  case K_PASSIVE:  //基于状态的转换的正常操作
    break;

  case K_JOINT_PD:
    this->nextStateName = FSM_StateName::JOINT_PD;                
    break;

  case K_STAND_UP:
    this->nextStateName = FSM_StateName::STAND_UP;              
    break;

  case K_RECOVERY_STAND:
    this->nextStateName = FSM_StateName::RECOVERY_STAND;        
    break;

  default:
    std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
              << K_PASSIVE << " to "
              << this->_data->controlParameters->control_mode << std::endl;
  }
  return this->nextStateName;                                   //得到下一个状态
}



/**
 *功能：状态转移， 处理机器人在状态之间的实际转换
 * @return 如果转换完成，则为true
 */
template <typename T>
TransitionData<T> FSM_State_Passive<T>::transition()
{
  this->transitionData.done = true;   //完成过渡
  return this->transitionData;        //将转换数据返回到FSM
}



/**
*在退出状态时清除状态信息。
 */
template <typename T>
void FSM_State_Passive<T>::onExit()
{
//退出时无需清理
}

// template class FSM_State_Passive<double>;
template class FSM_State_Passive<float>;
