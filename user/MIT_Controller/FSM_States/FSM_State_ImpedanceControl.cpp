/*========================= Impedance Control =========================*/
/**
  *允许每一条腿在笛卡尔空间进行PD阻抗控制的FSM状态
 */

#include "FSM_State_ImpedanceControl.h"

/*
 * 功能：向传递状态特定信息的FSM状态的构造函数，generif FSM状态构造函数。
 * @param _controlFSMData 保存所有相关的控制数据
 */
template <typename T>
FSM_State_ImpedanceControl<T>::FSM_State_ImpedanceControl(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::IMPEDANCE_CONTROL,
                   "IMPEDANCE_CONTROL") 
{
//现在什么也不做
}


/**
 *功能：进入状态时要执行的行为
 */
template <typename T>
void FSM_State_ImpedanceControl<T>::onEnter() 
{
  this->nextStateName = this->stateName;//默认为不过渡
  this->transitionData.zero();          //重置转换数据
}



/**
 *功能： 调用要在每个控制循环迭代中执行的函数。
 */
template <typename T>
void FSM_State_ImpedanceControl<T>::run() 
{
  //不执行任何操作，所有命令都应以零开头 
}


/**
 *功能：管理用户可以转换为哪种状态
 *命令或状态事件触发器。
 * @return 要转换为的枚举FSM状态名
 */
template <typename T>
FSM_StateName FSM_State_ImpedanceControl<T>::checkTransition() 
{
  //获取下一个状态。无操作
  
  switch ((int)this->_data->controlParameters->control_mode) //切换FSM控制模式
  {
    case K_IMPEDANCE_CONTROL:                             //基于状态的转换的正常操作
      break;

    case K_BALANCE_STAND:                                 //要求更改平衡架
      this->nextStateName = FSM_StateName::BALANCE_STAND; //根据要求即时转换到移动状态
      this->transitionData.tDuration = 0.0;               //将调度程序中的下一步设置为
      this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << 0
                << " to " << this->_data->controlParameters->control_mode
                << std::endl;
  }

  return this->nextStateName;
}

/**
 *处理robot在状态之间的实际转换。
 *转换完成时返回true。
 * @return 如果转换完成，则为true
 */
template <typename T>
TransitionData<T> FSM_State_ImpedanceControl<T>::transition() 
{
  this->transitionData.done = true;
  return this->transitionData;//将转换数据返回到FSM
}



/**
 *在退出状态时清除状态信息。
 */
template <typename T>
void FSM_State_ImpedanceControl<T>::onExit() 
{
//退出时无需清理
}

// template class FSM_State_ImpedanceControl<double>;
template class FSM_State_ImpedanceControl<float>;