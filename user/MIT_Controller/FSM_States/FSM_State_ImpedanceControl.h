#ifndef FSM_STATE_IMPEDANCECONTROL_H
#define FSM_STATE_IMPEDANCECONTROL_H

#include "FSM_State.h"

/**
 *
 */
template <typename T>
class FSM_State_ImpedanceControl : public FSM_State<T> 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_ImpedanceControl(ControlFSMData<T>* _controlFSMData);
  void onEnter();                     //进入状态时要执行的行为
  void run();                         //运行状态的正常行为
  FSM_StateName checkTransition();    //检查是否有任何转换触发器
  TransitionData<T> transition();     //管理特定于状态的转换
  void onExit();                      //退出状态时要执行的行为

 private:
  int iter = 0;                       //跟踪控制迭代
};

#endif  // FSM_STATE_IMPEDANCECONTROL_H
