#ifndef FSM_STATE_PASSIVE_H
#define FSM_STATE_PASSIVE_H

#include "FSM_State.h"

/**
 *
 */
template <typename T>
class FSM_State_Passive : public FSM_State<T> 
{
 public:
  FSM_State_Passive(ControlFSMData<T>* _controlFSMData);
  void onEnter();                      //进入一种状态时的行为
  void run();                          //运行状态的正常行为
  FSM_StateName checkTransition();     //检查任何转换触发器
  TransitionData<T> transition();      //管理特定于状态的转换
  void onExit();                       //退出状态时要执行的行为
  TransitionData<T> testTransition();

 private:
  int iter = 0;                       //跟踪控制迭代
};

#endif  // FSM_STATE_PASSIVE_H
