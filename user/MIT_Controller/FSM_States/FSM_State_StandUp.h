#ifndef FSM_STATE_STANDUP_H
#define FSM_STATE_STANDUP_H

#include "FSM_State.h"


template <typename T>
class FSM_State_StandUp : public FSM_State<T> 
{
 public:
  FSM_State_StandUp(ControlFSMData<T>* _controlFSMData);
  void onEnter();                                           //进入该状态时要执行的行为
  void run();                                               //运行该状态的正常行为
  FSM_StateName checkTransition();                          //检查是否有触发状态转换器
  TransitionData<T> transition();                           //管理特定于状态的转换
  void onExit();                                            //退出该状态时要执行的行为
  TransitionData<T> testTransition();

 private:
  int iter = 0;                                             //跟踪控制迭代
  std::vector< Vec3<T> > _ini_foot_pos;
};

#endif  // FSM_STATE_STANDUP_H
