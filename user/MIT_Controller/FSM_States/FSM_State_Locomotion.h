#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include <Controllers/convexMPC/ConvexMPCLocomotion.h>
#include "FSM_State.h"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;
/**
 *
 */
template <typename T>
class FSM_State_Locomotion : public FSM_State<T> 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData);
  void onEnter();                               //进入状态时要执行的行为
  void run();                                   //运行状态的正常行为
  FSM_StateName checkTransition();              //检查任何转换触发器
  TransitionData<T> transition();               //管理特定于状态的转换
  void onExit();                                //退出状态时要执行的行为

 private:
  int iter = 0;                                 //跟踪控制迭代
  ConvexMPCLocomotion* cMPCOld;                 //MPC控制器
  WBC_Ctrl<T> * _wbc_ctrl;                      //WBC控制器
  LocomotionCtrlData<T> * _wbc_data;            //移动控制器数据
  void LocomotionControlStep();                 //将特定于触点的控件解析为腿部控制器
  bool locomotionSafe();
  void StanceLegImpedanceControl(int leg);      //运动中支撑腿的阻抗控制
};

#endif  // FSM_STATE_LOCOMOTION_H
