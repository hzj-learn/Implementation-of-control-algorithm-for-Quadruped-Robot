#ifndef FSM_STATE_BALANCESTAND_H
#define FSM_STATE_BALANCESTAND_H

#include "FSM_State.h"

template<typename T> class WBC_Ctrl;
template<typename T> class LocomotionCtrlData;

/**
 *
 */
template <typename T>
class FSM_State_BalanceStand : public FSM_State<T> 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData);
  void onEnter() override;        //进入状态时要执行的行为
  void run();                     //运行状态的正常行为
  FSM_StateName checkTransition();//检查是否有任何转换触发器
  TransitionData<T> transition(); //管理特定于状态的转换
  void onExit();                  //退出状态时要执行的行为



 private:
  int _iter = 0;                  //跟踪控制迭代
  void BalanceStandStep();        //将特定于触点的控件解析为腿部控制器

  WBC_Ctrl<T> * _wbc_ctrl;
  LocomotionCtrlData<T> * _wbc_data;

  T last_height_command = 0;

  Vec3<T> _ini_body_pos;
  Vec3<T> _ini_body_ori_rpy;
  T _body_weight;
};

#endif  // FSM_STATE_BALANCESTAND_H
