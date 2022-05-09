#ifndef FSM_STATE_RECOVERY_STANDUP_H
#define FSM_STATE_RECOVERY_STANDUP_H

#include "FSM_State.h"


template <typename T>
class FSM_State_RecoveryStand : public FSM_State<T> 
{
 public:
  FSM_State_RecoveryStand(ControlFSMData<T>* _controlFSMData);
  void onEnter();                               //进入状态时要执行的行为
  void run();                                   //运行状态的正常行为
  FSM_StateName checkTransition();              //检查是否有任何转换触发器
  TransitionData<T> transition();               //管理特定于状态的转换
  void onExit();                                //退出状态时要执行的行为
  TransitionData<T> testTransition();

 private:

  int iter = 0;                                  //跟踪控制迭代
  int _motion_start_iter = 0;

  static constexpr int StandUp = 0;
  static constexpr int FoldLegs = 1;
  static constexpr int RollOver = 2;

  unsigned long long _state_iter;
  int _flag = FoldLegs;

  // JPos
  Vec3<T> fold_jpos[4];
  Vec3<T> stand_jpos[4];
  Vec3<T> rolling_jpos[4];
  Vec3<T> initial_jpos[4];
  Vec3<T> zero_vec3;

  Vec3<T> f_ff;

  // iteration setup
  // 0.5 kHz
  const int rollover_ramp_iter = 150;
  const int rollover_settle_iter = 150;

  const int fold_ramp_iter = 400;
  const int fold_settle_iter = 700;

  const int standup_ramp_iter = 250;
  const int standup_settle_iter = 250;

  void _RollOver(const int & iter);
  void _StandUp(const int & iter);
  void _FoldLegs(const int & iter);

  bool _UpsideDown();
  void _SetJPosInterPts(
      const size_t & curr_iter, size_t max_iter, int leg, 
      const Vec3<T> & ini, const Vec3<T> & fin);

};

#endif  // FSM_STATE_RECOVERY_STANDUP_H
