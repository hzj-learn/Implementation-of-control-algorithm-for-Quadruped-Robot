#ifndef FSM_STATE_FRONTJUMP_H
#define FSM_STATE_FRONTJUMP_H

#include "FSM_State.h"
#include <Controllers/BackFlip/DataReader.hpp>
#include <Controllers/BackFlip/FrontJumpCtrl.hpp>


/**
 *
 */
template <typename T>
class FSM_State_FrontJump : public FSM_State<T> 
{
 public:
  FSM_State_FrontJump(ControlFSMData<T>* _controlFSMData);
  void onEnter();                     //进入状态时要执行的行为
  void run();                         //运行状态的正常行为
  FSM_StateName checkTransition();    //检查是否有任何转换触发器
  TransitionData<T> transition();     //管理特定于状态的转换
  void onExit();                      //退出状态时要执行的行为
  TransitionData<T> testTransition(); //跟踪控制迭代

 private:

  int iter = 0;
  int _motion_start_iter = 0;
  static constexpr int Preparation = 0;
  static constexpr int Flip = 1;
  static constexpr int Landing = 2;
  unsigned long long _state_iter;
  int _flag = Preparation;

  //关节位置
  Vec3<T> initial_jpos[4];
  Vec3<T> zero_vec3;
  Vec3<T> f_ff;
  
  void _SetJPosInterPts(
      const size_t & curr_iter, size_t max_iter, int leg, 
      const Vec3<T> & ini, const Vec3<T> & fin);

  DataReader* _data_reader;
  bool _b_running = true;
  bool _b_first_visit = true;
  int _count = 0;
  int _waiting_count = 6;
  float _curr_time = 0;
  FrontJumpCtrl<T>* front_jump_ctrl_;

  void SetTestParameter(const std::string& test_file);
  bool _Initialization();
  void ComputeCommand();
  void _SafeCommand();

};

#endif  // FSM_STATE_FrontJump_H
