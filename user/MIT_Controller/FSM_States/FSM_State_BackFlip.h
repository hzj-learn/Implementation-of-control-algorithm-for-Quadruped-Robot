#ifndef FSM_STATE_BACKFLIP_H
#define FSM_STATE_BACKFLIP_H

#include "FSM_State.h"
#include <Controllers/BackFlip/DataReader.hpp>
#include <Controllers/BackFlip/BackFlipCtrl.hpp>


template <typename T>
class FSM_State_BackFlip : public FSM_State<T> 
{
 public:
  FSM_State_BackFlip(ControlFSMData<T>* _controlFSMData);
  void onEnter();                     //（1）进入状态时要执行的行为
  void run();                         //（2）运行状态的正常行为
  FSM_StateName checkTransition();    //（3）检查是否有任何转换触发器
  TransitionData<T> transition();     //（4）管理特定于状态的转换
  void onExit();                      //（5）退出状态时要执行的行为
  TransitionData<T> testTransition();

 private:
  //跟踪控制迭代
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
  
  void _SetJPosInterPts(                                       //（3） 计算并设置关节位置，执行控制
      const size_t & curr_iter, size_t max_iter, int leg, 
      const Vec3<T> & ini, const Vec3<T> & fin);

  DataReader* _data_reader;
  bool _b_running = true;
  bool _b_first_visit = true;
  int _count = 0;
  int _waiting_count = 6;
  float _curr_time = 0;
  BackFlipCtrl<T>* backflip_ctrl_;

  void SetTestParameter(const std::string& test_file);
  bool _Initialization();   //（0）空翻状态初始化
  void ComputeCommand();    //（1）计算命令
  void _SafeCommand();      //（2）安全命令

};

#endif  // FSM_STATE_BACKFLIP_H
