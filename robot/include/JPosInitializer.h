/*!
 * @file JPosInitializer.h
 * @brief 在通电时控制器初始化腿的位置
 */

#ifndef JPOS_INITIALIZER
#define JPOS_INITIALIZER

#include "Controllers/LegController.h"
#include "Dynamics/Quadruped.h"
#include "Utilities/BSplineBasic.h"

/*!
 * 控制器在通电时初始化腿的位置
 */
template <typename T>
class JPosInitializer 
{
 public:
  JPosInitializer(T end_time, float dt);    //关节初始化，更新参数的构造函数
  ~JPosInitializer();                       //关节初始化，更新参数的析构函数

  bool IsInitialized(LegController<T>*);    //关节初始化，平稳运动函数【重点】

 private:
  void _UpdateParam();                      //更新控制器参数函数
  void _UpdateInitial(const LegController<T>* ctrl);//关节初始化，平稳移动函数
  bool _b_first_visit;
  T _end_time;
  T _curr_time;
  T _dt;

  std::vector<T> _ini_jpos;
  std::vector<T> _target_jpos;
  std::vector<T> _mid_jpos;

  BS_Basic<T, cheetah::num_act_joint, 3, 1, 2, 2> _jpos_trj;
};
#endif
