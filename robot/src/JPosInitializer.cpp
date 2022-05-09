/*!
 * @file JPosInitializer.cpp
 * @brief 功能：控制器在通电时初始化腿的位置
 */

#include "JPosInitializer.h"
#include "Utilities/Utilities_print.h"
#include "ParamHandler/ParamHandler.hpp"
#include <Configuration.h>




/*!
 * 功能：关节初始化构造函数（设置目标时间及间隔）
 */
template <typename T>   //C++模板
JPosInitializer<T>::JPosInitializer(T end_time, float dt)
    : _b_first_visit(true),
      _end_time(end_time),
      _curr_time(0.),
      _dt(dt),
      _ini_jpos(cheetah::num_act_joint) 
{
  _UpdateParam();
}



/*!
 * 功能：关节初始化，更新参数析构函数
 */
template <typename T>
JPosInitializer<T>::~JPosInitializer() {}



/*!
 * 功能：按照B样条曲线平稳移动跟踪到匍匐状态的函数（进行关节初始化）
 */
template <typename T>
bool JPosInitializer<T>::IsInitialized(LegController<T>* ctrl) 
{
  _curr_time += _dt;
  // 初始设置
  if (_b_first_visit) //在第一次进来的时候更新控制指令
  {
    _UpdateInitial(ctrl); //给定并B样条起始点、中间点、落脚点，目标时间点四个参数
    _b_first_visit = false;
  }
  // 进行B样条曲线跟踪
  if (_curr_time < _end_time) 
  {
    T jpos[cheetah::num_act_joint];
    //获取给定时间的B样条线位置,存到jpos
    
    _jpos_trj.getCurvePoint(_curr_time, jpos);

  //遍历4条腿的三个电机，进行位置控制B样条曲线跟踪
  for (size_t leg(0); leg < cheetah::num_leg; ++leg)              
    {
      for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) 
      {
        ctrl->commands[leg].tauFeedForward[jidx] = 0.;            //设置关节前馈力矩为0
        ctrl->commands[leg].qDes[jidx] = jpos[3 * leg + jidx];    //设置关节期望位置
        ctrl->commands[leg].qdDes[jidx] = 0.;                     //设置关节期望速度=0
      }
    }
    return false;
  }
  return true;
}



/*!
 * 功能：给定并B样条起始点、中间点、落脚点，目标时间点四个参数
 * 步骤：
 * （1）定义变量
 * （2）复位每条腿的参数
 * （3）设置每条腿的参数：起始点、中间点、落脚点，目标时间点
 * （4）根据上面填充的数据，使用B样条的方法，进行更新关节轨迹参数
 */
template <typename T>
void JPosInitializer<T>::_UpdateInitial(const LegController<T>* ctrl) 
{
/*（1）定义起始点、中间点、落脚点变量*/
  T ini[3 * cheetah::num_act_joint];       //定义起始点
  T fin[3 * cheetah::num_act_joint];       //定义落脚点
  T** mid = new T*[1];                     //定义中间点
  mid[0] = new T[cheetah::num_act_joint];

/*（2）复位每条腿起始点和落点的参数为0*/
  for (size_t i(cheetah::num_act_joint); i < 3 * cheetah::num_act_joint; ++i) /
  {
    ini[i] = 0.;
    fin[i] = 0.;
  }
/*（3）加载每条腿三个关节起始点、中间点和落点位置的参数*/
//起始点
  for (int leg(0); leg < 4; ++leg)          //四条腿
  {
    for (int jidx(0); jidx < 3; ++jidx)     //每条腿的三个关节
    {
      ini[3 * leg + jidx]       = ctrl->datas[leg].q[jidx];   //加载对应的参数
      _ini_jpos[3 * leg + jidx] = ctrl->datas[leg].q[jidx];
    }
  }
//中间点和落脚点
  for (size_t i(0); i < cheetah::num_act_joint; ++i)  //加载对应的关节位置参数
  {
    fin[i] = _target_jpos[i];
    mid[0][i] = _mid_jpos[i];
  }
/*（4）根据上面填充的数据，使用B样条的方法，进行更新关节轨迹参数*/
  _jpos_trj.SetParam(ini, fin, mid, _end_time);      
  delete[] mid[0];
  delete[] mid;
}



/*!
 * 功能：关节初始化，更新控制器参数函数
 * 步骤：
 * （1）从yaml文件加载关节位置控制初始化参数，更新参数
 * （2）获得target_jpos矩阵
 * （3）获得mid_jpos矩阵
 */
template <typename T>
void JPosInitializer<T>::_UpdateParam() 
{
  ParamHandler handler(THIS_COM "config/initial_jpos_ctrl.yaml");   //从yaml文件加载关节位置控制初始化参数
  handler.getVector<T>("target_jpos", _target_jpos);                //_target_jpos矩阵
  handler.getVector<T>("mid_jpos", _mid_jpos);                      //_mid_jpos矩阵
}


template class JPosInitializer<double>;
template class JPosInitializer<float>;
