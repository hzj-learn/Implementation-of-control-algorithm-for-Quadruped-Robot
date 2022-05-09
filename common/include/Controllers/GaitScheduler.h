/*!
 * @file GaitScheduler.h
 * @brief 固定步态计时逻辑
 */

#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include <iostream>

#include "cppTypes.h"
#include "../../user/MIT_Controller/MIT_UserParameters.h"

/**
 * 列举步态类型。预先计划的步态已定义。
 * 枚举类型15种
 */
enum class GaitType
{
  STAND,              //(1)站立STAND
  STAND_CYCLE,        //(2)站立周期STAND_CYCLE
  STATIC_WALK,        //(3)静态行走STATIC_WALK
  AMBLE,              //(4)慢跑AMBLE
  TROT_WALK,          //(5)快步走TROT_WALK
  TROT,               //(6)对角小跑TROT
  TROT_RUN,           //(7)快跑TROT_RUN
  PACE,               //(8)散步PACE
  BOUND,              //(9)跳步BOUND
  ROTARY_GALLOP,      //(10)旋转跑ROTARY_GALLOP
  TRAVERSE_GALLOP,    //(11)打横跑TRAVERSE_GALLOP
  PRONK,              //(12)俯卧PRONK
  THREE_FOOT,         //(13)三条腿THREE_FOOT
  CUSTOM,             //(14)自定义CUSTOM
  TRANSITION_TO_STAND //(15)过渡到站立TRANSITION_TO_STAND
};

/**
 * 功能：步态时间信息
 */
template <typename T>
struct GaitData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GaitData() { zero(); }
  void zero();            // 把所有的数据归零
  GaitType _currentGait;  // 当前GaitType
  GaitType _nextGait;     // 下一步GaitType过渡到
  std::string gaitName;   // 步态的名字字符串

  //  步态描述符
  T periodTimeNominal;     // 总周期时间
  T initialPhase;          // 初始偏移相位
  T switchingPhaseNominal; // 标称相位切换点
  int overrideable;        // 允许步态参数被覆写

  //  启用每只脚的标记
  Eigen::Vector4i gaitEnabled; //  启动步态控制腿

  // 基于时间的描述符
  Vec4<T> periodTime;          // 整个步态时间
  Vec4<T> timeStance;          // 总站立时间
  Vec4<T> timeSwing;           // 总摆动时间
  Vec4<T> timeStanceRemaining; // 剩余站立时间
  Vec4<T> timeSwingRemaining;  // 剩余摆动时间

  //  阶段基于描述符
  Vec4<T> switchingPhase; // 相切换到swing
  Vec4<T> phaseVariable;  // 每只脚的整体步态阶段
  Vec4<T> phaseOffset;    // 步态相位偏移
  Vec4<T> phaseScale;     // 相对于变量的相位刻度
  Vec4<T> phaseStance;    // 支撑相中相位
  Vec4<T> phaseSwing;     // 摆动相中相位

  //  预定的接触状态
  Eigen::Vector4i contactStateScheduled; //  脚的接触状态
  Eigen::Vector4i contactStatePrev;      //  脚的先前接触状态
  Eigen::Vector4i touchdownScheduled;    //  预定的触地事件标志
  Eigen::Vector4i liftoffScheduled;      //  预定离地事件标值
};

/**
 * 功能：处理步态数据和计划的步伐和摆动。
 */
template <typename T>
class GaitScheduler
{
public:
  GaitScheduler(MIT_UserParameters *_userParameters, float _dt);   //GaitScheduler的构造函数
  ~GaitScheduler(){};                                              //GaitScheduler的析构函数

  void initialize();         //初始化步态调度程序

  void step();               //调度程序逻辑的迭代步骤

  // 从预定义库创建新步态
  void modifyGait();             //修改步态
  void createGait();             //创建步态
  void calcAuxiliaryGaitData();  //计算辅助步态数据
  void printGaitInfo();          //打印特征信息和当前状态
  GaitData<T> gaitData;          //储存所有步态相关数据

  // 自然步态参数
  T period_time_natural = 0.5;         //周期时间
  T switching_phase_natural = 0.5;     //摆动向接触切换点
  T swing_time_natural = 0.25;          //摆动时间

private:

  // Quadruped<T>& _quadruped;         //四足动物模型
  MIT_UserParameters *userParameters;  //用户参数
  T dt;                                //控制回路时间步长变化
  T dphase;                            //每一步的相变
  int printNum = 5;                    //选择每隔N次迭代打印信息的频率 N*(0.001s) in simulation time
  int printIter = 0;                   // 跟踪自上次信息打印以来的迭代次数
};

#endif