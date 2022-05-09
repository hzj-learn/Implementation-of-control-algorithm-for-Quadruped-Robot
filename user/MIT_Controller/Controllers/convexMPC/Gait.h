#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H

#include <string>
#include <queue>

#include "cppTypes.h"


class Gait 
{
public:
  virtual ~Gait() = default;
//虚函数 在之后定义
  virtual Vec4<float> getContactState() = 0;                       //（1）获得四足接触状态
  virtual Vec4<float> getSwingState() = 0;                         //（2）获得四足摆动状态
  virtual int* getMpcTable() = 0;                                  //（3）获得mpc需要的gait数组 悬空与否
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;//（4）设置MPC迭代
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;   //（5）获得这次支撑的持续时间
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;    //（6）获得这次摆动的持续时间
  virtual int getCurrentGaitPhase() = 0;                          //（7）获得当前步态所处相位0~1
  virtual void debugPrint() {}

protected:
  std::string _name;
};

using Eigen::Array4f;
using Eigen::Array4i;

class OffsetDurationGait : public Gait 
{
public:
//步态周期分段数 相位差 支撑持续时间（按分段算） 步态名称
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~OffsetDurationGait();
  //同上
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  int* _mpc_table;
  //（按分段算）
  Array4i _offsets;         // offset in mpc segments 相位差
  Array4i _durations;       // duration of step in mpc segments 支撑持续时间
  //按百分比算
  Array4f _offsetsFloat;     // offsets in phase (0 to 1)相位差
  Array4f _durationsFloat;   // durations in phase (0 to 1)支撑持续时间
  
  int _stance;              //支撑时间，按分段算
  int _swing;               //摆动时间 分段算
  int _iteration;           //步态片段计数
  int _nIterations;         //步态片段数
  float _phase;             //当前相位
};


//基本同上
class MixedFrequncyGait : public Gait 
{
public:
  MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string& name);
  ~MixedFrequncyGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  float _duty_cycle;
  int* _mpc_table;
  Array4i _periods;
  Array4f _phase;
  int _iteration;
  int _nIterations;
};

#endif //PROJECT_GAIT_H
