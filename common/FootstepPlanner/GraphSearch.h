#ifndef CHEETAH_SOFTWARE_GRAPHSEARCH_H
#define CHEETAH_SOFTWARE_GRAPHSEARCH_H

#include <vector>
#include "cppTypes.h"

//接触状态结构体
struct ContactState 
{
  union 
  {
    bool contact[4]; //四条腿的接触状态
    struct           //四条腿的名称
    {
      bool fr, fl, rr, rl;
    };
  };
  ContactState(bool _fr, bool _fl, bool _rr, bool _rl) //写四条腿的接触状态函数
  {
    fr = _fr;
    fl = _fl;
    rr = _rr;
    rl = _rl;
  }
  ContactState() { }
};


//默认的步态：对角小跑、站立
struct DefaultGaits 
{
  std::vector<ContactState> trotting, standing;
};

//输入的轨迹状态
struct InputTrajectoryState 
{
  Vec2<float> p;    //脚的位置
  Vec2<float> v;    //脚的速度
  float theta;
};


//足端规划状态
struct FootplanFootState 
{
  Vec2<float> p;    //脚的位置
  bool contact;     //脚的接触状态
  float stateTime;  //脚的时间
};


//躯干规划状态
struct FootplanState 
{
  float t;                    //时间
  Vec2<float> pBase;          //躯干位置
  FootplanFootState feet[4];  //四只脚的足端规划状态
};


//躯干规划状态
struct FootplanStats 
{
  u64 nodesVisited;
  u64 maxMemory;

  FootplanStats() 
  {
  reset();
  }

  void reset() 
  {
    nodesVisited = 0;
    maxMemory = 0;
  }
};


struct FootplanGoal 
{
  Vec2<float> goalPos;//目标位置
};

using FootplanStateCost = float (*)(FootplanState&, FootplanGoal&);
using FootplanTransitionCost = float (*)(FootplanState&, FootplanState&, FootplanGoal&);

namespace FootplanCosts 
{
  float distanceToGoal(FootplanState& state, FootplanGoal& goal);//角现在和目标状态的距离
}


//足端规划器
class FootstepPlanner 
{
public:
  FootstepPlanner(bool verbose);                                                              //(1)足端规划器函数
  void reset();                                                                               //(2)足端规划器复位函数                 
  void buildInputTrajectory(float duration, float dt, InputTrajectoryState x0, float omega);  //(3)建立输入足端规划轨迹函数
  void planFixedEvenGait(std::vector<ContactState>& gait, float gait_period);                 //(4)建立适合的事件步态函数
  std::vector<InputTrajectoryState>& getInitialTrajectory()                                   //(5)获取初始化足端轨迹
  {
    return _inputTrajectory;
  }

  void addCost(FootplanStateCost cost)      //                                              
  {
    _stateCosts.push_back(cost);
  }

  void addCost(FootplanTransitionCost cost) //
  {
    _transitionCosts.push_back(cost);
  }

  FootplanGoal& getGoal()                   //
  {
    return _goal;
  }

  DefaultGaits defaults;
private:
  bool _verbose;
  FootplanStats _stats;
  FootplanGoal _goal;

  std::vector<FootplanStateCost> _stateCosts;
  std::vector<FootplanTransitionCost> _transitionCosts;
  std::vector<InputTrajectoryState> _inputTrajectory;
};


#endif //CHEETAH_SOFTWARE_GRAPHSEARCH_H
