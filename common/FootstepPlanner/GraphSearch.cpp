#include "GraphSearch.h"
#include "Math/orientation_tools.h"


/**
 * 功能：计算足端现在的状态和期望的状态的距离函数
 */
float FootplanCosts::distanceToGoal(FootplanState &state, FootplanGoal &goal) 
{
  Vec2<float> dp = state.pBase - goal.goalPos;
  return dp.norm();
}


/**
 * 功能：足端规划器函数
 */
FootstepPlanner::FootstepPlanner(bool verbose) : _verbose(verbose) 
{
  _stats.reset();
  defaults.trotting = {{true, false, false, true},
                       {false, true, true, false}};
  defaults.standing = {{true, true, true, true}};
}


/**
 * 功能：足端规划器复位函数
 */
void FootstepPlanner::reset() 
{
  _stats.reset();
  _stateCosts.clear();
  _transitionCosts.clear();
}

/**
 * 功能：建立输入足端规划轨迹函数
 */
void FootstepPlanner::buildInputTrajectory(float duration, float dt, InputTrajectoryState x0, float omega) 
{
  if(_verbose) 
  {
    printf("Input trajectory with %d steps\n", (int)(duration / dt));
  }
  _inputTrajectory.clear();
  _inputTrajectory.reserve(duration / dt);

  Vec3<float> velocity(x0.v[0], x0.v[1], 0.f);
  Vec3<float> position(x0.p[0], x0.p[1], 0.f);
  float theta = x0.theta;
  float t = 0;
  for(uint32_t i = 0; i < (duration / dt); i++) 
  {
    Vec3<float> vRot = ori::coordinateRotation(ori::CoordinateAxis::Z, theta).transpose() * velocity;
    _inputTrajectory.push_back({{position[0], position[1]}, {vRot[0], vRot[1]}, theta});
    position += vRot * dt;
    t += dt;
    theta += omega * dt;
  }
}


/**
 * 功能：建立适合的事件步态函数
 */
void FootstepPlanner::planFixedEvenGait(std::vector<ContactState> &gait, float gait_period) 
{
  (void)gait;
  (void)gait_period;
}