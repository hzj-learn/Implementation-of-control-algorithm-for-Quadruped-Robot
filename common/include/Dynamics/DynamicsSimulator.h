/*! @file DynamicsSimulator.h
 *  @brief  碰撞刚体动力学模拟器
 * 结合ABA、碰撞、积分器和任何其他外力来运行模拟。不做任何图形。
 */

#ifndef PROJECT_DYNAMICSSIMULATOR_H
#define PROJECT_DYNAMICSSIMULATOR_H

#include "Collision/CollisionBox.h"
#include "Collision/CollisionMesh.h"
#include "Collision/CollisionPlane.h"
#include "Collision/ContactConstraint.h"
#include "FloatingBaseModel.h"
#include "Math/orientation_tools.h"
#include "spatial.h"

using namespace ori;
using namespace spatial;
#include <eigen3/Eigen/Dense>

//设置机器人最初的参数
template <typename T>
struct RobotHomingInfo 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec3<T> position;   //躯干位置
  Vec3<T> rpy;        //躯干三元数
  double kp_lin;      //躯干的kp参数
  double kd_lin;      //躯干的kd参数
  double kp_ang;      //腿部的kp参数
  double kd_ang;      //腿部的kd参数
  bool active_flag;   //运动标志位
};

/*!
 * 功能：浮基系统动力学仿真类（包含状态）
 */
template <typename T>
class DynamicsSimulator 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DynamicsSimulator(FloatingBaseModel<T>& model,bool useSpringDamper = false);  //用给定的模型初始化模拟器
  
  void step(T dt, const DVec<T>& tau, T kp,T kd);                               //模拟前进一步
 
  void runABA(const DVec<T>& tau)                                               //用关节体算法求状态
  { _model.runABA(tau, _dstate); }
  
  void forwardKinematics()                                       //计算腿的正向运动学
  { _model.forwardKinematics(); }  
  
  void integrate(T dt);                                          //整合以找到新的状态
 
  void setState(const FBModelState<T>& state)                    //设置正在模拟的机器人的状态
  {
    _state = state;
    _model.setState(state);  //力重新计算动力学
  }
 
  void setHoming(const RobotHomingInfo<T>& homing)               //设置机器人最初的参数
  {
    _homing = homing;
  }
 
  const FBModelState<T>& getState() const                        //获取机器人的状态
  { return _state; }
 
  const FBModelStateDerivative<T>& getDState() const             //获取最近计算的状态导数（由runABA更新）
  { return _dstate; }
 
  void setAllExternalForces(const vectorAligned<SVec<T>>& forces)//增加外力。这些是加在地面反作用力之上的，第i力是作用在物体i上的空间力。在模拟器的每一步之后都会被清除
  {
    _model._externalForces = forces;
  }
 
  void addCollisionPlane(T mu, T rest, T height)                 //添加碰撞平面
  {
    _contact_constr->AddCollision(new CollisionPlane<T>(mu, rest, height));
  }
 
  void addCollisionBox(T mu, T rest, T depth, T width, T height, //添加碰撞框
                       const Vec3<T>& pos, const Mat3<T>& ori) {
    _contact_constr->AddCollision(
        new CollisionBox<T>(mu, rest, depth, width, height, pos, ori));
  }
 
  void addCollisionMesh(T mu, T rest, T grid_size,               //添加碰撞网格
                        const Vec3<T>& left_corner_loc,
                        const DMat<T>& height_map)  {
    _contact_constr->AddCollision(                               //添加碰撞网格
        new CollisionMesh<T>(mu, rest, grid_size, left_corner_loc, height_map));
  }
 
  size_t getNumBodies() { return _model._nDof; }
 
  const size_t& getTotalNumGC()                                  //获取接触到地面的腿的数目
  { return _model._nGroundContact; }
 
  const Vec3<T>& getContactForce(size_t idx)                     //获取接触到地面的腿的反馈力矩
  {
    return _contact_constr->getGCForce(idx);
  }
 
  const FloatingBaseModel<T>& getModel()                         //获取浮基模型
  { return _model; }

 private:
  void updateCollisions(T dt, T kp, T kd);                       //更新地面碰撞列表
  FBModelState<T> _state;
  FBModelStateDerivative<T> _dstate;
  FloatingBaseModel<T>& _model;
  vector<CollisionPlane<T>> _collisionPlanes;
  ContactConstraint<T>* _contact_constr;
  SVec<T> _lastBodyVelocity;
  bool _useSpringDamper;
  RobotHomingInfo<T> _homing;
};

#endif  // PROJECT_DYNAMICSSIMULATOR_H
