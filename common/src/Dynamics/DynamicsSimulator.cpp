/*! @file DynamicsSimulator.cpp
 *  @brief  碰撞刚体动力学模拟器
 *  结合ABA、碰撞、积分器和任何其他外力来运行模拟。不做任何图形。
 */

#include "Dynamics/DynamicsSimulator.h"
#include "Collision/ContactImpulse.h"
#include "Collision/ContactSpringDamper.h"
#include "Utilities/Utilities_print.h"

/*!
 * 功能：通过为ABA矩阵分配内存来初始化动力学模拟器
 */
template <typename T>
DynamicsSimulator<T>::DynamicsSimulator(FloatingBaseModel<T> &model,
                                        bool useSpringDamper)
    : _model(model), _useSpringDamper(useSpringDamper) 
{
  if (_useSpringDamper) //摆动腿。使用浮及动力学模型计算施加与地面接触的物体的外力
  {
    _contact_constr = new ContactSpringDamper<T>(&_model);
  }
  else                  //支撑腿。使用浮及动力学模型计算施加与地面接触的物体的外力
  {
    _contact_constr = new ContactImpulse<T>(&_model);
  }

  _state.bodyVelocity = SVec<T>::Zero();          //躯干速度
  _state.bodyPosition = Vec3<T>::Zero();          //躯干位置
  _state.bodyOrientation = Quat<T>::Zero();       //躯干方向
  _state.q = DVec<T>::Zero(_model._nDof - 6);     //腿的角度
  _state.qd = DVec<T>::Zero(_model._nDof - 6);    //腿的角速度
  _dstate.qdd = DVec<T>::Zero(_model._nDof - 6);  //腿的角加速度
  _lastBodyVelocity.setZero();
}

/*!
 * 功能：采取一个模拟步骤迭代
 * @param dt : 时间步持续时间
 * @param tau : 关节力矩
 */
template <typename T>
void DynamicsSimulator<T>::step(T dt, const DVec<T> &tau, T kp, T kd ) 
{
  forwardKinematics();           //计算腿的正向运动学
  updateCollisions(dt, kp, kd);  // 处理冲突
  // 进程归位
  if( _homing.active_flag) 
  {
    Mat3<T> R10_des = rpyToRotMat(_homing.rpy);              // R10_des
    Mat3<T> R10_act = _model.getOrientation(5).transpose();  // R10
    Mat3<T> eR01 = R10_des.transpose()*R10_act;              // eR * R01 = R01_des
    
    Vec4<T> equat = rotationMatrixToQuaternion(eR01.transpose());
    Vec3<T> angle_axis = quatToso3(equat); // in world frame

    Vec3<T> p = _model.getPosition(5);
    Vec3<T> f = _homing.kp_lin*(_homing.position - p)-_homing.kd_lin*_model.getLinearVelocity(5);

    //Note: External forces are spatial forces in the {0} frame.
    _model._externalForces.at(5) += forceToSpatialForce(f,p);
    _model._externalForces.at(5).head(3) += _homing.kp_ang*angle_axis - _homing.kd_ang*_model.getAngularVelocity(5);

  }

  runABA(tau);                   // 动力学算法
  integrate(dt);                 // 前进一步

  _model.setState(_state);
  _model.resetExternalForces();  // 清除外力
  _model.resetCalculationFlags();
}


template <typename T>
void DynamicsSimulator<T>::updateCollisions(T dt, T kp, T kd) 
{
  _model.forwardKinematics();
  _contact_constr->UpdateExternalForces(kp, kd, dt);
}



/*!
 * 功能：整合浮动基态
 * @param dt timestep
 */
template <typename T>
void DynamicsSimulator<T>::integrate(T dt) 
{
  if (_useSpringDamper) {
    Vec3<T> omegaBody = _state.bodyVelocity.template block<3, 1>(0, 0);
    Mat6<T> X = createSXform(quaternionToRotationMatrix(_state.bodyOrientation),
                             _state.bodyPosition);
    RotMat<T> R = rotationFromSXform(X);
    Vec3<T> omega0 = R.transpose() * omegaBody;

    // 实际集成
    _state.qd += _dstate.qdd * dt;
    _state.q += _state.qd * dt;

    _state.bodyVelocity += _dstate.dBodyVelocity * dt;
    _state.bodyPosition += _dstate.dBodyPosition * dt;
    _state.bodyOrientation = integrateQuat(_state.bodyOrientation, omega0, dt);
  } else {
    //实际集成
    //加速度积分速度更新
    _state.qd += _dstate.qdd * dt;
    _state.bodyVelocity += _dstate.dBodyVelocity * dt;

    //接触约束速度已更新
    _contact_constr->UpdateQdot(_state);

    //准备体速度积分
    RotMat<T> R_body = quaternionToRotationMatrix(_state.bodyOrientation);

    _dstate.dBodyPosition =
        R_body.transpose() * _state.bodyVelocity.template block<3, 1>(3, 0);
    Vec3<T> omegaBody = _state.bodyVelocity.template block<3, 1>(0, 0);

    //位置更新
    _state.q += _state.qd * dt;
    _state.bodyPosition += _dstate.dBodyPosition * dt;
    _state.bodyOrientation =
        integrateQuatImplicit(_state.bodyOrientation, omegaBody, dt);
    _dstate.dBodyVelocity = (_state.bodyVelocity - _lastBodyVelocity) / dt;
    _lastBodyVelocity = _state.bodyVelocity;
  }
}

template class DynamicsSimulator<double>;

template class DynamicsSimulator<float>;
