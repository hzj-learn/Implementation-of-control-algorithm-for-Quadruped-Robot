/*! @file ActuatorModel.h
 *  @brief 执行器模型包括摩擦、最大转矩和电机转矩转速曲线。
 *  getTorque用于关节处的扭矩，而不是电机处的扭矩。
 *  提供的摩擦是用于连接处的扭矩，而不是电机处的扭矩
 *  R/KT是给马达的
 */

#ifndef PROJECT_ACTUATORMODEL_H
#define PROJECT_ACTUATORMODEL_H

#include "Utilities/utilities.h"

template <typename T>
class ActuatorModel 
{
 public:
 //（1）建立关节执行器模型
  ActuatorModel(T gearRatio, T motorKT, T motorR, T batteryV, T damping,
                T dryFriction, T tauMax)
      : _gr(gearRatio),
        _kt(motorKT),
        _R(motorR),
        _V(batteryV),
        _damping(damping),
        _dryFriction(dryFriction),
        _tauMax(tauMax) {}

  ActuatorModel() {}

  //（2）考虑摩擦（干燥和阻尼）、电压限制和扭矩限制的基础上，给定期望扭矩和速度，计算实际的执行机构扭矩。
  T getTorque(T tauDes, T qd) 
  {
    T tauDesMotor = tauDes / _gr;        // 计算电动机转矩
    T iDes = tauDesMotor / (_kt * 1.5);  // 计算期望电流，公式：i = tau / KT     
    T bemf = qd * _gr * _kt * 2.;        // 计算反电动势，公式：T bemf =  qd * _gr * _kt * 1.732;
    T vDes = iDes * _R + bemf;           // 计算期望速度，公式：v = I*R + emf
    T vActual = coerce(vDes, -_V, _V);   // 电池电压限制
    T tauActMotor = 1.5 * _kt * (vActual - bemf) / _R;       // 计算执行电机力矩中间式，公式：tau = Kt * I = Kt * V / R
    T tauAct = _gr * coerce(tauActMotor, -_tauMax, _tauMax); // 计算执行电机力矩

    //增加阻尼和干摩擦
    if (_frictionEnabled)
      tauAct = tauAct - _damping * qd - _dryFriction * sgn(qd);

    return tauAct;// 执行电机力矩
  }

  void setFriction(bool enabled) { _frictionEnabled = enabled; }

 private:
  T _gr, _kt, _R, _V, _damping, _dryFriction, _tauMax;
  bool _frictionEnabled = true;
};

#endif  // PROJECT_ACTUATORMODEL_H
