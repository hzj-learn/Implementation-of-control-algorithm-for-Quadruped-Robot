//摆动腿轨迹跟踪控制器，参数文件

#ifndef PROJECT_JPOSUSERPARAMETERS_H
#define PROJECT_JPOSUSERPARAMETERS_H

#include "ControlParameters/ControlParameters.h"


//JPosUserParameters是ControlParameters 的一个友元类
class JPosUserParameters : public ControlParameters 
{
public:
  JPosUserParameters()
      : ControlParameters("user-parameters"),
        INIT_PARAMETER(tau_ff),
        INIT_PARAMETER(kp),
        INIT_PARAMETER(kd),
        INIT_PARAMETER(zero),
        INIT_PARAMETER(calibrate)
      {}

  DECLARE_PARAMETER(double, tau_ff);
  DECLARE_PARAMETER(double, kp);
  DECLARE_PARAMETER(double, kd);
  DECLARE_PARAMETER(double, zero);
  DECLARE_PARAMETER(double, calibrate);
};

#endif //PROJECT_JPOSUSERPARAMETERS_H
