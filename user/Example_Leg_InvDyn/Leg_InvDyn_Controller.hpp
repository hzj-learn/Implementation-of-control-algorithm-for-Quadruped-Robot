//设置并启动腿部逆动力学控制器h文件

#ifndef JPOS_CONTROLLER
#define JPOS_CONTROLLER

#include <RobotController.h>
#include "Leg_InvDyn_UserParameters.h"


//Leg_InvDyn_Controller是RobotController的一个友元类
class Leg_InvDyn_Controller:public RobotController
{
  public:
    Leg_InvDyn_Controller():RobotController(){}  //计算腿部关节你动力学解算，最终输出每个关节的力矩
    virtual ~Leg_InvDyn_Controller(){}
    virtual void initializeController(){}
    virtual void runController();
    virtual void updateVisualization(){}
    virtual ControlParameters* getUserControlParameters() 
    {
      return &userParameters;
    }
  protected:
    Leg_InvDyn_UserParameters userParameters;
};

#endif
