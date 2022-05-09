//摆动腿轨迹跟踪控制器H文件
#ifndef JPOS_CONTROLLER
#define JPOS_CONTROLLER

#include <RobotController.h>
#include "JPosUserParameters.h"

//JPos_Controller是RobotController 的一个友元类
class JPos_Controller:public RobotController
{
  public:
    JPos_Controller():RobotController(),_jpos_ini(cheetah::num_act_joint)
    {
    _jpos_ini.setZero();
    }
    virtual ~JPos_Controller(){}

    virtual void initializeController(){}                            //初始化什么都不做
    virtual void runController();
    virtual void updateVisualization(){}                             //不更新可视化
    virtual ControlParameters* getUserControlParameters() 
    {
      return &userParameters;
    }
  protected:
    DVec<float> _jpos_ini;
  JPosUserParameters userParameters;
};

#endif
