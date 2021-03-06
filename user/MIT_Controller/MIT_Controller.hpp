#ifndef MIT_CONTROLLER
#define MIT_CONTROLLER

#include <RobotController.h>
#include "Controllers/GaitScheduler.h"
#include "Controllers/ContactEstimator.h"
#include "FSM_States/ControlFSM.h"
#include "MIT_UserParameters.h"
//#include <gui_main_control_settings_t.hpp>

class MIT_Controller: public RobotController
{
public:
  MIT_Controller();
  virtual ~MIT_Controller(){}
  virtual void initializeController();
  virtual void runController();
  virtual void updateVisualization(){}
  virtual ControlParameters* getUserControlParameters() 
  {
    return &userParameters;
  }
  virtual void Estop()
  {_controlFSM->initialize(); }
protected:
  ControlFSM<float>* _controlFSM;
  GaitScheduler<float>* _gaitScheduler;   //步态调度器控制脚的名义接触时间表
  MIT_UserParameters userParameters;
};


#endif
