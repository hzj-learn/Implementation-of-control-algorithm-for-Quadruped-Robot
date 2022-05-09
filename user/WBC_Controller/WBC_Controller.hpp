#ifndef WBC_CONTROLLER
#define WBC_CONTROLLER

#include <RobotController.h>
#include "WBC_States/Cheetah_DynaCtrl_Definition.h"
#include <gui_main_control_settings_t.hpp>

template<typename T> class Test;

class WBC_Controller: public RobotController
{
public:
  WBC_Controller();                                       //WBC基类控制器
  virtual ~WBC_Controller(){}
  //对于WBC状态（测试）
  virtual void initializeController();                    //（1）初始化WBC函数
  virtual void runController();                           //（2）使用WBC逻辑计算leg控制器的命令函数
  virtual void updateVisualization();                     //（3）更新可视化函数
  virtual ControlParameters* getUserControlParameters()   //（4）获取用户控制参数函数
  {
    return nullptr;
  }

protected:
  Test<float>* _wbc_state;
  Cheetah_Data<float>* _data;
  Cheetah_Extra_Data<float>* _extra_data;
  gui_main_control_settings_t main_control_settings;

//（5）可视化内容
  void _StepLocationVisualization();  //（1）落脚可视化
  void _BodyPathVisualization();      //（2）躯干路径可视化
  void _BodyPathArrowVisualization(); //（3）躯干路径方向可视化
  void _testDebugVisualization();     //（4）测试调试可视化

  float _ini_yaw;
};



#endif
