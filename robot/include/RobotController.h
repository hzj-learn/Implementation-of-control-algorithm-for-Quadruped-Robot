/*!
 * @file RobotController.h
 * @brief 
 * 通用机器人运行的控制器父类。
 * 这是 cheetah mini and cheetah 3控制代码和通用硬件代码之间的通用接口
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Controllers/LegController.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/DesiredStateCommand.h"
#include "SimUtilities/VisualizationData.h"
#include "SimUtilities/GamepadCommand.h"

/*!
 * 用户机器人控制器的父类
 */
class RobotController
{
  friend class RobotRunner;      //RobotRunner是RobotController的友元类
public:
 // 仅仅调用一次
  RobotController(){}            //将运行框架加入任务管理器函数，构造函数
  virtual ~RobotController(){}   //RobotController的析构函数
  virtual void initializeController() = 0;

 //下面的函数每次控制循环调用一次
  virtual void runController() = 0; 
  virtual void updateVisualization() = 0;
  virtual ControlParameters* getUserControlParameters() = 0;
  virtual void Estop() {}

protected:
  Quadruped<float>* _quadruped = nullptr;                   //四足数据
  FloatingBaseModel<float>* _model = nullptr;               //运动学树
  LegController<float>* _legController = nullptr;           //腿部控制
  StateEstimatorContainer<float>* _stateEstimator = nullptr;//状态估计器
  StateEstimate<float>* _stateEstimate = nullptr;           //估计数据
  GamepadCommand* _driverCommand = nullptr;
  RobotControlParameters* _controlParameters = nullptr;
  DesiredStateCommand<float>* _desiredStateCommand = nullptr;
  VisualizationData* _visualizationData = nullptr;
  RobotType _robotType;
};

#endif
