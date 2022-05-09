#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include <ControlParameters/RobotParameters.h>
#include <MIT_UserParameters.h>
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"

template <typename T>
struct ControlFSMData 
{
  Quadruped<T>* _quadruped;                     //（1）机器人相关数据
  StateEstimatorContainer<T>* _stateEstimator;  //（2）状态估计容器 
  LegController<T>* _legController;             //（3）腿部控制器
  GaitScheduler<T>* _gaitScheduler;             //（4）步态计划
  DesiredStateCommand<T>* _desiredStateCommand; //（5）期望状态命令
  RobotControlParameters* controlParameters;    //（6）控制参数
  MIT_UserParameters* userParameters;           //（7）用户配置参数
  VisualizationData* visualizationData;         //（8）可视化数据
};

template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

#endif  // CONTROLFSM_H