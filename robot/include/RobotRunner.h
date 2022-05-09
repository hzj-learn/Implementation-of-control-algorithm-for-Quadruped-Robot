/*!
 * @file RobotRunner.h
 * @brief 运行机器人控制器的通用框架。
 * 此代码是mini cheetah和cheetah 3的控制代码和硬件/仿真之间的通用接口
 */

#ifndef PROJECT_ROBOTRUNNER_H
#define PROJECT_ROBOTRUNNER_H

#include "ControlParameters/ControlParameterInterface.h"
#include "ControlParameters/RobotParameters.h"
#include "Controllers/StateEstimatorContainer.h"
#include "SimUtilities/IMUTypes.h"
#include "rt/rt_rc_interface.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/LegController.h"
#include "Dynamics/Quadruped.h"
#include "JPosInitializer.h"
#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/VisualizationData.h"
#include "Utilities/PeriodicTask.h"
#include "cheetah_visualization_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "RobotController.h"
#include <lcm-cpp.hpp>

class RobotRunner : public PeriodicTask 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotRunner(RobotController* , PeriodicTaskManager*, float, std::string);//定义机器人运行器
  using PeriodicTask::PeriodicTask;                                        //周期任务
  void init() override;                                                    //初始化机器人模型，状态估计器，腿部控制器，机器人数据，以及任何控制逻辑特定的数据
  void run() override;                                                     //通过调用每个主要组件来运行整个机器人控制系统
  void cleanup() override;

  // 初始化状态估计器，默认没有作弊模式
  void initializeStateEstimator(bool cheaterMode = false);          //初始化状态估计器
  virtual ~RobotRunner();
  RobotController* _robot_ctrl;                                     //机器人控制器
  GamepadCommand* driverCommand;                                    //手柄命令，从硬件桥来数据
  RobotType robotType;                                              //机器人类型,从硬件桥来数据
  VectorNavData* vectorNavData;                                     //imu数据，从硬件桥来数据
  CheaterState<double>* cheaterState;                               //未使用 
  
  //硬件命令 数据
  SpiData* spiData; 
  SpiCommand* spiCommand;                           //从硬件桥来数据
  TiBoardCommand* tiBoardCommand;                   //腿部控制命令数组 ，从硬件桥来数据
  TiBoardData* tiBoardData;                         //腿部控制器返回数据数组,从硬件桥来数据 
  RobotControlParameters* controlParameters;        //控制参数 ，从硬件桥来数据
  VisualizationData* visualizationData;             //可视化数据 ，从硬件桥来数据
  CheetahVisualization* cheetahMainVisualization;   //可视化 //从硬件桥来数据

 private:
  float _ini_yaw;
  int iter = 0;
  void setupStep();
  void finalizeStep();
  JPosInitializer<float>* _jpos_initializer;          //初始化开机时腿的位置
  Quadruped<float> _quadruped;                        //机器人类型
  LegController<float>* _legController = nullptr;     //腿部控制器 
  StateEstimate<float> _stateEstimate;                //状态估计结果
  StateEstimatorContainer<float>* _stateEstimator;    //状态估计 所有估计器
  bool _cheaterModeEnabled = false;
  DesiredStateCommand<float>* _desiredStateCommand;   //期望状态命令  
  rc_control_settings rc_control;
  
  //自定义消息
  lcm::LCM _lcm;
  leg_control_command_lcmt leg_control_command_lcm;   //腿部控制命令
  state_estimator_lcmt state_estimator_lcm;           //状态估计
  leg_control_data_lcmt leg_control_data_lcm;         //腿部数据
  
  //用于计算估计力和接触的接触估计器
  FloatingBaseModel<float> _model;
  u64 _iterations = 0;
};

#endif  // PROJECT_ROBOTRUNNER_H
