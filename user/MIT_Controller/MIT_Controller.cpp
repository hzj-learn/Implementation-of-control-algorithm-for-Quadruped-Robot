#include "MIT_Controller.hpp"


/**
 * 功能：创建一个MIT_Controller控制器，MIT_Controller是继承RobotController的
 */
MIT_Controller::MIT_Controller():RobotController()
{}

/**
 * 功能：初始化控制FSM。
 */
void MIT_Controller::initializeController() 
{
  //(1)创建一个新的步态规划器（GaitScheduler），fun(用户参数，间隔时间)
  _gaitScheduler = new GaitScheduler<float>(&userParameters, _controlParameters->controller_dt);
  //(2)创建控制FSM
  _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                      _legController, _gaitScheduler,
                                      _desiredStateCommand, _controlParameters, 
                                      _visualizationData, &userParameters);
}



/**
 * 使用ControlFSM逻辑计算腿部控制器的命令。
 */
void MIT_Controller::runController() 
{
  _gaitScheduler->step();                         //(1)步态规划器【选择步态】+相位变化计算【支撑相或腾空相步态调度相位增量迭代】
  _desiredStateCommand->convertToStateCommands(); //(2)计算期望的状态转换为躯干的状态--即躯干期望的COM轨迹生成【手柄目标命令+状态估计器当前状态】计算想要的状态轨迹（12个状态变量）
  _controlFSM->runFSM();                          //(3)【重点！！！】运行控制FSM代码，运行状态机切换任务
}


