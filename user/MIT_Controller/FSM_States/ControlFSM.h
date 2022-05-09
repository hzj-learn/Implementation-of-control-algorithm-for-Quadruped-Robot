#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>
#include "ControlFSMData.h"//包含所有与控制相关的数据
#include "SafetyChecker.h"//检查机器人状态和安全命令

// FSM状态
#include "../FSM_States/FSM_State.h"
#include "../FSM_States/FSM_State_BalanceStand.h"
#include "../FSM_States/FSM_State_ImpedanceControl.h"
#include "../FSM_States/FSM_State_JointPD.h"
#include "../FSM_States/FSM_State_Locomotion.h"
#include "../FSM_States/FSM_State_Passive.h"
#include "../FSM_States/FSM_State_StandUp.h"
#include "../FSM_States/FSM_State_RecoveryStand.h"
#include "../FSM_States/FSM_State_Vision.h"
#include "../FSM_States/FSM_State_BackFlip.h"
#include "../FSM_States/FSM_State_FrontJump.h"

/**
 * 枚举所有操作模式
 */
enum class FSM_OperatingMode 
{ 
  NORMAL,             //正常运行状态
  TRANSITIONING,      //状态切换状态
  ESTOP,              //停止状态
  EDAMP               //
};

/**
 * 运行状态机FSM状态列表
 */
template <typename T>
struct FSM_StatesList 
{
  FSM_State<T>* invalid;                          //（1）FSM是否使用
  FSM_State_Passive<T>* passive;                  //（2）被人为操纵调试状态
  FSM_State_JointPD<T>* jointPD;                  //（3）关节PD控制器
  FSM_State_ImpedanceControl<T>* impedanceControl;//（4）阻抗控制
  FSM_State_StandUp<T>* standUp;                  //（5）站立过程
  FSM_State_BalanceStand<T>* balanceStand;        //（6）站立平衡
  FSM_State_Locomotion<T>* locomotion;            //（7）运动
  FSM_State_RecoveryStand<T>* recoveryStand;      //（8）恢复站立
  FSM_State_Vision<T>* vision;                    //（9）可视化
  FSM_State_BackFlip<T>* backflip;                //（10）后空翻
  FSM_State_FrontJump<T>* frontJump;              //（11）向前跳
};

template <typename T>
struct FSM_ControllerList 
{
};


/**
 * 处理FSM状态的父类
 */
template <typename T>
class ControlFSM 
{
 public:
  ControlFSM(Quadruped<T>* _quadruped,
             StateEstimatorContainer<T>* _stateEstimator,
             LegController<T>* _legController, GaitScheduler<T>* _gaitScheduler,
             DesiredStateCommand<T>* _desiredStateCommand,
             RobotControlParameters* controlParameters,
             VisualizationData* visualizationData,
             MIT_UserParameters* userParameters);
  void initialize();                    //（1）初始化有限状态机
  void runFSM();                        //（2）运行FSM逻辑，处理状态转换和正常运行
  FSM_OperatingMode safetyPreCheck();   //这将被删除并放入SafetyChecker类
  FSM_OperatingMode safetyPostCheck();
  FSM_State<T>* getNextState(FSM_StateName stateName);  //（3）从请求时创建的状态列表中获取下一个FSM_State
  void printInfo(int opt);              //打印当前状态
  ControlFSMData<T> data;               //包含所有控制相关数据
  /*FSM状态信息*/ 
  FSM_StatesList<T> statesList;         //（1）保存fsm所有状态statesList
  FSM_State<T>* currentState;           //（2）FSM当前状态currentState
  FSM_State<T>* nextState;              //（3）FSM下一个状态nextState
  FSM_StateName nextStateName;          //（4）FSM下一个状态名nextStateName
  SafetyChecker<T>* safetyChecker;      //（5）检查所有输入和命令是否安全safetyChecker
  TransitionData<T> transitionData;

 private:
  FSM_OperatingMode operatingMode;     //FSM的工作模式
  int printNum = 10000;                //控制打印频率,模拟时间N*（0.001s）
  int printIter = 0;                   //跟踪自上次信息打印以来的迭代次数 使大于printNum不打印
  int iter = 0;
};

#endif  // CONTROLFSM_H
