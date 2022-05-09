#ifndef FSM_State_H
#define FSM_State_H

#include <stdio.h>

#include "ControlFSMData.h"
#include "TransitionData.h"
#include "Controllers/GaitScheduler.h"

#include <Controllers/BalanceController/BalanceController.hpp>

// 正常机器人状态
#define K_PASSIVE       0
#define K_STAND_UP      1
#define K_BALANCE_STAND 3
#define K_LOCOMOTION    4
#define K_LOCOMOTION_TEST 5
#define K_RECOVERY_STAND 6
#define K_VISION        8
#define K_BACKFLIP      9
#define K_FRONTJUMP     11

//特定控制状态
#define K_JOINT_PD          51
#define K_IMPEDANCE_CONTROL 52
#define K_INVALID           100

/**
 * 枚举所有FSM状态，以便我们可以跟踪它们。
 */
enum class FSM_StateName
{
  INVALID,
  PASSIVE,
  JOINT_PD,
  IMPEDANCE_CONTROL,
  STAND_UP,
  BALANCE_STAND,
  LOCOMOTION,
  RECOVERY_STAND,
  VISION,
  BACKFLIP,
  FRONTJUMP
};

/**
 *状态机状态父类目前看到的状态，大部分参数没有用到
 */
template <typename T>
class FSM_State
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //所有状态的泛型构造函数
  FSM_State(ControlFSMData<T> *_controlFSMData, FSM_StateName stateNameIn,
            std::string stateStringIn);
  virtual void onEnter() = 0;               //进入一种状态时的行为
  virtual void run() = 0;                   //运行状态的正常行为
  virtual FSM_StateName checkTransition()   //管理特定于状态的转换
  { return FSM_StateName::INVALID; }

  virtual TransitionData<T> transition()    //运行转换行为，并在完成转换时返回true
  { return transitionData; }
  virtual void onExit() = 0;                //退出状态时要执行的行为
  void jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes);  //关节PD控制
  void cartesianImpedanceControl(int leg, Vec3<T> pDes, Vec3<T> vDes,  //笛卡尔阻抗控制
                                 Vec3<double> kp_cartesian,
                                 Vec3<double> kd_cartesian);
                  
  void footstepHeuristicPlacement(int leg);  //脚踏启发式布置  未启用 

  //运行控制器
  void runControls();
  void runBalanceController();
  void runWholeBodyController();
  void runConvexModelPredictiveController();
  void runRegularizedPredictiveController();

  //参数安全检测 置真/假
  void turnOnAllSafetyChecks();
  void turnOffAllSafetyChecks();

  //保存所有相关的控制数据
  ControlFSMData<T> *_data;

  //FSM状态信息
  FSM_StateName stateName;     //当前状态名枚举类 每种状固定不变 
  FSM_StateName nextStateName; //下一个状态名枚举类
  std::string stateString;     //状态名

  //状态转移参数
  T transitionDuration;             //转移时间间隔
  T tStartTransition;               //转移开始时间
  TransitionData<T> transitionData; //相关数据的结构，可在转换期间用于在状态间传递数据。

  //预先控制安全检查
  bool checkSafeOrientation = false; //检查横摇和纵摇

  //控制后安全检查
  bool checkPDesFoot = false;         //不要把脚弄得太远
  bool checkForceFeedForward = false; //不使用较大力
  bool checkLegSingularity = false;   //不用腿

  //整个机器人的腿部控制器命令占位符(3x4矩阵)
  Mat34<T> jointFeedForwardTorques; //前馈关节力矩
  Mat34<T> jointPositions;          //关节角度
  Mat34<T> jointVelocities;         //关节角速度
  Mat34<T> footFeedForwardForces;   //前馈力
  Mat34<T> footPositions;           //足端位置
  Mat34<T> footVelocities;          //足端速度

  //下一步的脚步位置
  Mat34<T> footstepLocations;

  //高级机器人身体控制器
  BalanceController balanceController;

private:
  // 增益 直角坐标
  Mat3<float> kpMat;

  // 增益 直角坐标
  Mat3<float> kdMat;
};

#endif // FSM_State_H
