/*!
 * @file DesiredStateCommand.h
 * @brief 将操纵杆指令转换为机器人所需轨迹的逻辑
 * 将操纵杆命令转换为机器人所需轨迹的逻辑
 * 这将生成一个状态轨迹，很容易用于模型预测控制器
 */

#ifndef DESIRED_STATE_COMMAND_H
#define DESIRED_STATE_COMMAND_H

#include <iostream>
#include "Controllers/StateEstimatorContainer.h"
#include "cppTypes.h"
#include "SimUtilities/GamepadCommand.h"
#include "robot/include/rt/rt_rc_interface.h"

/**
 * 功能：遥控器期望状态参考COM轨迹数据
 */
template <typename T>
struct DesiredStateData 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DesiredStateData() { zero(); }
  void zero();                            // 把所有数据归零
  Vec12<T> stateDes;                      // 瞬时期望状态指令
  Vec12<T> pre_stateDes;                  //上一次期望状态指令
  Eigen::Matrix<T, 12, 10> stateTrajDes;  // 期望的未来状态轨迹(最多10个时间步长MPC)
};

/**
 * 功能：躯干状态参考COM轨迹规划命令
 */
template <typename T>
class DesiredStateCommand 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //使用Gamepad Command结构初始化，fun(游戏手柄命令、手柄参数设置、机器人控制参数、状态估计器数据)
  DesiredStateCommand(GamepadCommand* command, rc_control_settings* rc_command,
                      RobotControlParameters* _parameters,
                      StateEstimate<T>* sEstimate, float _dt) 
{
    gamepadCommand = command;     //
    rcCommand = rc_command;       //
    stateEstimate = sEstimate;    //
    parameters = _parameters;     //

    data.stateDes.setZero();      //
    data.pre_stateDes.setZero();  //
    leftAnalogStick.setZero();    //
    rightAnalogStick.setZero();   //

    dt = _dt;
  }

  void convertToStateCommands();
  void setCommandLimits(T minVelX_in, T maxVelX_in,
                        T minVelY_in, T maxVelY_in, T minTurnRate_in, T maxTurnRate_in);
  void desiredStateTrajectory(int N, Vec10<T> dtVec);
  void printRawInfo();
  void printStateCommandInfo();
  float deadband(float command, T minVal, T maxVal);

  // 这些应该来自接口
  T maxRoll = 0.4;
  T minRoll = -0.4;
  T maxPitch = 0.4;
  T minPitch = -0.4;
  T maxVelX = 3.0;
  T minVelX = -3.0;

  T maxVelY = 2.0;
  T minVelY = -2.0;

  T maxTurnRate = 2.5;
  T minTurnRate = -2.5;

  Vec2<float> leftAnalogStick;
  Vec2<float> rightAnalogStick;

  // 保持瞬时期望状态和未来期望状态轨迹
  DesiredStateData<T> data;
  const rc_control_settings* rcCommand;
  const GamepadCommand* gamepadCommand;
  bool trigger_pressed = false;

private:
  StateEstimate<T>* stateEstimate;
  RobotControlParameters* parameters;
  Mat12<T> A;                 //离散时间近似的动力学矩阵
  T dt;                       //控制回路时间步长变化
  T deadbandRegion = 0.075;   //模拟斗杆死区的值切断
  const T filter = 0.1;
  int printNum = 5;           //选择每隔N次迭代打印信息的频率// N*(0.001s) in simulation time
  int printIter = 0;          // 跟踪自上次信息打印以来的迭代次数
};

#endif
