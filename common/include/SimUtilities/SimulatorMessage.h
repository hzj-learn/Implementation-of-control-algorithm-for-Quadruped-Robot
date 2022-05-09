/*! @file SimulatorMessage.h
 *  @brief 发送到/来自开发模拟器的消息
 *这些消息包含机器人程序之间交换的所有数据以及使用共享内存的模拟器。
 * 基本上这就是一切除了.用于调试日志，这些日志由LCM处理
 */

#ifndef PROJECT_SIMULATORTOROBOTMESSAGE_H
#define PROJECT_SIMULATORTOROBOTMESSAGE_H

#include "ControlParameters/ControlParameterInterface.h"
#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/VisualizationData.h"
#include "SimUtilities/ti_boardcontrol.h"
#include "Utilities/SharedMemory.h"

/*!
 * 模拟器的模式
 */
enum class SimulatorMode 
{
  RUN_CONTROL_PARAMETERS,  // （1）不运行机器人控制器，仅仅是处理参数的一个进程                           
  RUN_CONTROLLER,          // （2）运行机器人控制器
  DO_NOTHING,              // （3）仅仅做检查连接时候成功
  EXIT                     // （4）停止
};

/*!
 * 从模拟器到机器人PC的简单信息
 */
struct SimulatorToRobotMessage 
{
  GamepadCommand gamepadCommand;  //（1）遥控手柄命令
  RobotType robotType;            //（2）模拟哪个机器人取决于模拟器

  //（3）imu数据
  VectorNavData vectorNav;        
  CheaterState<double> cheaterState;
  //（4）腿部SPI数据
  SpiData spiData;
  TiBoardData tiBoardData[4];
  //（5）控制应答的参数
  ControlParameterRequest controlParameterRequest;
  //（6）模拟器的模式
  SimulatorMode mode;
};

/*!
 * 从机器人PC到模拟器的简单信息
 */
struct RobotToSimulatorMessage 
{
  RobotType robotType;                              //（1）机器人类型
  SpiCommand spiCommand;                            //（2）SPI命令
  TiBoardCommand tiBoardCommand[4];                 //（3）upboard命令
  VisualizationData visualizationData;              //（4）可视化数据
  CheetahVisualization mainCheetahVisualization;    //（5）可视化，从硬件桥来数据
  ControlParameterResponse controlParameterResponse;//（6）控制应答的参数
};

/*!
 * 机器人和模拟器之间共享的所有数据
 */
struct SimulatorMessage 
{
  RobotToSimulatorMessage robotToSim;//（1）从机器人PC到模拟器的简单信息
  SimulatorToRobotMessage simToRobot;//（2）从模拟器到机器人PC的简单信息
};

/*!
 *模拟器同步的信息
 *SimulatorSyncronizedMessage存储在共享内存中，由
 *模拟器和机器人模拟器和机器人轮流有对整个消息的独占访问。预定顺序为：
 *  - robot: waitForSimulator()
 *  - simulator: *simulates robot* (模拟器可以读/写，机器人不能做任何事情)
 *  - simulator: simDone()
 *  - simulator: waitForRobot()
 *  - robot: *runs controller*    (模拟器可以读/写，机器人不能做任何事情)
 *  - robot: robotDone();
 *  - robot: waitForSimulator()
 *  ...
 */
struct SimulatorSyncronizedMessage : public SimulatorMessage 
{

  void init()               //（1）初始化共享内存信号量   init（）方法只能在*共享内存连接后调用！
  {
    robotToSimSemaphore.init(0);//1)共享从真实机器人到仿真的内存信号量
    simToRobotSemaphore.init(0);//2)共享从仿真到真实机器人的内存信号量
  }

  void waitForSimulator()   //（2）等待模拟器响应
  { simToRobotSemaphore.decrement(); }

  void simulatorIsDone()    //（3）模拟器响应完成
  { simToRobotSemaphore.increment(); }

  void waitForRobot()       //（4）等待机器人响应
  { robotToSimSemaphore.decrement(); }

  bool tryWaitForRobot()    //（5）尝试等待模拟器响应
  { return robotToSimSemaphore.tryDecrement(); }

  bool waitForRobotWithTimeout()  //（6）等待模拟器响应超时
  {
    return robotToSimSemaphore.decrementTimeout(1, 0);
  }

  void robotIsDone()        //（7）机器人响应完成
  { robotToSimSemaphore.increment(); }

 private:
  SharedMemorySemaphore robotToSimSemaphore, simToRobotSemaphore;//共享内存信号量
};

#endif  // PROJECT_SIMULATORTOROBOTMESSAGE_H
