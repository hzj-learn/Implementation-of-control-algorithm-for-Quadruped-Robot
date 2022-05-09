/*! @file SimulationBridge.h
 *  @brief  SimulationBridge运行一个RobotController，并使用共享内存将其连接到模拟器。它是硬件桥的模拟版本。
 */

#ifndef PROJECT_SIMULATIONDRIVER_H
#define PROJECT_SIMULATIONDRIVER_H

#include <thread>

#include "ControlParameters/RobotParameters.h"
#include "RobotRunner.h"
#include "SimUtilities/SimulatorMessage.h"
#include "Types.h"
#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"


/*模拟器连接桥*/
class SimulationBridge 
{
 public:
  explicit SimulationBridge(RobotType robot, RobotController* robot_ctrl) :_robot(robot)  //(1)声明SimulationBridge这个类的构造函数，RobotType这个类型仅仅枚举了CHEETAH_3, MINI_CHEETAH两款机器人
    {
     _fakeTaskManager = new PeriodicTaskManager;                                          //1）创建定期任务内存空间
     _robotRunner = new RobotRunner(robot_ctrl, _fakeTaskManager, 0, "robot-task");       //2）创建运行框架加入任务管理器构造函数
     _userParams = robot_ctrl->getUserControlParameters();                                //3）获取机器人控制参数
    }
  ~SimulationBridge()                                                                     //声明SimulationBridge这个类的析造函数
  {
    delete _fakeTaskManager;                                                              //删除定期任务内存空间                                                              
    delete _robotRunner;                                                                  //删除运行框架加入任务管理器构造函数
  }
  void run_sbus();                  //(2)运行手柄RC receive线程
  void run();                       //(3)连接到仿真函数，运行四足动物控制器
  void handleControlParameters();   //(4)处理来自模拟器的控制参数消息
  void runRobotControl();           //(5)运行机器人控制器

 private:
  PeriodicTaskManager taskManager;
  bool _firstControllerRun = true;
  PeriodicTaskManager* _fakeTaskManager = nullptr;
  RobotType _robot;
  RobotRunner* _robotRunner = nullptr;
  SimulatorMode _simMode;
  SharedMemoryObject<SimulatorSyncronizedMessage> _sharedMemory;
  RobotControlParameters _robotParams;
  ControlParameters* _userParams = nullptr;
  u64 _iterations = 0;

  std::thread* sbus_thread;
};

#endif  // PROJECT_SIMULATIONDRIVER_H
