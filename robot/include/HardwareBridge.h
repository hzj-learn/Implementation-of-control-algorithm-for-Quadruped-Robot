/*!
 * @file HardwareBridge.h
 * @brief 机器人代码与机器人硬件的接口
 * 此类初始化两个机器人的硬件并允许机器人控制器来访问它。
 */

#ifndef PROJECT_HARDWAREBRIDGE_H
#define PROJECT_HARDWAREBRIDGE_H

//#ifdef linux 

#define MAX_STACK_SIZE 16384  // 16KB  of stack                                 分配内存
#define TASK_PRIORITY 49      // linux priority, this is not the nice value     linux优先级

#include <string>
#include <lcm-cpp.hpp>
#include <lord_imu/LordImu.h>
#include "RobotRunner.h"
#include "Utilities/PeriodicTask.h"
#include "control_parameter_request_lcmt.hpp"
#include "control_parameter_respones_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "microstrain_lcmt.hpp"
#include "ecat_command_t.hpp"
#include "ecat_data_t.hpp"



/*!
 * 机器人与硬件的接口的类
 */
class HardwareBridge 
{
 public:
  HardwareBridge(RobotController* robot_ctrl)                                   //（1）定义通用机器人硬件桥
      : statusTask(&taskManager, 0.5f),
        _interfaceLCM(getLcmUrl(255)),
        _visualizationLCM(getLcmUrl(255)) 
  {
    _controller = robot_ctrl;
    _userControlParameters = robot_ctrl->getUserControlParameters();
  }
  ~HardwareBridge() { delete _robotRunner; }
  void prefaultStack();                                                         //（2）内存报错函数
  void setupScheduler();                                                        //（3）设置任务，实时优先级配置调度函数
  void initCommon();                                                            //（4）初始化命令，两条狗对应的LCM通讯和线程初始化。
  void initError(const char* reason, bool printErrno = false);                  //（5）打印初始化时错误，如果在初始化过程中发生错误，在启动电机之前，打印错误并退出。
  void handleGamepadLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,//（6）手柄命令设置，接受到的消息设置到手柄命令中，用于gamepad消息的LCM处理程序
                        const gamepad_lcmt* msg);
  void handleInterfaceLCM();                                                    //（7）运行接口lcm通信
  void handleControlParameter(const lcm::ReceiveBuffer* rbuf,                   //（8）控制参数获取，LCM处理程序 并响应
                              const std::string& chan,
                              const control_parameter_request_lcmt* msg);
  void publishVisualizationLCM();                                               //（9）LCM发送LCM可视化数据
  void run_sbus();                                                              //（10）运行手柄总线

 protected:
  PeriodicTaskManager taskManager;                         //（1）任务管理器
  PrintTaskStatus statusTask;                              //（2）打印任务管理器中所有任务的状态的定期任务 打印内容被注释了
  GamepadCommand _gamepadCommand;                          //（3）手柄命令.
  VisualizationData _visualizationData;                    //（4）运动可视化数据
  CheetahVisualization _mainCheetahVisualization;          //（5）机器人外形可视化
  lcm::LCM _interfaceLCM;                                  //（6）接口LCM
  lcm::LCM _visualizationLCM;                              //（7）可视化LCM
  control_parameter_respones_lcmt _parameter_response_lcmt;//（8）控制参数回应
  SpiData _spiData;                                        //（9）控制器SPI数据
  SpiCommand _spiCommand;                                  //（10）控制器SPI命令
  
//四腿控制器 数据 命令
  TiBoardCommand _tiBoardCommand[4];                       //(11)upboard指令
  TiBoardData _tiBoardData[4];                             //(12)upboard数据
  bool _firstRun = true;
  RobotRunner* _robotRunner = nullptr;
  RobotControlParameters _robotParams;                     //(13)机器人控制参数
  u64 _iterations = 0;
  std::thread _interfaceLcmThread;                         //(14)接口LCM线程
  volatile bool _interfaceLcmQuit = false;                 //(15)接口LCM退出
  RobotController* _controller = nullptr;                  //(16)机器人控制器
  ControlParameters* _userControlParameters = nullptr;     //(17)控制参数
  int _port;
};


/*!
 *猎豹mini专用硬件的接口的类
 */ 
class MiniCheetahHardwareBridge : public HardwareBridge 
{
 public:
  MiniCheetahHardwareBridge(RobotController* rc, bool load_parameters_from_file);  //（1）定义 MiniCheetah硬件桥
  void runSpi();                              //（2）运行迷你猎豹SPI
  void initHardware();                        //（3）初始化Mini Cheetah特定硬件，初始化SPI、初始化IMU
  void run();                                 //（4）小型猎豹启动配置
  void runMicrostrain();                      //（5）接收IMU的数据
  void logMicrostrain();                      //（6）更新LCM数据，并发布
  void abort(const std::string& reason);      
  void abort(const char* reason);             

 private:
  VectorNavData _vectorNavData;               //（1）imu导航数据
  lcm::LCM _spiLcm;                           //（2）用LCM发布SPI数据
  lcm::LCM _microstrainLcm;                   
  std::thread _microstrainThread;             //（3）开启线程
  LordImu _microstrainImu;                   
  microstrain_lcmt _microstrainData;          //
  bool _microstrainInit = false;              //
  bool _load_parameters_from_file;            //
};



/*!
 *猎豹3专用硬件的接口的类
 */ 
class Cheetah3HardwareBridge : public HardwareBridge 
{
public:
  Cheetah3HardwareBridge(RobotController* rc); //（1）定义 MiniCheetah硬件桥
  void runEcat();                              //（2）运行lcm获取和发送信息
  void initHardware();                         //（3）初始化Cheetah3特定的硬件
  void run();                                  //（4）Cheetah3运行
  void publishEcatLCM();                       //（5）通过以太网LCM，发送四条腿自定义消息

private:
  VectorNavData _vectorNavData;     //（1）imu数据
  lcm::LCM _ecatLCM;                //（2）自定义消息用lcm
  ecat_command_t ecatCmdLcm;        //（3）两种消息类型
  ecat_data_t ecatDataLcm;

};
#endif // END of #ifdef linux
#endif  // PROJECT_HARDWAREBRIDGE_H
