/*! @file SimulationBridge.cpp
 *  @brief  SimulationBridge运行一个RobotController，并使用共享内存将其连接到模拟器。它是硬件桥的模拟版本。
 */

#include "SimulationBridge.h"
#include "Utilities/SegfaultHandler.h"
#include "Controllers/LegController.h"
#include "rt/rt_rc_interface.h"
#include "rt/rt_sbus.h"

/*!
 * 功能：运行仿真桥驱动
 * 步骤：
 * （1）配置内存环境
 * （2）根据种模式，默认运行四足动物控制器
 */
void SimulationBridge::run() 
{
  /*（1）配置内存环境*/
  _sharedMemory.attach(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME);
  _sharedMemory().init();                                             //初始化共享内存 
  install_segfault_handler(_sharedMemory().robotToSim.errorMessage);  //安装segfault处理程序函数


  /*（2）根据4种模式，初始化运行四足动物控制器*/
  try 
  {
    printf("[Simulation Driver] Starting main loop...\n");
    bool firstRun = true;                         //定义一个布尔变量firstRun
    for (;;) 
    {
      _sharedMemory().waitForSimulator();        //（1）等待模拟器响应

      //(2)如果是第一次运行，判断机器人类型类型是否正确
      if (firstRun)                              
      {
        firstRun = false;
        if (_robot != _sharedMemory().simToRobot.robotType)   //若检查机器人类型，不是仿真类型打印错误并抛出错误
        {
          printf(
            "simulator and simulatorDriver don't agree on which robot we are "
            "simulating (robot %d, sim %d)\n",
            (int)_robot, (int)_sharedMemory().simToRobot.robotType);
          throw std::runtime_error("robot mismatch!");
         }
      }

      //（3）获取模拟器的4种运行模式,并运行相应的的模式
      _simMode = _sharedMemory().simToRobot.mode;//从共享内存中获取模拟器的4种运行模式
      switch (_simMode)
      {
        case SimulatorMode::RUN_CONTROL_PARAMETERS: //（1）运行控制参数设置模式
          handleControlParameters(); //处理来自模拟器的控制参数消息
          break;

        case SimulatorMode::RUN_CONTROLLER:         //（2）运行控制模式：（默认是运行这个模式）
          _iterations++;
          runRobotControl();         //运行机器人控制器，和真实的机器人运行同一段程序【重点！！】
          break;

        case SimulatorMode::DO_NOTHING:            //（3）啥都不做模式：模拟器只是在检查我们是否还在运行
          break;

        case SimulatorMode::EXIT:                  //（4）模拟器退出模式
          printf("[Simulation Driver] Transitioned to exit mode\n");//打印退出提示，并退出
          return;
          break;

        default:                                  //（5）如果都不是上面枚举的模式，就抛出错误，表示是未识别的模拟器
          throw std::runtime_error("unknown simulator mode");
       }

      _sharedMemory().robotIsDone();             //告诉模拟器我们完成了 
    }
  } 
  
   //（3）若仿真硬件和脚本有问题，抛出错误
  catch (std::exception& e)
  {
    strncpy(_sharedMemory().robotToSim.errorMessage, e.what(), sizeof(_sharedMemory().robotToSim.errorMessage));
    _sharedMemory().robotToSim.errorMessage[sizeof(_sharedMemory().robotToSim.errorMessage) - 1] = '\0';
    throw e;
  }
}


/*!
 * 功能：处理来自模拟器的控制参数消息
 * 步骤：
 * （1）内存搬数据
 *  (2)健全性检查记录
 * （3）处理请求种类
 */
void SimulationBridge::handleControlParameters() 
{
  /*（1）内存搬数据*/
  ControlParameterRequest& request =                          //把内存种控制参数请求数据，搬过来函数变量
      _sharedMemory().simToRobot.controlParameterRequest;
  ControlParameterResponse& response =                        //把内存种控制参数应答数据，搬过来函数变量
      _sharedMemory().robotToSim.controlParameterResponse;

 /*(2)健全性检查记录*/
  if (request.requestNumber <= response.requestNumber)        //请求数小于应答数的时候，抛出警告
  {
    // nothing to do!
    printf(
        "[SimulationBridge] Warning: the simulator has run a ControlParameter "
        "iteration, but there is no new request!\n");
    return;
  }
  u64 nRequests = request.requestNumber - response.requestNumber;  //请求数-应答数
  assert(nRequests == 1);
  response.nParameters = _robotParams.collection._map              //记录日志
                             .size();  
/*（3）处理请求种类*/
  switch (request.requestKind)          
  {
    case ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME:     //按名称设置“机器人”参数
    {
      std::string name(request.name);                                 //定义一个请求名称
      ControlParameter& param = _robotParams.collection.lookup(name); //按名称查找控制参数
      if (param._kind != request.parameterKind)                       //如果机器人参数种类和请求的机器人参数种类不一样，报错
      {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(request.parameterKind));
      }
      param.set(request.value, request.parameterKind);                //根据应答的种类做实际的设置

      // 应答部分
      response.requestNumber =
          request.requestNumber;                                      //把请求数目赋值给应答数据，这么就默认了是一致的
      response.parameterKind =
          request.parameterKind;                                      // 只是为了调试打印语句
      response.value = request.value;                                 // 只是为了调试打印语句
      strcpy(response.name,
             name.c_str());                                           // 只是为了调试打印语句
      response.requestKind = request.requestKind;
      printf("%s\n", response.toString().c_str());
    } break;

    case ControlParameterRequestKind::SET_USER_PARAM_BY_NAME:      //按名称设置用户参数
    {
      std::string name(request.name);                              //定义一个请求名称
      if(!_userParams)                                             //如果不是用户的参数，抛出警告
      {
        printf("[Simulation Bridge] Warning: tried to set user parameter, but the robot does not have any!\n");
      }
      else                                                         //如果是用户的参数，就按名称查找控制参数
      {
        ControlParameter& param = _userParams->collection.lookup(name);//按名称查找控制参数
        if (param._kind != request.parameterKind)                      //如果机器人参数种类和请求的机器人参数种类不一样，报错
        {
          throw std::runtime_error(
              "type mismatch for parameter " + name + ", robot thinks it is " +
              controlParameterValueKindToString(param._kind) +
              " but received a command to set it to " +
              controlParameterValueKindToString(request.parameterKind));
        }
        param.set(request.value, request.parameterKind);              //根据应答的种类做实际的设置
      }

      // 应答部分
      response.requestNumber =
          request.requestNumber;                   //把请求数目赋值给应答数据，这么就默认了是一致的
      response.parameterKind =
          request.parameterKind;                   //只是为了调试打印语句
      response.value = request.value;              //只是为了调试打印语句
      strcpy(response.name,
             name.c_str());                        // 只是为了调试打印语句
      response.requestKind = request.requestKind;
      printf("%s\n", response.toString().c_str());
    } break;

    case ControlParameterRequestKind::GET_ROBOT_PARAM_BY_NAME:     //按名称获取“机器人”参数
    {
      std::string name(request.name);                                  //定义一个请求名称
      ControlParameter& param = _robotParams.collection.lookup(name);  //按名称查找控制参数
      if (param._kind != request.parameterKind)                        //如果不是用户的参数，抛出警告
      {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(request.parameterKind));
      }

      // 应答部分
      response.value = param.get(request.parameterKind);
      response.requestNumber = request.requestNumber;   //把请求数目赋值给应答数据，这么就默认了是一致的
      response.parameterKind =
          request.parameterKind;                        //只是为了调试打印语句
      strcpy(response.name,
             name.c_str());                             //只是为了调试打印语句
      response.requestKind =
          request.requestKind;                          //只是为了调试打印语句
      printf("%s\n", response.toString().c_str());
    } break;
    default:
      throw std::runtime_error("unhandled get/set");
  }
}


/*!
 *  功能：运行机器人控制器
 *  步骤：
 * （1）判断是否是第一次运行
 *      1）检查是否设置了机器人所有参数的值
 *      2）加载用户设置的控制参数
 *      3）设置机器人参数
 * （2）通过调用每个主要组件来运行整个机器人控制系统
 * 
 */
void SimulationBridge::runRobotControl() 
{
  /*（1）判断是否是第一次运行*/
  if (_firstControllerRun)
   {
    /*（1）检查是否设置了机器人所有参数的值 */
    printf("[Simulator Driver] First run of robot controller...\n");
    if (_robotParams.isFullyInitialized())   //如果检查到设置了机器人所有参数的值 
    {
      printf("\tAll %ld control parameters are initialized\n",
             _robotParams.collection._map.size());
    } 
    else                                     //如果检查没有设置了机器人所有参数的值，抛出一个错误 
    {
      printf(
          "\tbut not all control parameters were initialized. Missing:\n%s\n",
          _robotParams.generateUnitializedList().c_str());
      throw std::runtime_error(
          "not all parameters initialized when going into RUN_CONTROLLER");
    }

   /*（2）加载用户设置的控制参数*/
    auto* userControlParameters = _robotRunner->_robot_ctrl->getUserControlParameters();//加载用户设置的控制参数
    if(userControlParameters)         //如果检查用户是设置了参数
     {
      if (userControlParameters->isFullyInitialized())   //如果检查到设置了机器人所有参数的值 
      {
        printf("\tAll %ld user parameters are initialized\n",
               userControlParameters->collection._map.size());
        _simMode = SimulatorMode::RUN_CONTROLLER;   //设置运行控制器模式标志位
      }
       else                                              //如果检查没有设置了机器人所有参数的值 
       {  //抛出一个错误
        printf(
            "\tbut not all control parameters were initialized. Missing:\n%s\n",
            userControlParameters->generateUnitializedList().c_str());
        throw std::runtime_error(
            "not all parameters initialized when going into RUN_CONTROLLER");
      }
     } 
    else                              //如果检查用户没有设置了参数
    {
      _simMode = SimulatorMode::RUN_CONTROLLER;//直接运行控制器
    }

  /*（3）设置并初始化机器人参数*/
  //设置参数
    _robotRunner->driverCommand =&_sharedMemory().simToRobot.gamepadCommand;//1)手柄命令，从硬件桥来数据

    _robotRunner->spiData = &_sharedMemory().simToRobot.spiData;            //2)SPI腿部数据

    _robotRunner->spiCommand = &_sharedMemory().robotToSim.spiCommand;      //3)SPI命令，从硬件桥来数据     

    _robotRunner->tiBoardData = _sharedMemory().simToRobot.tiBoardData;     //4)腿部控制器返回数据数组,从硬件桥来数据

    _robotRunner->tiBoardCommand =_sharedMemory().robotToSim.tiBoardCommand;//5)腿部控制命令数组 //从硬件桥来数据

    _robotRunner->robotType = _robot;                                       //6)机器人类型,从硬件桥来数据
    
    _robotRunner->vectorNavData = &_sharedMemory().simToRobot.vectorNav;    //7)imu数据，从硬件桥来数据
    
    _robotRunner->cheaterState = &_sharedMemory().simToRobot.cheaterState;  //8)检查模式的状态，未使用
    
    _robotRunner->controlParameters = &_robotParams;                        //9)机器人控制参数，从硬件桥来数据
    
    _robotRunner->visualizationData =&_sharedMemory().robotToSim.visualizationData;//10)视觉数据，从硬件桥来数据
    
    _robotRunner->cheetahMainVisualization =&_sharedMemory().robotToSim.mainCheetahVisualization;//11）可视化数据，从硬件桥来数据

    _robotRunner->init();                                                   //使用上面的数据进行机器人的初始化，初始化机器人模型，状态估计器，腿部控制器，机器人数据，以及任何控制逻辑特定的数据
    
    _firstControllerRun = false;

    sbus_thread = new std::thread(&SimulationBridge::run_sbus, this);       //开启一个线程接受遥控手柄的信息
  }
 /*（2）通过调用每个主要组件来运行整个机器人控制系统*/
  _robotRunner->run();                //运行整个机器人控制系统                                      
}


/*!
 * 功能：运行手柄RC receive线程
 */
void SimulationBridge::run_sbus() 
{
  printf("[run_sbus] starting...\n");//（1）打印提示运行RC手柄接收
  int port = init_sbus(true);        //（2）初始化手柄串口的端口
  //（3）死循环运行
  while (true) 
  {
    if (port > 0) 
    {
      int x = receive_sbus(port);//1）接收手柄数据
      if (x) 
      {
        sbus_packet_complete();  //2）解包
      }
    }
    usleep(5000);
  }
}
