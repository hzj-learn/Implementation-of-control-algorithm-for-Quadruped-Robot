/*!
 * @file HardwareBridge.cpp
 * @brief 机器人代码和机器人硬件之间的接口
 * 这个类初始化两个机器人的硬件并允许机器人
 * 控制器来访问它
 */
//#ifdef linux 

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include "Configuration.h"
#include "HardwareBridge.h"
//#include "rt/rt_rc_interface.h"
#include "rt/rt_sbus.h"
#include "rt/rt_spi.h"
#include "rt/rt_vectornav.h"
#include "rt/rt_ethercat.h"
#include "Utilities/Utilities_print.h"

#define USE_MICROSTRAIN

/*!
 * 功能：初始化命令，如果在初始化过程中发生错误，在启动电机之前，打印错误并退出。
 * @param reason 错误消息字符串
 * @param printErrno 如果为真，那么打印C errno
 */
void HardwareBridge::initError(const char* reason, bool printErrno) 
{
  printf("FAILED TO INITIALIZE HARDWARE: %s\n", reason);

  if (printErrno) {
    printf("Error: %s\n", strerror(errno));
  }
  exit(-1);
}



/*!
 * 功能：内存堆栈、LCM线程初始化
 * 步骤：步骤都是在Cheetah 3和Mini Cheetah之间通用的
 * （1）申请内存
 * （2）任务优先级配置
 * （3）LCM通讯接口初始化
 * （4）LCM订阅手柄信息
 * （5）LCM订阅控制参数请求
 * （6）开启多线程
 */
void HardwareBridge::initCommon() 
{
  /*（1）初始化堆栈申请内存,若有错误抛出并处理*/
  printf("[HardwareBridge] Init stack\n");
  prefaultStack();                            
 
  /*（2）配置配置多线程任务的优先级*/
  printf("[HardwareBridge] Init scheduler\n");
  setupScheduler();                           

  /*（3）检查LCM通讯接口是否有问题，订阅手柄信息和控制参数请求，并开启LCM通讯接口线程*/
  if (!_interfaceLCM.good())                                                      //若LCM通讯接口准备失败
  {
    initError("_interfaceLCM failed to initialize\n", false);
  }
  printf("[HardwareBridge] Subscribe LCM\n");
  _interfaceLCM.subscribe("interface", &HardwareBridge::handleGamepadLCM, this);  //LCM订阅手柄信息：接收到的消息设置到手柄命令中，用于gamepad消息的LCM处理程序，运行handleGamepadLCM（）
  _interfaceLCM.subscribe("interface_request",
                          &HardwareBridge::handleControlParameter, this);         //LCM订阅控制参数请求
  printf("[HardwareBridge] Start interface LCM handler\n");
  _interfaceLcmThread = std::thread(&HardwareBridge::handleInterfaceLCM, this);   //开启LCM通讯接口线程
}


/*!
 * 功能：运行接口lcm通信
 * 其中lcm.handle()
 * LCM自动解码消息，再传给回调函数，回调函数可以识别消息类型。
 * 因为回调函数在lcm.handle()方法中调度，所以不需要并发执行，这些都在一个单线程中完成。
 * 调用lcm.handle()非常重要，函数会保持阻塞直到有任务需要做。
 */
void HardwareBridge::handleInterfaceLCM() 
{
  while (!_interfaceLcmQuit) _interfaceLCM.handle();
}


/*!
* 功能：内存申请错误函数
  写入堆栈上的一个16 KB缓冲区。如果我们的堆栈使用4K页面，这将确保在堆栈增长时不会出现页面错误。
  还有mlock与当前进程关联的所有页面，这可以防止猎豹软件被换出。
  如果内存用完，机器人程序将被OOM进程杀手杀死(并留下日志)，而不是变得没有响应。
 */
void HardwareBridge::prefaultStack() 
{
  printf("[Init] Prefault stack...\n");
  volatile char stack[MAX_STACK_SIZE];
  memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) 
  {
    initError(
        "mlockall failed.  This is likely because you didn't run robot as "
        "root.\n",
        true);
  }
}


/*!
 * 功能：设置任务，实时优先级配置调度函数
 */
void HardwareBridge::setupScheduler() 
{
  printf("[Init] Setup RT Scheduler...\n");
  struct sched_param params;
  params.sched_priority = TASK_PRIORITY;//任务优先级
  if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) 
  {
    initError("sched_setscheduler failed.\n", true);
  }
}


/*!
 * 功能：手柄命令设置，接收到的消息设置到手柄命令中，用于gamepad消息的LCM处理程序
 */
void HardwareBridge::handleGamepadLCM(const lcm::ReceiveBuffer* rbuf,
                                      const std::string& chan,
                                      const gamepad_lcmt* msg) 
{
  (void)rbuf;
  (void)chan;
  _gamepadCommand.set(msg);//接收到的消息设置到手柄命令中
}


/*!
 * 功能：控制参数获取，LCM处理程序 并响应
 * 步骤：
 * （1）当自定义的控制消息数量（设备）小于LCM的消息数量时，抛出警告
 * （2）硬件完整性检查
 * （3）请求参数类型设置  通过参数名设置用户类型或通过参数名设置机器人类型
 *      1）控制参数获取
 *      2）控制类型检查
 *      3）做实际的参数类型设置
 *      4）参数类型做响应
 */
void HardwareBridge::handleControlParameter(
    const lcm::ReceiveBuffer* rbuf,            //LCM接收到的数据缓冲器
    const std::string& chan,                   //频道名
    const control_parameter_request_lcmt* msg) //自定义消息类型
{
  (void)rbuf;
  (void)chan;
  if (msg->requestNumber <= _parameter_response_lcmt.requestNumber) //（1）当自定义的控制消息数量（设备）小于LCM的消息数量时，抛出警告
  {
    printf(
        "[HardwareBridge] Warning: the interface has run a ControlParameter "
        "iteration, but there is no new request!\n");
  }

  //（2）硬件完整性检查
  s64 nRequests = msg->requestNumber - _parameter_response_lcmt.requestNumber;//检查硬件设备有没有少。
  if (nRequests != 1) 
  {
    printf("[ERROR] Hardware bridge: we've missed %ld requests\n",
           nRequests - 1);
  }

  switch (msg->requestKind) //请求参数类型
  {  
    case (s8)ControlParameterRequestKind::SET_USER_PARAM_BY_NAME:  //通过参数名设置用户类型
    {
      if(!_userControlParameters) //（1）当前类型不是用户的控制类型，打印报错
      {
        printf("[Warning] Got user param %s, but not using user parameters!\n",(char*)msg->name);
      }
      else                        //当前类型是用户的控制类型
      {
        std::string name((char*)msg->name);
        ControlParameter& param = _userControlParameters->collection.lookup(name); //（1）控制参数获取
        if ((s8)param._kind != msg->parameterKind)                                 //（2）类型检查，若参数类型不对
        {
          throw std::runtime_error(
              "type mismatch for parameter " + name + ", robot thinks it is " +
              controlParameterValueKindToString(param._kind) +
              " but received a command to set it to " +
              controlParameterValueKindToString(
                  (ControlParameterValueKind)msg->parameterKind));
        }

        ControlParameterValue v;
        memcpy(&v, msg->value, sizeof(v));
        param.set(v, (ControlParameterValueKind)msg->parameterKind);               //（3）做实际的参数类型设置

                                                                                   //（4）参数类型做响应
        _parameter_response_lcmt.requestNumber =
            msg->requestNumber;  // 承认设置已经发生
			
        _parameter_response_lcmt.parameterKind =
            msg->parameterKind;  // 仅用于调试打印语句
        memcpy(_parameter_response_lcmt.value, msg->value, 64);
        strcpy((char*)_parameter_response_lcmt.name,
               name.c_str());   // 用于调试打印语句
        _parameter_response_lcmt.requestKind = msg->requestKind;
        printf("[User Control Parameter] set %s to %s\n", name.c_str(),
               controlParameterValueToString(
                   v, (ControlParameterValueKind)msg->parameterKind)
                   .c_str());
      }
    } break;
    case (s8)ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME: //通过参数名设置机器人类型
    {
      std::string name((char*)msg->name);
      ControlParameter& param = _robotParams.collection.lookup(name);
      // 类型检查
      if ((s8)param._kind != msg->parameterKind) {
        throw std::runtime_error(
            "type mismatch for parameter " + name + ", robot thinks it is " +
            controlParameterValueKindToString(param._kind) +
            " but received a command to set it to " +
            controlParameterValueKindToString(
                (ControlParameterValueKind)msg->parameterKind));
      }

      // 做实际的设定
      ControlParameterValue v;
      memcpy(&v, msg->value, sizeof(v));
      param.set(v, (ControlParameterValueKind)msg->parameterKind);

      //响应
      _parameter_response_lcmt.requestNumber =
          msg->requestNumber;  //承认已经发生了 
		  
      _parameter_response_lcmt.parameterKind =
          msg->parameterKind;  // 只是为了调试打印语句
		  
      memcpy(_parameter_response_lcmt.value, msg->value, 64);
      //_parameter_response_lcmt.value = _parameter_request_lcmt.value; // just
	  
      //用于调试打印语句
      strcpy((char*)_parameter_response_lcmt.name,
             name.c_str());  // 只是为了调试打印语句
      _parameter_response_lcmt.requestKind = msg->requestKind;

      printf("[Robot Control Parameter] set %s to %s\n", name.c_str(),
             controlParameterValueToString(
                 v, (ControlParameterValueKind)msg->parameterKind)
                 .c_str());

    } break;

    default: 
    {
      throw std::runtime_error("parameter type unsupported");//参数类型不支持错误
    }
    break;
  }
  _interfaceLCM.publish("interface_response", &_parameter_response_lcmt);         //发送回应
}


/*!
 * 功能：从文件中加载猎豹mini的参数
 */
MiniCheetahHardwareBridge::MiniCheetahHardwareBridge(RobotController* robot_ctrl, bool load_parameters_from_file)
    : HardwareBridge(robot_ctrl), _spiLcm(getLcmUrl(255)), _microstrainLcm(getLcmUrl(255)) 
{
  _load_parameters_from_file = load_parameters_from_file;
}


/*!
 * 功能：小型猎豹启动配置
 * 步骤：
 * (1)LCM和线程初始化
 * (2)Mini Cheetah特定硬件初始化
 * (3)从文件加载参数，或者等待加载参数
 */
void MiniCheetahHardwareBridge::run() 
{
  ///////////////////////////*（1） LCM和线程初始化*///////////////////////////
  
  initCommon();                   
  ///////////////////////////*（2）Mini Cheetah特定硬件初始化*////////////////////////////
  initHardware();                 
  //////////////////////////*（3）文件加载机器人参数*/////////////////////////////////////
  //若加载参数文件标志位打开，允许从Yaml文件加载机器人参数
  if(_load_parameters_from_file)  
  {
    printf("[Hardware Bridge] Loading parameters from file...\n");
    try                      
    {
      _robotParams.initializeFromYamlFile(THIS_COM "config/mini-cheetah-defaults.yaml");//从Yaml（config/mini-cheetah-defaults.yaml）文件初始化机器人参数
    }
    catch(std::exception& e) //从Yaml文件初始化机器人参数失败，报错
    {
      printf("Failed to initialize robot parameters from yaml file: %s\n", e.what());
      exit(1);
    }
    if(!_robotParams.isFullyInitialized()) //从文件加载机器人参数不全，报错
    {
      printf("Failed to initialize all robot parameters\n");
      exit(1);
    }
    printf("Loaded robot parameters\n");

    if(_userControlParameters)    //用户控制参数加载
    {
      try                      
      {
        _userControlParameters->initializeFromYamlFile(THIS_COM "config/mc-mit-ctrl-user-parameters.yaml"); //从Yaml（config/mc-mit-ctrl-user-parameters.yaml）文件初始化用户参数
      } 
      catch(std::exception& e)  //从Yaml文件初始化用户参数失败，报错
      {
        printf("Failed to initialize user parameters from yaml file: %s\n", e.what());
        exit(1);
      }
      if(!_userControlParameters->isFullyInitialized()) //从Yaml文件加载机器人参数不全，报错
      {
        printf("Failed to initialize all user parameters\n");
        exit(1);
      }
      printf("Loaded user parameters\n");
    } 
    else 
    {
      printf("Did not load user parameters because there aren't any\n");
    }
  } 
  
  //加载参数文件标志位关闭,使用终端参数输入
  else  
  {
    printf("[Hardware Bridge] Loading parameters over LCM...\n");
    while (!_robotParams.isFullyInitialized()) //从文件加载机器人参数不全，等待机器人参数输入，等1秒
    {
      printf("[Hardware Bridge] Waiting for robot parameters...\n");
      usleep(1000000);
    }
    if(_userControlParameters)//用户控制参数加载，等待机器人参数输入，等1秒
    {
      while (!_userControlParameters->isFullyInitialized()) 
      {
        printf("[Hardware Bridge] Waiting for user parameters...\n");
        usleep(1000000);
      }
    }
  }

  printf("[Hardware Bridge] Got all parameters, starting up!\n");
 
  ///////////////////////*（4）使用RobotRunner这个类创建一个对象_robotRunner,并对命令和数据和对象的变量建立映射关系*/////////////////////
  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

  _robotRunner->driverCommand = &_gamepadCommand;                         //1)手柄命令数据映射到driverCommand
  _robotRunner->spiData = &_spiData;                                      //2)控制器SPI数据映射到spiData
  _robotRunner->spiCommand = &_spiCommand;                                //3)控制器SPI命令映射到spiCommand
  _robotRunner->robotType = RobotType::MINI_CHEETAH;                      //4)机器人类型MINI_CHEETAH映射到robotType
  _robotRunner->vectorNavData = &_vectorNavData;                          //5)imu导航数据映射到vectorNavData
  _robotRunner->controlParameters = &_robotParams;                        //6)机器人参数映射到controlParameters
  _robotRunner->visualizationData = &_visualizationData;                  //7)可视化数据_visualizationData映射到visualizationData
  _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;    //8)机器人外形可视化_mainCheetahVisualization映射到cheetahMainVisualization
  _firstRun = false;

  /////////////////////*(5)初始化控制任务，启动任务管理器*//////////////////////////////////////////////////////////////////////////
  
  /*(1)开始运行定期打印任务管理器中所有任务的状态任务*/
   statusTask.start();


  /*(2)开始运行spi任务*/ 
  PeriodicMemberFunction<MiniCheetahHardwareBridge> spiTask(                    //创建一个SPI通讯任务：fun（任务管理器,周期，任务名称，运行的函数，对象）
      &taskManager, .002, "spi", &MiniCheetahHardwareBridge::runSpi, this);     
  spiTask.start();                                                              //开始运行spi任务


  /*（3）开始运行接收IMU的数据任务*/
  if(_microstrainInit)//如果IMU初始化完成后
    _microstrainThread = std::thread(&MiniCheetahHardwareBridge::runMicrostrain, this);


  /*（4）【重点！！！】启动任务管理器，开始运行机器人控制器任务*/
  _robotRunner->start();


  /*（5）开始运行可视化任务*/
  PeriodicMemberFunction<MiniCheetahHardwareBridge> visualizationLCMTask(     //创建一个可视化任务：fun（任务管理器,周期，任务名称，运行的函数，对象）
      &taskManager, .0167, "lcm-vis",
      &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
  visualizationLCMTask.start();                                               //开始运行可视化任务


  /*（6）开始运行rc控制器任务*/ 
  _port = init_sbus(false);  // 不是模拟模式，是读真正串口数据到_port端口中
  PeriodicMemberFunction<HardwareBridge> sbusTask(                            //创建一个遥控手柄任务
      &taskManager, .005, "rc_controller", &HardwareBridge::run_sbus, this);
  sbusTask.start();                                                           //开始运行遥控手柄任务

  /*（7）开始运行更新并发布LCM数据任务*/ 
  PeriodicMemberFunction<MiniCheetahHardwareBridge> microstrainLogger(
      &taskManager, .001, "microstrain-logger", &MiniCheetahHardwareBridge::logMicrostrain, this);
  microstrainLogger.start();

  for (;;) 
  {
    usleep(1000000);
    // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
  }
}


/*!
 * 功能：用SBUS接收RC，通过接收到的手柄数据，设置移动参数、模式
 */ 
void HardwareBridge::run_sbus() 
{
  if (_port > 0) //若串口收到数据
  {
    int x = receive_sbus(_port);  //（1）从port口接收手柄的数据，并保存在变量x中
    if (x) 
    {
      sbus_packet_complete();     //（2）通过接收到的手柄数据，设置移动参数、模式
    }
  }
}


/*!
 * 功能：接收IMU的数据
 */ 
void MiniCheetahHardwareBridge::runMicrostrain() 
{
  while(true) 
  {
    //////////*（1）接收更新IMU的数据*////////////////////////////
    _microstrainImu.run();    

    /////////*（2）若使用了IMU传感器，就搬运IMU数据*///////////////
#ifdef USE_MICROSTRAIN
    //1）加速度数据
    _vectorNavData.accelerometer = _microstrainImu.acc;
    //2)四元数数据
    _vectorNavData.quat[0] = _microstrainImu.quat[1];  
    _vectorNavData.quat[1] = _microstrainImu.quat[2];  
    _vectorNavData.quat[2] = _microstrainImu.quat[3];  
    _vectorNavData.quat[3] = _microstrainImu.quat[0];  
    //3)陀螺仪数据
    _vectorNavData.gyro = _microstrainImu.gyro;        
#endif
  }
}


/*!
 * 功能：更新LCM数据，并发布
 */
void MiniCheetahHardwareBridge::logMicrostrain() 
{
  _microstrainImu.updateLCM(&_microstrainData);               //（1）更新IMU_LCM数据
  _microstrainLcm.publish("microstrain", &_microstrainData);  //（2）发布_microstrainData消息
}


/*!
 * 初始化Mini Cheetah特定硬件，初始化SPI、初始化IMU
 */
void MiniCheetahHardwareBridge::initHardware() 
{
  /*（1）初始化IMU*/
  _vectorNavData.quat << 1, 0, 0, 0;                          //（1）定义imu导航数据
#ifndef USE_MICROSTRAIN
  printf("[MiniCheetahHardware] Init vectornav\n");
  if (!init_vectornav(&_vectorNavData))                       //（2）若初始化Vectornav矢量导航通信并设置传感器函数不成功
  {
    printf("Vectornav failed to initialize\n");
    //initError("failed to initialize vectornav!\n", false);
  }
#endif
  _microstrainInit = _microstrainImu.tryInit(0, 921600);//(3)初始化IMU

  /*（2）初始化SPI*/
  init_spi();                                           //初始化SPI

}



/*!
 *  初始化Cheetah3特定的硬件
 */ 
void Cheetah3HardwareBridge::initHardware() 
{
  _vectorNavData.quat << 1, 0, 0, 0;
  printf("[Cheetah 3 Hardware] Init vectornav\n");
  if (!init_vectornav(&_vectorNavData)) 
  {
    printf("Vectornav failed to initialize\n");
    printf_color(PrintColor::Red, "****************\n"
                                  "**  WARNING!  **\n"
                                  "****************\n"
                                  "  IMU DISABLED  \n"
                                  "****************\n\n");
  }
}



/*!
 * 运行迷你猎豹SPI
 * 步骤：
 * （1）获取SPI命令
 * （2）获取SPI数据
 * （3）SPI模块运行
 * （4）发布spi数据
 */
void MiniCheetahHardwareBridge::runSpi() 
{	
  spi_command_t* cmd = get_spi_command();           //（1）获取SPI命令
  spi_data_t* data = get_spi_data();                //（2）获取SPI数据

  memcpy(cmd, &_spiCommand, sizeof(spi_command_t)); //（3）把SPI的命令复制到CMD的变量里面
  spi_driver_run();                                 //（4）SPI模块运行，调用spi的库函数实现的
  memcpy(&_spiData, data, sizeof(spi_data_t));      //（5）把SPI的数据复制到变量_spiData里面

  _spiLcm.publish("spi_data", data);                //（6）发布spi数据
  _spiLcm.publish("spi_command", cmd);              //（7）发布SPI命令
}


/*!
 * 功能：运行lcm获取和发送信息
 * 步骤：
 * （1）通过以太网设置upboard命令
 * （2）运行以太网通讯
 * （3）通过以太网获取upboard数据
 * （4）通过LCM发布数据
 */ 
void Cheetah3HardwareBridge::runEcat() 
{	
  rt_ethercat_set_command(_tiBoardCommand);//通过以太网设置upboard命令
  rt_ethercat_run();                       //运行以太网通讯
  rt_ethercat_get_data(_tiBoardData);      //通过以太网获取upboard数据
  publishEcatLCM();                        //通过LCM发布数据
}


/*!
 * 功能：通过以太网LCM，发送四条腿自定义消息
 */ 
void Cheetah3HardwareBridge::publishEcatLCM() 
{
  for(int leg = 0; leg < 4; leg++) 
  {
    //期望位置
    ecatCmdLcm.x_des[leg] = _tiBoardCommand[leg].position_des[0];
    ecatCmdLcm.y_des[leg] = _tiBoardCommand[leg].position_des[1];
    ecatCmdLcm.z_des[leg] = _tiBoardCommand[leg].position_des[2];
    //期望速度
    ecatCmdLcm.dx_des[leg] = _tiBoardCommand[leg].velocity_des[0];
    ecatCmdLcm.dy_des[leg] = _tiBoardCommand[leg].velocity_des[1];
    ecatCmdLcm.dz_des[leg] = _tiBoardCommand[leg].velocity_des[2];
    //位置KP
    ecatCmdLcm.kpx[leg] = _tiBoardCommand[leg].kp[0];
    ecatCmdLcm.kpy[leg] = _tiBoardCommand[leg].kp[1];
    ecatCmdLcm.kpz[leg] = _tiBoardCommand[leg].kp[2];
    //位置KD
    ecatCmdLcm.kdx[leg] = _tiBoardCommand[leg].kd[0];
    ecatCmdLcm.kdy[leg] = _tiBoardCommand[leg].kd[1];
    ecatCmdLcm.kdz[leg] = _tiBoardCommand[leg].kd[2];
    //使能
    ecatCmdLcm.enable[leg] = _tiBoardCommand[leg].enable;
    //腿零位置
    ecatCmdLcm.zero_joints[leg] = _tiBoardCommand[leg].zero;
    //反作用力力矩
    ecatCmdLcm.fx_ff[leg] = _tiBoardCommand[leg].force_ff[0];
    ecatCmdLcm.fy_ff[leg] = _tiBoardCommand[leg].force_ff[1];
    ecatCmdLcm.fz_ff[leg] = _tiBoardCommand[leg].force_ff[2];
    
    ecatCmdLcm.tau_abad_ff[leg] = _tiBoardCommand[leg].tau_ff[0];
    ecatCmdLcm.tau_hip_ff[leg] = _tiBoardCommand[leg].tau_ff[1];
    ecatCmdLcm.tau_knee_ff[leg] = _tiBoardCommand[leg].tau_ff[2];

    ecatCmdLcm.q_des_abad[leg] = _tiBoardCommand[leg].q_des[0];
    ecatCmdLcm.q_des_hip[leg] = _tiBoardCommand[leg].q_des[1];
    ecatCmdLcm.q_des_knee[leg] = _tiBoardCommand[leg].q_des[2];

    ecatCmdLcm.qd_des_abad[leg] = _tiBoardCommand[leg].qd_des[0];
    ecatCmdLcm.qd_des_hip[leg] = _tiBoardCommand[leg].qd_des[1];
    ecatCmdLcm.qd_des_knee[leg] = _tiBoardCommand[leg].qd_des[2];

    ecatCmdLcm.kp_joint_abad[leg] = _tiBoardCommand[leg].kp_joint[0];
    ecatCmdLcm.kp_joint_hip[leg] = _tiBoardCommand[leg].kp_joint[1];
    ecatCmdLcm.kp_joint_knee[leg] = _tiBoardCommand[leg].kp_joint[2];

    ecatCmdLcm.kd_joint_abad[leg] = _tiBoardCommand[leg].kd_joint[0];
    ecatCmdLcm.kd_joint_hip[leg] = _tiBoardCommand[leg].kd_joint[1];
    ecatCmdLcm.kd_joint_knee[leg] = _tiBoardCommand[leg].kd_joint[2];
    //最大力矩
    ecatCmdLcm.max_torque[leg] = _tiBoardCommand[leg].max_torque;
  }

  for(int leg = 0; leg < 4; leg++) {
    ecatDataLcm.x[leg] = _tiBoardData[leg].position[0];
    ecatDataLcm.y[leg] = _tiBoardData[leg].position[1];
    ecatDataLcm.z[leg] = _tiBoardData[leg].position[2];
    ecatDataLcm.dx[leg] = _tiBoardData[leg].velocity[0];
    ecatDataLcm.dy[leg] = _tiBoardData[leg].velocity[1];
    ecatDataLcm.dz[leg] = _tiBoardData[leg].velocity[2];
    ecatDataLcm.fx[leg] = _tiBoardData[leg].force[0];
    ecatDataLcm.fy[leg] = _tiBoardData[leg].force[1];
    ecatDataLcm.fz[leg] = _tiBoardData[leg].force[2];
    ecatDataLcm.q_abad[leg] = _tiBoardData[leg].q[0];
    ecatDataLcm.q_hip[leg] = _tiBoardData[leg].q[1];
    ecatDataLcm.q_knee[leg] = _tiBoardData[leg].q[2];
    ecatDataLcm.dq_abad[leg] = _tiBoardData[leg].dq[0];
    ecatDataLcm.dq_hip[leg] = _tiBoardData[leg].dq[1];
    ecatDataLcm.dq_knee[leg] = _tiBoardData[leg].dq[2];
    ecatDataLcm.tau_abad[leg] = _tiBoardData[leg].tau[0];
    ecatDataLcm.tau_hip[leg] = _tiBoardData[leg].tau[1];
    ecatDataLcm.tau_knee[leg] = _tiBoardData[leg].tau[2];
    ecatDataLcm.tau_des_abad[leg] = _tiBoardData[leg].tau_des[0];
    ecatDataLcm.tau_des_hip[leg] = _tiBoardData[leg].tau_des[1];
    ecatDataLcm.tau_des_knee[leg] = _tiBoardData[leg].tau_des[2];
    ecatDataLcm.loop_count_ti[leg] = _tiBoardData[leg].loop_count_ti;
    ecatDataLcm.ethercat_count_ti[leg] = _tiBoardData[leg].ethercat_count_ti;
    ecatDataLcm.microtime_ti[leg] = _tiBoardData[leg].microtime_ti;
  }

  _ecatLCM.publish("ecat_cmd", &ecatCmdLcm);
  _ecatLCM.publish("ecat_data", &ecatDataLcm);
}


/*!
 * LCM发送LCM可视化数据
 */
void HardwareBridge::publishVisualizationLCM() 
{
  /*(1)定义一个可视化的变量*/
  cheetah_visualization_lcmt visualization_data;

  /*(2)赋值猎豹可视化位置p*/
  for (int i = 0; i < 3; i++) 
  {
    visualization_data.x[i] = _mainCheetahVisualization.p[i];                     
  }

  /*(3)赋值猎豹可视化quat、颜色和透明度*/
  for (int i = 0; i < 4; i++) 
  {
    visualization_data.quat[i] = _mainCheetahVisualization.quat[i];               
    visualization_data.rgba[i] = _mainCheetahVisualization.color[i];             
  }

  /*（4）赋值猎豹可视化电机角度*/
  for (int i = 0; i < 12; i++) 
  {
    visualization_data.q[i] = _mainCheetahVisualization.q[i];                      
  }

  /*（5）发布LCM可视化数据*/
  _visualizationLCM.publish("main_cheetah_visualization", &visualization_data);  
}


/*!
 * Cheetah3HardwareBridge实例化
 */
Cheetah3HardwareBridge::Cheetah3HardwareBridge(RobotController *rc) : HardwareBridge(rc),  _ecatLCM(getLcmUrl(255)) 
{
}


/*!
 * 功能：Cheetah3运行
 * 步骤：
 * （1）两条狗对应的LCM和线程初始化
 * （2）初始化Cheetah3特定的硬件
 * （3）等待参数加载
 * （4）开始运行
 */
void Cheetah3HardwareBridge::run() 
{
  initCommon();     //订阅消息
  initHardware();   //初始化Cheetah3特定的硬件

//等待参数加载
  printf("[Hardware Bridge] Loading parameters over LCM...\n");
  while (!_robotParams.isFullyInitialized())                          //若没有加载完机器人参数，打印出来，等一秒
  {
    printf("[Hardware Bridge] Waiting for robot parameters...\n");
    usleep(1000000);
  }
  if(_userControlParameters) 
  {
    while (!_userControlParameters->isFullyInitialized()) 
    {
      printf("[Hardware Bridge] Waiting for user parameters...\n");
      usleep(1000000);
    }
  }

//开始运行
  printf("[Hardware Bridge] Got all parameters, starting up!\n");

//（1）实例化运行器 传入控制器，任务管理器 参数 名称
  _robotRunner =
      new RobotRunner(_controller, &taskManager, _robotParams.controller_dt, "robot-control");

//（2）之前获得相关参数给运行器
  _robotRunner->driverCommand = &_gamepadCommand;
  _robotRunner->tiBoardData = _tiBoardData;
  _robotRunner->tiBoardCommand = _tiBoardCommand;
  _robotRunner->robotType = RobotType::CHEETAH_3;
  _robotRunner->controlParameters = &_robotParams;
  _robotRunner->visualizationData = &_visualizationData;
  _robotRunner->cheetahMainVisualization = &_mainCheetahVisualization;
  _robotRunner->vectorNavData = &_vectorNavData;

//（3）初始化运行器 
  _robotRunner->init();
  _firstRun = false;

  //（4）初始化控制线程 
  statusTask.start();

  rt_ethercat_init();
  // （5）自定义消息传输任务开始
  PeriodicMemberFunction<Cheetah3HardwareBridge> ecatTask(
      &taskManager, .001, "ecat", &Cheetah3HardwareBridge::runEcat, this);//任务管理器  时间间隔1khz 回调函数（类的成员函数），对象
  ecatTask.start();

  // （6）机器人控制器开始
  _robotRunner->start();

  // （7）可视化开始 同上
  PeriodicMemberFunction<Cheetah3HardwareBridge> visualizationLCMTask(
      &taskManager, .0167, "lcm-vis",
      &MiniCheetahHardwareBridge::publishVisualizationLCM, this);
  visualizationLCMTask.start();

//（8）主循环 定时打印任务状态
  for (;;) 
  {
    usleep(100000);
    taskManager.printStatus();
    // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
  }
}

#endif
