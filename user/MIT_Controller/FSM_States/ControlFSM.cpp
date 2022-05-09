/*============================ Control FSM ============================*/
/*
  管理机器人控制的有限状态机。处理对FSM状态函数的调用，并管理所有状态之间的转换。
 */

#include "ControlFSM.h"
#include <rt/rt_rc_interface.h>

/**
 * 功能：控件FSM的构造函数
 * 步骤：
 * （1）传入所有必要的数据并将其存储在结构中。使用启动状态和操作模式初始化FSM。
 * （2）传入参数从robotrunner的给RobotController* _robot_ctrl赋值来
 * @param _quadruped            四足的信息
 * @param _stateEstimator       估计状态的信息
 * @param _legController        与腿部控制器的接口
 * @param _gaitScheduler        步态规划器，控制预定的足部接触模式
 * @param _desiredStateCommand  所需的COM状态轨迹
 * @param controlParameters     从GUI中传入控制参数
 * @param VisualizationData     GUI可视化数据
 * @param MIT_UserParameters    用户参数
 */
template <typename T>
ControlFSM<T>::ControlFSM(Quadruped<T> *_quadruped,
                          StateEstimatorContainer<T> *_stateEstimator,
                          LegController<T> *_legController,
                          GaitScheduler<T> *_gaitScheduler,
                          DesiredStateCommand<T> *_desiredStateCommand,
                          RobotControlParameters *controlParameters,
                          VisualizationData *visualizationData,
                          MIT_UserParameters *userParameters)
{
  /*（1）将指针形参赋值到data结构体*/
  data._quadruped = _quadruped;                       //（1）四足的信息数据
  data._stateEstimator = _stateEstimator;             //（2）估计状态的信息数据
  data._legController = _legController;               //（3）与腿部控制器的接口数据
  data._gaitScheduler = _gaitScheduler;               //（4）步态规划器，控制预定的足部接触模式数据
  data._desiredStateCommand = _desiredStateCommand;   //（5）所需的COM状态轨迹数据
  data.controlParameters = controlParameters;         //（6）从GUI中传入控制参数数据
  data.visualizationData = visualizationData;         //（7）GUI可视化数据
  data.userParameters = userParameters;               //（8）用户参数数据

  /*（2）初始化相关FSM状态列表*/
  statesList.invalid            = nullptr;                                  //（1）初始化invalid->FSM是否使用状态数据及相关控制器
  statesList.passive            = new FSM_State_Passive<T>(&data);          //（2）初始化passive->被人为操纵调试状态数据及相关控制器
  statesList.jointPD            = new FSM_State_JointPD<T>(&data);          //（3）初始化jointPD->关节PD控制器状态数据及相关控制器
  statesList.impedanceControl   = new FSM_State_ImpedanceControl<T>(&data); //（4）初始化impedanceControl->阻抗控制状态数据及相关控制器
  statesList.standUp            = new FSM_State_StandUp<T>(&data);          //（5）初始化standUp->站立过程状态数据及相关控制器
  statesList.balanceStand       = new FSM_State_BalanceStand<T>(&data);     //（6）初始化balanceStand->站立平衡状态数据及相关控制器
  statesList.locomotion         = new FSM_State_Locomotion<T>(&data);       //（7）初始化locomotion->运动状态数据及相关控制器
  statesList.recoveryStand      = new FSM_State_RecoveryStand<T>(&data);    //（8）初始化recoveryStand->恢复站立状态数据及相关控制器
  statesList.vision             = new FSM_State_Vision<T>(&data);           //（9）初始化vision->可视化状态数据及相关控制器
  statesList.backflip           = new FSM_State_BackFlip<T>(&data);         //（10）初始化backflip->后空翻状态数据及相关控制器
  statesList.frontJump          = new FSM_State_FrontJump<T>(&data);        //（11）初始化frontJump->向前跳状态数据及相关控制器
  /*（3）创建安全检查器，进行安全检查并限制操作*/
  safetyChecker = new SafetyChecker<T>(&data); 
  /*（4）使用被动FSM状态初始化FSM*/
  initialize();      //初始化控件FSM                        
}



/**
 * 功能：初始化控件FSM。
 * 应设置为被动状态和正常工作模式。
 * 赋值设置操作而已                          
 */
template <typename T>
void ControlFSM<T>::initialize()
{
  currentState = statesList.passive;          //（1）用控制数据初始化一个新的FSM状态
  currentState->onEnter();                    //（2）进入新的当前状态
  nextState = currentState;                   //（3）初始化为不处于过渡状态状态进行迭代
  operatingMode = FSM_OperatingMode::NORMAL;  //（4）初始化FSM模式为正常操作
}



/**
 * 功能：运行状态机
 * 步骤：
 * （1）运行安全检查
 * （2）判断是否使用遥控器，若使用遥控器，根据手柄控制步态调度模式（这样遥控器的优先级高一点）
 * （3）若操作模式不是停止模式，就启动机器人运行状态机FSM
 * （4）操作模式是停止模式，停止运行机器人控制代码
 * （5）打印FSM的当前状态
 * （6）增加迭代计数器
 */
template <typename T>
void ControlFSM<T>::runFSM()
{
  /*（1）检查在运行站立状态时，躯干方向是否安全*/
  //即检测躯干的横滚角度和俯仰角度是否超过不安全的阈值
  operatingMode = safetyPreCheck();               

  /*（2）判断是否使用遥控器，若使用遥控器，使用手柄设置输入步态类型（这样遥控器的优先级高一点）*/
  if (data.controlParameters->use_rc)             
  {                                                          
    int rc_mode = data._desiredStateCommand->rcCommand->mode;    //（1）根据手柄控制设定模式，把手柄的指令赋值给rc_mode变量
  
    /*若是恢复站立模式1*/
    if (rc_mode == RC_mode::RECOVERY_STAND)                                     
    {
      data.controlParameters->control_mode = K_RECOVERY_STAND;
    }

     /*若是运动状态模式2*/
    else if (rc_mode == RC_mode::LOCOMOTION)                                   
    {
      data.controlParameters->control_mode = K_LOCOMOTION;
    }

     /*若是站立平衡模式3*/
    else if (rc_mode == RC_mode::QP_STAND)                                     
    {
      data.controlParameters->control_mode = K_BALANCE_STAND;
    }

    /*若是可视化模式4*/
    else if (rc_mode == RC_mode::VISION)                                        
    {
      data.controlParameters->control_mode = K_VISION;
    }

    /*若是后空翻模式5*/
    else if (rc_mode == RC_mode::BACKFLIP || rc_mode == RC_mode::BACKFLIP_PRE)  
    {
      data.controlParameters->control_mode = K_BACKFLIP;
    }
  }

  /*（3）【重点！！！】判断操作模式是否处于停止模式，决定是否启动机器人运行状态机FSM*/
  if (operatingMode != FSM_OperatingMode::ESTOP) 
  {
    /*如果操作模式正常则运行正常控件，这段是必须运行的*/
    if (operatingMode == FSM_OperatingMode::NORMAL)    
    {
      nextStateName = currentState->checkTransition(); //获取的当前转换状态命令，判断机器人FSM状态有没有改变

      if (nextStateName != currentState->stateName)    //若有检测到切换命令，即下一个状态不等于本状态
      {
        operatingMode = FSM_OperatingMode::TRANSITIONING;//1）将FSM工作模式设置为transitioning
        nextState = getNextState(nextStateName);         //2) 按名称获取下一个FSM状态
      }
      else                                             //若没检测到切换命令
      { 
        //【重点！！！】运行当前状态控制器
        //在每个控制循环迭代中执行的函数，不同状态就会对应进去不同的run()函数
        //注意这里的状态先粗略的分为5大类，包括RECOVERY_STAND、LOCOMOTION、QP_STAND、VISION、BACKFLIP
        //其中LOCOMOTION中有细分了比较多种的步态，目前调的比较好的状态有，trotting\walking2\trotrunning三个状态
        currentState->run();    
      }
    }

    /*如果检测到切换状态*/
    if (operatingMode == FSM_OperatingMode::TRANSITIONING) 
    {
      transitionData = currentState->transition();      //3）获得转换数据，进行状态数据转换操作
      safetyPostCheck();                                //4）检查机器人状态以确保安全操作，每次状态切换都要做的
      if (transitionData.done)                          //5）若状态转换已经完成                        
      {
        currentState->onExit();                          //（1）退出当前状态
        currentState = nextState;                        //（2）切换到下一个状态
        currentState->onEnter();                         //（3）进入新状态
        operatingMode = FSM_OperatingMode::NORMAL;       //（4）操作模式设置为正常模式，表示没有接收到切换指令,相当于切换状态复位
      }
    }
   
    /*如果没检测到切换状态*/
    else                                                    
    {
      safetyPostCheck();//非转换状态 仅会进行检查 即任何状态下进行检查并限制
    }
  }
 
  /*（3）操作模式是停止模式，停止运行机器人控制代码*/
  else                                            
  { 
    currentState = statesList.passive;          //设置当前状态为手动被动调试
    currentState->onEnter();                    //进行当前状态检测
    nextStateName = currentState->stateName;    //状态机状态更替迭代
  }
 
  /*（4）打印FSM的当前状态*/
  printInfo(0);

  /*（5）增加迭代计数器*/
  iter++;
}




/**
 * 功能：检查机器人状态是否符合安全启动操作条件。如果处于不安全状态，则在安全之前不会运行常规状态机控制
 * @return 适当的操作模式
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck()
{
  if (currentState->checkSafeOrientation && data.controlParameters->control_mode != K_RECOVERY_STAND)  //根据当前状态，检查安全方向 
  {
    if (!safetyChecker->checkSafeOrientation()) //如果姿态角不安全，处理逻辑如下
    {
      operatingMode = FSM_OperatingMode::ESTOP;                        //1）停止状态机切换
      std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;//2）打印提示
    }
  }
  return operatingMode;
}


/**
 * 功能：检查脚的安全位置和检查期望的安全前馈力
 * 打印出哪个命令是不安全的。每个状态都有一个选项来开关控制它所关心的命令的检查。
 * @return 适当的操作模式
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck()
{
  if (currentState->checkPDesFoot)           //检查脚的安全位置
  {
    safetyChecker->checkPDesFoot();
  }
  if (currentState->checkForceFeedForward)   //检查期望的安全前馈力
  {
    safetyChecker->checkForceFeedForward();
  }
  return operatingMode;  //默认是返回当前的操作模式
}




/**
 * 功能：命令时返回下一个FSM的批准状态。
 * @param  next 下一个命令枚举的状态名
 * @return next FSM state 下一个状态
 */
template <typename T>
FSM_State<T> *ControlFSM<T>::getNextState(FSM_StateName stateName)
{
//通过枚举状态名选择正确的FSM状态，通过状态列表执行不同的任务
  switch (stateName)
  {
  case FSM_StateName::INVALID:        //不合法的状态
    return statesList.invalid;

  case FSM_StateName::PASSIVE:        //被人为操纵调试状态
    return statesList.passive;

  case FSM_StateName::JOINT_PD:       //关节PD控制器
    return statesList.jointPD;

  case FSM_StateName::IMPEDANCE_CONTROL://阻抗控制
    return statesList.impedanceControl;

  case FSM_StateName::STAND_UP:       //站立过程
    return statesList.standUp;

  case FSM_StateName::BALANCE_STAND:  //站立平衡
    return statesList.balanceStand;

  case FSM_StateName::LOCOMOTION:     //运动
    return statesList.locomotion;

  case FSM_StateName::RECOVERY_STAND: //恢复站立
    return statesList.recoveryStand;

  case FSM_StateName::VISION:         //可视化
    return statesList.vision;

  case FSM_StateName::BACKFLIP:       //后空翻
    return statesList.backflip;

  case FSM_StateName::FRONTJUMP:      //向前跳
    return statesList.frontJump;

  default:
    return statesList.invalid;       //不合法的状态
  }
}




/**
 *功能：定期打印控制FSM信息和重要事件，例如转换初始化和终结。
 *注意：独立功能不要把实际的代码弄乱。
 * @param printing  常规或事件模式选项
 */
template <typename T>
void ControlFSM<T>::printInfo(int opt)
{
  switch (opt)
  {
  case 0: //定期正常打印状态机信息      
    printIter++;                //增量打印迭代
    if (printIter == printNum)  //按命令频率打印状态机的信息
    {
      std::cout << "[CONTROL FSM] Printing FSM Info...\n";
      std::cout
          << "---------------------------------------------------------\n";
      std::cout << "Iteration: " << iter << "\n";
      if (operatingMode == FSM_OperatingMode::NORMAL)             //状态机在正常运行状态，则打印正常信息
      {
        std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                  << "\n";
      }
      else if (operatingMode == FSM_OperatingMode::TRANSITIONING) //状态机在状态切换状态，则打印切换信息
      {
        std::cout << "Operating Mode: TRANSITIONING from "
                  << currentState->stateString << " to "
                  << nextState->stateString << "\n";
      }
      else if (operatingMode == FSM_OperatingMode::ESTOP)         //状态机在停止状态，则打印停止信息
      {
        std::cout << "Operating Mode: ESTOP\n";
      }
      std::cout << "Gait Type: " << data._gaitScheduler->gaitData.gaitName
                << "\n";
      std::cout << std::endl;
      printIter = 0;        //增量打印迭代复位
    }
    break;
 
  case 1: //初始化FSM状态转换 
    std::cout << "[CONTROL FSM] Transition initialized from "
              << currentState->stateString << " to " << nextState->stateString
              << "\n"
              << std::endl;
    break;
 
  case 2: //FSM完成状态转换
    std::cout << "[CONTROL FSM] Transition finalizing from "
              << currentState->stateString << " to " << nextState->stateString
              << "\n"
              << std::endl;
    break;
  }
}

template class ControlFSM<float>;
