/*============================ Locomotion =============================*/
/**
 * 用于机器人移动的FSM状态。管理特定于接触的逻辑并处理对控制器的接口调用。
 * 这种状态应该独立于控制器、步态和期望的轨迹。
 */

#include "FSM_State_Locomotion.h"
#include <Utilities/Timer.h>
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
//#include <rt/rt_interface_lcm.h>

/**
 * 功能：将状态特定信息传递给通用FSM状态构造函数
 * @param _controlFSMData  保存所有相关的控制数据
 * 步骤：
 * （0）根据机型实例化MPC控制器
 * （1）开启所有的安全检查
 * （2）关闭足部pos命令，因为它在WBC中被设置为操作任务
 * （3）将GRF初始化为零
 * （4）将footstep初始化为零
 * （5）建立模型
 * （6）发布运动信息
 */
template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T> *_controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  //（1）根据机器人机型，创建MPC控制器
  if (_controlFSMData->_quadruped->_robotType == RobotType::MINI_CHEETAH)  //实例化MINI_CHEETAHMPC控制器
  {
    //在这里可以改MPC的迭代次数、MPC的预测未来状态数量、控制周期三个形参。默认的参数请到"ConvexMPCLocomotion.h"里面改
    //上面的27是MPC计算的时间间隔27ms，相当于控制频率
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
                                      27 / (1000. * _controlFSMData->controlParameters->controller_dt),
                                      _controlFSMData->userParameters);
    
  }
  else if (_controlFSMData->_quadruped->_robotType == RobotType::CHEETAH_3)//实例化CHEETAH_3MPC控制器
  {
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
                                      33 / (1000. * _controlFSMData->controlParameters->controller_dt),
                                      _controlFSMData->userParameters);
  }
  else                                                                     //既不是MINI_CHEETAH，也不是CHEETAH_3，机型对应不上，报错
  {
    assert(false);
  }

  this->turnOnAllSafetyChecks();                    //（2）开启所有的安全检查
  this->checkPDesFoot = false;                      //（3）关闭足部位置检查命令，因为它在WBC中被设置为操作任务
  this->footFeedForwardForces = Mat34<T>::Zero();   //（4）将反作用力GRF初始化为零
  this->footstepLocations = Mat34<T>::Zero();       //（5）将footstep初始化为零
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());   //（6）建立机器人模型
  _wbc_data = new LocomotionCtrlData<T>();                                        //（7）发布运动信息
}


/**
 *功能：进入状态时要执行的行为
 */
template <typename T>
void FSM_State_Locomotion<T>::onEnter()
{
  this->nextStateName = this->stateName;  //默认是不转换状态
  this->transitionData.zero();            //重置转换数据
  cMPCOld->initialize();                  //初始化CMPC
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;//切换状态时默认trot步态
  printf("[FSM LOCOMOTION] On Enter\n");
}



/**
 * 功能：调用要在每个控制循环迭代中执行的函数
 */
template <typename T>
void FSM_State_Locomotion<T>::run()
{
  LocomotionControlStep();  //为这个迭代调用移动控制逻辑
}




extern rc_control_settings rc_control;
/**
 * 功能：检查能不能进行状态转移，即管理哪些状态可以由用户命令或状态事件触发器切换。
 * @return 要转换为的枚举FSM状态名
 */
template <typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition()
{
  iter++;//获取下一个状态
  if (locomotionSafe())//当前姿态安全才可以切换状态
  {
    switch ((int)this->_data->controlParameters->control_mode)  //切换FSM控制模式
    {
    case K_LOCOMOTION:
      break;

    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;     //要求更改平衡表
      this->transitionDuration = 0.0;                         //过渡时间很快
      break;

    case K_PASSIVE:                                           //啥都不干 
      this->nextStateName = FSM_StateName::PASSIVE;           //要求更改平衡表
      this->transitionDuration = 0.0;                         //过渡时间是直接的
      break;

    case K_STAND_UP:                                            //站起
      this->nextStateName = FSM_StateName::STAND_UP;
      this->transitionDuration = 0.;
      break;

    case K_RECOVERY_STAND:                                    //摔倒翻身
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = 0.;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      this->transitionDuration = 0.;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_LOCOMOTION << " to "
                << this->_data->controlParameters->control_mode << std::endl;
    }
  }
  else                ///当前姿态不安全，不可以切换状态，强制设置为恢复站立状态
  {
    this->nextStateName = FSM_StateName::RECOVERY_STAND;//强制设置为恢复站立状态
    this->transitionDuration = 0.;
    rc_control.mode = RC_mode::RECOVERY_STAND;
  }
  return this->nextStateName;                         //将下一个状态名返回给FSM
}




/**
 * 功能：状态转移，即处理机器人在状态之间的实际转换。
 * @return 在转换完成时返回true。
 */
template <typename T>
TransitionData<T> FSM_State_Locomotion<T>::transition()
{
  switch (this->nextStateName)                        //切换FSM控制模式
  {
    case FSM_StateName::BALANCE_STAND:    //站立平衡状态
      LocomotionControlStep(); //从移动转换到站立时缓慢过度状态  状态转换完成
      iter++;
      if (iter >= this->transitionDuration * 1000)//大于转换时间，表示已经切换完成
      {
        this->transitionData.done = true;
      }
      else                                        //小于转换时间，表示没有切换完成
      {
        this->transitionData.done = false;
      }
      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }
  return this->transitionData;                        //将转换数据返回到FSM
}




/**
 * 功能：移动是否安全有效
 * @return 在有效时返回true,超过设定限幅参数返回false
 * 步骤：
 * （1）获取状态估计器的所有数据
 * （2）当状态估计器返回的roll角度（即rpy[0]）大于最大的翻滚角roll，报警并返回无效
 * （3）当状态估计器返回的pitch角度（即rpy[2]）大于最大的翻滚角pitch，报警并返回无效
 * 
 * （4）获取腿部控制器hip、y-position两个数据
 * （5）如果hip超过限制，报警并返回无效
 * （6）y-position超过限制，报警并返回无效
 * 
 * （7）获取腿部控制器速度数据
 * （8）腿的速度超过限制，报警并返回无效            
 */
template <typename T>
bool FSM_State_Locomotion<T>::locomotionSafe()
{
  auto &seResult = this->_data->_stateEstimator->getResult();//（1）获取状态估计器的所有数据
  const T max_roll = 40;      //定义翻滚角roll的最大角度
  const T max_pitch = 40;     //定义平移角pitch的最大角度

  if (std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll))  //（2）当状态估计器返回的roll角度（即rpy[0]）大于最大的翻滚角roll，报警并返回无效
  {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if (std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) //（3）当状态估计器返回的pitch角度（即rpy[2]）大于最大的翻滚角pitch，报警并返回无效
  {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for (int leg = 0; leg < 4; leg++)//四条腿都要检查
  {
    auto p_leg = this->_data->_legController->datas[leg].p;//（4）获取腿部控制器hip、y-position两个数据
    if (p_leg[2] > 0)                                      //（5）如果hip超过限制，报警并返回无效
    {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if (std::fabs(p_leg[1] > 0.18))                        //（6）y-position超过限制，报警并返回无效
    {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();//（7）获取腿部控制器速度数据
    if (std::fabs(v_leg) > 9.)                             //（8）腿的速度超过限制，报警并返回无效                          
    {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;
}



/**
 * 功能：在退出状态时清除状态信息。
 */
template <typename T>
void FSM_State_Locomotion<T>::onExit()
{
  iter = 0;  //退出时无需清理
}



/**
 * 功能：控制运动状态：通过调用适当的平衡控制器，并解析每个姿态或摆动腿的结果，计算每个脚的腿部控制器的命令。
 * 备注：原理很重要
 */
template <typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep()
{
  ///////////////*（1）【很重要！！】运行CMPC控制器，得到每条腿的反作用力*////////////////////////////
  cMPCOld->run<T>(*this->_data);    //MPC控制器，输入_data数据，输出反作用力

  ///////////////*(2)【很重要！！】运行WBC控制器*//////////////////////////////////////////////////////////////////
//（1）定义期望位置、期望速度、KP\KD，并用腿部控制器数据更新
  //1）定义期望位置、期望速度、KP\KD
  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  //2）用腿部控制器数据更新期望位置、期望速度、KP\KD
  for (int leg(0); leg < 4; ++leg)                
  {
    pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;     //腿部控制器更新每条腿的位置
    vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;     //腿部控制器更新每条腿的速度
    Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;//腿部控制器更新每条腿的KP
    Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;//腿部控制器更新每条腿的KD
  }

//（2）把MPC控制器输出的数据传给WBC控制器的躯干数据和腿部数据中
  if (this->_data->userParameters->use_wbc > 0.9)
  {
     //1）把MPC控制器输出的数据传给WBC控制器的躯干数据中
    _wbc_data->pBody_des = cMPCOld->pBody_des;            //1）把MPC的期望躯干位置传给WBC的期望位置
    _wbc_data->vBody_des = cMPCOld->vBody_des;            //2）把MPC的期望躯干速度传给WBC的期望速度
    _wbc_data->aBody_des = cMPCOld->aBody_des;            //3）把MPC的期望躯干加速度传给WBC的期望加速度
    _wbc_data->pBody_RPY_des = cMPCOld->pBody_RPY_des;    //4）把MPC的期望躯干欧拉角传给WBC的期望欧拉角
    _wbc_data->vBody_Ori_des = cMPCOld->vBody_Ori_des;    //5）把MPC的期望躯干方向传给WBC的期望方向

    //2）把MPC控制器输出的数据传给WBC控制器的腿部数据中
    for (size_t i(0); i < 4; ++i)
    {
      _wbc_data->pFoot_des[i] = cMPCOld->pFoot_des[i];    //1）把MPC的期望腿部位置传给WBC的期望位置
      _wbc_data->vFoot_des[i] = cMPCOld->vFoot_des[i];    //2）把MPC的期望腿部速度传给WBC的期望速度
      _wbc_data->aFoot_des[i] = cMPCOld->aFoot_des[i];    //3）把MPC的期望腿部加速度传给WBC的期望加速度
      _wbc_data->Fr_des[i] = cMPCOld->Fr_des[i];          //4）把MPC的期望腿部反作用力传给WBC的期望反作用力【重要！！！】
    }
    //3）把MPC的期望腿部接触状态传给WBC的腿部接触状态
    _wbc_data->contact_state = cMPCOld->contact_state;    

//（3）运行WBC控制器【重要！！！】
    _wbc_ctrl->run(_wbc_data, *this->_data);              //运行WBC控制器【重要！！！！！！】
  }

//////////////////////////////*（3）更新腿部控制命令*////////////////////////////////////////////////////////////////////////////////
  for (int leg(0); leg < 4; ++leg)//腿控制器
  {
    //this->_data->_legController->commands[leg].pDes = pDes_backup[leg];     //位置指令
    this->_data->_legController->commands[leg].vDes = vDes_backup[leg];       //速度指令
    //this->_data->_legController->commands[leg].kpCartesian = Kp_backup[leg];//笛卡尔KP增益
    this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];  //笛卡尔KD增益
  }
}



/**
 * 功能：用于支撑腿独立阻抗控制。防止腿部打滑和弹跳，以及在高速时跟踪脚部速度
 * 备注：未看到引用
 */
template <typename T>
void FSM_State_Locomotion<T>::StanceLegImpedanceControl(int leg)
{
  //支撑腿的阻抗控制
  this->cartesianImpedanceControl(
      leg, this->footstepLocations.col(leg), Vec3<T>::Zero(),
      this->_data->controlParameters->stand_kp_cartesian,
      this->_data->controlParameters->stand_kd_cartesian);
}
// template class FSM_State_Locomotion<double>;
template class FSM_State_Locomotion<float>;
