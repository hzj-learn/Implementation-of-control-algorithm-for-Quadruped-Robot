/*=========================== Balance Stand ===========================*/
/*
  *FSM状态，强制所有支腿落地，并使用QP，用于瞬时平衡控制的平衡控制器。
 */

#include "FSM_State_BalanceStand.h"
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

/*
 * 功能：向传递状态特定信息的FSM状态的构造函数   通用FSM状态构造函数
 * @param _controlFSMData 保存所有相关的控制数据
 */
template <typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND,"BALANCE_STAND") 
{  
  this->turnOnAllSafetyChecks();                  //设置预控安全检查
  this->checkPDesFoot = false;                    //关闭脚踏pos命令，因为它在WBC中设置为操作任务
  this->footFeedForwardForces = Mat34<T>::Zero(); //将GRF初始化为0s

  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();
  _wbc_ctrl->setFloatingBaseWeight(1000.);
}


/*
 * 功能：进入站立平衡状态时要执行的行为
 */
template <typename T>
void FSM_State_BalanceStand<T>::onEnter()
 {
  this->nextStateName = this->stateName;//默认为不过渡
  this->transitionData.zero();//重置转换数据
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;//始终将步态设置为站在这种状态
  _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;

  if(_ini_body_pos[2] < 0.2) 
  {
    _ini_body_pos[2] = 0.3;
  }
  last_height_command = _ini_body_pos[2];
  _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
  _body_weight = this->_data->_quadruped->_bodyMass * 9.81;
}



/**
 * 功能：调用要在每个控制循环迭代中执行的函数。
 */
template <typename T>
void FSM_State_BalanceStand<T>::run() 
{
  Vec4<T> contactState;
  contactState<< 0.5, 0.5, 0.5, 0.5; //四条腿都要接触
  this->_data->_stateEstimator->setContactPhase(contactState);
  BalanceStandStep();   
}




/**
 * 功能：管理可以由用户命令或状态事件触发器转换为哪些状态。
 * @return 要转换为的枚举FSM状态名
 */
template <typename T>
FSM_StateName FSM_State_BalanceStand<T>::checkTransition() 
{
  _iter++;//获取下一个状态
  switch ((int)this->_data->controlParameters->control_mode) //切换FSM控制模式
  {
    case K_BALANCE_STAND:
      //基于状态的转换的正常操作
      //需要一个工作状态估计器
      /*
      if (velocity > v_max) 
      {
        this->nextStateName = FSM_StateName::LOCOMOTION; //通知该状态下一个状态即将到来
        this->transitionDuration = 0.0; //根据要求即时转换到移动状态
        this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;//将调度程序中的下一步设置为
      }*/

      //测试：就地显示非用户请求的自动转换
      /*if (_iter >= 5458) 
      {
        this->nextStateName = FSM_StateName::LOCOMOTION;
        this->_data->controlParameters->control_mode = K_LOCOMOTION;
        this->transitionDuration = 0.0;
        this->_data->_gaitScheduler->gaitData._nextGait =
            GaitType::AMBLE;  // TROT; // Or get whatever is in
                              // main_control_settings
        _iter = 0;
      }
      */
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;                    //要求更改平衡架
      this->transitionDuration = 0.0;                                     // 根据要求即时转换到移动状态
      this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;   //将调度程序中的下一步设置为
      break;

    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;
      this->transitionDuration = 0.0;      //过渡时间很快

      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      this->transitionDuration = 0.0;      //过渡时间很快
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = 0.0;      //过渡时间很快
      break;

    case K_BACKFLIP:
      this->nextStateName = FSM_StateName::BACKFLIP;
      this->transitionDuration = 0.;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_BALANCE_STAND << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
  return this->nextStateName;  //将下一个状态名返回给FSM 
}


/**
 *功能：处理robot在状态之间的实际转换。
 * @return 如果转换完成，则为true
 */
template <typename T>
TransitionData<T> FSM_State_BalanceStand<T>::transition() 
{
  switch (this->nextStateName)             //切换FSM控制模式
  {
    case FSM_StateName::LOCOMOTION:        //平衡站立模式
      BalanceStandStep();
      _iter++;
      if (_iter >= this->transitionDuration * 1000) 
      {
        this->transitionData.done = true;
      } 
      else 
      {
        this->transitionData.done = false;
      }
      break;

    case FSM_StateName::PASSIVE:           //人为调试模式
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:    //恢复站立模式
      this->transitionData.done = true;
      break;

    case FSM_StateName::BACKFLIP:          //后空翻模式
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:            //视觉模式
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }
  return this->transitionData;  //将转换数据返回到FSM
}



/**
 * 功能：在退出状态时清除状态信息。
 */
template <typename T>
void FSM_State_BalanceStand<T>::onExit() 
{
  _iter = 0;
}



/**
 * 功能：计算平衡站立的时候，每只脚的腿部控制器的命令。
 */
template <typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep() 
{
  _wbc_data->pBody_des = _ini_body_pos;         //初始化躯干位置
  _wbc_data->vBody_des.setZero();               //设置躯干的线速度为0
  _wbc_data->aBody_des.setZero();               //设置躯干的加速度为0
  _wbc_data->pBody_RPY_des = _ini_body_ori_rpy; //初始化躯干rpy方向
  
  //计算躯干的方向和高度
  if(this->_data->controlParameters->use_rc)    //若在使用手柄，计算躯干的方向和高度
  {
    const rc_control_settings* rc_cmd = this->_data->_desiredStateCommand->rcCommand;
    //使用手柄时方向计算
    _wbc_data->pBody_RPY_des[0] = rc_cmd->rpy_des[0]*1.4;
    _wbc_data->pBody_RPY_des[1] = rc_cmd->rpy_des[1]*0.46;
    _wbc_data->pBody_RPY_des[2] -= rc_cmd->rpy_des[2];
    //使用手柄时高度计算
    _wbc_data->pBody_des[2] += 0.12 * rc_cmd->height_variation;
  }
  else                                          //若不使用手柄，计算躯干的方向和高度
  {
    //不使用手柄时方向计算
    _wbc_data->pBody_RPY_des[0] = 
     0.6* this->_data->_desiredStateCommand->gamepadCommand->leftStickAnalog[0];
     _wbc_data->pBody_RPY_des[1] = 
      0.6*this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];
    _wbc_data->pBody_RPY_des[2] -= 
      this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[1];
    //不使用手柄时高度
    _wbc_data->pBody_des[2] += 
      0.12 * this->_data->_desiredStateCommand->gamepadCommand->rightStickAnalog[0];
  }
 
  _wbc_data->vBody_Ori_des.setZero();           //设置躯干线速度为0

  for(size_t i(0); i<4; ++i)              //设置每条腿的参数
  {
    _wbc_data->pFoot_des[i].setZero();    //脚的位置设置为0
    _wbc_data->vFoot_des[i].setZero();    //脚的速度设置为0
    _wbc_data->aFoot_des[i].setZero();    //脚的加速度设置为0
    _wbc_data->Fr_des[i].setZero();       //
    _wbc_data->Fr_des[i][2] = _body_weight/4.;
    _wbc_data->contact_state[i] = true;
  }
  
  if(this->_data->_desiredStateCommand->trigger_pressed) 
  {
    _wbc_data->pBody_des[2] = 0.05;

    if(last_height_command - _wbc_data->pBody_des[2] > 0.001) 
    {
      _wbc_data->pBody_des[2] = last_height_command - 0.001;
    }
  }
  last_height_command = _wbc_data->pBody_des[2];
  _wbc_ctrl->run(_wbc_data, *this->_data);
}

// template class FSM_State_BalanceStand<double>;
template class FSM_State_BalanceStand<float>;
