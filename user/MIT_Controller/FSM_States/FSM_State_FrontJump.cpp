/*============================= Recovery Stand ==============================*/
/**
.*要求机器人站起来的过渡状态
 *平衡控制模式
 */

#include "FSM_State_FrontJump.h"
#include <Utilities/Utilities_print.h>

/**
 *功能：向传递状态特定信息的FSM状态的构造函数
 *备注：通用FSM状态构造函数。
 * @param _controlFSMData 保存所有相关的控制数据
 */
template <typename T>
FSM_State_FrontJump<T>::FSM_State_FrontJump(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP")
{
  this->checkSafeOrientation = false;   //关闭设置预控安全检查
  this->checkPDesFoot = false;          //关闭位置控制安全检查
  this->checkForceFeedForward = false;  //关闭反馈力矩控制安全检查
  zero_vec3.setZero();                  //设置矩阵为零矩阵
  f_ff << 0.f, 0.f, -25.f;               //写入三个关节的参数
  _data_reader = new DataReader(this->_data->_quadruped->_robotType, FSM_StateName::FRONTJUMP);
  front_jump_ctrl_ = new FrontJumpCtrl<T>(_data_reader, this->_data->controlParameters->controller_dt);
  front_jump_ctrl_->SetParameter();
}


/**
 *功能：进入状态时要执行的行为
 */
template <typename T>
void FSM_State_FrontJump<T>::onEnter() 
{
  this->nextStateName = this->stateName;  //默认为不过渡
  this->transitionData.zero();            //重置转换数据
   //重置迭代计数器
  iter = 0;                              
  _state_iter = 0;
  _count = 0;
  _curr_time = 0;
  _motion_start_iter = 0;
  _b_first_visit = true;
  for(size_t i(0); i < 4; ++i)           //初始位置配置
  {
    initial_jpos[i] = this->_data->_legController->datas[i].q;
  }
  front_jump_ctrl_->SetParameter();      //设置前跳控制参数
}



/**
 * 调用要在每个控制循环迭代中执行的函数。
 */
template <typename T>
void FSM_State_FrontJump<T>::run() 
{
  if (_b_running)//命令计算
   {
    if (!_Initialization()) 
    {
      ComputeCommand();
    }
  } 
  else 
  {
    _SafeCommand();
  }

  ++_count;
  _curr_time += this->_data->controlParameters->controller_dt;

}


/**
 * 功能：前跳状态初始化
*/
template <typename T>
bool FSM_State_FrontJump<T>::_Initialization() 
{ 
  static bool test_initialized(false);
  if (!test_initialized) 
  {
    test_initialized = true;
    printf("[Cheetah Test] Test initialization is done\n");
  }
  if (_count < _waiting_count) //未超时
  {
    for (int leg = 0; leg < 4; ++leg)     //更新关节的5个参数
    {
      this->_data->_legController->commands[leg].qDes = initial_jpos[leg];    //关节的位置信息
      for (int jidx = 0; jidx < 3; ++jidx)
      {
        this->_data->_legController->commands[leg].tauFeedForward[jidx] = 0.;//关节的反馈力矩信息
        this->_data->_legController->commands[leg].qdDes[jidx] = 0.;         //关节的速度信息
        this->_data->_legController->commands[leg].kpJoint(jidx,jidx) = 20.; //关节的KP反馈增益信息
        this->_data->_legController->commands[leg].kdJoint(jidx,jidx) = 2.;  //关节的Kd反馈增益信息
      }
    }
    return true;
  }
  return false;
}


/**
 * 功能：设置计算机命令
*/
template <typename T>
void FSM_State_FrontJump<T>::ComputeCommand() 
{
  if (_b_first_visit)                       //第一次访问，记录当系统时间
  {
    front_jump_ctrl_->FirstVisit(_curr_time);
    _b_first_visit = false;
  }
  if(this->_data->controlParameters->use_rc)//查看有使用手柄
  {
    if(this->_data->_desiredStateCommand->rcCommand->mode == RC_mode::BACKFLIP_PRE)      //手柄设置后空翻模式
    {
      front_jump_ctrl_->OneStep(_curr_time, true, this->_data->_legController->commands);
    }
    else                                                                                 //手柄没有设置后空翻模式
    {
      front_jump_ctrl_->OneStep(_curr_time, false, this->_data->_legController->commands);
    }
  }
  else                                      //查看没有使用手柄
  {
    front_jump_ctrl_->OneStep(_curr_time, false, this->_data->_legController->commands);
  }

  if (front_jump_ctrl_->EndOfPhase(this->_data->_legController->datas))
  {
    front_jump_ctrl_->LastVisit();
  }
}



/**
 * 功能：设置安全命令
*/
template <typename T>
void FSM_State_FrontJump<T>::_SafeCommand() 
{
  for (int leg = 0; leg < 4; ++leg) //仅仅设置每条腿的位置、速度、力矩信息
  {
    for (int jidx = 0; jidx < 3; ++jidx) 
    {
      this->_data->_legController->commands[leg].tauFeedForward[jidx] = 0.;
      this->_data->_legController->commands[leg].qDes[jidx] = this->_data->_legController->datas[leg].q[jidx];
      this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
    }
  }
}


/**
 * 功能：计算并设置关节位置，执行控制
 * 步骤：
 * （1）如果我们完成插值 
 * （2）计算设定值
 * （3）执行控制 
*/
template <typename T>
void FSM_State_FrontJump<T>::_SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, int leg, 
    const Vec3<T> & ini, const Vec3<T> & fin)
{
    float a(0.f);
    float b(1.f);
    if(curr_iter <= max_iter) //如果我们完成插值
    {
      b = (float)curr_iter/(float)max_iter;
      a = 1.f - b;
    }
    Vec3<T> inter_pos = a * ini + b * fin;//计算设定值
    this->jointPDControl(leg, inter_pos, zero_vec3);//执行控制
}




/*
 * 功能：管理用户可以转换为哪种状态命令或状态事件触发器。
 * @return 要转换为的枚举FSM状态名
 * 步骤：
 * （1）把这一次的状态名称赋值给下一次的状态名称
 * （2）切换FSM控制模式 
 * （3）获取下一个状态 
 */
template <typename T>
FSM_StateName FSM_State_FrontJump<T>::checkTransition() 
{
  this->nextStateName = this->stateName;//把这一次的状态名称赋值给下一次的状态名称
  iter++;
  switch ((int)this->_data->controlParameters->control_mode)//切换FSM控制模式
   {
    case K_FRONTJUMP:
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_PASSIVE: 
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case K_BALANCE_STAND: 
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_FRONTJUMP << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
  return this->nextStateName;//获取下一个状态
}




/**
 *处理robot在状态之间的实际转换。
 * @return 如果转换完成，则为true
 */
template <typename T>
TransitionData<T> FSM_State_FrontJump<T>::transition() 
{
  switch (this->nextStateName)        //选择下一个状态
  {
    case FSM_StateName::PASSIVE:  
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }
  return this->transitionData;//将转换数据返回到FSM
}



/**
 * 功能：在退出状态时清除状态信息
 * 退出状态时要执行的行为
 */
template <typename T>
void FSM_State_FrontJump<T>::onExit() //在退出状态时清除状态信息。
{
  //没什么要清理的
}

template class FSM_State_FrontJump<float>;
