/*============================= Recovery Stand ==============================*/
/*
  *要求机器人站起来的过渡状态
  *平衡控制模式
 */

#include "FSM_State_RecoveryStand.h"
#include <Utilities/Utilities_print.h>


/**
 *功能：向传递状态特定信息的FSM状态的构造函数
 * @param _controlFSMData 定义所有相关的初始化关节位置控制数据
 */
template <typename T>
FSM_State_RecoveryStand<T>::FSM_State_RecoveryStand(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP")
  {
  this->checkSafeOrientation = false;//设置预控安全检查
  this->checkPDesFoot = false;       //位置控制安全检查
  this->checkForceFeedForward = false;
  zero_vec3.setZero();

  /*目标配置*/
 //（1）匍匐状态的关节位置的初始参数
  fold_jpos[0] << -0.0f, -1.4f, 2.7f;
  fold_jpos[1] << 0.0f, -1.4f, 2.7f;
  fold_jpos[2] << -0.0f, -1.4f, 2.7f;
  fold_jpos[3] << 0.0f, -1.4f, 2.7f;

 //（2）站力状态的关节位置的初始参数
  for(size_t i(0); i<4; ++i)
  {
    stand_jpos[i] << 0.f, -.8f, 1.6f;
  }
//（3）滚动状态的关节位置的初始参数
  rolling_jpos[0] << 1.5f, -1.6f, 2.77f;
  rolling_jpos[1] << 1.3f, -3.1f, 2.77f;
  rolling_jpos[2] << 1.5f, -1.6f, 2.77f;
  rolling_jpos[3] << 1.3f, -3.1f, 2.77f;
  f_ff << 0.f, 0.f, -25.f;
}


/**
 *功能：进入状态时要执行的行为
 */
template <typename T>
void FSM_State_RecoveryStand<T>::onEnter() 
{
  this->nextStateName = this->stateName;//默认为不过渡
  this->transitionData.zero();          //重置转换数据
  iter = 0;                             //重置迭代计数器
  _state_iter = 0;
  for(size_t i(0); i < 4; ++i)          //初始配置，位置
  {
    initial_jpos[i] = this->_data->_legController->datas[i].q;
  }
  T body_height =  this->_data->_stateEstimator->getResult().position[2];
  _flag = FoldLegs;

  if( !_UpsideDown() )                 //正确定位
  {
    if (  (0.2 < body_height) && (body_height < 0.45) )
    {
      printf("[Recovery Balance] body height is %f; Stand Up \n", body_height);
      _flag = StandUp;
    }
    else
    {
      printf("[Recovery Balance] body height is %f; Folding legs \n", body_height);
    }
  }
  else                                 //不正确定位
  {
      printf("[Recovery Balance] UpsideDown (%d) \n", _UpsideDown() );
  }
  _motion_start_iter = 0;
}


/**
 *功能：判断是否正确定位
 */
template <typename T>
bool FSM_State_RecoveryStand<T>::_UpsideDown()
{
  if(this->_data->_stateEstimator->getResult().rBody(2,2) < 0)
  {
    return true;
  }
  return false;
}



/**
 *功能：调用要在每个控制循环迭代中执行的函数。
 */
template <typename T>
void FSM_State_RecoveryStand<T>::run() 
{
  switch(_flag)
  {
    case StandUp://（3）从匍匐状态转换成站立状态
      _StandUp(_state_iter - _motion_start_iter);
      break;
    case FoldLegs://（1）折叠腿部
      _FoldLegs(_state_iter - _motion_start_iter);
      break;
    case RollOver://（2）翻身动作
      _RollOver(_state_iter - _motion_start_iter);
      break;
  }
 ++_state_iter;
}



/**
 *功能：计算关节位置，并执行控制
 */
template <typename T>
void FSM_State_RecoveryStand<T>::_SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, int leg, 
    const Vec3<T> & ini, const Vec3<T> & fin)
{
    float a(0.f);
    float b(1.f);
    if(curr_iter <= max_iter)                         //如果我们完成插值
    {
      b = (float)curr_iter/(float)max_iter;
      a = 1.f - b;
    }
    Vec3<T> inter_pos = a * ini + b * fin;            //计算设定值
    this->jointPDControl(leg, inter_pos, zero_vec3);  //执行控制
}




/**
 *功能：滚动翻身过程运动函数
 */
template <typename T>
void FSM_State_RecoveryStand<T>::_RollOver(const int & curr_iter)
{
  for(size_t i(0); i<4; ++i)
  {
    _SetJPosInterPts(curr_iter, rollover_ramp_iter, i, 
        initial_jpos[i], rolling_jpos[i]);
  }

  if(curr_iter > rollover_ramp_iter + rollover_settle_iter)
  {
    _flag = FoldLegs;
    for(size_t i(0); i<4; ++i) initial_jpos[i] = rolling_jpos[i];
    _motion_start_iter = _state_iter+1;
  }
}



/**
 *功能：从匍匐到恢复站立过程运动函数
 */
template <typename T>
void FSM_State_RecoveryStand<T>::_StandUp(const int & curr_iter)
{
  T body_height = this->_data->_stateEstimator->getResult().position[2];
  bool something_wrong(false);

  if( _UpsideDown() || (body_height < 0.1 ) ) 
  { 
    something_wrong = true;
  }

//如果因为某种原因身体高度太低，即使站起来的动作快结束了，当紧急停止处于其他状态中间时可能发生
  if( (curr_iter > floor(standup_ramp_iter*0.7) ) && something_wrong)
  {
    for(size_t i(0); i < 4; ++i) 
    {
      initial_jpos[i] = this->_data->_legController->datas[i].q;
    }
    _flag = FoldLegs;
    _motion_start_iter = _state_iter+1;

    printf("[Recovery Balance - Warning] body height is still too low (%f) or UpsideDown (%d); Folding legs \n", 
        body_height, _UpsideDown() );

  }
  else                                                               //正常情况
  {
    for(size_t leg(0); leg<4; ++leg)
    {
      _SetJPosInterPts(curr_iter, standup_ramp_iter, 
          leg, initial_jpos[leg], stand_jpos[leg]);
    }
  }
  //机器人的前馈质量。
  //for(int i = 0; i < 4; i++)
  //this->_data->_legController->commands[i].forceFeedForward = f_ff;
  Vec4<T> se_contactState(0.5,0.5,0.5,0.5);
  this->_data->_stateEstimator->setContactPhase(se_contactState);
}


/**
 *功能：把四条腿折叠起来
 */
template <typename T>
void FSM_State_RecoveryStand<T>::_FoldLegs(const int & curr_iter)
{
  for(size_t i(0); i<4; ++i)    //设置四个关节位置，计算关节位置，并执行控制
  {
    _SetJPosInterPts(curr_iter, fold_ramp_iter, i, 
        initial_jpos[i], fold_jpos[i]);
  }
  if(curr_iter >= fold_ramp_iter + fold_settle_iter)//如果现在折叠的角度过大
  {
    if(_UpsideDown())//向上倒
    {
      _flag = RollOver;
      for(size_t i(0); i<4; ++i) initial_jpos[i] = fold_jpos[i];
    }
    else             //向下倒
    {
      _flag = StandUp;
      for(size_t i(0); i<4; ++i) initial_jpos[i] = fold_jpos[i];
    }
    _motion_start_iter = _state_iter + 1;
  }
}




/**
 * 功能：状态转移检查：管理哪些状态可以由用户命令或状态事件触发器切换
 * @return 要转换为的枚举FSM状态名
 */
template <typename T>
FSM_StateName FSM_State_RecoveryStand<T>::checkTransition() 
{
  this->nextStateName = this->stateName;    //把这一次的状态名称赋值给下一次的状态名称
  iter++;
  switch ((int)this->_data->controlParameters->control_mode)//切换FSM控制模式
  {
    case K_RECOVERY_STAND:
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

    case K_BACKFLIP: 
      this->nextStateName = FSM_StateName::BACKFLIP;
      break;

    case K_FRONTJUMP: 
      this->nextStateName = FSM_StateName::FRONTJUMP;
      break;

    case K_VISION: 
      this->nextStateName = FSM_StateName::VISION;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_RECOVERY_STAND << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }
  return this->nextStateName;//获取下一个状态
}


/**
 *功能：状态转移， 处理机器人在状态之间的实际转换
 * @return 如果转换完成，则为true
 */
template <typename T>
TransitionData<T> FSM_State_RecoveryStand<T>::transition() 
{
//完成转换
  switch (this->nextStateName)
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

    case FSM_StateName::BACKFLIP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::FRONTJUMP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }
  return this->transitionData;//将转换数据返回到FSM
}



/**
 * 功能：在退出状态时清除状态信息。
 */
template <typename T>
void FSM_State_RecoveryStand<T>::onExit() 
{
  //退出时无需清理
}

// template class FSM_State_RecoveryStand<double>;
template class FSM_State_RecoveryStand<float>;
