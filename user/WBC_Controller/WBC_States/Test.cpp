#include "Test.hpp"
#include <Utilities/Utilities_print.h>
#include <ParamHandler/ParamHandler.hpp>


/**
 * 功能：测试的构造函数
 */
template <typename T>
Test<T>::Test(FloatingBaseModel<T>* robot, const RobotType& robot_type)
    : _b_first_visit(true),
      _b_save_file(false),
      _robot(robot),
      _count(0),
      _waiting_count(5),
      _b_running(true) 
{
  _robot_type = robot_type;
  _sp = StateProvider<T>::getStateProvider();
  _ParameterSetting();
  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
}


/**
 * 功能：测试的析构函数
 */
template <typename T>
Test<T>::~Test() {}


/**
 * 功能：获取命令函数
 */
template <typename T>
void Test<T>::GetCommand(const Cheetah_Data<T>* data,
                         LegControllerCommand<T>* command,
                         Cheetah_Extra_Data<T>* ext_data) 
{
  /*（1）手柄命令更新*/
  _sp->_mode = data->mode;                      //模式
  _sp->_dir_command[0] = data->dir_command[0];  //左右移动命令
  _sp->_dir_command[1] = data->dir_command[1];  //前后一定命令

  for (size_t i(0); i < 3; ++i) _sp->_ori_command[i] = data->ori_command[i];//方向欧拉角指令

  /*（2）每个关节的角度和速度更新*/
  for (size_t i(0); i < cheetah::num_act_joint; ++i) 
  {
    _state.q[i] = data->jpos[i];        //角度
    _state.qd[i] = data->jvel[i];       //角速度
    _sp->_Q[i + 6] = data->jpos[i];     //角度
    _sp->_Qdot[i + 6] = data->jvel[i];  //角速度
  }

  /*（3）躯干的位置和速度初始化*/
  _state.bodyPosition.setZero();
  _state.bodyVelocity.setZero();

  /*（4）躯干方向更新*/
  for (size_t i(0); i < 4; ++i) 
  {
    _state.bodyOrientation[i] = data->body_ori[i];
  }
  _sp->_Q[cheetah::dim_config] = _state.bodyOrientation[0];
  _sp->_Q[0] = _state.bodyOrientation[1];
  _sp->_Q[1] = _state.bodyOrientation[2];
  _sp->_Q[2] = _state.bodyOrientation[3];
  _body_rpy = ori::quatToRPY(_state.bodyOrientation);//把四元数转换成为欧拉角

  /*（5）躯干速度更新*/ 
  _state.bodyVelocity[0] = data->ang_vel[0];
  _state.bodyVelocity[1] = data->ang_vel[1];
  _state.bodyVelocity[2] = data->ang_vel[2];

  /*（6）躯干位置更新*/
  if (data->cheater_mode) 
  {
    _state.bodyPosition[0] = data->global_body_pos[0];
    _state.bodyPosition[1] = data->global_body_pos[1];
    _state.bodyPosition[2] = data->global_body_pos[2];
    //重新更新新的机器人模型
    _robot->setState(_state);
    _robot->forwardKinematics();
  } 
  else 
  {
   //重新更新新的机器人模型
    _robot->setState(_state);
    _robot->forwardKinematics();
   //与接触点的偏移
    Vec3<T> ave_foot;
    Vec3<T> ave_foot_vel;
    //设置足端位置、速度都为0
    ave_foot.setZero();     
    ave_foot_vel.setZero(); 
    //计算足端的位置和速度
    for (size_t i(0); i < _sp->_num_contact; ++i) 
    {
      ave_foot += (1. / _sp->_num_contact) * _robot->_pGC[_sp->_contact_pt[i]];
      ave_foot_vel +=
          (1. / _sp->_num_contact) * _robot->_vGC[_sp->_contact_pt[i]];
    }
    //计算躯干的位置
    _state.bodyPosition = -ave_foot;
    _state.bodyPosition += _sp->_local_frame_global_pos;

    Quat<T> quat = _state.bodyOrientation;                  //获取躯干的四元数
    Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);    //根据四元数转换成旋转矩阵
    //计算
    _state.bodyVelocity.tail(3) = -Rot * ave_foot_vel;
  
   //重新更新新的机器人模型
    _robot->setState(_state);
    _robot->forwardKinematics();
  }
   //StateProvider位置变量更新
  _sp->_Q[3] = _state.bodyPosition[0];
  _sp->_Q[4] = _state.bodyPosition[1];
  _sp->_Q[5] = _state.bodyPosition[2];


  /*（7）更新接触雅可比矩阵、质量矩阵（H），广义引力（G），科里奥利力*/
  _robot->contactJacobians();             //计算速度的接触雅可比矩阵（3xn矩阵）
  _robot->massMatrix();                   //计算逆动力学公式中的质量矩阵（H）
  _robot->generalizedGravityForce();      //计算逆动力学中的广义引力（G）
  _robot->generalizedCoriolisForce();     //计算逆动力学中广义科里奥利力


  /*（8）躯干的速度更新*/
  for (size_t i(0); i < 6; ++i) {
    _sp->_Qdot[i] = _state.bodyVelocity[i];
  }

  /*（9）躯干方位检查（是否翻转）*/
  //_SafetyCheck(); // dont use this for backflip test

  /*（10）命令计算*/
  if (_b_running) 
  {
    if (!_Initialization(data, command)) //若没有进行数据及命令初始化
    {
      _UpdateTestOneStep();
      ComputeCommand(command);
    }
  } 
  else 
  {
    _SafeCommand(data, command);
  }
  (void)ext_data;
  //_copy_cmd = command;
  _UpdateExtraData(ext_data);

  ++_count;
  _sp->_curr_time += cheetah::servo_rate;
}


/**
 * 功能：安全检查函数
 */
template <typename T>
void Test<T>::_SafetyCheck() 
{
  // pretty_print(_body_rpy, std::cout, "body rpy");
  if (fabs(_body_rpy[0]) > _roll_limit) 
  {
    _b_running = false;
  }
  if (fabs(_body_rpy[1]) > _pitch_limit) 
  {
    _b_running = false;
  }
}



/**
 * 功能：设置命令函数
 */
template <typename T>
void Test<T>::_SafeCommand(const Cheetah_Data<T>* data,
                           LegControllerCommand<T>* command) 
{
  for (size_t leg(0); leg < cheetah::num_leg; ++leg) 
  {
    for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) 
    {
      command[leg].tauFeedForward[jidx] = 0.;
      command[leg].qDes[jidx] = data->jpos[3 * leg + jidx];
      command[leg].qdDes[jidx] = 0.;
    }
  }
}



/**
 * 功能：初始化函数
 */
template <typename T>
bool Test<T>::_Initialization(const Cheetah_Data<T>* data,
                              LegControllerCommand<T>* command) 
{
  static bool test_initialized(false);
  if (!test_initialized) 
  {
    _TestInitialization();
    test_initialized = true;
    printf("[Cheetah Test] Test initialization is done\n");
  }
  if (_count < _waiting_count) 
  {
    for (size_t leg(0); leg < cheetah::num_leg; ++leg) 
    {
      for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) 
      {
        command[leg].tauFeedForward[jidx] = 0.;
        command[leg].qDes[jidx] = data->jpos[3 * leg + jidx];
        command[leg].qdDes[jidx] = 0.;
      }
    }
    return true;
  }
  return false;
}



/**
 * 功能：参数设置函数
 */
template <typename T>
void Test<T>::_ParameterSetting() 
{
  ParamHandler handler(CheetahConfigPath "INTERFACE_setup.yaml");
  handler.getValue<T>("roll_limit", _roll_limit);
  handler.getValue<T>("pitch_limit", _pitch_limit);
}



/**
 * 功能：计算指令函数
 */
template <typename T>
void Test<T>::ComputeCommand(void* command) 
{
  if (_b_first_visit) 
  {
    _state_list[_phase]->FirstVisit();
    _b_first_visit = false;
  }

  _state_list[_phase]->OneStep(command);

  if (_state_list[_phase]->EndOfPhase()) 
  {
    _state_list[_phase]->LastVisit();
    _phase = _NextPhase(_phase);
    _b_first_visit = true;
  }
}

template class Test<double>;
template class Test<float>;
