#include "WBC_Ctrl.hpp"
#include <Utilities/Utilities_print.h>
#include <Utilities/Timer.h>


/**
 * 功能：WBC_Ctrl构造函数
 */
template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model):
  _full_config(cheetah::num_act_joint + 7),
  _tau_ff(cheetah::num_act_joint),
  _des_jpos(cheetah::num_act_joint),
  _des_jvel(cheetah::num_act_joint),
  _wbcLCM(getLcmUrl(255))
{
  _iter = 0;
  _full_config.setZero();

  _model = model;
  _kin_wbc = new KinWBC<T>(cheetah::dim_config);

  _wbic = new WBIC<T>(cheetah::dim_config, &(_contact_list), &(_task_list));
  _wbic_data = new WBIC_ExtraData<T>();

  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);
  //_wbic_data->_W_floating = DVec<T>::Constant(6, 50.);
  //_wbic_data->_W_floating[5] = 0.1;
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(cheetah::num_leg_joint, 5.);
  _Kd_joint.resize(cheetah::num_leg_joint, 1.5);

  //_Kp_joint_swing.resize(cheetah::num_leg_joint, 10.);
  //_Kd_joint_swing.resize(cheetah::num_leg_joint, 1.5);

  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
}

/**
 * 功能：WBC_Ctrl析构函数
 */
template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl()
{
  delete _kin_wbc;
  delete _wbic;
  delete _wbic_data;

  typename std::vector<Task<T> *>::iterator iter = _task_list.begin();
  while (iter < _task_list.end()) 
  {
    delete (*iter);
    ++iter;
  }
  _task_list.clear();

  typename std::vector<ContactSpec<T> *>::iterator iter2 = _contact_list.begin();
  while (iter2 < _contact_list.end()) 、
  {
    delete (*iter2);
    ++iter2;
  }
  _contact_list.clear();
}


/**
 * 功能：计算WBC
 */
template <typename T>
void WBC_Ctrl<T>::_ComputeWBC() 
{
  // （1）找到WBC的配置参数
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel);

  // （2）更新浮动机体模型（的参数）
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);

  // （3）输入反作用力和WBC的数据，计算关节扭矩
  _wbic->MakeTorque(_tau_ff, _wbic_data);
}


/**
 * 功能：WBC_Ctrl运行函数
 */
template<typename T>
void WBC_Ctrl<T>::run(void* input, ControlFSMData<T> & data)
{
  ++_iter;//迭代计数

  //（1）更新浮动机体模型
  _UpdateModel(data._stateEstimator->getResult(), data._legController->datas);

  // (2)接触状态任务更新
  _ContactTaskUpdate(input, data);

  // (3)WBC计算
  _ComputeWBC();

  // (4)更新腿部指令
  _UpdateLegCMD(data);

  // (5)发布LCM
  _LCM_PublishData();
}


/**
 * 功能：更新腿部命令
 */
template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(ControlFSMData<T> & data)
{
  //（1）把腿部控制器的命令传给中间变量，用于后续传值运算
  LegControllerCommand<T> * cmd = data._legController->commands;     

  //（2）传入每个关节的力矩、角度、角速度、KD/KP增益数据
  for (size_t leg(0); leg < cheetah::num_leg; ++leg) //遍历四条腿
  {
    cmd[leg].zero();  //把每条腿的指令清零

    for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) //遍历每个关节
    {
      cmd[leg].tauFeedForward[jidx] = _tau_ff[cheetah::num_leg_joint * leg + jidx];//传入关节力矩数据
      cmd[leg].qDes[jidx] = _des_jpos[cheetah::num_leg_joint * leg + jidx];        //传入关节角度数据
      cmd[leg].qdDes[jidx] = _des_jvel[cheetah::num_leg_joint * leg + jidx];       //传入关节角速度数据
      cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];                              //传入关节KP增益数据
      cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];                              //传入关节KD增益数据
    }
  }

  //(3)四条腿膝关节数据限幅
  for(size_t leg(0); leg<4; ++leg)
  {
    if(cmd[leg].qDes[2] < 0.3)//膝关节最小期望角度限幅
    {
      cmd[leg].qDes[2] = 0.3;
    }
    if(data._legController->datas[leg].q[2] < 0.3)//膝关节最小力矩限幅
    {
      T knee_pos = data._legController->datas[leg].q[2]; 
      cmd[leg].tauFeedForward[2] = 1./(knee_pos * knee_pos + 0.02);
    }
  }
}


/**
 * 功能：更新WBC的浮动基体模型
 * 输入量是状态估计器数据、腿部控制器数据
 */
template<typename T>
void WBC_Ctrl<T>::_UpdateModel(const StateEstimate<T> & state_est, 
    const LegControllerData<T> * leg_data)
{
///////////////////*（1）把状态估计器的方向、位置、角速度、线速度传给浮动机体模型*/////////////////
  _state.bodyOrientation = state_est.orientation;   //把状态估计器的方向数据传给浮动机体模型的方向
  _state.bodyPosition = state_est.position;         //把状态估计器的位置数据传给浮动机体模型的位置
  for(size_t i(0); i<3; ++i)
  {
    _state.bodyVelocity[i] = state_est.omegaBody[i];//把状态估计器的角速度数据传给浮动机体模型的角速度
    _state.bodyVelocity[i+3] = state_est.vBody[i];  //把状态估计器的线速度数据传给浮动机体模型的线速度

    for(size_t leg(0); leg<4; ++leg)
    {
      _state.q[3*leg + i] = leg_data[leg].q[i];     //把腿部控制器的腿角度传给传给浮动机体模型的腿角度
      _state.qd[3*leg + i] = leg_data[leg].qd[i];   //把腿部控制器的腿角速度传给传给浮动机体模型的腿角速度

      _full_config[3*leg + i + 6] = _state.q[3*leg + i];
    }
  }
///////////////////*（2）用上面设置的数据设置浮基模型*///////////////////////////////////////////
  _model.setState(_state);            

///////////////////*（3）计算雅可比矩阵、质量矩阵（H）、广义引力（G）、广义科里奥利力，并赋值给中间变量做后续调用*///////////////////////////////////////////
  _model.contactJacobians();          //计算速度的接触雅可比矩阵（3xn矩阵）
  _model.massMatrix();                //计算逆动力学公式中的质量矩阵（H）
  _model.generalizedGravityForce();   //计算逆动力学中的广义引力（G）
  _model.generalizedCoriolisForce();  //计算逆动力学中广义科里奥利力


  _A = _model.getMassMatrix();            //把上面计算的质量矩阵（H）赋值给变量_A
  _grav = _model.getGravityForce();       //把上面计算的广义引力（G）赋值给变量_grav
  _coriolis = _model.getCoriolisForce();  //把上面计算的广义科里奥利力赋值给变量_coriolis
  _Ainv = _A.inverse();                   //把上面计算的逆质量矩阵（H）赋值给变量_Ainv
}


template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
