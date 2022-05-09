#include "LocomotionCtrl.hpp"
#include <WBC_Ctrl/ContactSet/SingleContact.hpp>
#include <WBC_Ctrl/TaskSet/BodyOriTask.hpp>
#include <WBC_Ctrl/TaskSet/BodyPosTask.hpp>
//#include <WBC_Ctrl/TaskSet/BodyPostureTask.hpp>
#include <WBC_Ctrl/TaskSet/LinkPosTask.hpp>


/**
 * 功能：运动控制器构造函数
 */
template<typename T>
LocomotionCtrl<T>::LocomotionCtrl(FloatingBaseModel<T> model):
  WBC_Ctrl<T>(model)
{
  _body_pos_task = new BodyPosTask<T>(&(WBCtrl::_model));
  _body_ori_task = new BodyOriTask<T>(&(WBCtrl::_model));


  _foot_contact[0] = new SingleContact<T>(&(WBCtrl::_model), linkID::FR);
  _foot_contact[1] = new SingleContact<T>(&(WBCtrl::_model), linkID::FL);
  _foot_contact[2] = new SingleContact<T>(&(WBCtrl::_model), linkID::HR);
  _foot_contact[3] = new SingleContact<T>(&(WBCtrl::_model), linkID::HL);

  _foot_task[0] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::FR);
  _foot_task[1] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::FL);
  _foot_task[2] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::HR);
  _foot_task[3] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::HL);
}


/**
 * 功能：运动控制器析构函数
 */
template<typename T>
LocomotionCtrl<T>::~LocomotionCtrl()
{
  delete _body_pos_task;
  delete _body_ori_task;

  for(size_t i (0); i<4; ++i){
    delete _foot_contact[i];
    delete _foot_task[i];
  }
}


/**
 * 功能：接触状态任务更新
 */
template<typename T>
void LocomotionCtrl<T>::_ContactTaskUpdate(void* input, ControlFSMData<T> & data)
{
  _input_data = static_cast<LocomotionCtrlData<T>* >(input);//（1）把WBC输入的数据赋值给中间变量做后续调用

  _ParameterSetup(data.userParameters);                     //（2）用WBC的用户数据设置参数：机体的位置、方向的KP/KD、腿部的KP/KD、配置关节的KP/KD
  
  _CleanUp();                                               //（3）清除之前的数据

  _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);   //（4）根据期望机体欧拉角，经过转换设置期望的四元数

//（5）更新机体的方向、位置，并把结果发布
  Vec3<T> zero_vec3; zero_vec3.setZero();
  _body_ori_task->UpdateTask(&_quat_des, _input_data->vBody_Ori_des, zero_vec3);//更新机体的方向任务数据
  _body_pos_task->UpdateTask(                                                   //更新机体的位置任务数据
      &(_input_data->pBody_des), 
      _input_data->vBody_des, 
      _input_data->aBody_des);
  WBCtrl::_task_list.push_back(_body_ori_task);   //发布运行机体的方向任务的结果，这个似乎是一个任务进程
  WBCtrl::_task_list.push_back(_body_pos_task);   //发布运行机体的位置任务的结果


//（6）更新腿部的接触，并把结果发布
  for(size_t leg(0); leg<4; ++leg)
  {
    if(_input_data->contact_state[leg] > 0.)//接触（支撑时）
    { 
      _foot_contact[leg]->setRFDesired((DVec<T>)(_input_data->Fr_des[leg]));//更新腿部的接触任务数据
      _foot_contact[leg]->UpdateContactSpec();
      WBCtrl::_contact_list.push_back(_foot_contact[leg]);                  //发布腿部的接触任务的结果

    }
    else                                    // 非接触（摆动）
    { 
      _foot_task[leg]->UpdateTask(                          //更新腿部的接触任务数据
          &(_input_data->pFoot_des[leg]), 
          _input_data->vFoot_des[leg], 
          _input_data->aFoot_des[leg]);
      WBCtrl::_task_list.push_back(_foot_task[leg]);        //发布腿部的接触任务的结果
    }
  }
}


/**
 * 功能：接触任务更新测试
 */
template<typename T>
void LocomotionCtrl<T>::_ContactTaskUpdateTEST(void* input, ControlFSMData<T> & data)
{
  (void)data;
  _input_data = static_cast<LocomotionCtrlData<T>* >(input);

  for(size_t i(0); i<3; ++i)
  {
    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = 10.;
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = 3.;

    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = 10.;
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = 3.;

    for(size_t j(0); j<4; ++j)
    {
      ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = 70;
      ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = 3.;
    }  
  }
    _CleanUp();  // 清除之前的数据

  _quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);

  Vec3<T> zero_vec3; zero_vec3.setZero();
  _body_ori_task->UpdateTask(&_quat_des, _input_data->vBody_Ori_des, zero_vec3);
  _body_pos_task->UpdateTask(
      &(_input_data->pBody_des), 
      _input_data->vBody_des, 
      _input_data->aBody_des);

  WBCtrl::_task_list.push_back(_body_ori_task);
  WBCtrl::_task_list.push_back(_body_pos_task);

  for(size_t leg(0); leg<4; ++leg)
  {
    if(_input_data->contact_state[leg] > 0.) //接触（支撑时）
    {
      _foot_contact[leg]->setRFDesired((DVec<T>)(_input_data->Fr_des[leg]));
      _foot_contact[leg]->UpdateContactSpec();
      WBCtrl::_contact_list.push_back(_foot_contact[leg]);

    }
    else                                     //非接触，摆动时
    { 
      _foot_task[leg]->UpdateTask(
          &(_input_data->pFoot_des[leg]), 
          _input_data->vFoot_des[leg], 
          _input_data->aFoot_des[leg]);
          //zero_vec3);
      WBCtrl::_task_list.push_back(_foot_task[leg]);
    }
  }
}


/**
 * 功能：参数配置
 * (1)配置机体的位置、方向的KP/KD   
 * (2)配置腿部的KP/KD
 * (3)配置关节的KP/KD
 */
template<typename T>
void LocomotionCtrl<T>::_ParameterSetup(const MIT_UserParameters* param)
{
  for(size_t i(0); i<3; ++i)
  {
    //(1)配置机体的位置、方向的KP/KD   
    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = param->Kp_body[i];
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = param->Kd_body[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = param->Kp_ori[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = param->Kd_ori[i];
    //(2)配置腿部的KP/KD
    for(size_t j(0); j<4; ++j)
    {
      ((LinkPosTask<T>*)_foot_task[j])->_Kp[i] = param->Kp_foot[i];
      ((LinkPosTask<T>*)_foot_task[j])->_Kd[i] = param->Kd_foot[i];
    }
    //(3)配置关节的KP/KD
    WBCtrl::_Kp_joint[i] = param->Kp_joint[i];
    WBCtrl::_Kd_joint[i] = param->Kd_joint[i];
   }
}


/**
 * 功能：清除
 */
template<typename T>
void LocomotionCtrl<T>::_CleanUp()
{
  WBCtrl::_contact_list.clear();
  WBCtrl::_task_list.clear();
}


/**
 * 功能：LCM数据发布
 */
template<typename T>
void LocomotionCtrl<T>::_LCM_PublishData() 
{
  int iter(0);
  for(size_t leg(0); leg<4; ++leg){
    _Fr_result[leg].setZero();
    
    if(_input_data->contact_state[leg]>0.)
    {
      for(size_t i(0); i<3; ++i)
      {
        _Fr_result[leg][i] = WBCtrl::_wbic_data->_Fr[3*iter + i];
      }
      ++iter;
    }

    if(_input_data->contact_state[leg] > 0.)      //接触
    { 
      WBCtrl::_wbc_data_lcm.contact_est[leg] = 1;
    }
    else                                          //非接触
    {
      WBCtrl::_wbc_data_lcm.contact_est[leg] = 0;
    }
  }

  for(size_t i(0); i<3; ++i)
  {
    WBCtrl::_wbc_data_lcm.foot_pos[i] = WBCtrl::_model._pGC[linkID::FR][i];
    WBCtrl::_wbc_data_lcm.foot_vel[i] = WBCtrl::_model._vGC[linkID::FR][i];

    WBCtrl::_wbc_data_lcm.foot_pos[i + 3] = WBCtrl::_model._pGC[linkID::FL][i];
    WBCtrl::_wbc_data_lcm.foot_vel[i + 3] = WBCtrl::_model._vGC[linkID::FL][i];

    WBCtrl::_wbc_data_lcm.foot_pos[i + 6] = WBCtrl::_model._pGC[linkID::HR][i];
    WBCtrl::_wbc_data_lcm.foot_vel[i + 6] = WBCtrl::_model._vGC[linkID::HR][i];

    WBCtrl::_wbc_data_lcm.foot_pos[i + 9] = WBCtrl::_model._pGC[linkID::HL][i];
    WBCtrl::_wbc_data_lcm.foot_vel[i + 9] = WBCtrl::_model._vGC[linkID::HL][i];


    for(size_t leg(0); leg<4; ++leg)
    {
      WBCtrl::_wbc_data_lcm.Fr_des[3*leg + i] = _input_data->Fr_des[leg][i];
      WBCtrl::_wbc_data_lcm.Fr[3*leg + i] = _Fr_result[leg][i];

      WBCtrl::_wbc_data_lcm.foot_pos_cmd[3*leg + i] = _input_data->pFoot_des[leg][i];
      WBCtrl::_wbc_data_lcm.foot_vel_cmd[3*leg + i] = _input_data->vFoot_des[leg][i];
      WBCtrl::_wbc_data_lcm.foot_acc_cmd[3*leg + i] = _input_data->aFoot_des[leg][i];

      WBCtrl::_wbc_data_lcm.jpos_cmd[3*leg + i] = WBCtrl::_des_jpos[3*leg + i];
      WBCtrl::_wbc_data_lcm.jvel_cmd[3*leg + i] = WBCtrl::_des_jvel[3*leg + i];

      WBCtrl::_wbc_data_lcm.jpos[3*leg + i] = WBCtrl::_state.q[3*leg + i];
      WBCtrl::_wbc_data_lcm.jvel[3*leg + i] = WBCtrl::_state.qd[3*leg + i];

    }

    WBCtrl::_wbc_data_lcm.body_pos_cmd[i] = _input_data->pBody_des[i];
    WBCtrl::_wbc_data_lcm.body_vel_cmd[i] = _input_data->vBody_des[i];
    WBCtrl::_wbc_data_lcm.body_ori_cmd[i] = _quat_des[i];

    Quat<T> quat = WBCtrl::_state.bodyOrientation;
    Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
    Vec3<T> global_body_vel = Rot.transpose() * WBCtrl::_state.bodyVelocity.tail(3);

    WBCtrl::_wbc_data_lcm.body_pos[i] = WBCtrl::_state.bodyPosition[i];
    WBCtrl::_wbc_data_lcm.body_vel[i] = global_body_vel[i];
    WBCtrl::_wbc_data_lcm.body_ori[i] = WBCtrl::_state.bodyOrientation[i];
    WBCtrl::_wbc_data_lcm.body_ang_vel[i] = WBCtrl::_state.bodyVelocity[i];
  }
  WBCtrl::_wbc_data_lcm.body_ori_cmd[3] = _quat_des[3];
  WBCtrl::_wbc_data_lcm.body_ori[3] = WBCtrl::_state.bodyOrientation[3];

  WBCtrl::_wbcLCM.publish("wbc_lcm_data", &(WBCtrl::_wbc_data_lcm) );
}

template class LocomotionCtrl<float>;
template class LocomotionCtrl<double>;
