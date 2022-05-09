#include "WBIC.hpp"
#include <Utilities/Timer.h>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>


/**
 * 功能：WBIC， 直接力控
 */
  template <typename T>
WBIC<T>::WBIC(size_t num_qdot, const std::vector<ContactSpec<T>*>* contact_list,
    const std::vector<Task<T>*>* task_list)
  : WBC<T>(num_qdot), _dim_floating(6) 
  {
    _contact_list = contact_list;
    _task_list = task_list;

    _eye = DMat<T>::Identity(WB::num_qdot_, WB::num_qdot_);
    _eye_floating = DMat<T>::Identity(_dim_floating, _dim_floating);
  }


/**
 * 功能：输入反作用力和WBC的数据，计算关节扭矩
 */
template <typename T>
void WBIC<T>::MakeTorque(DVec<T>& cmd, void* extra_input) 
{
  //（1）若还没有准备好WBC的输入参数，报错
  if (!WB::b_updatesetting_) 
  {
    printf("[Wanning] WBIC setting is not done\n");
  }

  //（2）若有输入WBC的数据就把它搬到中间变量中，进行后续运算
  //求WBIC论文 式（16）之后的
  if (extra_input)
  {
    _data = static_cast<WBIC_ExtraData<T>*>(extra_input);
  } 

  //(3)设置整理优化的维度，有利于计算机计算，包括整理一下几个量G, g0, CE, ce0, CI, ci0,具体的整理方法看公式，也就是公式的表达形式转换一下
  _SetOptimizationSize();
  
  //(4)设置成本函数，即设置G矩阵
  _SetCost();

 //（5）首要任务是接触腿力控 输出支撑腿加速度
  DVec<T> qddot_pre;
  DMat<T> JcBar;
  DMat<T> Npre;
  if (_dim_rf > 0)
  {
    //建立接触设置
    _ContactBuilding();

    //设置独立的不等式约束
    _SetInEqualityConstraint();

    //计算满秩fat矩阵
    WB::_WeightedInverse(_Jc, WB::Ainv_, JcBar); //求动态一致伪逆 WBC-式(23)
    qddot_pre = JcBar * (-_JcDotQdot);           //式21-2
    Npre = _eye - JcBar * _Jc;                   //式20-1 N0
  } 
  else          
  {
    //摆动腿投影到支撑腿零空间运行
    qddot_pre = DVec<T>::Zero(WB::num_qdot_);
    Npre = _eye;
  }

  //（6）次要任务是摆动腿力控，输出摆动腿的加速度
  Task<T>* task;
  DMat<T> Jt, JtBar, JtPre;
  DVec<T> JtDotQdot, xddot;
  //执行零空间矩阵法的迭代任务
  for (size_t i(0); i < (*_task_list).size(); ++i) 
  {
    task = (*_task_list)[i];                  //把任务列表赋值过来
    task->getTaskJacobian(Jt);                //获取任务雅可比矩阵
    task->getTaskJacobianDotQdot(JtDotQdot);  //获取任务雅可比矩阵
    task->getCommand(xddot);                  //获取命令

    JtPre = Jt * Npre;                              //式19-1 J1|pre
    WB::_WeightedInverse(JtPre, WB::Ainv_, JtBar);  //式23 J1|pre^-1

    qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);//式18 qddot_pre i=1
    Npre = Npre * (_eye - JtBar * JtPre);                      //式19-2+式20-2 N1
  }


 //（7）设置相等约束
  _SetEqualityConstraint(qddot_pre);  

  //（8）进行QP优化，获取优化结果，得到关节扭矩
  // Timer timer;
  T f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
  (void)f;

  for (size_t i(0); i < _dim_floating; ++i) qddot_pre[i] += z[i];
  _GetSolution(qddot_pre, cmd);

  _data->_opt_result = DVec<T>(_dim_opt);
  for (size_t i(0); i < _dim_opt; ++i) 
  {
    _data->_opt_result[i] = z[i];         //获取优化结果，得到关节扭矩
  }
}


/**
 * 功能：设置相等约束 （动力学），重载函数
 */
template <typename T>
void WBIC<T>::_SetEqualityConstraint(const DVec<T>& qddot) 
{
  if (_dim_rf > 0) 
  {
    _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
      WB::A_.block(0, 0, _dim_floating, _dim_floating);
    _dyn_CE.block(0, _dim_floating, _dim_eq_cstr, _dim_rf) =
      -WB::Sv_ * _Jc.transpose();
    _dyn_ce0 = -WB::Sv_ * (WB::A_ * qddot + WB::cori_ + WB::grav_ -
        _Jc.transpose() * _Fr_des);
  } 
  else 
  {
    _dyn_CE.block(0, 0, _dim_eq_cstr, _dim_floating) =
      WB::A_.block(0, 0, _dim_floating, _dim_floating);
    _dyn_ce0 = -WB::Sv_ * (WB::A_ * qddot + WB::cori_ + WB::grav_);
  }

  for (size_t i(0); i < _dim_eq_cstr; ++i) 
  {
    for (size_t j(0); j < _dim_opt; ++j) 
    {
      CE[j][i] = _dyn_CE(i, j);
    }
    ce0[i] = -_dyn_ce0[i];
  }

}



/**
 * 功能：设置相等约束 （动力学），重载函数
 */
template <typename T>
void WBIC<T>::_SetInEqualityConstraint() 
{
  _dyn_CI.block(0, _dim_floating, _dim_Uf, _dim_rf) = _Uf;
  _dyn_ci0 = _Uf_ieq_vec - _Uf * _Fr_des;

  for (size_t i(0); i < _dim_Uf; ++i) 
  {
    for (size_t j(0); j < _dim_opt; ++j) 
    {
      CI[j][i] = _dyn_CI(i, j);
    }
    ci0[i] = -_dyn_ci0[i];
  }
}


/**
 * 功能：建立接触
 */
template <typename T>
void WBIC<T>::_ContactBuilding() 
{
  DMat<T> Uf;
  DVec<T> Uf_ieq_vec;
  //初始化
  DMat<T> Jc;
  DVec<T> JcDotQdot;
  size_t dim_accumul_rf, dim_accumul_uf;
  (*_contact_list)[0]->getContactJacobian(Jc);
  (*_contact_list)[0]->getJcDotQdot(JcDotQdot);
  (*_contact_list)[0]->getRFConstraintMtx(Uf);
  (*_contact_list)[0]->getRFConstraintVec(Uf_ieq_vec);

  dim_accumul_rf = (*_contact_list)[0]->getDim();
  dim_accumul_uf = (*_contact_list)[0]->getDimRFConstraint();

  _Jc.block(0, 0, dim_accumul_rf, WB::num_qdot_) = Jc;
  _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
  _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
  _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;
  _Fr_des.head(dim_accumul_rf) = (*_contact_list)[0]->getRFDesired();

  size_t dim_new_rf, dim_new_uf;

  for (size_t i(1); i < (*_contact_list).size(); ++i) 
  {
    (*_contact_list)[i]->getContactJacobian(Jc);
    (*_contact_list)[i]->getJcDotQdot(JcDotQdot);

    dim_new_rf = (*_contact_list)[i]->getDim();
    dim_new_uf = (*_contact_list)[i]->getDimRFConstraint();

    //雅可比矩阵扩展 
    _Jc.block(dim_accumul_rf, 0, dim_new_rf, WB::num_qdot_) = Jc;

    // JcDotQdot扩展
    _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

    // Uf
    (*_contact_list)[i]->getRFConstraintMtx(Uf);
    _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

    // Uf inequality vector
    (*_contact_list)[i]->getRFConstraintVec(Uf_ieq_vec);
    _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

    // Fr desired
    _Fr_des.segment(dim_accumul_rf, dim_new_rf) =
      (*_contact_list)[i]->getRFDesired();
    dim_accumul_rf += dim_new_rf;
    dim_accumul_uf += dim_new_uf;
  }
}


/**
 * 功能：获取反馈力矩
 */
template <typename T>
void WBIC<T>::_GetSolution(const DVec<T>& qddot, DVec<T>& cmd) 
{
  DVec<T> tot_tau;
  if (_dim_rf > 0) 
  {
    _data->_Fr = DVec<T>(_dim_rf);
    //获取反馈力矩
    for (size_t i(0); i < _dim_rf; ++i)
      _data->_Fr[i] = z[i + _dim_floating] + _Fr_des[i];
    tot_tau =
      WB::A_ * qddot + WB::cori_ + WB::grav_ - _Jc.transpose() * _data->_Fr;
  } 
  else 
  {
    tot_tau = WB::A_ * qddot + WB::cori_ + WB::grav_;
  }
  _data->_qddot = qddot;
  cmd = tot_tau.tail(WB::num_act_joint_);
}


/**
 * 功能：设置成本函数G
 */
template <typename T>
void WBIC<T>::_SetCost() 
{
  size_t idx_offset(0);         //定义一个偏移量，用于搬运数据的而已

  for (size_t i(0); i < _dim_floating; ++i) //遍历每个浮基维度
  {
    G[i + idx_offset][i + idx_offset] = _data->_W_floating[i];  //传入世界坐标系下的浮基模型
  }
  idx_offset += _dim_floating;
  for (size_t i(0); i < _dim_rf; ++i)       //遍历每个脚底到躯干的向量
  {
    G[i + idx_offset][i + idx_offset] = _data->_W_rf[i];        //传入世界坐标系下的脚底到躯干的向量
  }
}


/**
 * 功能：更新设置
 */
template <typename T>
void WBIC<T>::UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
    const DVec<T>& cori, const DVec<T>& grav,
    void* extra_setting) 
{
  WB::A_ = A;
  WB::Ainv_ = Ainv;
  WB::cori_ = cori;
  WB::grav_ = grav;
  WB::b_updatesetting_ = true;

  (void)extra_setting;
}


/**
 * 功能：设置整理优化的维度，包括整理一下几个量G, g0, CE, ce0, CI, ci0
 */
template <typename T>
void WBIC<T>::_SetOptimizationSize() 
{
  //（1）获取每条接触腿、力不等式约束的维度尺寸，并整理成机器人的状态变量维度
  _dim_rf = 0;  //定义每条接触腿3个维度
  _dim_Uf = 0;  //定义力不等式约束维数6个维度
  for (size_t i(0); i < (*_contact_list).size(); ++i) 
  {
    _dim_rf += (*_contact_list)[i]->getDim();               //获取每条接触腿3个维度
    _dim_Uf += (*_contact_list)[i]->getDimRFConstraint();   //获取力不等式约束维数6个维度
  }
  _dim_opt = _dim_floating + _dim_rf;             //整理组成的状态变量维度 机身6个+每条接触腿3个
  _dim_eq_cstr = _dim_floating;                   //等式约束维度 6


  //矩阵设置
  G.resize(0., _dim_opt, _dim_opt);         //整理广义引力（G）矩阵形式，(6+3n)*(6+3n)
  g0.resize(0., _dim_opt);                  //整理g0矩阵形式，          (6+3n)
  CE.resize(0., _dim_opt, _dim_eq_cstr);    //整理CE，                 (6+3n)*6
  ce0.resize(0., _dim_eq_cstr);             //整理ce0矩阵形式，         6

  // Eigen 矩阵设置
  _dyn_CE = DMat<T>::Zero(_dim_eq_cstr, _dim_opt);  //6*(6+3n)
  _dyn_ce0 = DVec<T>(_dim_eq_cstr);                 //6
  if (_dim_rf > 0) 
  {
    CI.resize(0., _dim_opt, _dim_Uf);              //(6+3n)*6n
    ci0.resize(0., _dim_Uf);                       //6n
    _dyn_CI = DMat<T>::Zero(_dim_Uf, _dim_opt);
    _dyn_ci0 = DVec<T>(_dim_Uf);

    _Jc = DMat<T>(_dim_rf, WB::num_qdot_);        //3n*18
    _JcDotQdot = DVec<T>(_dim_rf);                //3n
    _Fr_des = DVec<T>(_dim_rf);                   //3n

    _Uf = DMat<T>(_dim_Uf, _dim_rf);              //6n*3n
    _Uf.setZero();
    _Uf_ieq_vec = DVec<T>(_dim_Uf);               //6n*1
  } 
  else 
  {
    CI.resize(0., _dim_opt, 1);
    ci0.resize(0., 1);
  }
}

template class WBIC<double>;
template class WBIC<float>;
