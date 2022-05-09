#include "SingleContact.hpp"
#include <Utilities/Utilities_print.h>

/**
 * 功能：单一接触构造函数
 */
template <typename T>
SingleContact<T>::SingleContact(const FloatingBaseModel<T>* robot, int pt)
    : ContactSpec<T>(3), _max_Fz(1500.), _contact_pt(pt), _dim_U(6) 
{
  Contact::idx_Fz_ = 2;
  robot_sys_ = robot;
  Contact::Jc_ = DMat<T>(Contact::dim_contact_, cheetah::dim_config);
  Contact::JcDotQdot_ = DVec<T>::Zero(Contact::dim_contact_);
  Contact::Uf_ = DMat<T>::Zero(_dim_U, Contact::dim_contact_);

  T mu(0.4);

  Contact::Uf_(0, 2) = 1.;

  Contact::Uf_(1, 0) = 1.;
  Contact::Uf_(1, 2) = mu;
  Contact::Uf_(2, 0) = -1.;
  Contact::Uf_(2, 2) = mu;

  Contact::Uf_(3, 1) = 1.;
  Contact::Uf_(3, 2) = mu;
  Contact::Uf_(4, 1) = -1.;
  Contact::Uf_(4, 2) = mu;

  // 法向力上限
  Contact::Uf_(5, 2) = -1.;
}


/**
 * 功能：单一接触析构函数
 */
template <typename T>
SingleContact<T>::~SingleContact() {}


/**
 * 功能：更新雅可比矩阵
 */
template <typename T>
bool SingleContact<T>::_UpdateJc() 
{
  Contact::Jc_ = robot_sys_->_Jc[_contact_pt];
  return true;
}


/**
 * 功能：更新JcDotQdot
 */
template <typename T>
bool SingleContact<T>::_UpdateJcDotQdot() 
{
  Contact::JcDotQdot_ = robot_sys_->_Jcdqd[_contact_pt];
  return true;
}


/**
 * 功能：更新uf
 */
template <typename T>
bool SingleContact<T>::_UpdateUf() 
{
  return true;
}

/**
 * 功能：更新重力向量
 */
template <typename T>
bool SingleContact<T>::_UpdateInequalityVector() 
{
  Contact::ieq_vec_ = DVec<T>::Zero(_dim_U);
  Contact::ieq_vec_[5] = -_max_Fz;
  return true;
}

template class SingleContact<double>;
template class SingleContact<float>;
