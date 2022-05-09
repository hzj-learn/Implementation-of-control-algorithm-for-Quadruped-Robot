#include "FixedBodyContact.hpp"
#include <Dynamics/Quadruped.h>



/**
 * 功能：适应躯干接触构造函数
 */
template <typename T>
FixedBodyContact<T>::FixedBodyContact() : ContactSpec<T>(6) 
{
  Contact::Jc_ = DMat<T>::Zero(Contact::dim_contact_, cheetah::dim_config);

  for (size_t i(0); i < Contact::dim_contact_; ++i) Contact::Jc_(i, i) = 1.;
  Contact::Uf_ = DMat<T>::Zero(1, Contact::dim_contact_);
  Contact::ieq_vec_ = DVec<T>::Zero(1);
}


/**
 * 功能：适应躯干接触析构函数
 */
template <typename T>
FixedBodyContact<T>::~FixedBodyContact() {}



/**
 * 功能：更新Jc雅可比矩阵函数
 */
template <typename T>
bool FixedBodyContact<T>::_UpdateJc() 
{
  return true;
}


/**
 * 功能：更新JcDotQdot函数
 */
template <typename T>
bool FixedBodyContact<T>::_UpdateJcDotQdot() 
{
  Contact::JcDotQdot_ = DVec<T>::Zero(Contact::dim_contact_);
  return true;
}


/**
 * 功能：更新UF函数
 */
template <typename T>
bool FixedBodyContact<T>::_UpdateUf() 
{
  return true;
}


/**
 * 功能：更新质量向量函数
 */
template <typename T>
bool FixedBodyContact<T>::_UpdateInequalityVector() 
{
  return true;
}

template class FixedBodyContact<double>;
template class FixedBodyContact<float>;
