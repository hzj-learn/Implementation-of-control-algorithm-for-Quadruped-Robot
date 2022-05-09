#ifndef Cheetah_SINGLE_CONTACT
#define Cheetah_SINGLE_CONTACT

#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/Quadruped.h>
#include <WBC/ContactSpec.hpp>

template <typename T>
class SingleContact : public ContactSpec<T> 
{
 public:
  SingleContact(const FloatingBaseModel<T>* robot, int contact_pt);
  virtual ~SingleContact();

  void setMaxFz(T max_fz) { _max_Fz = max_fz; }

 protected:
  T _max_Fz;
  int _contact_pt;
  int _dim_U;

  virtual bool _UpdateJc();               //（1）更新Jc雅可比矩阵函数
  virtual bool _UpdateJcDotQdot();        //（2）更新JcDotQdot函数
  virtual bool _UpdateUf();               //（3）更新质量向量函数
  virtual bool _UpdateInequalityVector(); //（4）更新质量向量函数

  const FloatingBaseModel<T>* robot_sys_;
};

#endif
