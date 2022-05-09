#ifndef Cheetah_FIXED_BODY_CONTACT
#define Cheetah_FIXED_BODY_CONTACT

#include <WBC/ContactSpec.hpp>

template <typename T>
class FixedBodyContact : public ContactSpec<T> 
{
 public:
  FixedBodyContact();         //适应躯干接触构造函数
  virtual ~FixedBodyContact();

 protected:
  virtual bool _UpdateJc();               //（1）更新Jc雅可比矩阵函数
  virtual bool _UpdateJcDotQdot();        //（2）更新JcDotQdot函数
  virtual bool _UpdateUf();               //（3）更新质量向量函数
  virtual bool _UpdateInequalityVector(); //（4）更新质量向量函数
};

#endif
