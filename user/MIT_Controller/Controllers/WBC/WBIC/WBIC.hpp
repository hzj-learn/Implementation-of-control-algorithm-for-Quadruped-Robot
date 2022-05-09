#ifndef WHOLE_BODY_IMPULSE_CONTROL_H
#define WHOLE_BODY_IMPULSE_CONTROL_H

#include <Utilities/Utilities_print.h>
#include <Goldfarb_Optimizer/QuadProg++.hh>
#include <WBC/ContactSpec.hpp>
#include <WBC/Task.hpp>
#include <WBC/WBC.hpp>

template <typename T>
class WBIC_ExtraData 
{
 public:
  //输出
  DVec<T> _opt_result;  
  DVec<T> _qddot;
  DVec<T> _Fr;

  //输入
  DVec<T> _W_floating;    //世界坐标系下，的浮基模型
  DVec<T> _W_rf;          //世界坐标系下，脚底到躯干的向量

  WBIC_ExtraData() {}
  ~WBIC_ExtraData() {}
};



template <typename T>
class WBIC : public WBC<T> 
{
 public:
  WBIC(size_t num_qdot, const std::vector<ContactSpec<T>*>* contact_list,
       const std::vector<Task<T>*>* task_list);
  virtual ~WBIC() {}

  virtual void UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
                             const DVec<T>& cori, const DVec<T>& grav,
                             void* extra_setting = NULL);

  virtual void MakeTorque(DVec<T>& cmd, void* extra_input = NULL);

 private:
  const std::vector<ContactSpec<T>*>* _contact_list;
  const std::vector<Task<T>*>* _task_list;

  void _SetEqualityConstraint(const DVec<T>& qddot);
  void _SetInEqualityConstraint();
  void _ContactBuilding();

  void _GetSolution(const DVec<T>& qddot, DVec<T>& cmd);
  void _SetCost();
  void _SetOptimizationSize();

  size_t _dim_opt;      // 接触delta反馈力矩, 第一任务是delta反馈力矩
  size_t _dim_eq_cstr;  // 等价约束
  size_t _dim_rf;       // 不等价约束
  size_t _dim_Uf;       //力不等式约束

  size_t _dim_floating;

  WBIC_ExtraData<T>* _data;

  GolDIdnani::GVect<double> z;
  // 成本函数
  GolDIdnani::GMatr<double> G;
  GolDIdnani::GVect<double> g0;

  //等价
  GolDIdnani::GMatr<double> CE;
  GolDIdnani::GVect<double> ce0;

  //不等价
  GolDIdnani::GMatr<double> CI;
  GolDIdnani::GVect<double> ci0;

  DMat<T> _dyn_CE;
  DVec<T> _dyn_ce0;
  DMat<T> _dyn_CI;
  DVec<T> _dyn_ci0;

  DMat<T> _eye;
  DMat<T> _eye_floating;

  DMat<T> _S_delta;
  DMat<T> _Uf;
  DVec<T> _Uf_ieq_vec;

  DMat<T> _Jc;
  DVec<T> _JcDotQdot;
  DVec<T> _Fr_des;

  DMat<T> _B;
  DVec<T> _c;
  DVec<T> task_cmd_;
};

#endif
