#ifndef WHOLE_BODY_CONTROLLER
#define WHOLE_BODY_CONTROLLER

#include <Utilities/Utilities_print.h>
#include <Utilities/pseudoInverse.h>
#include <cppTypes.h>
#include <vector>
#include "ContactSpec.hpp"
#include "Task.hpp"

//假设前6个（在二维情况下为3个）关节用于表示浮基。

#define WB WBC<T>

template <typename T>
class WBC {
 public:
  WBC(size_t num_qdot) : num_act_joint_(num_qdot - 6), num_qdot_(num_qdot) {
    Sa_ = DMat<T>::Zero(num_act_joint_, num_qdot_);
    Sv_ = DMat<T>::Zero(6, num_qdot_);

    Sa_.block(0, 6, num_act_joint_, num_act_joint_).setIdentity();
    Sv_.block(0, 0, 6, 6).setIdentity();
  }
  virtual ~WBC() {}

  virtual void UpdateSetting(const DMat<T>& A, const DMat<T>& Ainv,
                             const DVec<T>& cori, const DVec<T>& grav,
                             void* extra_setting = NULL) = 0;

  virtual void MakeTorque(DVec<T>& cmd, void* extra_input = NULL) = 0;

 protected:
  //仅限全秩脂肪矩阵
  void _WeightedInverse(const DMat<T>& J, const DMat<T>& Winv, DMat<T>& Jinv,
                        double threshold = 0.0001) {
    DMat<T> lambda(J * Winv * J.transpose());
    DMat<T> lambda_inv;
    pseudoInverse(lambda, threshold, lambda_inv);
    Jinv = Winv * J.transpose() * lambda_inv;
  }

  size_t num_act_joint_;
  size_t num_qdot_;

  DMat<T> Sa_;  // 驱动关节
  DMat<T> Sv_;  // 虚拟关节

  DMat<T> A_;
  DMat<T> Ainv_;
  DVec<T> cori_;
  DVec<T> grav_;

  bool b_updatesetting_;
  bool b_internal_constraint_;
};

#endif
