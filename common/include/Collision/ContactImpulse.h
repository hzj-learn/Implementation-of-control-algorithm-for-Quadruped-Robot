/*!
 * @file ContactImpulse.h
 * @brief 基于脉冲的接触动力学实现
 */

#ifndef CONTACT_IMPULSE_H
#define CONTACT_IMPULSE_H

#include "ContactConstraint.h"
#include "Dynamics/FloatingBaseModel.h"

/*!
 * 基于脉冲的浮动基模型接触动力学
 */
template <typename T>
class ContactImpulse : public ContactConstraint<T> 
{
 public:

  /*!
   * 功能：使用给定的浮基模型构造一个新的接触动力学模型
   * @param model : 触头用浮动底座模型
   */
  ContactImpulse(FloatingBaseModel<T>* model) : ContactConstraint<T>(model) 
  {
    _iter_lim = 10;
    _nDof = CC::_model->_nDof;
    _b_debug = false;
  }

  virtual ~ContactImpulse() {}

  /*!
  *在地面接触点列表中运行单个碰撞平面的地面接触模型。允许地面在切线方向而不是法向上变形。相反，脚会穿透地面
  *地面还“记得”它在不同接触事件之间的变形。（但是它会很快反弹）
   * @param K 地面刚度
   * @param D 地面阻尼
   * @param dt 时间步长（用于偏转）
   */
  virtual void UpdateExternalForces(T K, T D, T dt) 
  {
    (void)K;
    (void)D;
    //此处设置穿透恢复率
    _penetration_recover_ratio = 0.0 / dt;
    _dt = dt;
  }

  virtual void UpdateQdot(FBModelState<T>& state);

 protected:
  size_t _iter_lim;
  bool _b_debug;
  T _tol = 1.e-6;
  T _penetration_recover_ratio;
  size_t _nDof;
  void _UpdateVelocity(DVec<T>& qdot);
  void _UpdateQdotOneDirection(size_t idx,
                               const vectorAligned<D3Mat<T> >& Jc_list,
                               const T* lambda_list,
                               const vectorAligned<DVec<T> > AinvB_list,
                               const T* des_vel_list, const T* min_list,
                               const T* max_list, DVec<T>& qdot);
  size_t _iter_sum;

 private:
  T _dt;
};

#endif
