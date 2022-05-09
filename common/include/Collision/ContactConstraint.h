/*!
 * @file 接触限制
 * @brief 接触约束逻辑的虚类
 * 要生成子类，需要实现虚函数，
 * UpdateExternalForces，UpdateQdot
 */

#ifndef CONTACT_CONSTRAINT_H
#define CONTACT_CONSTRAINT_H

#include <iostream>

#include "Collision/Collision.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Utilities/Utilities_print.h"
#include "cppTypes.h"

#define CC ContactConstraint<T>

/*!
 * 浮基模型的接触动力学
 */
template <typename T>
class ContactConstraint 
{
 public:
  /*!
   * 使用给定的浮基模型构造一个接触模型函数
   * @param model : 触点的浮动基本模型
   */
  ContactConstraint(FloatingBaseModel<T>* model)
      : _nContact(0), _nCollision(0) 
  {
    _model = model;
    for (size_t i(0); i < _model->_nGroundContact; ++i) 
    {
      _cp_force_list.push_back(Vec3<T>::Zero());
    }
  }

  virtual ~ContactConstraint() {}

  /*!
   * 添加碰撞对象函数
   * @param collision :碰撞对象
   */
  void AddCollision(Collision<T>* collision) 
  {
    _collision_list.push_back(collision);
    ++_nCollision;
  }

  /*!
   * 向浮动基础模型添加外力以响应碰撞
   * 用于基于弹簧阻尼器的接触约束方法
   * @param K : 弹簧常数
   * @param D : 阻尼常数
   * @param dt : 时间步长（秒）
   */
  virtual void UpdateExternalForces(T K, T D, T dt) = 0;

  /*!
  *调整浮基模型上的q峎u点以响应碰撞
  *用于基于脉冲的接触约束方法
   * @param state : 浮式系统的完整状态
   */
  virtual void UpdateQdot(FBModelState<T>& state) = 0;

  /*!
   *用于可视化
   * @return cp_pos_list：全局框架中的所有接触点位置。
   */
  const vectorAligned<Vec3<T>>& getContactPosList() 
  { return _cp_pos_list; }

  /*!
   * 用于可视化
   * @return cp_力_列表：全局描述的所有线性接触力
   * frame.
   */
  const Vec3<T>& getGCForce(size_t idx) 
  { return _cp_force_list[idx]; }

 protected:
  vectorAligned<Vec2<T>> deflectionRate;
  void _groundContactWithOffset(T K, T D);

  size_t _CheckContact();

  vectorAligned<Vec2<T>> _tangentialDeflections;

  size_t _nContact;
  size_t _nCollision;

  FloatingBaseModel<T>* _model;

  std::vector<Collision<T>*> _collision_list;
  std::vector<size_t> _idx_list;
  std::vector<T> _cp_resti_list;
  std::vector<T> _cp_mu_list;

  std::vector<T> _cp_penetration_list;
  vectorAligned<Vec3<T>> _cp_force_list;       // 适用于w.r.t Global的所有接触点
  vectorAligned<Vec3<T>> _cp_local_force_list;
  vectorAligned<Vec3<T>> _cp_pos_list;
  vectorAligned<Mat3<T>> _cp_frame_list;
};

#endif
