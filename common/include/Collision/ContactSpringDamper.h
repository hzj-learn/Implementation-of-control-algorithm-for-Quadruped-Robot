/*!
 * @file ContactSpringDamper.h
 * @brief 基于弹簧阻尼器的接触动力学实现
 */

#ifndef CONTACT_SPRING_DAMPER_H
#define CONTACT_SPRING_DAMPER_H

#include "ContactConstraint.h"
#include "Dynamics/FloatingBaseModel.h"

/*!
 * 表示FloatingBaseModel的接触动力学的类
 * 这将计算施加与地面接触的物体的外力
 */
template <typename T>
class ContactSpringDamper : public ContactConstraint<T> 
{
 public:
  /*!
  *建立一种新的接触动力学模型
   * @param model : 接触动力学的浮动基模型
   */
  ContactSpringDamper(FloatingBaseModel<T>* model)
      : ContactConstraint<T>(model) 
  {
    _nGC = CC::_model->_nGroundContact;
    for (size_t i(0); i < _nGC; ++i) 
    {
      _tangentialDeflections.push_back(Vec2<T>::Zero());
    }
    deflectionRate.resize(_nGC);
  }

  virtual ~ContactSpringDamper() {}


  /*!
  *在地面接触点列表中运行单个碰撞平面的地面接触模型。允许地面在切线方向而不是法向上变形。相反，脚会穿透地面
  *地面还“记得”它在不同接触事件之间的变形。（但是它会很快反弹）
   * @param K 地面刚度
   * @param D 地面阻尼
   * @param dt 时间步长（用于偏转）
   */
  virtual void UpdateExternalForces(T K, T D, T dt);

  virtual void UpdateQdot(FBModelState<T>& state) 
  {
    (void)state; /* Do nothing */
  }

 protected:
  size_t _nGC;

  void _groundContactWithOffset(T K, T D);

  vectorAligned<Vec2<T>> deflectionRate;
  vectorAligned<Vec2<T>> _tangentialDeflections;
};

#endif
