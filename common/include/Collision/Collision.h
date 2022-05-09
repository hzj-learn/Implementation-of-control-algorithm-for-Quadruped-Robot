/*! @file Collision.h
 *  @brief 碰撞逻辑的虚类
 *要生成子类，需要实现虚函数，
 *接触检测，检查点是否与几何体接触
 */

#ifndef COLLISION_H
#define COLLISION_H

#include "cppTypes.h"

/*!
 * 抽象碰撞类
 */
template <typename T>
class Collision 
{
 public:
  /*!
   *功能：制造新的碰撞
   * @param mu : 摩擦系数
   * @param resti : 恢复系数（回弹/冲击）
   */
  Collision(const T& mu, const T& resti) : _mu(mu), _restitution_coeff(resti) {}
  virtual ~Collision() {}

  /*!
   * 接触检测的虚拟函数
   * @param cp_pos :全局框架中的接触点
   * @param penetration : 对碰撞物体法线方向的穿透尺寸
   * @param cp_frame : 具有垂直于接触面法向轴（z）的局部框架
   */
  virtual bool ContactDetection(const Vec3<T>& cp_pos, T& penetration,
                                Mat3<T>& cp_frame) = 0;

  const T& getFrictionCoeff() { return _mu; }                     //获取摩擦系数
  const T& getRestitutionCoeff() { return _restitution_coeff; }   //获取恢复系数（回弹/冲击）

 protected:
  T _mu, _restitution_coeff;
};

#endif  // COLLISION_H
