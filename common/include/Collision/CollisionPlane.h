/*!
 * @file CollisionPlane.h
 * @brief 无限平面的碰撞逻辑
 * 最简单的碰撞，用于地板和全局边界框
 */

#ifndef COLLISIONPLANE_H
#define COLLISIONPLANE_H

#include <vector>

#include "Collision/Collision.h"
#include "cppTypes.h"

/*!
 * 类来表示无限碰撞平面（如平地）。
 */
template <typename T>
class CollisionPlane : public Collision<T> 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * 构造一个新的碰撞平面函数
   * @param mu : 输入摩擦系数
   * @param restitution  : 输入回弹率（v+/v-）
   * @param height  : 输入平面的高度
   */
  CollisionPlane(const T& mu, const T& restitution, const T& height)
      : Collision<T>(mu, restitution), _height(height) {}

  virtual ~CollisionPlane() {}

  virtual bool ContactDetection(const Vec3<T>& cp_pos, T& penetration,
                                Mat3<T>& cp_frame);

 private:
  T _height;
};

#endif  // COLLISION_PLANE_H
