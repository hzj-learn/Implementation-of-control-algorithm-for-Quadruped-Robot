/*!
 * @file CollisionBox.h
 * @brief 长方体的碰撞逻辑
 */

#ifndef COLLISION_BOX_H
#define COLLISION_BOX_H

#include <vector>

#include "Collision/Collision.h"
#include "Utilities/utilities.h"
#include "cppTypes.h"

/*!
 * 类来表示长方体碰撞
 */
template <typename T>
class CollisionBox : public Collision<T> 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * 功能：构造一个新的碰撞长方体
   * @param mu : 摩擦系数
   * @param restitution  : 回弹率（v+/v-）
   * @param depth : 盒子的深度
   * @param width : 盒子的宽度
   * @param height : 盒子的高度
   * @param position :框在全局框架中的位置
   * @param ori : 长方体在全局框架中的方向
   */
  CollisionBox(const T& mu, const T& restitution, const T& depth,
               const T& width, const T& height, const Vec3<T>& position,
               const Mat3<T>& ori)
      : Collision<T>(mu, restitution), _position(position), _orientation(ori) 
  {
    _size[0] = depth;
    _size[1] = width;
    _size[2] = height;
  }

  virtual ~CollisionBox() 
  {}
 
  virtual bool ContactDetection(const Vec3<T>& cp_pos, T& penetration,Mat3<T>& cp_frame);//检查躯干是否有接触函数

 private:
  T _size[3];

  Vec3<T> _position;
  Mat3<T> _orientation;
};

#endif  // COLLISION_BOX_H
