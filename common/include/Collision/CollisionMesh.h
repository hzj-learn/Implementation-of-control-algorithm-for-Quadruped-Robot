/*!
 * @file CollisionMesh.h
 * @brief 网格的碰撞逻辑
 */

#ifndef COLLISION_MESH_H
#define COLLISION_MESH_H

#include <vector>

#include "Collision/Collision.h"
#include "Utilities/utilities.h"
#include "cppTypes.h"

/*!
 * 类来表示网格碰撞
 */
template <typename T>
class CollisionMesh : public Collision<T> 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * 功能：构造新的碰撞网格函数
   * @param mu           : 输入摩擦系数
   * @param restitution  : 输入回弹率（v+/v-）
   * @param grid         : 输入给定高度地图的栅格大小（米）
   * @param left_corner_loc : 输入高度左下边缘的全局位置
   * @param height_map : 输入网格高度图
   */
  CollisionMesh(const T& mu, const T& restitution, const T& grid,
                const Vec3<T>& left_corner_loc, const DMat<T>& height_map)
      : Collision<T>(mu, restitution),
        _left_corner_loc(left_corner_loc),
        _height_map(height_map),
        _grid(grid) 
  {
    _x_max = (_height_map.rows() - 1) * _grid;
    _y_max = (_height_map.cols() - 1) * _grid;
  }

  virtual ~CollisionMesh() {}
  virtual bool ContactDetection(const Vec3<T>& cp_pos, T& penetration,
                                Mat3<T>& cp_frame);

 private:
  T _size[3];

  Vec3<T> _left_corner_loc;
  DMat<T> _height_map;

  T _grid;
  T _x_max;
  T _y_max;
};

#endif  // COLLISION_MESH_H
