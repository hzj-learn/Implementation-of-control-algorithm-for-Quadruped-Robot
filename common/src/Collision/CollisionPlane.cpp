#include "Collision/CollisionPlane.h"

/*!
 *功能：检查是否与平面有接触函数
 *若有接触，返回ture
 *若没有接触，返回false
 */
template <typename T>
bool CollisionPlane<T>::ContactDetection(const Vec3<T>& cp_pos, T& penetration,
                                         Mat3<T>& cp_frame) 
{
  if (cp_pos[2] < _height) 
  {
    penetration = cp_pos[2] - _height;
    cp_frame.setIdentity();
    return true;
  } 
  else 
  {
    return false;
  }
}

template class CollisionPlane<double>;
template class CollisionPlane<float>;
