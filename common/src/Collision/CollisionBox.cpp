#include "Collision/CollisionBox.h"
#include "Utilities/utilities.h"

/*!
 *功能：检查躯干是否有接触函数
 *若有接触，返回ture
 *若没有接触，返回false
 */
template <typename T>
bool CollisionBox<T>::ContactDetection(const Vec3<T>& cp_pos, T& penetration,
                                       Mat3<T>& cp_frame) 
{
  Vec3<T> center2cp_local = _orientation.transpose() * (cp_pos - _position);

  bool inside_box(true);      //默认防止了长方体
  T err[3] = {0};
  for (size_t i(0); i < 3; ++i) 
  {
    // 向正方向外
    if (center2cp_local[i] > _size[i] / 2.) 
    {
      inside_box = false;
    }
    // 从外到负方向
    else if (center2cp_local[i] < -_size[i] / 2.) 
    {
      inside_box = false;
    }
    // 距地面距离
    else 
    {
      err[i] = _size[i] / 2. - std::abs(center2cp_local[i]);
    }
  }

  if (inside_box) 
  {
    if (err[2] < std::min(err[0], err[1]))      //Z面接触
    {  
      if (center2cp_local[2] > 0.) {
        cp_frame = _orientation;
      } else {
        cp_frame = -_orientation;
      }
      penetration = -err[2];
    } 

    else if (err[1] < std::min(err[0], err[2])) //Y面接触
    { 
      if (center2cp_local[1] > 0.) {
        // X -->X
        cp_frame.template block<3, 1>(0, 0) =
            _orientation.template block<3, 1>(0, 0);
        // -Z --> Y
        cp_frame.template block<3, 1>(0, 1) =
            -_orientation.template block<3, 1>(0, 2);
        // Y --> Z
        cp_frame.template block<3, 1>(0, 2) =
            _orientation.template block<3, 1>(0, 1);
      } else {
        // X --> X
        cp_frame.template block<3, 1>(0, 0) =
            _orientation.template block<3, 1>(0, 0);
        // Z --> Y
        cp_frame.template block<3, 1>(0, 1) =
            _orientation.template block<3, 1>(0, 2);
        // -Y --> Z
        cp_frame.template block<3, 1>(0, 2) =
            -_orientation.template block<3, 1>(0, 1);
      }
      penetration = -err[1];
    } 
    
    else                                        //X面接触
    { 
      if (center2cp_local[0] > 0.) {
        // X -->X
        cp_frame.template block<3, 1>(0, 0) =
            _orientation.template block<3, 1>(0, 1);
        // -Z --> Y
        cp_frame.template block<3, 1>(0, 1) =
            -_orientation.template block<3, 1>(0, 2);
        // Y --> Z
        cp_frame.template block<3, 1>(0, 2) =
            _orientation.template block<3, 1>(0, 0);
      } else {
        // X -->X
        cp_frame.template block<3, 1>(0, 0) =
            _orientation.template block<3, 1>(0, 0);
        // -Z --> Y
        cp_frame.template block<3, 1>(0, 1) =
            _orientation.template block<3, 1>(0, 1);
        // Y --> Z
        cp_frame.template block<3, 1>(0, 2) =
            -_orientation.template block<3, 1>(0, 0);
      }
      penetration = -err[0];
    }
    return true;
  }
  return false;
}

template class CollisionBox<double>;
template class CollisionBox<float>;
