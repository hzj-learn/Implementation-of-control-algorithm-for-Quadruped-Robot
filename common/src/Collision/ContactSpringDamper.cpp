/*! @file ContactConstraint.cpp
 *  @brief ContactConstraint虚拟类
 */

#include "Collision/ContactSpringDamper.h"
#include "Dynamics/FloatingBaseModel.h"

/*!
 *计算与接触重合的接触力并更新外力在动力学模型中。
 */
template <typename T>
void ContactSpringDamper<T>::UpdateExternalForces(T K, T D, T dt) 
{
  CC::_nContact = CC::_CheckContact();  //检测腿有没有触地
  for (size_t i(0); i < _nGC; ++i) 
  {
    // 首先假设没有接触，所以地面“弹回”
    deflectionRate[i] = (-K / D) * _tangentialDeflections[i];
  }
  if (CC::_nContact > 0) 
  {
    _groundContactWithOffset(K, D);
    for (size_t i(0); i < CC::_nContact; ++i) 
    {
      CC::_model->_externalForces.at(CC::_model->_gcParent[CC::_idx_list[i]]) +=
          forceToSpatialForce(CC::_cp_force_list[CC::_idx_list[i]],
                              CC::_cp_pos_list[i]);
    }
    static int count(0);
    ++count;
    if (count > 5) 
    {
      // exit(0);
    }
  }

  for (size_t i(0); i < _nGC; ++i) {
    _tangentialDeflections[i] += dt * deflectionRate[i];
  }
}


/*!
 * 计算接触力宽度偏移函数
 */
template <typename T>
void ContactSpringDamper<T>::_groundContactWithOffset(T K, T D) 
{
  for (size_t i = 0; i < CC::_nContact; i++) 
  {
    Vec3<T> v =
        CC::_cp_frame_list[i].transpose() *
        CC::_model->_vGC[CC::_idx_list[i]];  //平面坐标中的速度
    T z = CC::_cp_penetration_list[i];       //穿透地面
    T zd = v[2];                             //穿透速度
    T zr = std::sqrt(std::max(T(0), -z));    //sqrt穿透地面，如果我们没有穿透
    T normalForce =
        zr *
        (-K * z - D * zd);  // 法向力为is spring-damper * sqrt(penetration)

    // 现在将输出力设置为零。
    CC::_cp_local_force_list[i][0] = 0;
    CC::_cp_local_force_list[i][1] = 0;
    CC::_cp_local_force_list[i][2] = 0;

    if (normalForce > 0) {
      CC::_cp_local_force_list[i][2] =
          normalForce;  // 设置法向力。这是在plane上的目前的坐标
      //首先，假设粘着
      //这意味着切向变形以脚的速度发生。
      deflectionRate[CC::_idx_list[i]] = v.template topLeftCorner<2, 1>();
      Vec2<T> tangentialSpringForce =
          K * zr *
          _tangentialDeflections[CC::_idx_list[i]];  // “弹簧”产生的切向力
      Vec2<T> tangentialForce =
          -tangentialSpringForce -
          D * zr * deflectionRate[CC::_idx_list[i]];  // 添加阻尼以获得总切线

      //检查是否打滑：
      T slipForce = CC::_cp_mu_list[i] *
                    normalForce;  //不打滑的最大力值
      T tangentialForceMagnitude =
          tangentialForce
              .norm();  // 如果我们假设粘着
      T r = tangentialForceMagnitude / slipForce;  // 力/最大力比

      if (r > 1) {
        //我们正在滑倒。
        tangentialForce =
            tangentialForce / r;  //调整切向力，避免打滑
        deflectionRate[CC::_idx_list[i]] =
            -(tangentialForce + tangentialSpringForce) / (D * zr);
      }
      // 设置力矩
      CC::_cp_local_force_list[i][0] = tangentialForce[0];
      CC::_cp_local_force_list[i][1] = tangentialForce[1];
    }
    //移回机器人框架
    CC::_cp_force_list[CC::_idx_list[i]] =
        CC::_cp_frame_list[i] * CC::_cp_local_force_list[i];
  }
}

template class ContactSpringDamper<double>;
template class ContactSpringDamper<float>;
