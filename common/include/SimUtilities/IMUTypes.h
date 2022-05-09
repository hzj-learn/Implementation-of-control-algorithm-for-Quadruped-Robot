/*! @file IMUTypes.h
 *  @brief 来自IMU的数据
 */

#ifndef PROJECT_IMUTYPES_H
#define PROJECT_IMUTYPES_H

#include "cppTypes.h"

/*!
 * Mini Cheetah's IMU
 */
struct VectorNavData //矢量导航数据
{
  Vec3<float> accelerometer;    //（1）加速度计
  Vec3<float> gyro;             //（2）陀螺仪
  Quat<float> quat;             //（3）四元数
  // todo is there status for the vectornav?
};

template <typename T>
struct CheaterState //机器人躯干仿真状态
{
  Quat<T> orientation;      //（1）躯干方向
  Vec3<T> position;         //（2）躯干位置
  Vec3<T> omegaBody;        //（3）躯干俯仰角
  Vec3<T> vBody;            //（4）躯干线速度
  Vec3<T> acceleration;     //（5）躯干加速度
};

#endif  // PROJECT_IMUTYPES_H
