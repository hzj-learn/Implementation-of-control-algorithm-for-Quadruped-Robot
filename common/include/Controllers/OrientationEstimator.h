/*! @file OrientationEstimator.h
 *  @brief 方位估计算法
 *  方位估计器应计算：
 *  - （1）orientation: 表示方向的四元数， 通过imu获得的角度可以转换成旋转矩阵，将变量都映射到世界坐标系下
 *  - （2）rBody:       坐标变换矩阵，满足将世界坐标系的速度能映射成基躯干的速度(satisfies vBody = Rbody * vWorld)
 *  - （3）omegaBody:   躯干坐标系中的角速度
 *  - （4）omegaWorld:  世界坐标系中的角速度
 *  - （5）rpy角度:      roll滚动角度、 pitch倾斜角度、 yaw偏航角度
 */
#ifndef PROJECT_ORIENTATIONESTIMATOR_H
#define PROJECT_ORIENTATIONESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"

/*!
 * 功能：方向的估计器，在模拟中总是返回正确的值
 */
template <typename T>
class CheaterOrientationEstimator : public GenericEstimator<T> 
{
 public:
  virtual void run();
  virtual void setup() {}
};

/*!
 *功能：向量导航IMU的估计量
 *imu已经提供了方向,我们只需读取IMU数据并作相应的左边变换，返回那个IMU的值
 */
template <typename T>
class VectorNavOrientationEstimator : public GenericEstimator<T> 
{
 public:
  virtual void run();
  virtual void setup() {}
  
 protected:
  bool _b_first_visit = true;
  Quat<T> _ori_ini_inv;
};


#endif  // PROJECT_ORIENTATIONESTIMATOR_H
