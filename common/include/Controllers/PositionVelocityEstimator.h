/*! @file PositionVelocityEstimator.h
  *位置速度传感器应计算：
  *-世界/身体框架中的身体位置/速度
  *-身体/世界框架中的脚位置/速度
 */

#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"

/*!
 * 功能：真实调试模式下，基于卡尔曼滤波的位置和速度估计器。
 *      这是Mini Cheetah和Cheetah 3中使用的算法。
 */
template <typename T>
class LinearKFPositionVelocityEstimator : public GenericEstimator<T> 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LinearKFPositionVelocityEstimator();
  virtual void run();
  virtual void setup();

 private:
  Eigen::Matrix<T, 18, 1> _xhat;  //状态估计值矩阵：世界坐标下[p v p1 p2 p3 p4] 
  Eigen::Matrix<T, 12, 1> _ps;    //储存状态矩阵p
  Eigen::Matrix<T, 12, 1> _vs;    //储存状态矩阵v
  Eigen::Matrix<T, 18, 18> _A;    //状态转移阵
  Eigen::Matrix<T, 18, 18> _Q0;   //初始状态估计噪声矩阵
  Eigen::Matrix<T, 18, 18> _P;    //初始不确定性矩阵
  Eigen::Matrix<T, 28, 28> _R0;   //初始观测噪声矩阵
  Eigen::Matrix<T, 18, 3> _B;     //输入阵
  Eigen::Matrix<T, 28, 18> _C;    //观测阵
};


/*!
 * 功能：仿真模式下，位置和速度估计器
 * 将返回正确的位置和在模拟中运行时的速度。
 */
template<typename T>
class CheaterPositionVelocityEstimator : public GenericEstimator<T> 
{
public:
  virtual void run();
  virtual void setup() {}
};

#endif  // PROJECT_POSITIONVELOCITYESTIMATOR_H
