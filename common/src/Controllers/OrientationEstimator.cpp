/*! @file OrientationEstimator.h
 *  @brief 全方位估计算法
 *
 *  此文件将包含所有方向算法。
 *  方位估计器应计算：
 *  - （1）orientation: 表示方向的四元数， 通过imu获得的角度可以转换成旋转矩阵，将变量都映射到世界坐标系下
 *  - （2）rBody:       坐标变换矩阵，满足将世界坐标系的速度能映射成基躯干的速度(satisfies vBody = Rbody * vWorld)
 *  - （3）omegaBody:   躯干坐标系中的角速度
 *  - （4）omegaWorld:  世界坐标系中的角速度
 *  - （5）rpy角度:      roll滚动角度、 pitch倾斜角度、 yaw偏航角度
 */


#include "Controllers/OrientationEstimator.h"
/*!
 * 功能：在cheater模式的时候，运行orientation方向估计
 */
template <typename T>
void CheaterOrientationEstimator<T>::run() 
{
  //估计躯干方向
  this->_stateEstimatorData.result->orientation =
      this->_stateEstimatorData.cheaterState->orientation.template cast<T>();
  
  //估计旋转矩阵
  //通过orientation得到的旋转矩阵表示 从世界坐标变换到机体坐标的变换， 从而实现 vBody = Rbody * vWorld
  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData.result->orientation);
  
  //估计躯干坐标系中的角速度
  this->_stateEstimatorData.result->omegaBody =
      this->_stateEstimatorData.cheaterState->omegaBody.template cast<T>();
  
  //估计世界坐标系中的角速度
  this->_stateEstimatorData.result->omegaWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->omegaBody;
  
  //估计rpy角度
  this->_stateEstimatorData.result->rpy =
      ori::quatToRPY(this->_stateEstimatorData.result->orientation);
  
  //估计躯干坐标系的加速度
  this->_stateEstimatorData.result->aBody =
      this->_stateEstimatorData.cheaterState->acceleration.template cast<T>();
  
  //估计世界坐标系的加速度
  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->aBody;
}

/*!
 * 功能：真正运行机器人时候，运行的orientation方向估计
 * 备注：我们只需直接读取IMU已经提供的四元数方向,陀螺仪角速度、加速度计加速度，并作相应的坐标变换，返回读取后经过转换的IMU数据
 */
template <typename T>
void VectorNavOrientationEstimator<T>::run() 
{
/////////////////////////////////////////////*（1）用于表示方向的数据*///////////////////////////////////////////////////

  //（1）读取在躯干坐标系中IMU表示方向的四元数，用于表示躯干方向的数据
			  //orientation[0]: w :vectorNavData->quat[3]
			  //orientation[1]: x :vectorNavData->quat[0]
			  // …………………………………………
   this->_stateEstimatorData.result->orientation[0] = this->_stateEstimatorData.vectorNavData->quat[3]; //
   this->_stateEstimatorData.result->orientation[1] = this->_stateEstimatorData.vectorNavData->quat[0];
   this->_stateEstimatorData.result->orientation[2] = this->_stateEstimatorData.vectorNavData->quat[1];
   this->_stateEstimatorData.result->orientation[3] = this->_stateEstimatorData.vectorNavData->quat[2];
  
  //（2）计算躯干坐标系中的旋转矩阵rBody，用于表示躯干方向的数据
  //原理：通过IMU提供的四元素转换得到得到的旋转矩阵
  this->_stateEstimatorData.result->rBody =
			ori::quaternionToRotationMatrix(this->_stateEstimatorData.result->orientation);

  //（3）计算rpy角度：roll滚动角度、 pitch倾斜角度、 yaw偏航角度，用于表示躯干方向的数据   
  //原理：将四元数转换为RPY，返回角度（横摇、俯仰、偏航）
  this->_stateEstimatorData.result->rpy = ori::quatToRPY(this->_stateEstimatorData.result->orientation);


/////////////////////////////////////////////*（2）用于表示角速度的数据*///////////////////////////////////////////////////

  //（4）读取躯干坐标系中的角速度omegaBody，用于表示角速度的数据
  //原理： 直接读取imu测量提供的陀螺仪数据
  this->_stateEstimatorData.result->omegaBody =
			this->_stateEstimatorData.vectorNavData->gyro.template cast<T>();

  //（5）计算世界坐标系中的角速度omegaWorld，用于表示角速度的数据 
  //原理：omegaWorld世界坐标系中的角速度=rBody旋转矩阵转换后*omegaBody躯干坐标系中的角速度
  this->_stateEstimatorData.result->omegaWorld =
			this->_stateEstimatorData.result->rBody.transpose() *this->_stateEstimatorData.result->omegaBody;
 
/////////////////////////////////////////////*（3）用于表示加速度的数据*///////////////////////////////////////////////////

  //（6）读取躯干坐标系的加速度aBody，用于表示加速度的数据 
  //原理：直接读取imu加速度计测量提供的acc
  this->_stateEstimatorData.result->aBody =
      this->_stateEstimatorData.vectorNavData->accelerometer.template cast<T>();
      
  //（7）计算世界坐标系的加速度aWorld，用于表示加速度的数据 
  //原理：世界坐标系的加速度aWorld=躯干坐标系的加速度aBody转换后*躯干坐标系的加速度aBody
  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *this->_stateEstimatorData.result->aBody;
}


template class CheaterOrientationEstimator<float>;
template class CheaterOrientationEstimator<double>;

template class VectorNavOrientationEstimator<float>;
template class VectorNavOrientationEstimator<double>;