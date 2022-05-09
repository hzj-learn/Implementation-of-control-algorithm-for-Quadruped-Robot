/*! @file ImuSimulator.h
 *  @brief  模拟IMU
 */
#include "SimUtilities/ImuSimulator.h"
#include "Dynamics/spatial.h"
#include "Math/orientation_tools.h"
#include "Utilities/utilities.h"

//（1）计算加速度
template <typename T>
void ImuSimulator<T>::computeAcceleration(
    const FBModelState<T> &robotState,
    const FBModelStateDerivative<T> &robotStateD, Vec3<float> &acc,
    std::uniform_real_distribution<float> &dist, const RotMat<float> &R_body) 
{
  // 加速度计噪声
  fillEigenWithRandom(acc, _mt, dist);

  // 重力（机器人直立时应为正）
  acc += (R_body * Vec3<float>(0, 0, 9.81));
 

  // 加速
  acc += spatial::spatialToLinearAcceleration(robotStateD.dBodyVelocity,
                                              robotState.bodyVelocity)
             .template cast<float>();
}


//（2）更新矢量导航
template <typename T>
void ImuSimulator<T>::updateVectornav(
    const FBModelState<T> &robotState,
    const FBModelStateDerivative<T> &robotStateD, VectorNavData *data) 
{
  // body orientation
  RotMat<float> R_body = quaternionToRotationMatrix(
      robotState.bodyOrientation.template cast<float>());

  Quat<float> ori_quat;
  // printf("imu ori: %f, %f, %f, %f\n",
  // robotState.bodyOrientation[0],
  // robotState.bodyOrientation[1],
  // robotState.bodyOrientation[2],
  // robotState.bodyOrientation[3] );

  // 加速度
  computeAcceleration(robotState, robotStateD, data->accelerometer,
                      _vectornavAccelerometerDistribution, R_body);

  // gyro
  fillEigenWithRandom(data->gyro, _mt, _vectornavGyroDistribution);
  data->gyro +=
      robotState.bodyVelocity.template head<3>().template cast<float>();

  // 四元数
  if (_vectorNavOrientationNoise) {
    Vec3<float> omegaNoise;
    fillEigenWithRandom(omegaNoise, _mt, _vectornavQuatDistribution);
    Quat<float> floatQuat = robotState.bodyOrientation.template cast<float>();
    // data->quat = integrateQuat(floatQuat, omegaNoise, 1.0f);
    ori_quat = integrateQuat(floatQuat, omegaNoise, 1.0f);
    // printf("imu noised ori: %f, %f, %f, %f\n",
    // robotState.bodyOrientation[0],
    // robotState.bodyOrientation[1],
    // robotState.bodyOrientation[2],
    // robotState.bodyOrientation[3] );
  } else {
    // data->quat = robotState.bodyOrientation.template cast<float>();
    ori_quat = robotState.bodyOrientation.template cast<float>();
  }
  data->quat[3] = ori_quat[0];
  data->quat[0] = ori_quat[1];
  data->quat[1] = ori_quat[2];
  data->quat[2] = ori_quat[3];
}

//（3）更新仿真器状态
template <typename T>
void ImuSimulator<T>::updateCheaterState(
    const FBModelState<T> &robotState,
    const FBModelStateDerivative<T> &robotStateD, CheaterState<T> &state) 
{
  RotMat<T> R_body = quaternionToRotationMatrix(robotState.bodyOrientation);
  state.acceleration = (R_body * Vec3<T>(0, 0, 9.81)) +
                       spatial::spatialToLinearAcceleration(
                           robotStateD.dBodyVelocity, robotState.bodyVelocity);
  state.orientation = robotState.bodyOrientation;
  state.position = robotState.bodyPosition;
  state.omegaBody = robotState.bodyVelocity.template head<3>();
  state.vBody = robotState.bodyVelocity.template tail<3>();
}

template class ImuSimulator<float>;
template class ImuSimulator<double>;
