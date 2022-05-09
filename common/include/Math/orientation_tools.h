/*! @file orientation_tools.h
 *  @brief 三维旋转的实用功能
 *他的文件包含旋转实用程序。我们通常用“坐标”
 *与位移变换相反
 *常见于图形中。为了描述物体的方位，我们使用
 *从世界坐标转换为身体坐标的旋转矩阵。这是
 *将身体自身旋转到正确位置的矩阵的转置
 *方向。

 *这遵循了罗伊·费瑟斯通（Roy Featherstone）的优秀著作《刚体》（Rigid Body）的惯例
 *动力学算法及其附带的spatial_v2Matlab库。
 *注意，我们不使用四元数的空间法则！
 */

#ifndef LIBBIOMIMETICS_ORIENTATION_TOOLS_H
#define LIBBIOMIMETICS_ORIENTATION_TOOLS_H

#include "Math/MathUtilities.h"
#include "cppTypes.h"

#include <eigen3/Eigen/Dense>

#include <cmath>
#include <iostream>
#include <type_traits>

namespace ori {

static constexpr double quaternionDerviativeStabilization = 0.1;

enum class CoordinateAxis { X, Y, Z };

/*!
 * 将弧度转换为角度函数
 */
template <typename T>
T rad2deg(T rad) 
{
  static_assert(std::is_floating_point<T>::value,
                "must use floating point value");
  return rad * T(180) / T(M_PI);
}

/*!
 * 将角度转换为弧度函数
 */
template <typename T>
T deg2rad(T deg) 
{
  static_assert(std::is_floating_point<T>::value,
                "must use floating point value");
  return deg * T(M_PI) / T(180);
}

/*!
*计算坐标变换的旋转矩阵函数
*坐标轴（坐标轴：X，.1）*v将使v旋转-.1弧度-
*这将转换成一个旋转0.1弧度的帧！。
 */
template <typename T>
Mat3<T> coordinateRotation(CoordinateAxis axis, T theta) 
{
  static_assert(std::is_floating_point<T>::value,
                "must use floating point value");
  T s = std::sin(theta);
  T c = std::cos(theta);

  Mat3<T> R;

  if (axis == CoordinateAxis::X) {
    R << 1, 0, 0, 0, c, s, 0, -s, c;
  } else if (axis == CoordinateAxis::Y) {
    R << c, 0, -s, 0, 1, 0, s, 0, c;
  } else if (axis == CoordinateAxis::Z) {
    R << c, s, 0, -s, c, 0, 0, 0, 1;
  }

  return R;
}

/*!
 * 从rpy转到旋转矩阵函数
 */
template <typename T>
Mat3<typename T::Scalar> rpyToRotMat(const Eigen::MatrixBase<T>& v) 】
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "must have 3x1 vector");
  Mat3<typename T::Scalar> m = coordinateRotation(CoordinateAxis::X, v[0]) *
                               coordinateRotation(CoordinateAxis::Y, v[1]) *
                               coordinateRotation(CoordinateAxis::Z, v[2]);
  return m;
}

/*!
 * 将3x1向量转换为斜对称3x3矩阵函数
 */
template <typename T>
Mat3<typename T::Scalar> vectorToSkewMat(const Eigen::MatrixBase<T>& v) 
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  Mat3<typename T::Scalar> m;
  m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return m;
}

/*!
 * 将3x3矩阵的斜对称分量转化为3x1向量函数
 */
template <typename T>
Vec3<typename T::Scalar> matToSkewVec(const Eigen::MatrixBase<T>& m) {
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                "Must have 3x3 matrix");
  return 0.5 * Vec3<typename T::Scalar>(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0),
                                        (m(1, 0) - m(0, 1)));
}

/*!
 * 将旋转矩阵转换为方向四元数函数
 */
template <typename T>
Quat<typename T::Scalar> rotationMatrixToQuaternion(
    const Eigen::MatrixBase<T>& r1) {
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                "Must have 3x3 matrix");
  Quat<typename T::Scalar> q;
  Mat3<typename T::Scalar> r = r1.transpose();
  typename T::Scalar tr = r.trace();
  if (tr > 0.0) {
    typename T::Scalar S = sqrt(tr + 1.0) * 2.0;
    q(0) = 0.25 * S;
    q(1) = (r(2, 1) - r(1, 2)) / S;
    q(2) = (r(0, 2) - r(2, 0)) / S;
    q(3) = (r(1, 0) - r(0, 1)) / S;
  } else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
    typename T::Scalar S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
    q(0) = (r(2, 1) - r(1, 2)) / S;
    q(1) = 0.25 * S;
    q(2) = (r(0, 1) + r(1, 0)) / S;
    q(3) = (r(0, 2) + r(2, 0)) / S;
  } else if (r(1, 1) > r(2, 2)) {
    typename T::Scalar S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
    q(0) = (r(0, 2) - r(2, 0)) / S;
    q(1) = (r(0, 1) + r(1, 0)) / S;
    q(2) = 0.25 * S;
    q(3) = (r(1, 2) + r(2, 1)) / S;
  } else {
    typename T::Scalar S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
    q(0) = (r(1, 0) - r(0, 1)) / S;
    q(1) = (r(0, 2) + r(2, 0)) / S;
    q(2) = (r(1, 2) + r(2, 1)) / S;
    q(3) = 0.25 * S;
  }
  return q;
}

/*!
 * 功能：将四元数转换为旋转矩阵函数
 */
template <typename T>
Mat3<typename T::Scalar> quaternionToRotationMatrix(
    const Eigen::MatrixBase<T>& q) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  typename T::Scalar e0 = q(0);
  typename T::Scalar e1 = q(1);
  typename T::Scalar e2 = q(2);
  typename T::Scalar e3 = q(3);

  Mat3<typename T::Scalar> R;

  R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
      2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
      1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
      2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
      1 - 2 * (e1 * e1 + e2 * e2);
  R.transposeInPlace();
  return R;
}

/*!
 * 功能：将四元数转换为RPY函数
 * 使用ZYX指令（偏航俯仰滚转），但返回角度（横摇、俯仰、偏航）。
 */
template <typename T>
Vec3<typename T::Scalar> quatToRPY(const Eigen::MatrixBase<T>& q) 
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  Vec3<typename T::Scalar> rpy;
  typename T::Scalar as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
  rpy(2) =
      std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                 square(q[0]) + square(q[1]) - square(q[2]) - square(q[3]));
  rpy(1) = std::asin(as);
  rpy(0) =
      std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                 square(q[0]) - square(q[1]) - square(q[2]) + square(q[3]));
  return rpy;
}

/*!
 * 功能：将RPY转换为四元数函数
 */
template <typename T>
Quat<typename T::Scalar> rpyToQuat(const Eigen::MatrixBase<T>& rpy) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 vec");
  Mat3<typename T::Scalar> R = rpyToRotMat(rpy);
  Quat<typename T::Scalar> q = rotationMatrixToQuaternion(R);
  return q;
}


/*!
 * 将四元数转换为欧拉角函数
 */
template <typename T>
Vec3<typename T::Scalar> quatToso3(const Eigen::MatrixBase<T>& q) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  Vec3<typename T::Scalar> so3;
  typename T::Scalar theta = 2. * std::acos(q[0]);
  so3[0] = theta * q[1] / std::sin(theta / 2.);
  so3[1] = theta * q[2] / std::sin(theta / 2.);
  so3[2] = theta * q[3] / std::sin(theta / 2.);
  return so3;
}


/*!
 * 功能：将旋转矩阵转换为RPY函数。返回角度（横摇、俯仰、偏航）。
 */
template <typename T>
Vec3<typename T::Scalar> rotationMatrixToRPY(const Eigen::MatrixBase<T>& R) 
{
  static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3,
                "Must have 3x3 matrix");
  Quat<typename T::Scalar> q = rotationMatrixToQuaternion(R);
  Vec3<typename T::Scalar> rpy = quatToRPY(q);
  return rpy;
}


/*!
 *四元数导数的计算函数
 *欧米茄是用身体框架来表示的
 * @param q
 * @param omega
 * @return
 */
template <typename T, typename T2>
Quat<typename T::Scalar> quatDerivative(const Eigen::MatrixBase<T>& q,
                                        const Eigen::MatrixBase<T2>& omega) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 omega");
  // first case in rqd
  Mat4<typename T::Scalar> Q;
  Q << q[0], -q[1], -q[2], -q[3], q[1], q[0], -q[3], q[2], q[2], q[3], q[0],
      -q[1], q[3], -q[2], q[1], q[0];

  Quat<typename T::Scalar> qq(
      quaternionDerviativeStabilization * omega.norm() * (1 - q.norm()),
      omega[0], omega[1], omega[2]);
  Quat<typename T::Scalar> dq = 0.5 * Q * qq;
  return dq;
}

/*!
 * 取两个四元数的乘积函数
 */
template <typename T>
Quat<typename T::Scalar> quatProduct(const Eigen::MatrixBase<T>& q1,
                                     const Eigen::MatrixBase<T>& q2) {
  typename T::Scalar r1 = q1[0];
  typename T::Scalar r2 = q2[0];
  Vec3<typename T::Scalar> v1(q1[1], q1[2], q1[3]);
  Vec3<typename T::Scalar> v2(q2[1], q2[2], q2[3]);

  typename T::Scalar r = r1 * r2 - v1.dot(v2);
  Vec3<typename T::Scalar> v = r1 * v2 + r2 * v1 + v1.cross(v2);
  Quat<typename T::Scalar> q(r, v[0], v[1], v[2]);
  return q;
}

/*!
 * 计算给定的新四元数函数
 * @param quat      旧四元数
 * @param omega     角速度（惯性坐标！）
 * @param dt        时间步
 * @return
 */
template <typename T, typename T2, typename T3>
Quat<typename T::Scalar> integrateQuat(const Eigen::MatrixBase<T>& quat,
                                       const Eigen::MatrixBase<T2>& omega,
                                       T3 dt) 
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 omega");
  Vec3<typename T::Scalar> axis;
  typename T::Scalar ang = omega.norm();
  if (ang > 0) {
    axis = omega / ang;
  } else {
    axis = Vec3<typename T::Scalar>(1, 0, 0);
  }

  ang *= dt;
  Vec3<typename T::Scalar> ee = std::sin(ang / 2) * axis;
  Quat<typename T::Scalar> quatD(std::cos(ang / 2), ee[0], ee[1], ee[2]);

  Quat<typename T::Scalar> quatNew = quatProduct(quatD, quat);
  quatNew = quatNew / quatNew.norm();
  return quatNew;
}

/*!
 * 计算给定的新四元数函数
 * @param quat      旧四元数
 * @param omega     角速度（惯性坐标！）
 * @param dt        时间步
 * @return
 */
template <typename T, typename T2, typename T3>
Quat<typename T::Scalar> integrateQuatImplicit(
    const Eigen::MatrixBase<T>& quat, const Eigen::MatrixBase<T2>& omega,
    T3 dt) 
{
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 4,
                "Must have 4x1 quat");
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 omega");
  Vec3<typename T::Scalar> axis;
  typename T::Scalar ang = omega.norm();
  if (ang > 0) {
    axis = omega / ang;
  } else {
    axis = Vec3<typename T::Scalar>(1, 0, 0);
  }

  ang *= dt;
  Vec3<typename T::Scalar> ee = std::sin(ang / 2) * axis;
  Quat<typename T::Scalar> quatD(std::cos(ang / 2), ee[0], ee[1], ee[2]);

  Quat<typename T::Scalar> quatNew = quatProduct(quat, quatD);
  quatNew = quatNew / quatNew.norm();
  return quatNew;
}


/*!
 * 将四元数转换为欧拉角函数
 */
template <typename T>
void quaternionToso3(const Quat<T> quat, Vec3<T>& so3) 
{
  so3[0] = quat[1];
  so3[1] = quat[2];
  so3[2] = quat[3];

  T theta =
      2.0 * asin(sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]));

  if (fabs(theta) < 0.0000001) {
    so3.setZero();
    return;
  }
  so3 /= sin(theta / 2.0);
  so3 *= theta;
}

/*!
 * 将欧拉角转换为四元数。
 */
template <typename T>
Quat<T> so3ToQuat(Vec3<T>& so3) 
{
  Quat<T> quat;
  T theta = sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]);
  if (fabs(theta) < 1.e-6) {
    quat.setZero();
    quat[0] = 1.;
    return quat;
  }
  quat[0] = cos(theta / 2.);
  quat[1] = so3[0] / theta * sin(theta / 2.);
  quat[2] = so3[1] / theta * sin(theta / 2.);
  quat[3] = so3[2] / theta * sin(theta / 2.);
  return quat;
}
}  // namespace ori

#endif  // LIBBIOMIMETICS_ORIENTATION_TOOLS_H
