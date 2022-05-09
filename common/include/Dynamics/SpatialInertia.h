/*! @file SpatialInertia.h
 *  @brief 表示空间惯性张量的类
 *
 */

#ifndef LIBBIOMIMETICS_SPATIALINERTIA_H
#define LIBBIOMIMETICS_SPATIALINERTIA_H

#include "Math/orientation_tools.h"
#include "spatial.h"

#include <cmath>
#include <iostream>
#include <type_traits>

#include <eigen3/Eigen/Dense>

using namespace ori;
using namespace spatial;

/*!
 * 刚体惯量的6x6空间惯量张量表示
 */
template <typename T>
class SpatialInertia {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /*!
   * 根据质量、质心和3x3转动构造空间惯性
   */
  SpatialInertia(T mass, const Vec3<T>& com, const Mat3<T>& inertia) {
    Mat3<T> cSkew = vectorToSkewMat(com);
    _inertia.template topLeftCorner<3, 3>() =
        inertia + mass * cSkew * cSkew.transpose();
    _inertia.template topRightCorner<3, 3>() = mass * cSkew;
    _inertia.template bottomLeftCorner<3, 3>() = mass * cSkew.transpose();
    _inertia.template bottomRightCorner<3, 3>() = mass * Mat3<T>::Identity();
  }

  /*!
   * 从6x6矩阵构造空间惯性
   */
  explicit SpatialInertia(const Mat6<T>& inertia) { _inertia = inertia; }

  /*!
   * 如果没有参数，则为零。
   */
  SpatialInertia() { _inertia = Mat6<T>::Zero(); }

  /*!
   * 从质量特性向量构造空间惯性
   */
  explicit SpatialInertia(const MassProperties<T>& a) {
    _inertia(0, 0) = a(4);
    _inertia(0, 1) = a(9);
    _inertia(0, 2) = a(8);
    _inertia(1, 0) = a(9);
    _inertia(1, 1) = a(5);
    _inertia(1, 2) = a(7);
    _inertia(2, 0) = a(8);
    _inertia(2, 1) = a(7);
    _inertia(2, 2) = a(6);
    Mat3<T> cSkew = vectorToSkewMat(Vec3<T>(a(1), a(2), a(3)));
    _inertia.template topRightCorner<3, 3>() = cSkew;
    _inertia.template bottomLeftCorner<3, 3>() = cSkew.transpose();
    _inertia.template bottomRightCorner<3, 3>() = a(0) * Mat3<T>::Identity();
  }

  /*!
   * 由伪惯性构造空间惯性。这在物理一致惯性参数的线性矩阵不等式
   * @param P
   */
  explicit SpatialInertia(const Mat4<T>& P) {
    Mat6<T> I;
    T m = P(3, 3);
    Vec3<T> h = P.template topRightCorner<3, 1>();
    Mat3<T> E = P.template topLeftCorner<3, 3>();
    Mat3<T> Ibar = E.trace() * Mat3<T>::Identity() - E;
    I.template topLeftCorner<3, 3>() = Ibar;
    I.template topRightCorner<3, 3>() = vectorToSkewMat(h);
    I.template bottomLeftCorner<3, 3>() = vectorToSkewMat(h).transpose();
    I.template bottomRightCorner<3, 3>() = m * Mat3<T>::Identity();
    _inertia = I;
  }

  /*!
   * 将空间惯性转换为质量特性矢量
   */
  MassProperties<T> asMassPropertyVector() {
    MassProperties<T> a;
    Vec3<T> h = matToSkewVec(_inertia.template topRightCorner<3, 3>());
    a << _inertia(5, 5), h(0), h(1), h(2), _inertia(0, 0), _inertia(1, 1),
        _inertia(2, 2), _inertia(2, 1), _inertia(2, 0), _inertia(1, 0);
    return a;
  }

  /*!
   * 获得6x6空间惯性
   */
  const Mat6<T>& getMatrix() const { return _inertia; }

  void setMatrix(const Mat6<T>& mat) { _inertia = mat; }

  void addMatrix(const Mat6<T>& mat) { _inertia += mat; }

  /*!
   * 获得质量
   */
  T getMass() { return _inertia(5, 5); }

  /*!
   * 获取质心位置
   */
  Vec3<T> getCOM() {
    T m = getMass();
    Mat3<T> mcSkew = _inertia.template topRightCorner<3, 3>();
    Vec3<T> com = matToSkewVec(mcSkew) / m;
    return com;
  }

  /*!
   * 得到3x3转动惯量
   */
  Mat3<T> getInertiaTensor() {
    T m = getMass();
    Mat3<T> mcSkew = _inertia.template topRightCorner<3, 3>();
    Mat3<T> I_rot = _inertia.template topLeftCorner<3, 3>() -
                    mcSkew * mcSkew.transpose() / m;
    return I_rot;
  }

  /*!
  *转换为4x4伪惯性矩阵。这在物理一致惯性参数的线性矩阵不等式
   */
  Mat4<T> getPseudoInertia() {
    Vec3<T> h = matToSkewVec(_inertia.template topRightCorner<3, 3>());
    Mat3<T> Ibar = _inertia.template topLeftCorner<3, 3>();
    T m = _inertia(5, 5);
    Mat4<T> P;
    P.template topLeftCorner<3, 3>() =
        0.5 * Ibar.trace() * Mat3<T>::Identity() - Ibar;
    P.template topRightCorner<3, 1>() = h;
    P.template bottomLeftCorner<1, 3>() = h.transpose();
    P(3, 3) = m;
    return P;
  }

  /*!
   * 绕轴翻转惯性矩阵。这不是有效的，但它是有效的！
   */
  SpatialInertia flipAlongAxis(CoordinateAxis axis) {
    Mat4<T> P = getPseudoInertia();
    Mat4<T> X = Mat4<T>::Identity();
    if (axis == CoordinateAxis::X)
      X(0, 0) = -1;
    else if (axis == CoordinateAxis::Y)
      X(1, 1) = -1;
    else if (axis == CoordinateAxis::Z)
      X(2, 2) = -1;
    P = X * P * X;
    return SpatialInertia(P);
  }

 private:
  Mat6<T> _inertia;
};

#endif  // LIBBIOMIMETICS_SPATIALINERTIA_H
