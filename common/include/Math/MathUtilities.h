/*! @file MathUtilities.h
 *  @brief 数学实用函数
 *
 */

#ifndef PROJECT_MATHUTILITIES_H
#define PROJECT_MATHUTILITIES_H

#include <eigen3/Eigen/Dense>

/*!
 * 平方数
 */
template <typename T>
T square(T a) {
  return a * a;
}

/*!
 * 两个特征矩阵几乎相等吗？
 */
template <typename T, typename T2>
bool almostEqual(const Eigen::MatrixBase<T>& a, const Eigen::MatrixBase<T>& b,
                 T2 tol) {
  long x = T::RowsAtCompileTime;
  long y = T::ColsAtCompileTime;

  if (T::RowsAtCompileTime == Eigen::Dynamic ||
      T::ColsAtCompileTime == Eigen::Dynamic) {
    assert(a.rows() == b.rows());
    assert(a.cols() == b.cols());
    x = a.rows();
    y = a.cols();
  }

  for (long i = 0; i < x; i++) {
    for (long j = 0; j < y; j++) {
      T2 error = std::abs(a(i, j) - b(i, j));
      if (error >= tol) return false;
    }
  }
  return true;
}

#endif  // PROJECT_MATHUTILITIES_H
