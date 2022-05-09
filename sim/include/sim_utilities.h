/*! @file sim_utilities.h
 *  @brief Utility functions that exist only in the simulator
 */

#ifndef PROJECT_SIM_UTILITIES_H
#define PROJECT_SIM_UTILITIES_H

#include "Dynamics/spatial.h"
#include "Math/orientation_tools.h"
#include "cppTypes.h"

#include <QMatrix4x4>

using namespace spatial;

/*!
  *将空间变换转换为Qt变换。
  *请注意，这通常还具有反转转换的效果：
  *空间变换是坐标变换，Qt不遵循这个约定
 */
template <typename T>
QMatrix4x4 spatialTransformToQT(const Eigen::MatrixBase<T> &X) {
  static_assert(T::ColsAtCompileTime == 6 && T::RowsAtCompileTime == 6,
                "Must have 6x6 matrix");
  Mat4<typename T::Scalar> H = sxformToHomogeneous(invertSXform(X));
  QMatrix4x4 M(H(0, 0), H(0, 1), H(0, 2), H(0, 3), H(1, 0), H(1, 1), H(1, 2),
               H(1, 3), H(2, 0), H(2, 1), H(2, 2), H(2, 3), H(3, 0), H(3, 1),
               H(3, 2), H(3, 3));
  return M;
}

#endif  // PROJECT_SIM_UTILITIES_H
