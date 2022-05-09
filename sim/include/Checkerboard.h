/*! @file Checkerboard.cpp
 *  @brief 棋盘图案的三维平面
 *
 * 以原点的中心和0,0,1法线为方向
 */

#ifndef PROJECT_CHECKERBOARD_H
#define PROJECT_CHECKERBOARD_H

#include <vector>

#include "Colors.h"
#include "cTypes.h"

class Checkerboard {
 public:
  Checkerboard(float xSize, float ySize, size_t xSquares, size_t ySquares);

  /*!
  *设置暗方块的颜色
  *在计算顶点之前执行此操作
   */
  void setDarkColor(const float* dark) { _darkColor = dark; }

  /*!
  *设置灯光方块的颜色
  *在计算顶点之前执行此操作
   */
  void setLightColor(const float* light) { _lightColor = light; }

  /*!
   * 得到一个x，y大小的数组（以平面的坐标，米为单位）
   */
  const float* getSize() { return _size; }

  void computeVertices(std::vector<float>& vertices,
                       std::vector<float>& normals, std::vector<float>& colors);

 private:
  const float* _darkColor = checkerboardDark;
  const float* _lightColor = checkerboardLight;
  float _size[2];
  size_t _squares[2];
};

#endif  // PROJECT_CHECKERBOARD_H
