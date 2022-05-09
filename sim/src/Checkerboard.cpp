/*! @file Checkerboard.cpp
 *  @brief 三维平面棋盘图案图形
 *
 */

#include "Checkerboard.h"

#include <stdio.h>

/*!
 * 在原点构建一个新的棋盘
 * @param xSize : 宽度，米
 * @param ySize : 高度，米
 * @param xSquares : 沿x的正方形数（必须至少为1）
 * @param ySquares : 沿y轴的正方形数（必须至少为2）
 */
Checkerboard::Checkerboard(float xSize, float ySize, size_t xSquares,
                           size_t ySquares) {
  _size[0] = xSize;
  _size[1] = ySize;
  _squares[0] = xSquares;
  _squares[1] = ySquares;
  //  computeVertices();
}

/*!
 * 更新当前大小的棋盘顶点
 */
void Checkerboard::computeVertices(std::vector<float>& vertices,
                                   std::vector<float>& normals,
                                   std::vector<float>& colors) {
  // 3个通道，n*m正方形，2个三角形，每个三角形3个顶点
  size_t dataSize = 3 * _squares[0] * _squares[1] * 3 * 2;
  vertices.clear();
  normals.clear();
  colors.clear();
  vertices.reserve(dataSize);
  normals.reserve(dataSize);
  colors.reserve(dataSize);

  const float xStep = _size[0] / (float)_squares[0];
  const float yStep = _size[1] / (float)_squares[1];

  for (size_t i = 0; i < _squares[0]; i++) {
    for (size_t j = 0; j < _squares[1]; j++) {
      // 添加法线和颜色
      const float* squareColor = (i + j) % 2 ? _darkColor : _lightColor;
      for (size_t k = 0; k < 6; k++) {
        for (size_t m = 0; m < 3; m++) {
          colors.push_back(squareColor[m]);
          normals.push_back(m == 2 ? 1 : 0);
        }
      }

      const float xStart = xStep * i;
      const float yStart = yStep * j;

      // tri 1
      vertices.push_back(xStart);
      vertices.push_back(yStart);
      vertices.push_back(0);

      vertices.push_back(xStart);
      vertices.push_back(yStart + yStep);
      vertices.push_back(0);

      vertices.push_back(xStart + xStep);
      vertices.push_back(yStart);
      vertices.push_back(0);

      // tri 2
      vertices.push_back(xStart);
      vertices.push_back(yStart + yStep);
      vertices.push_back(0);

      vertices.push_back(xStart + xStep);
      vertices.push_back(yStart + yStep);
      vertices.push_back(0);

      vertices.push_back(xStart + xStep);
      vertices.push_back(yStart);
      vertices.push_back(0);
    }
  }
}