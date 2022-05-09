/*! @file DrawList.h
 *  @brief 数据结构来存储要绘制的机器人模型。
 *存储机器人的所有数据（关节位置除外）。
 *知道如何从文件加载猎豹机器人。
 *将需要支持添加可变数量的项目（用于装载地形
 *需要绘制）
 *
 *  还支持具有重复对象
 */

#ifndef PROJECT_DRAWLIST_H
#define PROJECT_DRAWLIST_H

#include "Checkerboard.h"
#include "Collision/CollisionPlane.h"
#include "Colors.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/spatial.h"
#include "SimUtilities/VisualizationData.h"
#include "cppTypes.h"
#include "obj_loader.h"
#include "sim_utilities.h"

#include <QMatrix4x4>

#include <stdlib.h>
#include <vector>

class BoxInfo {
 public:
  double depth, width, height;
  float frame[16];  // SE3
                    /*
                       T[0] T[4] T[8]  T[12]
                       T[1] T[5] T[9]  T[13]
                       T[2] T[6] T[10] T[14]
                       T[3] T[7] T[11] T[15] = (0, 0, 0, 1)
                       */
};

struct ScrollInfo {
  size_t id;
  float xs, ys;
};

struct SolidColor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<float> rgba;
  bool useSolidColor;
};

class DrawList {
 public:
  VisualizationData *_visualizationData;

  DrawList() {
    _cameraOrigin = Vec3<double>::Zero();
    loadFiles();
  }
  size_t addCheetah3(Vec4<float> color, bool useOld, bool canHide);
  size_t addMiniCheetah(Vec4<float> color, bool useOld, bool canHide);
  void buildDrawList();
  void loadFiles();
  void doScrolling(Vec3<float> cameraPos);
  size_t addCheckerboard(Checkerboard &checkerBoard, bool setScrolling);
  size_t addDebugSphere(float radius);
  void addBox(double depth, double width, double height,
              const Vec3<double> &pos, const Mat3<double> &ori,
              bool transparent);
  void addMesh(double grid_size, const Vec3<double> &left_corner,
               const DMat<double> &height_map, bool transparent);

  /*!
   *调整大小以保留大小对象。
   */
  void resize(size_t nUniqueObject, size_t nTotalObjects) {
    _nUnique = nUniqueObject;
    _nTotal = nTotalObjects;
    _vertexData.resize(nUniqueObject);
    _normalData.resize(nUniqueObject);
    _colorData.resize(nUniqueObject);
    _offsetXforms.resize(nUniqueObject);
    _objectMap.resize(nTotalObjects);
  }

  /*!
   * 获取要绘制的对象总数
   */
  size_t getNumObjectsToDraw() { return _nTotal; }

  /*!
   * For the i-th object, get the offset into the model data.
   * For use with the glDrawArrays function.
   * Note that several objects may have the same geometry, so this will return
   * the same value for these objects!
   */
  size_t getGLDrawArrayOffset(size_t i) {
    return _glArrayOffsets.at(_objectMap.at(i));
  }

  /*!
   * 对于第i个对象，获取模型数据数组的大小
   */
  size_t getGLDrawArraySize(size_t i) {
    return _glArraySizes.at(_objectMap.at(i));
  }

  bool getCanBeHidden(size_t i) {
    return _canBeHidden.at(i);
  }

  /*!
  *获取包含所有顶点数据的数组。使用getGLDrawArrayOffset/Size
  *获取每个对象的索引和大小
   */
  float *getVertexArray() { return _glVertexData.data(); }

  /*!
  *获取包含所有正常数据的数组。使用getGLDrawArrayOffset/Size
  *获取每个对象的索引和大小
   */
  float *getNormalArray() { return _glNormalData.data(); }

  size_t getSizeOfAllData() { return _glVertexData.size(); }

  /*!
   *获取包含所有颜色数据的数组。
   */
  float *getColorArray() { return _glColorData.data(); }

  /*!
  *得到了模型应采用的Qt变换矩阵
  *几何图形这是为了纠正导出零件和移动零件时的错误
  *一切都回到原点。
   */
  QMatrix4x4 &getModelBaseTransform(size_t i) { return _modelOffsets[i]; }

  /*!
  *得到用于移动模型的Qt变换矩阵
  *从它的起源到它在世界上的位置
   */
  QMatrix4x4 &getModelKinematicTransform(size_t i) {
    return _kinematicXform[i];
  }

  /*!
  *获取GPU使用的数据大小（MB）
  *用于调试
   */
  float getGLDataSizeMB() {
    size_t bytes =
        _glColorData.size() + _glNormalData.size() + _glVertexData.size();
    bytes = bytes * sizeof(float);
    return (float)bytes / float(1 << 20);
  }

  /*!
   *如果更改了几何图形并需要重新加载，则一次返回true。
   */
  bool needsReload() {
    if (_reloadNeeded) {
      _reloadNeeded = false;
      return true;
    }
    return false;
  }

  /*!
    *使用动力学结果更新机器人身体的位置
    *模拟。不运行模拟器-只是从
    *动态模拟器
   * @param model  : 模拟器
   * @param id     : 从loadCheetah3或loadMiniCheetah函数返回的id。
   */
  template <typename T>
  void updateRobotFromModel(DynamicsSimulator<T> &model, size_t id,
                            bool updateOrigin = false) {
    for (size_t modelID = 5, graphicsID = id; modelID < model.getNumBodies();
         modelID++, graphicsID++) {
      _kinematicXform.at(graphicsID) =
          spatialTransformToQT(model.getModel()._Xa.at(modelID));
    }

    if (updateOrigin) {
      _cameraOrigin = model.getState().bodyPosition.template cast<T>();
    }
  }

  /*!
    *更新GUI绘制的附加信息
    *不能运行模拟器
    *或者只是从未来的数据中提取
    *动态模拟器
   * @param model  : 模拟器
   */
  template <typename T>
  void updateAdditionalInfo(DynamicsSimulator<T> &model) {
    if (_additionalInfoFirstVisit) {
      _nTotalGC = model.getTotalNumGC();
      _cp_touch.resize(_nTotalGC, false);
      _cp_pos.resize(_nTotalGC);
      _cp_force.resize(_nTotalGC);
      std::vector<double> tmp(3);
      for (size_t i(0); i < _nTotalGC; ++i) {
        _cp_pos[i] = tmp;
        _cp_force[i] = tmp;
      }
      _additionalInfoFirstVisit = false;
    }

    for (size_t i(0); i < _nTotalGC; ++i) {
      // TODO: check touch boolean
      _cp_touch[i] = true;
      for (size_t j(0); j < 3; ++j) {
        _cp_pos[i][j] = model.getModel()._pGC[i][j];
        _cp_force[i][j] = model.getContactForce(i)[j];
      }
    }
  }

  /*!
  *更新棋盘的位置以匹配无限碰撞平面
  *无限碰撞平面只指定方向，所以我们
   * @param model : 碰撞平面
   * @param id    : 创建棋盘时返回的id
   */
  template <typename T>
  void updateCheckerboardFromCollisionPlane(CollisionPlane<T> &model,
                                            size_t id) {
    // Mat4<T> H = sxformToHomogeneous(model.getLocation());
    _kinematicXform.at(id) = spatialTransformToQT(model.getLocation());
  }

  template <typename T>
  void updateCheckerboard(T height, size_t id) {
    QMatrix4x4 H;
    H.setToIdentity();
    H.translate(0., 0., height);
    _kinematicXform.at(id) = H;
  }

  template <typename T>
  void updateDebugSphereLocation(Vec3<T> &position, size_t id) {
    QMatrix4x4 H;
    H.setToIdentity();
    H.translate(position[0], position[1], position[2]);
    _kinematicXform.at(id) = H;
  }

  /*!
   * 用纯色填充颜色数据
   */
  static void setSolidColor(std::vector<float> &data, size_t size, float r,
                            float g, float b) {
    data.clear();
    data.resize(size);

    if ((size % 3) != 0) {
      throw std::runtime_error("setSolidColor invalid size");
    }

    for (size_t i = 0; i < size / 3; i++) {
      data[i * 3 + 0] = r;
      data[i * 3 + 1] = g;
      data[i * 3 + 2] = b;
    }
  }

/*获取函数*/
  const size_t &getTotalNumGC() { return _nTotalGC; }
  const std::vector<double> &getGCPos(size_t idx) { return _cp_pos[idx]; }
  const std::vector<double> &getGCForce(size_t idx) { return _cp_force[idx]; }
  const std::vector<BoxInfo> &getBoxInfoList() { return _box_list; }

  const DMat<double> &getHeightMap() { return _height_map; }
  const Vec3<double> &getHeightMapLeftCorner() {
    return _height_map_left_corner;
  }
  const double &getHeightMapMax() { return _height_map_max; }
  const double &getHeightMapMin() { return _height_map_min; }
  const double &getGridSize() { return _grid_size; }

  const Vec3<double> &getCameraOrigin() { return _cameraOrigin; }
  vectorAligned<SolidColor> _instanceColor;
  std::vector<QMatrix4x4> _kinematicXform;

 private:
  size_t _nUnique = 0, _nTotal = 0;
  std::vector<std::vector<float>> _vertexData;
  std::vector<std::vector<float>> _normalData;
  std::vector<std::vector<float>> _colorData;
  std::vector<u8> _canBeHidden;
  vectorAligned<Mat4<float>>
      _offsetXforms;  // these are NOT coordinate transformations!
  std::string _baseFileName = "../resources/";

  std::vector<size_t> _objectMap;

  std::vector<size_t> _glArrayOffsets;
  std::vector<size_t> _glArraySizes;

  std::vector<float> _glVertexData;
  std::vector<float> _glNormalData;
  std::vector<float> _glColorData;
  std::vector<ScrollInfo> _scrollIDs;

  std::vector<QMatrix4x4> _modelOffsets;

  bool _reloadNeeded = false;
  bool _additionalInfoFirstVisit = true;

  size_t _nTotalGC = 0;
  std::vector<bool> _cp_touch;
  std::vector<std::vector<double>> _cp_pos;
  std::vector<std::vector<double>> _cp_force;
  std::vector<BoxInfo> _box_list;

  double _grid_size;
  Vec3<double> _height_map_left_corner;
  DMat<double> _height_map;
  double _height_map_max, _height_map_min;

  Vec3<double> _cameraOrigin;

  size_t _cheetah3LoadIndex = 0, _miniCheetahLoadIndex = 0,
         _sphereLoadIndex = 0, _cubeLoadIndex = 0;
};

#endif  // PROJECT_DRAWLIST_H
