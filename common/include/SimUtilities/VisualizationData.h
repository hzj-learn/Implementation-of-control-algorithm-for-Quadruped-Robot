/*! @file GraphicsDebugData.h
 *  @brief 模拟器sim到可视化的数据
 */

#ifndef VISUALIZATION_DATA_H
#define VISUALIZATION_DATA_H

#define VISUALIZATION_MAX_PATH_POINTS 2000
#define VISUALIZATION_MAX_PATHS 10
#define VISUALIZATION_MAX_ITEMS 100
#define VISUALIZATION_MAX_CHEETAHS 0

#include "cppTypes.h"

//（1）球形的可视化
struct SphereVisualization 
{
  Vec3<float> position;
  Vec4<float> color;
  double radius;
};

//（2）长方体的可视化
struct BlockVisualization 
{
  Vec3<float> dimension;
  Vec3<float> corner_position;
  Vec3<float> rpy;
  Vec4<float> color;
};

//（3）箭头的可视化
struct ArrowVisualization 
{
  Vec3<float> base_position;
  Vec3<float> direction;
  Vec4<float> color;
  float head_width;
  float head_length;
  float shaft_width;
};

//（4）猎豹机器人的可视化
struct CheetahVisualization 
{
  Vec12<float> q;
  Quat<float> quat;
  Vec3<float> p;
  Vec4<float> color;
};

//（5）路径的可视化
struct PathVisualization 
{
  size_t num_points = 0;
  Vec4<float> color;
  Vec3<float> position[VISUALIZATION_MAX_PATH_POINTS];
  void clear() {
    num_points = 0;
  }
};

//（6）圆锥的可视化
struct ConeVisualization 
{
  Vec3<float> point_position;
  Vec3<float> direction;
  Vec4<float> color;
  double radius;
};

//可视化数据
struct VisualizationData 
{
  size_t num_paths = 0, num_arrows = 0, num_cones = 0, num_spheres = 0,
         num_blocks = 0;
  SphereVisualization spheres[VISUALIZATION_MAX_ITEMS];
  BlockVisualization blocks[VISUALIZATION_MAX_ITEMS];
  ArrowVisualization arrows[VISUALIZATION_MAX_ITEMS];
  ConeVisualization cones[VISUALIZATION_MAX_ITEMS];
  PathVisualization paths[VISUALIZATION_MAX_PATHS];

  void clear() {
    num_paths = 0, num_arrows = 0, num_cones = 0, num_spheres = 0, num_blocks = 0;
  }

  SphereVisualization* addSphere() 
  {
    if(num_spheres < VISUALIZATION_MAX_ITEMS) {
      return &spheres[num_spheres++];
    }
    return nullptr;
  }

  BlockVisualization* addBlock() 
  {
    if(num_blocks < VISUALIZATION_MAX_ITEMS) {
      return &blocks[num_blocks++];
    }
    return nullptr;
  }

  ArrowVisualization* addArrow() 
  {
    if(num_arrows < VISUALIZATION_MAX_ITEMS) {
      return &arrows[num_arrows++];
    }
    return nullptr;
  }

  ConeVisualization* addCone() 
  {
    if(num_cones < VISUALIZATION_MAX_ITEMS) {
      return &cones[num_cones++];
    }
    return nullptr;
  }

  PathVisualization* addPath() 
  {
    if(num_paths < VISUALIZATION_MAX_PATHS) {
      auto* path = &paths[num_paths++];
      path->clear();
      return path;
    }
    return nullptr;
  }
};

#endif