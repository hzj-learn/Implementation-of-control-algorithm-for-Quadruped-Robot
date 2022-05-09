#ifndef PROJECT_TYPES_H
#define PROJECT_TYPES_H

#include "cppTypes.h"

struct MasterConfig {            //结构体
  RobotType _robot;              //机器人的类型
  bool simulated = false;        //布尔型，不是0(非仿真)就是1（仿真）
};

#endif  // PROJECT_TYPES_H
