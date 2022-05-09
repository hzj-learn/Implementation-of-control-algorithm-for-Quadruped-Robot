#ifndef SAFETY_CHECKER_H
#define SAFETY_CHECKER_H

#include <iostream>
#include "ControlFSMData.h"//包含所有与控件相关的数据

/**
 * SafetyChecker处理ControlFSM请求的检查。
 */
template <typename T>
class SafetyChecker 
{
 public:
  SafetyChecker(ControlFSMData<T>* dataIn) : data(dataIn){};
  // 预先检查以确保控件可以安全运行
  bool checkSafeOrientation(); //机器人的方位控制是安全的
  //后检查以确保控制可以发送到机器人
  bool checkPDesFoot();          //所需的脚位置不太远
  bool checkForceFeedForward();  //期望的前馈力不是太大
  ControlFSMData<T>* data;      //存储来自ControlFSM的数据
 private:
};

#endif  // SAFETY_CHECKER_H