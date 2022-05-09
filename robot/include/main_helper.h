/*!
 * @file main_helper.h
 * @brief 该函数应该在main中调用以启动机器人控制代码
 */

#ifndef ROBOT_MAIN_H
#define ROBOT_MAIN_H

#include "Types.h"
#include "RobotController.h"

extern MasterConfig gMasterConfig;
int main_helper(int argc, char** argv, RobotController* ctrl);      //设置并运行给定的机器人控制器

#endif  // ROBOT_MAIN_H
