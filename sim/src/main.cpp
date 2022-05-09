/*! @file main.cpp
 *  @brief Main function for simulator
 */

#include "Collision/CollisionPlane.h"
#include "DrawList.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/DynamicsSimulator.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/Quadruped.h"
#include "Graphics3D.h"
#include "SimControlPanel.h"
#include "Simulation.h"
#include "Utilities/utilities.h"
#include "Utilities/SegfaultHandler.h"

#include <QApplication>
#include <QSurfaceFormat>

#include <stdio.h>
#include <unistd.h>
#include <thread>

/*!
 *设置QT并运行模拟
 */
int main(int argc, char *argv[]) 
{
//（1）不安装安全检测程序
  install_segfault_handler(nullptr);
//（2）使用Qt库函数设置参数a
  QApplication a(argc, argv);
//（3）显示开放模拟器用户界面
  SimControlPanel panel;
  panel.show();
//（4）运行Qt程序，槽函数跳转到运行后台程序
  a.exec();
  return 0;
}
