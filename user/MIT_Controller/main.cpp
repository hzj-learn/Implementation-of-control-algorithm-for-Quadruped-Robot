/*!
 * @file main.cpp
 * @brief WBC控制器的主要功能
 *main函数解析命令行参数并启动相应的驱动程序。
 */

#include <main_helper.h>
#include "MIT_Controller.hpp"

//功能：设置并运行给定的机器人控制器
//3代表猎豹3    m代表迷你猎豹”    s代表sim    r代表robot
int main(int argc, char** argv) 
{
  main_helper(argc, argv, new MIT_Controller());        //这个是继承main_helper的，用main_helper创建一个控制器（MIT_Controller）
  return 0;
}
