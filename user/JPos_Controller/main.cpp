/*!
 * @file main.cpp
 * @brief 关节控制器，摆动腿轨迹跟踪控制器
 */

#include <main_helper.h>
#include "JPos_Controller.hpp"

//启动关节控制器，摆动腿轨迹跟踪控制器线程
int main(int argc, char** argv) 
{
  main_helper(argc, argv, new JPos_Controller());
  return 0;
}
