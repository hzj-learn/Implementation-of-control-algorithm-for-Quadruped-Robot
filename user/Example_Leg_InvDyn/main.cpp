/*!
 * @file main.cpp
 * @brief 设置并启动腿部逆动力学控制器
 */


/*!
 * 功能：设置并启动腿部逆动力学控制器
 */
#include <main_helper.h>
#include "Leg_InvDyn_Controller.hpp"

int main(int argc, char** argv) 
{
  main_helper(argc, argv, new Leg_InvDyn_Controller());
  return 0;
}
