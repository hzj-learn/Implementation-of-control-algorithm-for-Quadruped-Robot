/*!
 * @file main.cpp
 * @brief  main函数解析命令行参数并启动相应的驱动程序。
 */

#include <cassert>
#include <iostream>

#include "HardwareBridge.h"
#include "SimulationBridge.h"
#include "main_helper.h"
#include "RobotController.h"

MasterConfig gMasterConfig;

/*!
 *功能： 打印描述robot程序的命令行标志的消息
    “用法：robot[robot id][sim or robot][来自文件的参数]”
    “此处机器人id:3代表猎豹3，m代表迷你猎豹”
    sim或robot:s代表sim，r代表robot
    param file:f用于从文件加载参数，l（或不加载）用于LCM
    此选项只能在robot模式下使用
 */
void printUsage() 
{
  printf(
      "Usage: robot [robot-id] [sim-or-robot] [parameters-from-file]\n"
      "\twhere robot-id:     3 for cheetah 3, m for mini-cheetah\n"
      "\t      sim-or-robot: s for sim, r for robot\n"
      "\t      param-file:   f for loading parameters from file, l (or nothing) for LCM\n"
      "                      this option can only be used in robot mode\n");
}


/*!
 * 功能：设置并运行给定的机器人控制器
 * 参数1：argc用于选择机器人
 * 参数2：argv[1][~]用于选择机器人，argv[2][~]用于选择是否是仿真，argv[2][~]用于选择是否加载参数文件
 * argv用于选择仿真还是robot
 * 注意：RobotController用于选择对应的控制器 
 * 步骤：
    (1)通过argv选择机器人类型
    (2)通过argv选择是否是仿真环境
    (3)通过argv选择是否加载文件参数
    (4)根据配置的仿真状态或者真实状态启动模拟器连接桥
    (5)运行run函数进行动作
  这里是主程序的入口
 */
int main_helper(int argc, char** argv, RobotController* ctrl) 
{
  /////////*（1）提示终端输入命令行操作，打印描述robot程序的命令行标志的消息*///////////
  
  if (argc != 3 && argc != 4) 
  {
    printUsage();     // 打印描述robot程序的命令行标志的消息
    return EXIT_FAILURE;
  }

  ////////////////////////////*（2）选择机器人类型*//////////////////////////////////
  if (argv[1][0] == '3')        //3代表猎豹3,先判断是不是猎豹3
  {
    gMasterConfig._robot = RobotType::CHEETAH_3;
  } 
  else if (argv[1][0] == 'm')   //m代表迷你猎豹，再判断是不是猎豹mini 
  {
    gMasterConfig._robot = RobotType::MINI_CHEETAH;
  } 
  else                          //既不是猎豹3又不是猎豹mini就报错
  {
    printUsage();
    return EXIT_FAILURE;
  }

////////////////////////////*（3）选择运行是仿真还是真实机器人*//////////////////////
  if (argv[2][0] == 's')              //s代表sim
  {
    gMasterConfig.simulated = true;
  } 
  else if (argv[2][0] == 'r')         //r代表robot
  {
    gMasterConfig.simulated = false;
  }
  else                                //既不是仿真，又不是sim就报错 
  {
    printUsage();
    return EXIT_FAILURE;
  }

  ////////////////////////////*（4）选择文件加载方式*//////////////////////////////
  if(argc == 4 && argv[3][0] == 'f')  //f用于从文件加载参数
  {
    gMasterConfig.load_from_file = true;
    printf("Load parameters from file\n");
  }
  else                                //不加载参数
   {
    gMasterConfig.load_from_file = false;
    printf("Load parameters from network\n");
   }
  printf("[Quadruped] Cheetah Software\n");
  printf("        Quadruped:  %s\n",
         gMasterConfig._robot == RobotType::MINI_CHEETAH ? "Mini Cheetah"
                                                         : "Cheetah 3");
  printf("        Driver: %s\n", gMasterConfig.simulated
                                     ? "Development Simulation Driver"
                                     : "Quadruped Driver");

///////////////////////////// /*根据上面选择的状态连接机器人合适的驱动（仿真驱动或者robot驱动）*///////////////
 /*机器人若选择的是仿真状态*/
  if (gMasterConfig.simulated) 
  {
    if(argc != 3)                                         //报错
    {
      printUsage();
      return EXIT_FAILURE;
    }
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH)  //仿真选择的是猎豹mini，仿真连接猎豹mini的驱动
    {
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);//设置模拟器连接桥
      simulationBridge.run();   //连接到仿真函数
      printf("[Quadruped] SimDriver run() has finished!\n");
    } 
    else if (gMasterConfig._robot == RobotType::CHEETAH_3)//仿真选择的是猎豹3，仿真连接猎豹3的驱动
    {
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);  //用SimulationBridge类定义一个simulationBridge的对象，根据上面设置的参数，加载猎豹mini的参数，设置模拟器连接桥
      simulationBridge.run();                                         //运行仿真桥驱动
    } 
    else                                                  //既不是选择猎豹3也不是选择猎豹mini，就没有对应的驱动，报错
    {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
  }

/*机器人在真实的运动状态*/
 else                          
  {
#ifdef linux
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH)    //仿真选择的是猎豹mini，仿真连接猎豹mini的驱动
     {
      MiniCheetahHardwareBridge hw(ctrl, gMasterConfig.load_from_file);//用MiniCheetahHardwareBridge类定义一个hw的对象，给定手柄数据和机器人参数定义 MiniCheetah硬件桥
      hw.run();                                                        //运行硬件桥驱动，运行的是MiniCheetahHardwareBridge类内的run
      printf("[Quadruped] SimDriver run() has finished!\n");
     }

    else if (gMasterConfig._robot == RobotType::CHEETAH_3)  //仿真选择的是猎豹3，仿真连接猎豹3的驱动
    {
      Cheetah3HardwareBridge hw(ctrl);;//用Cheetah3HardwareBridge类定义一个hw的变量
      hw.run();                        //运行硬件桥驱动，运行的是Cheetah3HardwareBridge类内的run
    }
     else                                                   //既不是选择猎豹3也不是选择猎豹mini，就没有对应的驱动，报错
    {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
#endif
 }
  return 0;
}
