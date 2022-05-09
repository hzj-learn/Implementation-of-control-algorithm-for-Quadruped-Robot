/**
*在计算控制迭代。打印出哪个命令不安全。每个状态都有启用对其关心的命令进行检查的选项。
 */

#include "SafetyChecker.h"

/**
 * 功能：检查躯干姿态是否安全
 * @return safePDesFoot如果需要安全放置脚，则为true
 */
template <typename T>
bool SafetyChecker<T>::checkSafeOrientation() 
{
  if (abs(data->_stateEstimator->getResult().rpy(0)) >= 0.5 ||    //rpy角度大于0.5是危险的
      abs(data->_stateEstimator->getResult().rpy(1)) >= 0.5) 
  {
    printf("Orientation safety check failed!\n");
    return false;
  } 
  else                                                            //rpy角度小于0.5是危险的
  {
    return true;
  }
}



/**
 * 功能：检查足端的位置输出是否安全
 * @return safePDesFoot如果需要安全放置脚，则为true
 */
template <typename T>
bool SafetyChecker<T>::checkPDesFoot() 
{
 bool safePDesFoot = true;  //假设可以安全启动 
  //安全参数 
  T maxAngle = 1.0472;                                          //最大安全角度60度（应更改）
  T maxPDes = data->_quadruped->_maxLegLength * sin(maxAngle);  //迈腿最大位置

  for (int leg = 0; leg < 4; leg++) //检查所有的腿
  {
    if (data->_legController->commands[leg].pDes(0) > maxPDes)    //保持脚在+x内远离身体
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(0)
                << " | modified: " << maxPDes << std::endl;
      data->_legController->commands[leg].pDes(0) = maxPDes;   //正位置限幅
      safePDesFoot = false;                                    //不安全启动了
    }
    if (data->_legController->commands[leg].pDes(0) < -maxPDes)   //保持脚在-x中远离身体
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(0)
                << " | modified: " << -maxPDes << std::endl;
      data->_legController->commands[leg].pDes(0) = -maxPDes; //负位置限幅
      safePDesFoot = false;                                   //不安全启动了
    }

    if (data->_legController->commands[leg].pDes(1) > maxPDes)      //保持脚在+y方向远离身体
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(1)
                << " | modified: " << maxPDes << std::endl;
      data->_legController->commands[leg].pDes(1) = maxPDes;
      safePDesFoot = false;
    }
    if (data->_legController->commands[leg].pDes(1) < -maxPDes)     //不要让脚离身体太远
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(1)
                << " | modified: " << -maxPDes << std::endl;
      data->_legController->commands[leg].pDes(1) = -maxPDes;//负位置限幅
      safePDesFoot = false;                                  //不安全启动了
    }

    if (data->_legController->commands[leg].pDes(2) >-data->_quadruped->_maxLegLength / 4) //将腿放在电机模块下方（不要抬起身体上方或撞击模块）
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(2)
                << " | modified: " << -data->_quadruped->_maxLegLength / 4
                << std::endl;
      data->_legController->commands[leg].pDes(2) =
          -data->_quadruped->_maxLegLength / 4;
      safePDesFoot = false;                                   //不安全启动了
    }
    if (data->_legController->commands[leg].pDes(2) <-data->_quadruped->_maxLegLength)     //将脚保持在运动限制范围内 
    {
      std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].pDes(2)
                << " | modified: " << -data->_quadruped->_maxLegLength
                << std::endl;
      data->_legController->commands[leg].pDes(2) =
          -data->_quadruped->_maxLegLength;
      safePDesFoot = false;                                   //不安全启动了
    }
  }
  return safePDesFoot;    //如果所有需要的位置都是安全的，则返回true
}



/**
 * 功能：检查足端的垂直力、侧向力是否安全
 * @return safePDesFoot如果需要安全放置脚，则为true
 */
template <typename T>
bool SafetyChecker<T>::checkForceFeedForward() 
{
  bool safeForceFeedForward = true;  //假设可以安全启动

  T maxLateralForce = 0;   //初始化最大垂直力
  T maxVerticalForce = 0;  //初始化最大侧向力

  if (data->_quadruped->_robotType == RobotType::CHEETAH_3)          //CHEETAH_3机器人的最大力限制
  {
    maxLateralForce = 1800;
    maxVerticalForce = 1800;
  } 
  else if (data->_quadruped->_robotType == RobotType::MINI_CHEETAH)  //MINI_CHEETAH机器人的最大力限制
  {
    maxLateralForce = 350;
    maxVerticalForce = 350;
  }

  for (int leg = 0; leg < 4; leg++)   //检查所有的腿
  {
    if (data->_legController->commands[leg].forceFeedForward(0) > maxLateralForce)     //限制+x车身框架中的侧向力限制
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(0)
                << " | modified: " << maxLateralForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(0) = maxLateralForce;
      safeForceFeedForward = false;
    }
    if (data->_legController->commands[leg].forceFeedForward(0) <-maxLateralForce)     //限制-x车身框架的侧向力限制
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(0)
                << " | modified: " << -maxLateralForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(0) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }
    if (data->_legController->commands[leg].forceFeedForward(1) > maxLateralForce)     //限制+y车身框架中的侧向力限制
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(1)
                << " | modified: " << maxLateralForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(1) = maxLateralForce;
      safeForceFeedForward = false;
    }
    if (data->_legController->commands[leg].forceFeedForward(1) < -maxLateralForce)    //限制-y车身骨架的侧向力限制
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(1)
                << " | modified: " << -maxLateralForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(1) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }
    if (data->_legController->commands[leg].forceFeedForward(2) > maxVerticalForce)    //限制+z身体框架中的垂直力限制
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(2)
                << " | modified: " << -maxVerticalForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(2) =
          maxVerticalForce;
      safeForceFeedForward = false;
    }
    if (data->_legController->commands[leg].forceFeedForward(2) <-maxVerticalForce)    //限制-z车身框架中的垂直力限制
     {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->commands[leg].forceFeedForward(2)
                << " | modified: " << maxVerticalForce << std::endl;
      data->_legController->commands[leg].forceFeedForward(2) =
          -maxVerticalForce;
      safeForceFeedForward = false;
    }
  }
  return safeForceFeedForward;   //如果所有前馈力都安全，则返回true
}

//模板类SafetyChecker<double>；这应该被修复。。。需要机器人运行模板
template class SafetyChecker<float>;
