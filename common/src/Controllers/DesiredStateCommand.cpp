#include "Controllers/DesiredStateCommand.h"

/*=========================== Gait Data（步态数据） ===============================*/
/**
 * 功能：躯干期望的COM轨迹置0函数（初始化时用）
 */
template <typename T>
void DesiredStateData<T>::zero() 
{
  stateDes = Vec12<T>::Zero();
  stateTrajDes = Eigen::Matrix<T, 12, 10>::Zero();
}


template struct DesiredStateData<double>;
template struct DesiredStateData<float>;


/**
 * 功能：躯干期望的COM轨迹生成【手柄目标命令+状态估计器当前状态】计算想要的状态轨迹（12个状态变量）
 */
template <typename T>
void DesiredStateCommand<T>::convertToStateCommands() 
{
  data.zero();    //（0）先把上次的期望状态轨迹数据清零

  //（1）设置前进线速度
  //     --用遥控手柄左上下操纵杆输入的模拟值限幅设置
  data.stateDes(6) =
      deadband(gamepadCommand->leftStickAnalog[1], minVelX, maxVelX);//遥控手柄左上下操纵杆最大或最小模拟数值死区限制

  //（2）设置横向线速度
  //     --用遥控手柄左的左右操纵杆输入的模拟值限幅设置
  data.stateDes(7) =
      deadband(gamepadCommand->leftStickAnalog[0], minVelY, maxVelY);//遥控手柄左左右操纵杆最大或最小模拟数值死区限制

  //（3）设置垂直线速度
  //     --默认没有垂直速度
  data.stateDes(8) = 0.0;

  //（4）设置躯干质心X位置
  //    --公式：躯干质心X位置=状态估计器对X位置的估计值+（手柄控制周期*手柄当前设置的躯干X位置）           解释：期望的x位置增量=手柄控制周期*手柄当前设置的躯干X位置，原理类似于比例控制器
  data.stateDes(0) = stateEstimate->position(0) + dt * data.stateDes(6);

  //（5）设置躯干质心Y位置
  //    --公式：躯干质心Y位置=状态估计器对Y位置的估计值+（手柄控制周期*手柄当前设置的躯干Y位置）           解释：期望的Y位置增量=手柄控制周期*手柄当前设置的躯干Y位置，原理类似于比例控制器
  data.stateDes(1) = stateEstimate->position(1) + dt * data.stateDes(7);

  //（6）设置Z高度位置
  //    --机器人的高度是给定的，默认是45CM
  data.stateDes(2) = 0.45;

  //（7）设置左右横滚角速度  
  //     --默认左右滚动速度为0，模型简化的时候假设左右滚动速度=0
  data.stateDes(9) = 0.0;

  //（8）设置前后俯仰角速度 
  //     --默认前后俯仰角速度为0，模型简化的时候假设前后倾斜速度=0
  data.stateDes(10) = 0.0;

  //（9）设置偏航角速度
  //    --通过手柄限幅设置当前躯干偏航速度
  data.stateDes(11) =
      deadband(gamepadCommand->rightStickAnalog[0], minTurnRate, maxTurnRate);

  //（10）设置左右横滚角度  
  //    --模型简化的时候假设左右滚动角度=0
  data.stateDes(3) = 0.0;

  //（11）设置前后俯仰角度
  //    --模型简化的时候假设前后俯仰角度=0
  data.stateDes(4) = 
     deadband(gamepadCommand->rightStickAnalog[1], minPitch, maxPitch);

  //（12）设置偏航角度 
  //   --公式：偏航角度Yaw=状态估计器对偏航角度Yaw的估计值+（手柄控制周期*手柄当前设置的躯干偏航角度Yaw）           解释：期望的偏航角度Yaw增量=手柄控制周期*手柄当前设置的躯干偏航角度Yaw，原理类似于比例控制器
  data.stateDes(5) = stateEstimate->rpy(2) + dt * data.stateDes(11);
}




/**
 *功能：设置12个关节电机的期望轨迹状态数据函数
 */
template <typename T>
void DesiredStateCommand<T>::desiredStateTrajectory(int N, Vec10<T> dtVec) 
{
  A = Mat12<T>::Zero();
  A(0, 0) = 1;
  A(1, 1) = 1;
  A(2, 2) = 1;
  A(3, 3) = 1;
  A(4, 4) = 1;
  A(5, 5) = 1;
  A(6, 6) = 1;
  A(7, 7) = 1;
  A(8, 8) = 1;
  A(9, 9) = 1;
  A(10, 10) = 1;
  A(11, 11) = 1;
  data.stateTrajDes.col(0) = data.stateDes;

  for (int k = 1; k < N; k++)   //设置12个关节电机的期望轨迹状态数据
  {
    A(0, 6) = dtVec(k - 1);
    A(1, 7) = dtVec(k - 1);
    A(2, 8) = dtVec(k - 1);
    A(3, 9) = dtVec(k - 1);
    A(4, 10) = dtVec(k - 1);
    A(5, 11) = dtVec(k - 1);
    data.stateTrajDes.col(k) = A * data.stateTrajDes.col(k - 1);
    for (int i = 0; i < 12; i++) //打印12个关节电机的期望轨迹状态数据
    {
      // std::cout << data.stateTrajDes(i, k) << " ";
    }
    // std::cout << std::endl;
  }
  // std::cout << std::endl;
}


/**
 *  功能：数值死区限制函数，用在遥控手柄摇杆上
 */
template <typename T>
float DesiredStateCommand<T>::deadband(float command, T minVal, T maxVal) 
{
  if (command < deadbandRegion && command > -deadbandRegion) //轨迹在合理范围
  {
    return 0.0;
  }
  else                                                       //轨迹在不合理范围
  {
    return (command / (2)) * (maxVal - minVal);//校正公式
  }
}


/**
 *  功能：实时打印手柄操作的状态指令
 *   最后面没有调用
 */
template <typename T>
void DesiredStateCommand<T>::printRawInfo() 
{
  //（1）增量打印迭代
  printIter++;
  // （2）按要求的频率打印
  if (printIter == printNum) 
  {
    std::cout << "[DESIRED STATE COMMAND] Printing Raw Gamepad Info...\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Button Start: " << gamepadCommand->start
              << " | Back: " << gamepadCommand->back << "\n";
    std::cout << "Button A: " << gamepadCommand->a
              << " | B: " << gamepadCommand->b << " | X: " << gamepadCommand->x
              << " | Y: " << gamepadCommand->y << "\n";
    std::cout << "Left Stick Button: " << gamepadCommand->leftStickButton
              << " | X: " << gamepadCommand->leftStickAnalog[0]
              << " | Y: " << gamepadCommand->leftStickAnalog[1] << "\n";
    std::cout << "Right Analog Button: " << gamepadCommand->rightStickButton
              << " | X: " << gamepadCommand->rightStickAnalog[0]
              << " | Y: " << gamepadCommand->rightStickAnalog[1] << "\n";
    std::cout << "Left Bumper: " << gamepadCommand->leftBumper
              << " | Trigger Switch: " << gamepadCommand->leftTriggerButton
              << " | Trigger Value: " << gamepadCommand->leftTriggerAnalog
              << "\n";
    std::cout << "Right Bumper: " << gamepadCommand->rightBumper
              << " | Trigger Switch: " << gamepadCommand->rightTriggerButton
              << " | Trigger Value: " << gamepadCommand->rightTriggerAnalog
              << "\n\n";
    std::cout << std::endl;

    //重置迭代计数器
    printIter = 0;
  }
}



/**
 * 功能：实时打印躯干轨迹的状态
 */
template <typename T>
void DesiredStateCommand<T>::printStateCommandInfo() 
{
  //增量打印迭代
  printIter++;

  //按要求的频率打印
  if (printIter == printNum) {
    std::cout << "[DESIRED STATE COMMAND] Printing State Command Info...\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Position X: " << data.stateDes(0)
              << " | Y: " << data.stateDes(1) << " | Z: " << data.stateDes(2)
              << "\n";
    std::cout << "Orientation Roll: " << data.stateDes(3)
              << " | Pitch: " << data.stateDes(4)
              << " | Yaw: " << data.stateDes(5) << "\n";
    std::cout << "Velocity X: " << data.stateDes(6)
              << " | Y: " << data.stateDes(7) << " | Z: " << data.stateDes(8)
              << "\n";
    std::cout << "Angular Velocity X: " << data.stateDes(9)
              << " | Y: " << data.stateDes(10) << " | Z: " << data.stateDes(11)
              << "\n";
    std::cout << std::endl;
    std::cout << std::endl;

    //重置迭代计数器
    printIter = 0;
  }
}

template class DesiredStateCommand<double>;
template class DesiredStateCommand<float>;