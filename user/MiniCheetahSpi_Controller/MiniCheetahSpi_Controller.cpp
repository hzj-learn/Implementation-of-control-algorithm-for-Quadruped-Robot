#include "MiniCheetahSpi_Controller.h"


/*!
 * 功能：运行SPI程序（发送数据）
 * MiniCheetahSpi控制器
 * 发送每个关节的期望位置、KP、KD三个参数
 */
void MiniCheetahSpi_Controller::runController() 
{
  /////////////////////*（1）SPI线程上锁，进入临界区*/////////////////////////////////////////////////
  _mutex.lock();     

  /////////////////////*（2）发布每个关节期望位置、KP增益反馈、Kd增益反馈*/////////////////////////////
  u32 idx = 0;
  for(u32 leg = 0; leg < 4; leg++) //四条腿
  {
    for(u32 joint = 0; joint < 3; joint++) //每条腿的三个关节
    {
      _legController->commands[leg].qDes[joint] = command.q_des[idx];               //关节期望位置
      _legController->commands[leg].kpJoint(joint,joint) = command.kp_joint[idx];   //关节KP增益反馈
      _legController->commands[leg].kdJoint(joint,joint) = command.kd_joint[idx];   //关节Kd增益反馈
      idx++;
    }
  }
  ////////////////////*（3）SPI线程解锁，退出临界区*/////////////////////////////////////////////////
  _mutex.unlock();  
}