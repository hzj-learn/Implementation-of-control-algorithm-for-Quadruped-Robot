/*! @file LegController.h
 *  @brief 常用的腿部控制接口和腿部控制算法
 *   为小型猎豹和猎豹3型机器人实现低水平的腿部控制，
 *   所有的量都在“腿坐标系”中，与身体坐标系有相同的方向，但是移动了，使得0,0,0在ab/ad轴上(“髋坐标系”)。
 */

#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include "cppTypes.h"
#include "leg_control_command_lcmt.hpp"
#include "leg_control_data_lcmt.hpp"
#include "Dynamics/Quadruped.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/ti_boardcontrol.h"

/*!
 * 功能：从“upboard控制器”返回到每个关节电机的指令类
 */
template <typename T>
struct LegControllerCommand 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() 
  { zero(); }
  void zero();                                                        //将支腿命令归零函数
  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;  //定义关节前馈转矩指令、关节反馈馈力矩指令，关节期望角度指令,关节期望角速度指令、足端期望位置指令、足端期望速度指令
  Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;                 //定义直角坐标系下的kp反馈增益指令、直角坐标系下的kd反馈增益指令、关节电机的kp反馈增益指令，关节电机的kd反馈增益指令
};




/*!
 * 功能：从每个关节电机返回到“upboard控制器”的数据类
 */
template <typename T>
struct LegControllerData 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() 
  { zero(); }

  void zero();                            //（1）数据清零函数：把关节角度、关节角速度、足端位置、足端速度、雅可比矩阵、估计的关节力矩数据清零
  void setQuadruped(Quadruped<T>& quad)   //（2）设置四足机器人类型
  { quadruped = &quad; }

  Vec3<T> q, qd, p, v;                //关节角度 关节角速度 足端位置 足端速度 
  Mat3<T> J;                          //雅可比
  Vec3<T> tauEstimate;                //估计关节力矩
  Quadruped<T>* quadruped;            //机器人类型 cheetah3 or mini
};






/*!
 * 功能：四足4条腿的控制器类
 * 备注：适用于小型猎豹和猎豹3
 */
template <typename T>
class LegController 
{
 public:
  LegController(Quadruped<T>& quad) : _quadruped(quad) 
  {
    for (auto& data : datas) data.setQuadruped(_quadruped);//设置机器人类型
  }

  void zeroCommand();                                                           //（1）腿部控制命令清零函数
  void edampCommand(RobotType robot, T gain);                                   //（2）设置每个关节的kd\kp增益函数
  void updateData(const SpiData* spiData);                                      //（3）更新从通讯板的消息到“腿部数据upboard板函数
  void updateData(const TiBoardData* tiBoardData);                              //（4）更新从upboard板的消息到“腿部数据leg data”的函数
  
  void updateCommand(SpiCommand* spiCommand);                                   //（5）更新通讯板消息到upboard板“leg命令”的函数
  void updateCommand(TiBoardCommand* tiBoardCommand);                           //（6）更新upbroad板到通讯板的“leg命令”函数
  void setEnabled(bool enabled)                                                 //（7）设置腿部控制器使能
  { _legsEnabled = enabled; };                    
  void setLcm(leg_control_data_lcmt* data, leg_control_command_lcmt* command);  //（8）设置LCM数据和指令
  void setMaxTorqueCheetah3(T tau)                                              //（9）设置最大扭矩，这只对猎豹3有效
   { _maxTorque = tau; }

  LegControllerCommand<T> commands[4];
  LegControllerData<T> datas[4];
  Quadruped<T>& _quadruped;
  bool _legsEnabled = false;
  T _maxTorque = 0; 
  bool _zeroEncoders = false;
  u32 _calibrateEncoders = 0;
};

template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J, //（10）计算脚的位置及其雅可比
                                   Vec3<T>* p, int leg);

#endif  // PROJECT_LEGCONTROLLER_H
