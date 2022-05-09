/*! @file LegController.cpp
 *  @brief 公共腿部控制接口
 * 功能：迷你猎豹和猎豹3机器人实现低水平腿部控制
 * 消除通讯板和upboard之间的差异
 * 所有数量都在“支腿框架”中，其方向与身体框架，但会移动，使0,0,0位于ab/ad轴（“臀部框架”）
 */

#include <eigen3/Eigen/Dense>

#include "Controllers/LegController.h"

/*!
 * 功能：将支腿命令归零函数
 * 目的：腿就不会输出扭矩
 */
template <typename T>
void LegControllerCommand<T>::zero() 
{
  tauFeedForward = Vec3<T>::Zero();
  forceFeedForward = Vec3<T>::Zero();
  qDes = Vec3<T>::Zero();
  qdDes = Vec3<T>::Zero();
  pDes = Vec3<T>::Zero();
  vDes = Vec3<T>::Zero();
  kpCartesian = Mat3<T>::Zero();
  kdCartesian = Mat3<T>::Zero();
  kpJoint = Mat3<T>::Zero();
  kdJoint = Mat3<T>::Zero();
}

/*!
 * 功能：数据清零函数：把角度、角速度、足端位置、足端速度、雅可比矩阵、估计的力矩数据清零
 */
template <typename T>
void LegControllerData<T>::zero() {
  q = Vec3<T>::Zero();
  qd = Vec3<T>::Zero();
  p = Vec3<T>::Zero();
  v = Vec3<T>::Zero();
  J = Mat3<T>::Zero();
  tauEstimate = Vec3<T>::Zero();
}


/*!
 * 功能：将所有腿部命令归零函数，把角度、角速度、足端位置、足端速度、雅可比矩阵、估计的力矩命令清零
 * 备注：这应该在任何控制代码之前运行，所以如果控制代码混乱，没有改变leg命令，leg不会记得最后的命令
 */
template <typename T>
void LegController<T>::zeroCommand() 
{
  for (auto& cmd : commands) 
  {
    cmd.zero();   //把角度、角速度、足端位置、足端速度、雅可比矩阵、估计的力矩命令置零
  }
  _legsEnabled = false;
}


/*!
 * 功能：设置mini-cheetah或者CHEETAH_3每个关节的kd\kp增益函数
 * 备注：这将覆盖所有命令数据并生成使用给定增益的紧急阻尼指令
 * 对于迷你猎豹增益为N m/（rad/s），猎豹3的增益为N/m
 */
template <typename T>
void LegController<T>::edampCommand(RobotType robot, T gain) 
{
  zeroCommand();        //把角度、角速度、足端位置、足端速度、雅可比矩阵、估计的力矩命令清零
  if (robot == RobotType::CHEETAH_3)   //设置CHEETAH_3每个关节的kd增益
  {
    for (int leg = 0; leg < 4; leg++) 
    {
      for (int axis = 0; axis < 3; axis++) 
      {
        commands[leg].kdCartesian(axis, axis) = gain;
      }
    }
  } 
  else                                 //设置mini-cheetah每个关节的kd参数
  {  
    for (int leg = 0; leg < 4; leg++) 
    {
      for (int axis = 0; axis < 3; axis++) 
      {
        commands[leg].kdJoint(axis, axis) = gain;
      }
    }
  }
}


/*!
 * 功能：更新从通讯板的消息更新“腿部数据”函数
 * 备注：每个关节的更新
 * 参数：
 * (1)q:每个关节的角度
 * (2)qd每个关节的角速度
 * (3)J and p雅可比矩阵级腿的位置
 * (4)v 脚的线速度
 */
template <typename T>
void LegController<T>::updateData(const SpiData* spiData) 
{
  for (int leg = 0; leg < 4; leg++) 
  {
    //(1) q:每个关节的角度
    datas[leg].q(0) = spiData->q_abad[leg];
    datas[leg].q(1) = spiData->q_hip[leg];
    datas[leg].q(2) = spiData->q_knee[leg];

    //(2) qd每个关节的角速度
    datas[leg].qd(0) = spiData->qd_abad[leg];
    datas[leg].qd(1) = spiData->qd_hip[leg];
    datas[leg].qd(2) = spiData->qd_knee[leg];

    //(3) J and p雅可比矩阵及腿的位置
    computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J),
                                     &(datas[leg].p), leg);
    //(4) v 脚的线速度
    datas[leg].v = datas[leg].J * datas[leg].qd;
  }
}



/*!
 * 功能：更新从upboard板的消息到“leg data”的函数
 * 备注：每个关节的更新
 */
template <typename T>
void LegController<T>::updateData(const TiBoardData* tiBoardData) 
{
  for (int leg = 0; leg < 4; leg++)          //每条腿            
  {
    for (int joint = 0; joint < 3; joint++)  //每个关节
    {
      datas[leg].q(joint) = tiBoardData[leg].q[joint];                                          //（1）关节角度q
      datas[leg].qd(joint) = tiBoardData[leg].dq[joint];                                        //（2）关节角速度qd
      datas[leg].p(joint) = tiBoardData[leg].position[joint];                                   //（3）脚的位置p
      datas[leg].v(joint) = tiBoardData[leg].velocity[joint];                                   //（4）脚的速度v
      computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &datas[leg].J,nullptr, leg);   //（5）计算脚的位置及其雅可比
      datas[leg].tauEstimate[joint] = tiBoardData[leg].tau[joint];                              //（6）估计的关节力矩
    }
  }
}

/*!
 * 功能：更新通讯板板消息到“leg命令”的函数
 * 参数
 * (1)更新tauFF：关节转矩
 * (2)更新forceFF：力矩
 * (3)更新cartesian PD：直角坐标系下的kp\kd反馈增益
 * (4)更新转矩Torque
 * 
 * (5)更新设置力矩命令
 *（6）joint space kd：关节坐标系下的kd反馈增益
 *（7）joint space kp：关节坐标系下的kp反馈增益
 *（8）每个关节的角度
 *（9）每个关节的角速度
 *（10）估计扭矩，公式：估计扭矩=关节转矩+kp*(期望角度-真实的角度)+kd*(期望的角速度-真实的角速度)，  原理是前馈+PD控制器，前馈是用于补偿腿的重力惯性的
 *（11）腿部控制器使能
 */
template <typename T>
void LegController<T>::updateCommand(SpiCommand* spiCommand) 
{
  for (int leg = 0; leg < 4; leg++)     //四条腿
  {
  /*数据更新部分*/
    // (1)更新tauFF：关节转矩
    Vec3<T> legTorque = commands[leg].tauFeedForward;

    // (2)更新forceFF：力矩
    Vec3<T> footForce = commands[leg].forceFeedForward;

    // (3)更新cartesian PD：直角坐标系下的kp\kd反馈增益
    footForce +=
        commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
    footForce +=
        commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v);

    // (4)更新转矩Torque
    legTorque += datas[leg].J.transpose() * footForce;

/*计算设置命令部分*/
    // (5)设置设置力矩命令
    spiCommand->tau_abad_ff[leg] = legTorque(0);              //由躯干到脚底方向，第3个关节
    spiCommand->tau_hip_ff[leg] = legTorque(1);               //由躯干到脚底方向，第2个关节
    spiCommand->tau_knee_ff[leg] = legTorque(2);              //由躯干到脚底方向，第1个关节

    //（6）设置joint space kd：关节坐标系下的kd反馈增益命令
    spiCommand->kd_abad[leg] = commands[leg].kdJoint(0, 0);   //由躯干到脚底方向，第3个关节
    spiCommand->kd_hip[leg] = commands[leg].kdJoint(1, 1);    //由躯干到脚底方向，第2个关节
    spiCommand->kd_knee[leg] = commands[leg].kdJoint(2, 2);   //由躯干到脚底方向，第1个关节

    //（7）设置joint space kp：关节坐标系下的kp反馈增益命令
    spiCommand->kp_abad[leg] = commands[leg].kpJoint(0, 0);   //由躯干到脚底方向，第1个关节
    spiCommand->kp_hip[leg] = commands[leg].kpJoint(1, 1);    //由躯干到脚底方向，第1个关节
    spiCommand->kp_knee[leg] = commands[leg].kpJoint(2, 2);   //由躯干到脚底方向，第1个关节

    //（8）设置每个关节的角度命令
    spiCommand->q_des_abad[leg] = commands[leg].qDes(0);
    spiCommand->q_des_hip[leg] = commands[leg].qDes(1);
    spiCommand->q_des_knee[leg] = commands[leg].qDes(2);

    //（9）设置每个关节的角速度命令
    spiCommand->qd_des_abad[leg] = commands[leg].qdDes(0);
    spiCommand->qd_des_hip[leg] = commands[leg].qdDes(1);
    spiCommand->qd_des_knee[leg] = commands[leg].qdDes(2);

    //（10）设置估计扭矩命令，公式：估计扭矩=关节转矩+kp*(期望角度-真实的角度)+kd*(期望的角速度-真实的角速度)，  原理是前馈+PD控制器，前馈是用于补偿腿的重力惯性的
    datas[leg].tauEstimate =
        legTorque +
        commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
        commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);
    //（11）设置腿部控制器使能命令
    spiCommand->flags[leg] = _legsEnabled ? 1 : 0;        
  }
}



/*!
 * 功能：更新upbroad板的“leg命令”函数
 * 腿的部分
 * （1）计算每条腿的前馈力矩，前馈力矩=kp*(期望角度-实际角度)+kd*(期望角速度-实际角速度)
 * 关节的部分
 * （1）kp
 * （2）kd
 * （3）力矩
 * （4）脚的位置
 * （5）脚的速度
 * （6）关节的转矩
 */
template <typename T>
void LegController<T>::updateCommand(TiBoardCommand* tiBoardCommand) 
{
  for (int leg = 0; leg < 4; leg++) //每条腿
  {
    Vec3<T> tauFF = commands[leg].tauFeedForward.template cast<T>();          //定义前馈力矩向量
    tauFF += commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +    //（1）计算每条腿的前馈力矩，前馈力矩=kp*(期望角度-实际角度)+kd*(期望角速度-实际角速度)
             commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);

    for (int joint = 0; joint < 3; joint++)   //每个关节
    {
      tiBoardCommand[leg].kp[joint] = commands[leg].kpCartesian(joint, joint);    //（1）设置kp命令
      tiBoardCommand[leg].kd[joint] = commands[leg].kdCartesian(joint, joint);    //（2）设置kd命令
      tiBoardCommand[leg].tau_ff[joint] = tauFF[joint];                           //（3）设置力矩命令
      tiBoardCommand[leg].position_des[joint] = commands[leg].pDes[joint];        //（4）设置脚的期望位置命令
      tiBoardCommand[leg].velocity_des[joint] = commands[leg].vDes[joint];        //（5）设置脚的期望速度命令
      tiBoardCommand[leg].force_ff[joint] =                                       //（6）设置关节的转矩命令
          commands[leg].forceFeedForward[joint];
    }

    tiBoardCommand[leg].enable = _legsEnabled ? 1 : 0;                            //（1）腿控制器使能
    tiBoardCommand[leg].max_torque = _maxTorque;                                  //（2）最大转矩
  }
}


/*!
 * 功能：设置LCM数据和指令
 */
template<typename T>
void LegController<T>::setLcm(leg_control_data_lcmt *lcmData, leg_control_command_lcmt *lcmCommand) 
{
    for(int leg = 0; leg < 4; leg++)        //每条腿
    {
        for(int axis = 0; axis < 3; axis++) //每个关节
        {
          /*设置LCM数据*/
            int idx = leg*3 + axis;                                                   //每条腿、每个关节的递推逻辑换算
            lcmData->q[idx] = datas[leg].q[axis];                                     //(1)关节的角度数据
            lcmData->qd[idx] = datas[leg].qd[axis];                                   //(2)关节的角速度数据
            lcmData->p[idx] = datas[leg].p[axis];                                     //(3)脚的位置数据
            lcmData->v[idx] = datas[leg].v[axis];                                     //(4)脚的速度数据
            lcmData->tau_est[idx] = datas[leg].tauEstimate[axis];                     //(5)关节估计的力矩数据
          /*设置LCM命令*/
            lcmCommand->tau_ff[idx] = commands[leg].tauFeedForward[axis];             //(1)前馈转矩命令
            lcmCommand->f_ff[idx] = commands[leg].forceFeedForward[axis];             //(2)前馈力矩命令
            lcmCommand->q_des[idx] = commands[leg].qDes[axis];                        //(3)关节的角度命令
            lcmCommand->qd_des[idx] = commands[leg].qdDes[axis];                      //(4)关节的角速度命令
            lcmCommand->p_des[idx] = commands[leg].pDes[axis];                        //(5)脚的位置命令
            lcmCommand->v_des[idx] = commands[leg].vDes[axis];                        //(6)脚的速度命令
            lcmCommand->kp_cartesian[idx] = commands[leg].kpCartesian(axis, axis);    //(7)直角坐标系kp命令
            lcmCommand->kd_cartesian[idx] = commands[leg].kdCartesian(axis, axis);    //(8)直角坐标系kd命令
            lcmCommand->kp_joint[idx] = commands[leg].kpJoint(axis, axis);            //(9)关节kp命令
            lcmCommand->kd_joint[idx] = commands[leg].kdJoint(axis, axis);            //(10)关节kd命令
        }
    }
}

template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;


/*!
 * 功能：计算足端的位置及其雅可比。
 * 目的：这是在本地完成的腿坐标系。如果J/p为空，则跳过计算。
 */
template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg) 
{
  T l1 = quad._abadLinkLength;        //从躯干到脚底，腿部第一个连杆长度L1
  T l2 = quad._hipLinkLength;         //从躯干到脚底，腿部第二个连杆长度L2
  T l3 = quad._kneeLinkLength;        //从躯干到脚底，腿部第三个连杆长度L3
  T sideSign = quad.getSideSign(leg); 

  T s1 = std::sin(q(0));    //第一个关节的sin值
  T s2 = std::sin(q(1));    //第二个关节的sin值
  T s3 = std::sin(q(2));    //第三个关节的sin值

  T c1 = std::cos(q(0));    //第一个关节的cos值   
  T c2 = std::cos(q(1));    //第二个关节的cos值
  T c3 = std::cos(q(2));    //第三个关节的cos值

  T c23 = c2 * c3 - s2 * s3;//c23=cos(q(1))*cos(q(2))-sin(q(1))*sin(q(2))
  T s23 = s2 * c3 + c2 * s3;//s23=sin(q(1))*cos(q(2))+cos(q(1))*sin(q(2))

  if (J) //若J不为空
  {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - l1 * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + l1 * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p) //若P不为空
  {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) = l1 * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = l1 * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
}

//双精度浮点型
template void computeLegJacobianAndPosition<double>(Quadruped<double>& quad,
                                                    Vec3<double>& q,
                                                    Mat3<double>* J,
                                                    Vec3<double>* p, int leg);
//单精度浮点型
template void computeLegJacobianAndPosition<float>(Quadruped<float>& quad,
                                                   Vec3<float>& q,
                                                   Mat3<float>* J,
                                                   Vec3<float>* p, int leg);