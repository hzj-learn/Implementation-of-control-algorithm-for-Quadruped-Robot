/*! @file RobotParameters.cpp
 *  @brief 各种机器人参数的声明
 *  此类包含机器人的所有控制参数。
 */

#ifndef PROJECT_ROBOTPARAMETERS_H
#define PROJECT_ROBOTPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class RobotControlParameters : public ControlParameters 
{
 public:
  RobotControlParameters()
      : ControlParameters("robot-parameters"),
        INIT_PARAMETER(myValue),                        //（1）自己的参数
        INIT_PARAMETER(control_mode),                   //（2）机器人控制模式
        INIT_PARAMETER(testValue),                      //（3）测试参数
        INIT_PARAMETER(controller_dt),                  //（4）控制周期
        INIT_PARAMETER(stand_kp_cartesian),             //（5）站立反馈增益KP
        INIT_PARAMETER(stand_kd_cartesian),             //（6）站立反馈增益KD
        INIT_PARAMETER(kpCOM),                          //（7）躯干质心轨迹反馈增益KP
        INIT_PARAMETER(kdCOM),                          //（8）躯干质心轨迹反馈增益KD
        INIT_PARAMETER(kpBase),                         //（9）浮基反馈增益KP
        INIT_PARAMETER(kdBase),                         //（10）浮基反馈增益KD
        INIT_PARAMETER(cheater_mode),                   //（11）机器人“检查”模式
        INIT_PARAMETER(imu_process_noise_position),     //（12）IMU过程位置噪声
        INIT_PARAMETER(imu_process_noise_velocity),     //（13）IMU过程速度噪声
        INIT_PARAMETER(foot_process_noise_position),    //（14）腿运动过程位置噪声
        INIT_PARAMETER(foot_sensor_noise_position),     //（15）腿运动电机传感器位置噪声
        INIT_PARAMETER(foot_sensor_noise_velocity),     //（16）腿运动电机传感器速度噪声
        INIT_PARAMETER(foot_height_sensor_noise) {}     //（17）腿运动电机传感器高度噪声

  DECLARE_PARAMETER(double, myValue)
  DECLARE_PARAMETER(double, control_mode)
  DECLARE_PARAMETER(double, testValue)
  DECLARE_PARAMETER(double, controller_dt)
  DECLARE_PARAMETER(Vec3<double>, stand_kp_cartesian)
  DECLARE_PARAMETER(Vec3<double>, stand_kd_cartesian)
  DECLARE_PARAMETER(Vec3<double>, kpCOM)
  DECLARE_PARAMETER(Vec3<double>, kdCOM)
  DECLARE_PARAMETER(Vec3<double>, kpBase)
  DECLARE_PARAMETER(Vec3<double>, kdBase)

  // state estimator
  DECLARE_PARAMETER(s64, cheater_mode)
  DECLARE_PARAMETER(double, imu_process_noise_position)
  DECLARE_PARAMETER(double, imu_process_noise_velocity)
  DECLARE_PARAMETER(double, foot_process_noise_position)
  DECLARE_PARAMETER(double, foot_sensor_noise_position)
  DECLARE_PARAMETER(double, foot_sensor_noise_velocity)
  DECLARE_PARAMETER(double, foot_height_sensor_noise)
};

#endif  // PROJECT_ROBOTPARAMETERS_H
