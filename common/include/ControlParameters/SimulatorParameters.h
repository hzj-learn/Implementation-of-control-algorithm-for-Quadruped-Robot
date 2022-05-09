/*! @file SimulatorParameters.cpp
 *  @brief 各种模拟器参数的声明
 *  这个类包含模拟器的所有控制参数。
 *  在大多数情况下，模拟器只是将控制参数加载到
 *  simulator-defaults.ini这没问题
 */

#ifndef PROJECT_SIMULATORPARAMETERS_H
#define PROJECT_SIMULATORPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

#define SIMULATOR_DEFAULT_PARAMETERS "/simulator-defaults.yaml"
#define MINI_CHEETAH_DEFAULT_PARAMETERS "/mini-cheetah-defaults.yaml"
#define CHEETAH_3_DEFAULT_PARAMETERS "/cheetah-3-defaults.yaml"

class SimulatorControlParameters : public ControlParameters 
{
 public:
  SimulatorControlParameters()
      : ControlParameters("simulator-parameters"),
        INIT_PARAMETER(vectornav_imu_accelerometer_noise),//（1）IMU矢量导航加速度噪声
        INIT_PARAMETER(vectornav_imu_gyro_noise),         //（2）IMU矢量导航陀螺仪噪声
        INIT_PARAMETER(vectornav_imu_quat_noise),         //（3）IMU矢量导航四元数噪声
        INIT_PARAMETER(game_controller_deadband),         //（4）手柄控制死区
        INIT_PARAMETER(simulation_speed),                 //（5）仿真速度频率
        INIT_PARAMETER(simulation_paused),                //（6）仿真暂停
        INIT_PARAMETER(high_level_dt),                    //（7）高层控制周期
        INIT_PARAMETER(low_level_dt),                     //（8）底层层控制周期
        INIT_PARAMETER(dynamics_dt),                      //（9）动态控制周期
        INIT_PARAMETER(floor_kp),                         //（10）地面的反馈增益KP
        INIT_PARAMETER(floor_kd),                         //（11）地面的反馈增益KD
        INIT_PARAMETER(use_spring_damper),                //（12）使用弹簧减震器
        INIT_PARAMETER(sim_state_lcm),                    //（13）仿真状态LCM
        INIT_PARAMETER(sim_lcm_ttl),                      //（14）仿真状态TTL
        INIT_PARAMETER(go_home),                          //（15）初始状态设置
        INIT_PARAMETER(home_pos),                         //（16）初始位置
        INIT_PARAMETER(home_rpy),                         //（17）初始rpy
        INIT_PARAMETER(home_kp_lin),                      //（18）初始线性反馈增益KP
        INIT_PARAMETER(home_kd_lin),                      //（19）初始线性反馈增益KD
        INIT_PARAMETER(home_kp_ang),                      //（20）初始角度反馈增益KP
        INIT_PARAMETER(home_kd_ang) {}                    //（21）初始角度反馈增益KD

  DECLARE_PARAMETER(float, vectornav_imu_accelerometer_noise)
  DECLARE_PARAMETER(float, vectornav_imu_gyro_noise)
  DECLARE_PARAMETER(float, vectornav_imu_quat_noise)

  DECLARE_PARAMETER(float, game_controller_deadband)

  DECLARE_PARAMETER(double, simulation_speed)
  DECLARE_PARAMETER(s64, simulation_paused)
  DECLARE_PARAMETER(double, high_level_dt)
  DECLARE_PARAMETER(double, low_level_dt)
  DECLARE_PARAMETER(double, dynamics_dt)

  DECLARE_PARAMETER(double, floor_kp)
  DECLARE_PARAMETER(double, floor_kd)
  DECLARE_PARAMETER(s64, use_spring_damper)
  DECLARE_PARAMETER(s64, sim_state_lcm)
  DECLARE_PARAMETER(s64, sim_lcm_ttl)

  DECLARE_PARAMETER(s64, go_home)
  DECLARE_PARAMETER(Vec3<double>,home_pos)
  DECLARE_PARAMETER(Vec3<double>,home_rpy)
  DECLARE_PARAMETER(double, home_kp_lin)
  DECLARE_PARAMETER(double, home_kd_lin)
  DECLARE_PARAMETER(double, home_kp_ang)
  DECLARE_PARAMETER(double, home_kd_ang)

};

#endif  // PROJECT_SIMULATORPARAMETERS_H
