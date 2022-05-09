/*
参考文献：
   [R1] M. Focchi, A. del Prete, I. Havoutis, R. Featherstone, D. G. Caldwell,
and C. Semini. High-slope terrain locomotion for torque-controlled quadruped
robots. Autonomous Robots, 2016.

   [R2] R. M. Murray, S. S. Sastry, and L. Zexiang. A Mathematical Introduction
to Robotic Manipulation. CRC Press, Inc., Boca Raton, FL, USA, 1st edition,
1994.

Cheetah-3-Documentation-Control:
   [C1] balanceController.pdf

   qpOASES variables are terminated with _qpOASES
*/

#include "BalanceController.hpp"
#include <iostream>

using namespace std;



/**
 * 功能：平衡控制器
 * 备注：看来接下来的5条线并没有起到任何作用，我必须在OptimizationThread类中设置打印级别
 */
BalanceController::BalanceController()
    : QProblemObj_qpOASES(NUM_VARIABLES_QP, NUM_CONSTRAINTS_QP) 
{
  Options options;
  options.printLevel = PL_NONE;
  QProblemObj_qpOASES.setOptions(options);
  QProblemObj_qpOASES.setPrintLevel(PL_NONE);
  QPFinished = false;

  lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");
  if (lcm->good()) //LCM成功
  {
    printf("LCM IN BALANCE CONTROL INITIALIZED\n");
  } 
  else             //LCM失败
  {
    printf("LCM IN BALANCE CONTROLLER FAILED\n");
    exit(-1);
  }

  // Eigen库计算QP矩阵 
  H_eigen.resize(NUM_VARIABLES_QP, NUM_VARIABLES_QP);
  A_eigen.resize(NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
  g_eigen.resize(NUM_VARIABLES_QP, 1);
  xOpt_eigen.resize(NUM_VARIABLES_QP, 1);
  yOpt_eigen.resize(NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP, 1);

  mass = 41;
  inertia = 0.01;

  /* 模型和世界参数及力限 */
  Ig.resize(3, 3);
  Ig.setIdentity();
  Ig = inertia * Ig;
  gravity.resize(3, 1);

  direction_normal_flatGround.resize(3, 1);
  direction_tangential_flatGround.resize(3, 1);

  /* 对地面上的所有脚进行初始化 */
  contact_state.resize(4, 1);
  contact_state << 1, 1, 1, 1;

  minNormalForces_feet.resize(4, 1);
  maxNormalForces_feet.resize(4, 1);

  /* 实际运动学 */
  x_COM_world.resize(3, 1);
  xdot_COM_world.resize(3, 1);
  omega_b_world.resize(3, 1);
  quat_b_world.resize(4, 1);
  R_b_world.resize(3, 3);
  p_feet.resize(3, 4);

  R_yaw_act.resize(3, 3);
  R_yaw_act.setIdentity();

  /* 期望运动学 */
  x_COM_world_desired.resize(3, 1);
  xdot_COM_world_desired.resize(3, 1);
  xddot_COM_world_desired.resize(3, 1);
  omega_b_world_desired.resize(3, 1);
  omegadot_b_world_desired.resize(3, 1);
  R_b_world_desired.resize(3, 3);
  orientation_error.resize(3, 1);

  error_x_rotated.setZero(3);
  error_dx_rotated.setZero(3);
  error_theta_rotated.setZero(3);
  error_dtheta_rotated.setZero(3);

  vbd_command_eigen.resize(3, 1);
  vbd_command_eigen.setZero();

  /* 临时内部矩阵 */
  omegaHat.resize(3, 3);
  tempSkewMatrix3.resize(3, 3);
  tempVector3.resize(3, 1);

  tempSkewMatrix3.setZero();
  tempVector3.setZero();

  A_control.resize(6, 3 * NUM_CONTACT_POINTS);
  b_control.resize(6, 1);
  b_control_Opt.resize(6, 1);
  S_control.resize(6, 6);
  W_control.resize(NUM_VARIABLES_QP, NUM_VARIABLES_QP);
  C_control.resize(NUM_CONSTRAINTS_QP, 3 * NUM_CONTACT_POINTS);

  C_times_f_control.resize(NUM_CONSTRAINTS_QP, 1);

  C_control.setZero();
  xOptPrev.setZero(12);
  yOptPrev.setZero(NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP);

  for (int i = 0; i < NUM_VARIABLES_QP; i++) {
    xOpt_qpOASES[i] = 0.0;
    xOpt_initialGuess[i] = 0.0;
  }

  xOpt_initialGuess[2] = 100;
  xOpt_initialGuess[5] = 100;
  xOpt_initialGuess[8] = 100;
  xOpt_initialGuess[11] = 100;

  for (int i = 0; i < NUM_VARIABLES_QP + NUM_CONSTRAINTS_QP; i++) {
    yOpt_qpOASES[i] = 0.0;
  }


  set_QPWeights();
  set_RobotLimits();
  set_worldData();

  x_COM_world_desired << -0.14, 0.0, 0.57;
  xdot_COM_world_desired << 0, 0, 0;
  omega_b_world_desired << 0, 0, 0;
  R_b_world_desired.setIdentity();

  Kp_COMx = 0;
  Kp_COMy = 0;
  Kp_COMz = 0;

  Kd_COMx = 0;
  Kd_COMy = 0;
  Kd_COMz = 0;

  Kp_Base_roll = 0;
  Kp_Base_pitch = 0;
  Kp_Base_yaw = 0;

  Kd_Base_roll = 0;
  Kd_Base_pitch = 0;
  Kd_Base_yaw = 0;

  yaw_act = 0;

  cpu_time = 0.001;
  cpu_time_fixed = 0.001;
  qp_exit_flag = -1.0;

  qp_not_init = 1.0;
}



void BalanceController::testFunction() 
{ printf("testfun "); }



/* ---------------主要接口  ------------------- */

/**
 * 功能：计算QP问题数据
 */
void BalanceController::updateProblemData(double* xfb_in, double* p_feet_in,
                                          double* p_des, double* p_act,
                                          double* v_des, double* v_act,
                                          double* O_err, double yaw_act_in) 
{
  // 打开输入
  copy_Array_to_Eigen(quat_b_world, xfb_in, 4, 0);
  copy_Array_to_Eigen(x_COM_world, xfb_in, 3, 4);
  copy_Array_to_Eigen(omega_b_world, xfb_in, 3, 7);
  copy_Array_to_Eigen(xdot_COM_world, xfb_in, 3, 10);
  copy_Array_to_Eigen(p_feet, p_feet_in, 12, 0);

  yaw_act = yaw_act_in;
  R_yaw_act.setZero();
  R_yaw_act(0, 0) = cos(yaw_act);
  R_yaw_act(0, 1) = -sin(yaw_act);
  R_yaw_act(1, 0) = sin(yaw_act);
  R_yaw_act(1, 1) = cos(yaw_act);
  R_yaw_act(2, 2) = 1;
  quaternion_to_rotationMatrix(R_b_world, quat_b_world);//四元数的旋转矩阵
  calc_PDcontrol();           //计算控制器矩阵。必须在计算H_qp和g_qp之前调用它们 
  update_A_control();

  //计算QP问题数据
  calc_H_qpOASES();
  calc_A_qpOASES();
  calc_g_qpOASES();

  update_log_variables(p_des, p_act, v_des, v_act, O_err);

  cpu_time = cpu_time_fixed;
  nWSR_qpOASES = nWSR_fixed;
}


/**
 * 功能：设置接触信息数据
 */
void BalanceController::SetContactData(double* contact_state_in,
                                       double* min_forces_in,
                                       double* max_forces_in) 
{
  //打开输入
  copy_Array_to_Eigen(contact_state, contact_state_in, 4, 0);
  copy_Array_to_Eigen(minNormalForces_feet, min_forces_in, 4, 0);
  copy_Array_to_Eigen(maxNormalForces_feet, max_forces_in, 4, 0);

  calc_lb_ub_qpOASES();
  calc_lbA_ubA_qpOASES();
}


/**
 * 功能：设置期望摆动的位置
 */
void BalanceController::set_desired_swing_pos(double* pFeet_des_in) 
{
  for (int i = 0; i < 12; i++) 
  {
    qp_controller_data.pfeet_des[i] = pFeet_des_in[i];
  }
}


/**
 * 功能：设置实际摆动的位置
 */
void BalanceController::set_actual_swing_pos(double* pFeet_act_in) 
{
  for (int i = 0; i < 12; i++) 
  {
    qp_controller_data.pfeet_act[i] = pFeet_act_in[i];
  }
}


/**
 * 功能：解算QP的非线性
 */
void BalanceController::solveQP_nonThreaded(double* xOpt) 
{
  // &cpu_time
  if (qp_not_init == 1.0) 
  {
    qp_exit_flag = QProblemObj_qpOASES.init(
        H_qpOASES, g_qpOASES, A_qpOASES, lb_qpOASES, ub_qpOASES, lbA_qpOASES,
        ubA_qpOASES, nWSR_qpOASES, &cpu_time, xOpt_initialGuess);
    qp_not_init = 0.0;

    nWSR_initial = nWSR_qpOASES;
    cpu_time_initial = cpu_time;
  } 
  else 
  {
    qp_exit_flag = QProblemObj_qpOASES.init(
        H_qpOASES, g_qpOASES, A_qpOASES, lb_qpOASES, ub_qpOASES, lbA_qpOASES,
        ubA_qpOASES, nWSR_qpOASES, &cpu_time, xOpt_qpOASES, yOpt_qpOASES,
        &guessedBounds, &guessedConstraints);
  }

  QProblemObj_qpOASES.getPrimalSolution(xOpt_qpOASES);
  QProblemObj_qpOASES.getDualSolution(yOpt_qpOASES);

  QProblemObj_qpOASES.getBounds(guessedBounds);
  QProblemObj_qpOASES.getConstraints(guessedConstraints);

  qp_controller_data.exit_flag = qp_exit_flag;
  qp_controller_data.nWSR = nWSR_qpOASES;
  qp_controller_data.cpu_time_microseconds = cpu_time * 1.0e6;

  copy_real_t_to_Eigen(xOpt_eigen, xOpt_qpOASES, 12);

  b_control_Opt = A_control * xOpt_eigen;

  for (int i = 0; i < 6; i++) 
  {
    qp_controller_data.b_control_Opt[i] = b_control_Opt(i);
  }

  xOptPrev = xOpt_eigen;

  for (int i = 0; i < NUM_CONTACT_POINTS; i++) 
  {
    tempVector3 = -R_b_world.transpose() * xOpt_eigen.segment(3 * i, 3);
    xOpt[3 * i] = tempVector3(0);
    xOpt[3 * i + 1] = tempVector3(1);
    xOpt[3 * i + 2] = tempVector3(2);
  }
  QProblemObj_qpOASES.reset();

  calc_constraint_check();
  qp_controller_data_publish = qp_controller_data;
}


/**
 * 功能：验证模型
 */
void BalanceController::verifyModel(double* vbd_command) 
{
  vbd_command_eigen =
      (1.0 / mass) * A_control.block<3, 12>(0, 0) * xOpt_eigen - gravity;
  copy_Eigen_to_double(vbd_command, vbd_command_eigen, 3);
}




/* --------------- Control Math ------------ */

/**
 * 功能：计算偏航旋转坐标的误差
 */
void BalanceController::calc_PDcontrol() 
{
  error_x_rotated = R_yaw_act.transpose() * (x_COM_world_desired - x_COM_world);
  error_dx_rotated =
      R_yaw_act.transpose() * (xdot_COM_world_desired - xdot_COM_world);
  matrixLogRot(R_yaw_act.transpose() * R_b_world_desired *
                   R_b_world.transpose() * R_yaw_act,
               orientation_error);
  error_dtheta_rotated =
      R_yaw_act.transpose() * (omega_b_world_desired - omega_b_world);

  xddot_COM_world_desired(0) +=
      Kp_COMx * error_x_rotated(0) + Kd_COMx * error_dx_rotated(0);
  xddot_COM_world_desired(1) +=
      Kp_COMy * error_x_rotated(1) + Kd_COMy * error_dx_rotated(1);
  xddot_COM_world_desired(2) +=
      Kp_COMz * error_x_rotated(2) + Kd_COMz * error_dx_rotated(2);

  omegadot_b_world_desired(0) = Kp_Base_roll * orientation_error(0) +
                                Kd_Base_roll * error_dtheta_rotated(0);
  omegadot_b_world_desired(1) = Kp_Base_pitch * orientation_error(1) +
                                Kd_Base_pitch * error_dtheta_rotated(1);
  omegadot_b_world_desired(2) = Kp_Base_yaw * orientation_error(2) +
                                Kd_Base_yaw * error_dtheta_rotated(2);

  //使用[R1]的（4）和[R2]的命题2.5计算方位误差 
  Ig << .35, 0, 0, 0, 2.1, 0, 0, 0, 2.1;
  MatrixXd II = R_yaw_act.transpose() * R_b_world * Ig * R_b_world.transpose() *
                R_yaw_act;

  //见方程式（5）的RHS，[R1]
  b_control << mass * (xddot_COM_world_desired + gravity),
      II * omegadot_b_world_desired;
}


/**
 * 功能：计算限制检查
 */
void BalanceController::calc_constraint_check() 
{
  C_times_f_control = C_control * xOpt_eigen;

  for (int i = 0; i < NUM_VARIABLES_QP; i++) 
  {
    qp_controller_data.xOpt[i] = xOpt_eigen(i);
  }

  for (int i = 0; i < NUM_CONSTRAINTS_QP; i++) 
  {
    qp_controller_data.lbA[i] = lbA_qpOASES[i];
    qp_controller_data.ubA[i] = ubA_qpOASES[i];
    qp_controller_data.C_times_f[i] = C_times_f_control(i);
  }

  for (int i = 0; i < 6; i++) 
  {
    qp_controller_data.b_control[i] = b_control(i);
  }
}




/* --------------- QP Matrices and Problem DataQP   矩阵和问题数据 ------------ */

/**
 * 功能：更新控制器符号A*f=b中的A矩阵
 */
void BalanceController::update_A_control() 
{
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) 
  {
    A_control.block<3, 3>(0, 3 * i) << R_yaw_act.transpose();
    tempVector3 << contact_state(i) * p_feet.col(i);
    crossMatrix(tempSkewMatrix3, tempVector3);
    A_control.block<3, 3>(3, 3 * i) << R_yaw_act.transpose() * tempSkewMatrix3;
  }
}


/**
 * 功能：使用A矩阵计算QP成本矩阵H
 */
void BalanceController::calc_H_qpOASES() 
{
  H_eigen = 2 * (A_control.transpose() * S_control * A_control +
                 (alpha_control + 1e-3) * W_control);
  //复制到实数组（qpOASES数据类型）
  copy_Eigen_to_real_t(H_qpOASES, H_eigen, NUM_VARIABLES_QP, NUM_VARIABLES_QP);
}


/**
 * 功能：计算QPA矩阵
 */
void BalanceController::calc_A_qpOASES() 
{
  Eigen::Vector3d t1x;
  t1x << 1, 0, 0;
  Eigen::Vector3d t2y;
  t2y << 0, 1, 0;
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) 
  {
    C_control.block<1, 3>(5 * i + 0, 3 * i)
        << -mu_friction * direction_normal_flatGround.transpose() +
               t1x.transpose();
    C_control.block<1, 3>(5 * i + 1, 3 * i)
        << -mu_friction * direction_normal_flatGround.transpose() +
               t2y.transpose();
    C_control.block<1, 3>(5 * i + 2, 3 * i)
        << mu_friction * direction_normal_flatGround.transpose() +
               t2y.transpose();
    C_control.block<1, 3>(5 * i + 3, 3 * i)
        << mu_friction * direction_normal_flatGround.transpose() +
               t1x.transpose();
    C_control.block<1, 3>(5 * i + 4, 3 * i)
        << direction_normal_flatGround.transpose();
  }

  if (use_hard_constraint_pitch == 1) 
  {
    C_control.row(NUM_CONSTRAINTS_QP - 1) =
        A_control.row(5 - 1);  // 在俯仰控制上添加硬约束
  }
  else 
  {
    C_control.row(NUM_CONSTRAINTS_QP - 1) = 0 * A_control.row(5 - 1);
  }

  copy_Eigen_to_real_t(A_qpOASES, C_control, NUM_CONSTRAINTS_QP,
                       NUM_VARIABLES_QP);
}



/**
 * 功能：计算QP q矩阵
 */
void BalanceController::calc_g_qpOASES() 
{
  g_eigen = -2 * A_control.transpose() * S_control * b_control;
  g_eigen += -2 * xOptPrev.transpose() * alpha_control;
  // 复制到实数组（qpOASES数据类型）
  copy_Eigen_to_real_t(g_qpOASES, g_eigen, NUM_VARIABLES_QP, 1);
}


/**
 * 功能：计算QP lb_ub矩阵
 */
void BalanceController::calc_lb_ub_qpOASES() 
{
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) 
  {
    for (int j = 0; j < NUM_VARIABLES_PER_FOOT; j++) 
    {
      lb_qpOASES[NUM_VARIABLES_PER_FOOT * i + j] =
          contact_state(i) * NEGATIVE_NUMBER;
      ub_qpOASES[NUM_VARIABLES_PER_FOOT * i + j] =
          contact_state(i) * POSITIVE_NUMBER;
    }
  }

  //在f_y=0上添加跳跃约束 
  if (use_hard_constraint_pitch == 1) 
  {
    for (int i = 0; i < NUM_CONTACT_POINTS; i++) 
    {
      lb_qpOASES[NUM_VARIABLES_PER_FOOT * i + 1] = 0;
      ub_qpOASES[NUM_VARIABLES_PER_FOOT * i + 1] = 0;
    }
  }
}


/**
 * 功能：计算QP lbA_ubA矩阵
 */
void BalanceController::calc_lbA_ubA_qpOASES() 
{
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) 
  {
    lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i] =
        contact_state(i) * NEGATIVE_NUMBER;
    lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 1] =
        contact_state(i) * NEGATIVE_NUMBER;
    lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 2] = 0;
    lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 3] = 0;
    lbA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 4] =
        contact_state(i) * minNormalForces_feet(i);

    ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i] = 0;
    ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 1] = 0;
    ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 2] =
        contact_state(i) * POSITIVE_NUMBER;
    ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 3] =
        contact_state(i) * POSITIVE_NUMBER;
    ubA_qpOASES[NUM_CONSTRAINTS_PER_FOOT * i + 4] =
        contact_state(i) * maxNormalForces_feet(i);
  }

  // 在俯仰控制上添加硬约束
  if (use_hard_constraint_pitch == 1) 
  {
    lbA_qpOASES[NUM_CONSTRAINTS_QP - 1] = b_control(5 - 1);
    ubA_qpOASES[NUM_CONSTRAINTS_QP - 1] = b_control(5 - 1);
  }

  else if (use_hard_constraint_pitch > 1) 
  {
    lbA_qpOASES[NUM_CONSTRAINTS_QP - 1] = POSITIVE_NUMBER;
    ubA_qpOASES[NUM_CONSTRAINTS_QP - 1] = NEGATIVE_NUMBER;
  }

  else 
  {
    lbA_qpOASES[NUM_CONSTRAINTS_QP - 1] = NEGATIVE_NUMBER;
    ubA_qpOASES[NUM_CONSTRAINTS_QP - 1] = POSITIVE_NUMBER;
  }
}



/**
 * 功能：LCM发布数据
 */
void BalanceController::publish_data_lcm() 
{
  lcm->publish("CONTROLLER_qp_controller_data", &qp_controller_data_publish);
}


/**
 * 功能：更新变量日志
 */
void BalanceController::update_log_variables(double* p_des, double* p_act,
                                             double* v_des, double* v_act,
                                             double* O_err) 
{
  copy_Eigen_to_double(p_des, x_COM_world_desired, 3);
  copy_Eigen_to_double(p_act, x_COM_world, 3);
  copy_Eigen_to_double(v_des, xdot_COM_world_desired, 3);
  copy_Eigen_to_double(v_act, xdot_COM_world, 3);
  copy_Eigen_to_double(O_err, orientation_error, 3);

  for (int i = 0; i < 3; i++) 
  {
    qp_controller_data.p_des[i] = x_COM_world_desired(i);
    qp_controller_data.p_act[i] = x_COM_world(i);
    qp_controller_data.v_des[i] = xdot_COM_world_desired(i);
    qp_controller_data.v_act[i] = xdot_COM_world(i);
    qp_controller_data.O_err[i] = orientation_error(i);
    qp_controller_data.omegab_des[i] = omega_b_world_desired(i);
    qp_controller_data.omegab_act[i] = omega_b_world(i);
  }
}



/* ------------ Set Parameter Values -------------- */


/**
 * 功能：设置PD增益参数
 */
void BalanceController::set_PDgains(double* Kp_COM_in, double* Kd_COM_in,
                                    double* Kp_Base_in, double* Kd_Base_in) 
{
  Kp_COMx = Kp_COM_in[0];
  Kp_COMy = Kp_COM_in[1];
  Kp_COMz = Kp_COM_in[2];

  Kd_COMx = Kd_COM_in[0];
  Kd_COMy = Kd_COM_in[1];
  Kd_COMz = Kd_COM_in[2];

  Kp_Base_roll = Kp_Base_in[0];
  Kp_Base_pitch = Kp_Base_in[1];
  Kp_Base_yaw = Kp_Base_in[2];

  Kd_Base_roll = Kd_Base_in[0];
  Kd_Base_pitch = Kd_Base_in[1];
  Kd_Base_yaw = Kd_Base_in[2];
}


/**
 * 功能：设置期望的轨迹数据
 */
void BalanceController::set_desiredTrajectoryData(
    double* rpy_des_in, double* p_des_in, double* omegab_des_in,
    double* v_des_in)  //, double* vdot_des_in)
{
  x_COM_world_desired << p_des_in[0], p_des_in[1], p_des_in[2];
  rpyToR(R_b_world_desired, rpy_des_in);
  omega_b_world_desired << omegab_des_in[0], omegab_des_in[1], omegab_des_in[2];
  xdot_COM_world_desired << v_des_in[0], v_des_in[1], v_des_in[2];
  // xddot_COM_world_desired << vdot_des_in[0], vdot_des_in[1], vdot_des_in[2];
}


/**
 * 功能：设置权重
 */
void BalanceController::set_wrench_weights(double* COM_weights_in,
                                           double* Base_weights_in) 
{
  S_control(0, 0) = COM_weights_in[0];
  S_control(1, 1) = COM_weights_in[1];
  S_control(2, 2) = COM_weights_in[2];

  S_control(3, 3) = Base_weights_in[0];
  S_control(4, 4) = Base_weights_in[1];
  S_control(5, 5) = Base_weights_in[2];
}


/**
 * 功能：设置QP权重
 */
void BalanceController::set_QP_options(double use_hard_constraint_pitch_in) 
{
  use_hard_constraint_pitch = use_hard_constraint_pitch_in;
}


/**
 * 功能：设置QP权重
 */
void BalanceController::set_QPWeights() 
{
  S_control.setIdentity();
  W_control.setIdentity();
  alpha_control = .1;
}


/**
 * 功能：设置世界坐标系的数据
 */
void BalanceController::set_worldData() 
{
  direction_normal_flatGround << 0, 0, 1;
  gravity << 0, 0, 9.81;
  direction_tangential_flatGround << 0.7071, 0.7071, 0;
  mu_friction = 0.05;
}


/**
 * 功能：设置摩擦系数
 */
void BalanceController::set_friction(double mu_in)
{ mu_friction = mu_in; }


/**
 * 功能：设置alpha控制
 */
void BalanceController::set_alpha_control(double alpha_control_in) 
{
  alpha_control = alpha_control_in;
}


/**
 * 功能：设置重量数据
 */
void BalanceController::set_mass(double mass_in) { mass = mass_in; }

/**
 * 功能：设置机器人限制的力矩
 */
void BalanceController::set_RobotLimits() 
{
  minNormalForces_feet << 10, 10, 10, 10;
  maxNormalForces_feet << 160, 160, 160, 160;
}

/* ------------ Utilities（公共的数据） -------------- */
/**
 * 功能：获取QPFinished
 */
bool BalanceController::getQPFinished() 
{ return QPFinished; }


/**
 * 功能：打印QP数据
 */
void BalanceController::print_QPData() 
{
  std::cout << "\n\n";
  std::cout << "\n\nH = ";
  print_real_t(H_qpOASES, NUM_VARIABLES_QP, NUM_VARIABLES_QP);
  std::cout << "\n\nA = ";
  print_real_t(A_qpOASES, NUM_CONSTRAINTS_QP, NUM_VARIABLES_QP);
  std::cout << "\n\ng = ";
  print_real_t(g_qpOASES, NUM_VARIABLES_QP, 1);
  std::cout << "\n\nlb = ";
  print_real_t(lb_qpOASES, NUM_VARIABLES_QP, 1);
  std::cout << "\n\nub = ";
  print_real_t(ub_qpOASES, NUM_VARIABLES_QP, 1);
  std::cout << "\n\nlbA = ";
  print_real_t(lbA_qpOASES, NUM_CONSTRAINTS_QP, 1);
  std::cout << "\n\nubA = ";
  print_real_t(ubA_qpOASES, NUM_CONSTRAINTS_QP, 1);
}


/**
 * 功能：打印matrix、nRows、nCols
 */
void BalanceController::print_real_t(real_t* matrix, int nRows, int nCols) 
{
  int count = 0;
  for (int i = 0; i < nRows; i++) 
  {
    for (int j = 0; j < nCols; j++) 
    {
      std::cout << matrix[count] << "\t";
      count++;
    }
    std::cout << "\n";
  }
}


/**
 * 功能：数据转移：从Eigen复制数据到real
 */
void BalanceController::copy_Eigen_to_real_t(real_t* target,
                                             Eigen::MatrixXd& source, int nRows,
                                             int nCols) 
{
  int count = 0;

  //奇怪的行为：特征矩阵矩阵（count）是按列（而不是行）存储的 
  for (int i = 0; i < nRows; i++) 
  {
    for (int j = 0; j < nCols; j++) 
    {
      target[count] = source(i, j);
      count++;
    }
  }
}

/**
 * 功能：数据转移：从Eigen复制数据到double
 */
void BalanceController::copy_Eigen_to_double(double* target,
                                             Eigen::VectorXd& source,
                                             int length) 
{
  for (int i = 0; i < length; i++) 
  {
    target[i] = source(i);
  }
}


/**
 * 功能：数据转移：从Array复制数据到Eigen
 */
void BalanceController::copy_Array_to_Eigen(Eigen::VectorXd& target,
                                            double* source, int len,
                                            int startIndex) 
{
  for (int i = 0; i < len; i++) 
  {
    target(i) = source[i + startIndex];
  }
}


/**
 * 功能：数据转移：从Array复制数据到Eigen
 */
void BalanceController::copy_Array_to_Eigen(Eigen::MatrixXd& target,
                                            double* source, int len,
                                            int startIndex) 
{
  for (int i = 0; i < len; i++) 
  {
    target(i) = source[i + startIndex];
  }
}


/**
 * 功能：数据转移：从real复制数据到Eigen
 */
void BalanceController::copy_real_t_to_Eigen(Eigen::VectorXd& target,
                                             real_t* source, int len) {
  for (int i = 0; i < len; i++) {
    target(i) = source[i];
  }
}


/**
 * 功能：矩阵对数
 */
void BalanceController::matrixLogRot(const Eigen::MatrixXd& R,
                                     Eigen::VectorXd& omega) 
{
  // 公式：theta = acos( (Trace(R) - 1)/2 )
  double theta;
  double tmp = (R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2;
  if (tmp >= 1.) 
  {
    theta = 0;
  } 
  else if (tmp <= -1.) 
  {
    theta = M_PI;
  }
  else 
  {
    theta = acos(tmp);
  }
  // Matrix3F omegaHat = (R-R.transpose())/(2 * sin(theta));
  // crossExtract(omegaHat,omega);
  omega << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
  if (theta > 10e-5) 
  {
    omega *= theta / (2 * sin(theta));
  } 
  else 
  {
    omega /= 2;
  }
}


/**
 * 功能：交叉矩阵
 */
void BalanceController::crossMatrix(Eigen::MatrixXd& R,
                                    const Eigen::VectorXd& omega) 
{
  R(0, 1) = -omega(2)
  ;
  R(0, 2) = omega(1);
  R(1, 0) = omega(2);
  R(1, 2) = -omega(0);
  R(2, 0) = -omega(1);
  R(2, 1) = omega(0);
}


/**
 * 功能：矩阵表达式交叉
 */
void BalanceController::matrixExpOmegaCross(const Eigen::VectorXd& omega,
                                            Eigen::MatrixXd& R) 
{
  double theta = omega.norm();
  R.setIdentity();

  if (theta > 1e-9) 
  {
    omegaHat.setZero();
    crossMatrix(omegaHat, omega / theta);
    // R = I + omegaHat sin(theta) + omegaHat^2 (1-cos(theta))
    R += omegaHat * sin(theta) + omegaHat * omegaHat * (1 - cos(theta));
  }
}

/**
 * 功能：四元数旋转矩阵
 */
void BalanceController::quaternion_to_rotationMatrix(Eigen::MatrixXd& R,
                                                     Eigen::VectorXd& quat) 
{
  //维基百科
  R(0, 0) = 1 - 2 * quat(2) * quat(2) - 2 * quat(3) * quat(3);
  R(0, 1) = 2 * quat(1) * quat(2) - 2 * quat(0) * quat(3);
  R(0, 2) = 2 * quat(1) * quat(3) + 2 * quat(0) * quat(2);
  R(1, 0) = 2 * quat(1) * quat(2) + 2 * quat(0) * quat(3);
  R(1, 1) = 1 - 2 * quat(1) * quat(1) - 2 * quat(3) * quat(3);
  R(1, 2) = 2 * quat(2) * quat(3) - 2 * quat(1) * quat(0);
  R(2, 0) = 2 * quat(1) * quat(3) - 2 * quat(2) * quat(0);
  R(2, 1) = 2 * quat(2) * quat(3) + 2 * quat(1) * quat(0);
  R(2, 2) = 1 - 2 * quat(1) * quat(1) - 2 * quat(2) * quat(2);
}


/**
 * 功能：rpyToR
 */
void BalanceController::rpyToR(Eigen::MatrixXd& R, double* rpy_in) 
{
  Eigen::Matrix3d Rz, Ry, Rx;

  Rz.setIdentity();
  Ry.setIdentity();
  Rx.setIdentity();

  Rz(0, 0) = cos(rpy_in[2]);
  Rz(0, 1) = -sin(rpy_in[2]);
  Rz(1, 0) = sin(rpy_in[2]);
  Rz(1, 1) = cos(rpy_in[2]);

  Ry(0, 0) = cos(rpy_in[1]);
  Ry(0, 2) = sin(rpy_in[1]);
  Ry(2, 0) = -sin(rpy_in[1]);
  Ry(2, 2) = cos(rpy_in[1]);

  Rx(1, 1) = cos(rpy_in[0]);
  Rx(1, 2) = -sin(rpy_in[0]);
  Rx(2, 1) = sin(rpy_in[0]);
  Rx(2, 2) = cos(rpy_in[0]);

  R = Rz * Ry * Rx;
}
