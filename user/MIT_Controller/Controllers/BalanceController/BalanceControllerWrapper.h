#ifndef BALANCECONTROLLERWRAPPER_H
#define BALANCECONTROLLERWRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

void balanceControl_init();                                                  //(1)平衡控制器初始化函数
void balanceControl_set_desiredTrajectoryData(                               //(2)设置期望轨迹数据函数
    double* rpy_des_in, double* p_des_in, double* omegab_des_in,
    double* v_des_in);  
void balanceControl_set_desired_swing_pos(double* pFeet_des_in);             //(3)设置期望摆腿位置函数
void balanceControl_set_actual_swing_pos(double* pFeet_act_in);              //(4)设置实际轨迹数据函数
void balanceControl_set_PDgains(double* Kp_COM_in, double* Kd_COM_in,        //(5)设置实际PD增益函数
                                double* Kp_Base_in, double* Kd_Base_in);
void balanceControl_set_wrench_weights(double* COM_weights_in,               //(6)设置wrench权重函数
                                       double* Base_weights_in);
void balanceControl_set_QP_options(double use_hard_constraint_pitch_in);     //(7)设置QP选项函数
void balanceControl_updateProblemData(double* xfb_in, double* pFeet_in,      //(8)更新问题数据函数
                                      double yaw_in);
void balanceControl_SetContactData(double* contact_state_in,                 //(9)设置接触数据函数
                                   double* min_forces_in,
                                   double* max_forces_in);
void balanceControl_solveQP_nonThreaded();                                   //(10)处理QP非线性函数
void balanceControl_set_friction(double mu_in);                              //(12)设置摩檫力函数
void balanceControl_set_alpha_control(double alpha_control_in);              //(13)设置alpha控制函数
void balanceControl_set_mass(double mass_in);                                //(14)设置质量函数
void balanceControl_publish_data_lcm();                                      //(15)发布LCM数据函数
double balanceControl_get_fOpt_matlab(int index);                            //(16)从matlab获取数据函数
void bcPrint();                                                              

#ifdef __cplusplus
}
#endif

#endif
