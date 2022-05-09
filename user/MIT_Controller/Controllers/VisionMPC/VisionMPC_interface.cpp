#include "VisionMPC_interface.h"
#include "../convexMPC/common_types.h"
#include "VisionSolverMPC.h"
#include <eigen3/Eigen/Dense>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#define V_NUM_LEGS 4


vision_mpc_problem_setup v_problem_config;      //mpc参数 ：时间 摩擦系数 最大力 分段数
vision_mpc_update_data_t v_update;              //位置速度等
pthread_mutex_t vision_problem_cfg_mt;          //无用
pthread_mutex_t vision_mpc_update_mt;           //无用
pthread_t vision_mpc_solve_thread;              //无用

u8 v_first_run = 1;


/**
 * 功能：初始化MPC可视化
 */
void vision_initialize_mpc()
{
  //printf("Initializing MPC!\n");
  if(pthread_mutex_init(&vision_problem_cfg_mt,NULL)!=0)//无用
    printf("[MPC ERROR] Failed to initialize problem configuration mutex.\n");

  if(pthread_mutex_init(&vision_mpc_update_mt,NULL)!=0)//无用
    printf("[MPC ERROR] Failed to initialize update data mutex.\n");
}

/**
 * 功能：可视化配置问题
 */
void vision_setup_problem(double dt, int horizon, double mu, double f_max)
{
	//无用
  if(v_first_run) 
  {
    v_first_run = false;
    vision_initialize_mpc();
  }

  //pthread_mutex_lock(&problem_cfg_mt);
  v_problem_config.horizon = horizon; //分段数
  v_problem_config.f_max = f_max;     //最大力
  v_problem_config.mu = mu;           //摩擦系数
  v_problem_config.dt = dt;           //时间
  //pthread_mutex_unlock(&problem_cfg_mt);
  vision_resize_qp_mats(horizon);      //分配内存，重置Qp矩阵
}



/**
 * 功能：数组类型转换 matlab 转c++
 */
inline void vision_mpf_to_flt(flt* dst, mfp* src, s32 n_items) 
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}


/**
 * 功能：数组类型转换 int 转u8
 */
inline void vision_mint_to_u8(u8* dst, mint* src, s32 n_items) 
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}


int vision_has_solved = 0;//已解？

/**
 * 功能：安全复制问题数据并启动解算器
 * 备注：未用
 */
void vision_update_problem_data(double* p, double* v, double* q, double* w,
								double* r, double yaw, double* weights, 
								double* state_trajectory, double alpha, int* gait)
{
  vision_mpf_to_flt(v_update.p,p,3);
  vision_mpf_to_flt(v_update.v,v,3);
  vision_mpf_to_flt(v_update.q,q,4);
  vision_mpf_to_flt(v_update.w,w,3);
  vision_mpf_to_flt(v_update.r,r,12);
  v_update.yaw = yaw;
  vision_mpf_to_flt(v_update.weights,weights,12);
  //这是安全的，解算器没有运行，并且v_update_problem_data和setup_problem是从同一线程调用的
  vision_mpf_to_flt(v_update.traj,state_trajectory,12*v_problem_config.horizon);
  v_update.alpha = alpha;
  vision_mint_to_u8(v_update.gait,gait,4*v_problem_config.horizon);
  vision_solve_mpc(&v_update, &v_problem_config);
  vision_has_solved = 1;
}



/**
 * 功能：更新问题参数和解
 */
void vision_update_problem_data_floats(float* p, float* v, float* q, float* w,
										float* r, float yaw, float* weights,
										float* state_trajectory, float alpha, int* gait)
{
  //将传入参数复制给类变量
  v_update.alpha = alpha;
  v_update.yaw = yaw;
  vision_mint_to_u8(v_update.gait,gait,4*v_problem_config.horizon);
  memcpy((void*)v_update.p,(void*)p,sizeof(float)*3);
  memcpy((void*)v_update.v,(void*)v,sizeof(float)*3);
  memcpy((void*)v_update.q,(void*)q,sizeof(float)*4);
  memcpy((void*)v_update.w,(void*)w,sizeof(float)*3);
  memcpy((void*)v_update.r,(void*)r,sizeof(float)*12);
  memcpy((void*)v_update.weights,(void*)weights,sizeof(float)*12);
  memcpy((void*)v_update.traj,(void*)state_trajectory, sizeof(float) * 12 * v_problem_config.horizon);

  vision_solve_mpc(&v_update, &v_problem_config);  //解mpc
  vision_has_solved = 1; //已解
}


/**
 * 功能：获取索引号的结果
 */
double vision_get_solution(int index)
{
  if(!vision_has_solved) return 0.f;
  
  mfp* qs = vision_get_q_soln();
  return qs[index];
}
