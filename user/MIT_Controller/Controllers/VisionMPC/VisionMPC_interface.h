#ifndef _Vision_mpc_interface
#define _Vision_mpc_interface

#define V_MAX_GAIT_SEGMENTS 36

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

//mpc参数 
struct vision_mpc_problem_setup
{
  float dt;       //时间
  float mu;       //摩擦系数
  float f_max;    //最大力
  int horizon;    //分段数
};

struct vision_mpc_update_data_t
{
  float p[3];                           //位置
  float v[3];                           //速度
  float q[4];                           //四元数
  float w[3];                           //角速度
  float r[12];                          //com到足端向量
  float yaw;                            //偏航角
  float weights[12];                    //权重
  float traj[12*V_MAX_GAIT_SEGMENTS];   //参考轨迹 最大一次性V_MAX_GAIT_SEGMENTS
  float alpha;                          //mpc参数 K 式（31）
  unsigned char gait[V_MAX_GAIT_SEGMENTS];//步态 脚是否悬空
  unsigned char hack_pad[1000];         //无用
  int max_iterations;                   //最大迭代次数
  float x_drag;                         //不清楚
};

EXTERNC void vision_setup_problem(double dt, int horizon, double mu, double f_max);
EXTERNC void vision_update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait);
EXTERNC double vision_get_solution(int index);
EXTERNC void vision_update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float yaw, float* weights,
                                        float* state_trajectory, float alpha, int* gait);

void update_x_drag(float x_drag);
#endif
