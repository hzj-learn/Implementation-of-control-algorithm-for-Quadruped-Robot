#ifndef REFERENCEGRF_H
#define REFERENCEGRF_H
/*
+References:
+   [R1] M. Focchi, A. del Prete, I. Havoutis, R. Featherstone, D. G. Caldwell, and C. Semini. High-slope terrain
+   locomotion for torque-controlled quadruped robots. Autonomous Robots, 2016.
+   [R2] R. M. Murray, S. S. Sastry, and L. Zexiang. A Mathematical Introduction to Robotic Manipulation. CRC
+   Press, Inc., Boca Raton, FL, USA, 1st edition, 1994.
+Cheetah-3-Documentation-Control:
+   [C1] balanceController.pdf
+   qpOASES variables are terminated with _qpOASES
+*/
   
#ifndef EIGEN_NO_DEBUG
#define EIGEN_NO_DEBUG
#endif

#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>
#include <lcm/lcm-cpp.hpp>
#include "sim_command_t.hpp"
#include "qp_controller_data_t.hpp"


static const int NUM_VARIABLES_QP_DES = 4;
static const int NUM_CONSTRAINTS_QP_DES = 4;
static const int NUM_CONTACT_POINTS_DES = 4;
static const int NUM_VARIABLES_PER_FOOT_DES = 1;
static const int NUM_CONSTRAINTS_PER_FOOT_DES = 1;

//static const double PI_CONST = 3.1415;
static const double NEGATIVE_NUMBER_DES = -1000000.0;
static const double POSITIVE_NUMBER_DES =  1000000.0;

using namespace Eigen;
using namespace qpOASES;

class ReferenceGRF
{
   public:
      ReferenceGRF();                                                                                                                   //（1）计算参考反作用力函数
      ~ReferenceGRF(){};
      void updateProblemData(double* p_feet_desired_in,double* p_des);                                                                  //（2）使用新的运动学测量更新QP
      void SetContactData(double* contact_state_in,double* min_forces_in,double* max_forces_in, double threshold, int stance_legs_in);  //（3）设置QP约束的最小/最大力
      void solveQP_nonThreaded(double* xOpt);                                                                                           //（4）处理QP非线性函数
      void set_RobotLimits();                                                                                                           //（5）设置机器人限制
      void set_worldData();                                                                                                             //（6）设置世界坐标系数据
      void set_QPWeights();                                                                                                             //（7）设置QP权重
      void set_mass(double mass_in);                                                                                                    //（8）设置质量参数
      void set_alpha_control(double alpha_control_in);                                                                                  //（9）设置机器人alpha控制


   private:
      /* 固定大小qpOASES数据 */           
      QProblemB QProblemObj_qpOASES;  
      int_t nWSR_qpOASES = 100;
      int_t nWSR_fixed = 100;
      real_t cpu_time;
      real_t cpu_time_fixed;
      int_t qp_exit_flag;
      int nWSR_initial;
      double cpu_time_initial;
      double xOpt_local[4];
      double qp_not_init;

      Bounds guessedBounds;
      Constraints guessedConstraints;
      
      /* 用于HJB优化的QP变量 */
      real_t    H_qpOASES[NUM_VARIABLES_QP_DES*NUM_VARIABLES_QP_DES];
      real_t    g_qpOASES[NUM_VARIABLES_QP_DES];
      real_t   lb_qpOASES[NUM_VARIABLES_QP_DES];
      real_t   ub_qpOASES[NUM_VARIABLES_QP_DES];  
      real_t xOpt_qpOASES[NUM_VARIABLES_QP_DES];
      real_t yOpt_qpOASES[NUM_VARIABLES_QP_DES+NUM_CONSTRAINTS_QP_DES];   
      real_t xOpt_initialGuess[NUM_VARIABLES_QP_DES];

      /* 与qpOASES变量匹配的特征变量 */
      Eigen::MatrixXd    H_eigen;
      Eigen::MatrixXd    g_eigen;
      Eigen::VectorXd xOpt_eigen;
      Eigen::VectorXd yOpt_eigen;

      /* 用于构造QP矩阵的机器人控制变量，见[R1]的（5）和（6） */
      Eigen::MatrixXd A_control;            
      Eigen::VectorXd b_control;
      double alpha_control;

      /* 模型和世界参数及力限 */
      double mass;
      Eigen::VectorXd gravity;

      Eigen::VectorXd minNormalForces_feet;
      Eigen::VectorXd maxNormalForces_feet;

      /* 脚部接触信息，1表示在地上 */
      Eigen::VectorXd contact_state;

      /* 期望运动学 */
      Eigen::VectorXd x_COM_world_desired;
      Eigen::MatrixXd p_feet_desired; //new 

      /* 临时内部矩阵 */
      Eigen::MatrixXd omegaHat;   
      Eigen::MatrixXd tempSkewMatrix3;
      Eigen::VectorXd tempVector3;   

      /* 内部QP管理数据 */
      bool QPFinished;
      Eigen::VectorXd xOptPrev;
      Eigen::VectorXd yOptPrev;

      /* 接口功能 */     
      bool getQPFinished();    
     
      void update_A_control();
      void calc_H_qpOASES();
      void calc_g_qpOASES();     
      void calc_lb_ub_qpOASES();

      /* 公共函数 */
      void copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd &source, int nRows, int nCols );                  
      void copy_Array_to_Eigen(Eigen::VectorXd &target, double* source, int len, int startIndex);
      void copy_Array_to_Eigen(Eigen::MatrixXd &target, double* source, int len, int startIndex);
      void copy_real_t_to_Eigen(Eigen::VectorXd &target, real_t* source, int len);
      void crossMatrix(Eigen::MatrixXd &R, const Eigen::VectorXd &omega);
};

#endif
