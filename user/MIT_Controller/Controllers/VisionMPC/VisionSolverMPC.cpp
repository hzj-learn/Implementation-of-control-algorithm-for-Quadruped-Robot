#include "VisionSolverMPC.h"
#include "../convexMPC/common_types.h"
#include "VisionMPC_interface.h"
#include "VisionRobotState.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>
#include <stdio.h>
#include <sys/time.h>

#define V_BIG_NUMBER 5e10
VisionRobotState v_rs;
using std::cout;
using std::endl;
using Eigen::Dynamic;

Matrix<fpt,Dynamic,13> vA_qp;
Matrix<fpt,Dynamic,Dynamic> vB_qp;
Matrix<fpt,13,12> vBdt;
Matrix<fpt,13,13> vAdt;
Matrix<fpt,25,25> vABc,v_expmm;
Matrix<fpt,Dynamic,Dynamic> vS;
Matrix<fpt,Dynamic,1> vX_d;
Matrix<fpt,Dynamic,1> vU_b;
Matrix<fpt,Dynamic,Dynamic> v_fmat;

Matrix<fpt,Dynamic,Dynamic> v_qH;
Matrix<fpt,Dynamic,1> v_qg;

Matrix<fpt,Dynamic,Dynamic> v_eye_12h;

qpOASES::real_t* vH_qpoases;
qpOASES::real_t* vg_qpoases;
qpOASES::real_t* vA_qpoases;
qpOASES::real_t* vlb_qpoases;
qpOASES::real_t* vub_qpoases;
qpOASES::real_t* vq_soln;

qpOASES::real_t* vH_red;
qpOASES::real_t* vg_red;
qpOASES::real_t* vA_red;
qpOASES::real_t* vlb_red;
qpOASES::real_t* vub_red;
qpOASES::real_t* vq_red;
u8 v_real_allocated = 0;


char v_var_elim[2000];
char v_con_elim[2000];


/**
 * 功能：获取结果
 */
mfp* vision_get_q_soln() 
{
  return vq_soln;
}


/**
 * 功能：零附近
 */
s8 v_near_zero(fpt a)
{
  return (a < 0.01 && a > -.01) ;
}


/**
 * 功能：一附近
 */
s8 v_near_one(fpt a)
{
  return v_near_zero(a-1);
}


/**
 * 功能：矩阵格式抓换成求解器矩阵格式
 */
void v_matrix_to_real(qpOASES::real_t* dst, Matrix<fpt,Dynamic,Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for(s16 r = 0; r < rows; r++)
  {
    for(s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r,c);
      a++;
    }
  }
}




/**
 * 功能：离散化并转快速qp算法矩阵 ，通过将状态变量表示为当前状态和输入序列的函数，可以将状态变量从优化问题的决策变量中剔除
 */
void vision_c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc,fpt dt,s16 horizon)
{
  vABc.setZero();
  vABc.block(0,0,13,13) = Ac;     //式（25）
  vABc.block(0,13,13,12) = Bc;    //式（25）
  //离散化公式 通过（25）使用线性定常系统其次离散化，
  //[A,B;0,0]是A 离散化后A=e^(A*dt)
  vABc = dt*vABc;
  v_expmm = vABc.exp();//离散化公式
  vAdt = v_expmm.block(0,0,13,13);
  vBdt = v_expmm.block(0,13,13,12);
  if(horizon > 19) 
  {
    throw std::runtime_error("horizon is too long!");
  }

//构造快速QP问题矩阵
  Matrix<fpt,13,13> powerMats[20];
  powerMats[0].setIdentity();           //单位矩阵13*13
  for(int i = 1; i < horizon+1; i++) 
  {
    powerMats[i] = vAdt * powerMats[i-1];//（Adt，Adt*Adt，Adt*Adt*Adt。。。）
  }

  for(s16 r = 0; r < horizon; r++)
  {
    vA_qp.block(13*r,0,13,13) = powerMats[r+1];//A_qp=[Adt,Adt*Adt,Adt*Adt*Adt,....]^T=13K*13
    for(s16 c = 0; c < horizon; c++)
    {
      if(r >= c)
      {
        s16 a_num = r-c;
        vB_qp.block(13*r,12*c,13,12) = powerMats[a_num] * vBdt;
      }
    }
  }

}



/**
 * 功能：重置矩阵，分配和释放内存
 */
void vision_resize_qp_mats(s16 horizon)
{
  int mcount = 0;//无用
  int h2 = horizon*horizon;

  vA_qp.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon*1;

  vB_qp.resize(13*horizon, 12*horizon);
  mcount += 13*h2*12;

  vS.resize(13*horizon, 13*horizon);
  mcount += 13*13*h2;

  vX_d.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;

  vU_b.resize(20*horizon, Eigen::NoChange);
  mcount += 20*horizon;

  v_fmat.resize(20*horizon, 12*horizon);
  mcount += 20*12*h2;

  v_qH.resize(12*horizon, 12*horizon);
  mcount += 12*12*h2;

  v_qg.resize(12*horizon, Eigen::NoChange);
  mcount += 12*horizon;

  v_eye_12h.resize(12*horizon, 12*horizon);
  mcount += 12*12*horizon;

  //printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  vA_qp.setZero();
  vB_qp.setZero();
  vS.setZero();
  vX_d.setZero();
  vU_b.setZero();
  v_fmat.setZero();
  v_qH.setZero();
  v_eye_12h.setIdentity();

  //TODO: use realloc instead of free/malloc on size changes

  if(v_real_allocated)
  {
    free(vH_qpoases);
    free(vg_qpoases);
    free(vA_qpoases);
    free(vlb_qpoases);
    free(vub_qpoases);
    free(vq_soln);
    free(vH_red);
    free(vg_red);
    free(vA_red);
    free(vlb_red);
    free(vub_red);
    free(vq_red);
  }

  vH_qpoases = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  vg_qpoases = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  vA_qpoases = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  vlb_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  vub_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  vq_soln = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;

  vH_red = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  vg_red = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  vA_red = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  vlb_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  vub_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  vq_red = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  v_real_allocated = 1;

  //printf("malloc'd %d floating point numbers.\n",mcount);
#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n",horizon);
#endif
}



/**
 * 功能：式16 I^-1[r]x
 */
inline Matrix<fpt,3,3> cross_mat(Matrix<fpt,3,3> I_inv, Matrix<fpt,3,1> r)
{
  Matrix<fpt,3,3> cm;
  cm << 0.f, -r(2), r(1),
     r(2), 0.f, -r(0),
     -r(1), r(0), 0.f;
  return I_inv * cm;
}



/**
 * 功能：连续时间状态空间矩阵
 */
void vision_ct_ss_mats(Matrix<fpt,3,3> vI_world, fpt m, Matrix<fpt,3,4> r_feet, 
    Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B, float x_drag)
{
  A.setZero();
  A(3,9) = 1.f;
  A(9,9) = x_drag;
  A(4,10) = 1.f;
  A(5,11) = 1.f;

  A(11,12) = 1.f;
  A.block(0,6,3,3) = R_yaw.transpose();

  B.setZero();
  Matrix<fpt,3,3> I_inv = vI_world.inverse();

  for(s16 b = 0; b < 4; b++)
  {
    B.block(6,b*3,3,3) = cross_mat(I_inv,r_feet.col(b));
    B.block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
  }
}



/**
 * 功能：四元数转欧拉角
 */
void vision_quat_to_rpy(Quaternionf q, Matrix<fpt,3,1>& rpy)
{
  fpt as = vision_t_min(-2.*(q.x()*q.z()-q.w()*q.y()),.99999);
  rpy(0) = atan2(2.f*(q.x()*q.y()+q.w()*q.z()),vision_sq(q.w()) + vision_sq(q.x()) - vision_sq(q.y()) - vision_sq(q.z()));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f*(q.y()*q.z()+q.w()*q.x()),vision_sq(q.w()) - vision_sq(q.x()) - vision_sq(q.y()) + vision_sq(q.z()));
}


Matrix<fpt,13,1> v_x_0;   //初始状态
Matrix<fpt,3,3> vI_world; //惯性张量 世界坐标系下
Matrix<fpt,13,13> vA_ct;  //储存连续状态转移矩阵
Matrix<fpt,13,12> vB_ct_r;//储存连续控制矩阵


/**
 * 功能：视觉处理MPC
 */
void vision_solve_mpc(vision_mpc_update_data_t* update, vision_mpc_problem_setup* setup)
{
  v_rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw);

  //roll pitch yaw
  //四元数转欧拉角
  Matrix<fpt,3,1> rpy;
  vision_quat_to_rpy(v_rs.q,rpy);

  //初始状态(13个状态表示)
  //欧拉角（3x1），机身位置（3x1），机身角速度（3*1），机身速度（3*1），重力
  v_x_0 << rpy(2), rpy(1), rpy(0), v_rs.p , v_rs.w, v_rs.v, -9.8f;
  
  //式15 世界坐标系下惯性矩阵
  vI_world = v_rs.R_yaw * v_rs.I_body * v_rs.R_yaw.transpose(); //original
  
  //求式（16）和式（17）连续状态空间方程
  vision_ct_ss_mats(vI_world,v_rs.m,v_rs.r_feet,v_rs.R_yaw,vA_ct,vB_ct_r, update->x_drag);

  //QP matrices得到优化QP矩阵
  vision_c2qp(vA_ct,vB_ct_r,setup->dt,setup->horizon);

  //weights权重
  Matrix<fpt,13,1> full_weight;
  for(u8 i = 0; i < 12; i++)
    full_weight(i) = update->weights[i];

  full_weight(12) = 0.f;
  vS.diagonal() = full_weight.replicate(setup->horizon,1);//复制horizon次 13horizon*13horizon

  //trajectory 参考轨迹 这一步中所有参考
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < 12; j++)
      vX_d(13*i+j,0) = update->traj[12*i+j];
  }

   //输入力边界 ，x,y方向力无穷的上边界 
  s16 k = 0;
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < 4; j++)
    {
      vU_b(5*k + 0) = V_BIG_NUMBER;
      vU_b(5*k + 1) = V_BIG_NUMBER;
      vU_b(5*k + 2) = V_BIG_NUMBER;
      vU_b(5*k + 3) = V_BIG_NUMBER;
      vU_b(5*k + 4) = update->gait[i*4 + j] * setup->f_max;//update->gait[i*4 + j]=0/1//哪条腿在悬空中=0
      k++;
    }
  }

  //约束
  Matrix<fpt,5,3> f_block;
  fpt mu = 1.f/setup->mu;//式（22）~（24）变换形式
  f_block <<  mu, 0,  1.f,//式（22）~（24）
          -mu, 0,  1.f,
          0,  mu, 1.f,
          0, -mu, 1.f,
          0,   0, 1.f;

  for(s16 i = 0; i < setup->horizon*4; i++)//组合成约束矩阵
  {
    v_fmat.block(i*5,i*3,5,3) = f_block;//对角阵 size= horizon*20,horizon*12
  }

  v_qH = 2*(vB_qp.transpose()*vS*vB_qp + update->alpha*v_eye_12h);        //式（31）
  v_qg = 2*vB_qp.transpose()*vS*(vA_qp*v_x_0 - vX_d);                     //式（32）
  v_matrix_to_real(vH_qpoases,v_qH,setup->horizon*12, setup->horizon*12); //式（31）
  v_matrix_to_real(vg_qpoases,v_qg,setup->horizon*12, 1);                 //式（32）
  v_matrix_to_real(vA_qpoases,v_fmat,setup->horizon*20, setup->horizon*12);//所求量约束矩阵
  v_matrix_to_real(vub_qpoases,vU_b,setup->horizon*20, 1);                 //约束上边界

  for(s16 i = 0; i < 20*setup->horizon; i++)
    vlb_qpoases[i] = 0.0f;                                                  //约束下边界

  s16 num_constraints = 20*setup->horizon;                                  //约束数
  s16 num_variables = 12*setup->horizon;                                    //变量数
  qpOASES::int_t nWSR = 100;                                                //指定在初始同伦期间要执行的最大工作集重新计算次数（在输出时它包含实际执行的工作集重新计算的次数！）
  int new_vars = num_variables;                                             //不悬空变量数
  int new_cons = num_constraints;                                           //不悬空约束数

//清空 准备简化QP矩阵
  for(int i =0; i < num_constraints; i++)
    v_con_elim[i] = 0;

  for(int i = 0; i < num_variables; i++)
    v_var_elim[i] = 0;


  for(int i = 0; i < num_constraints; i++)//约束数，遍历所有约束上下边界 如果边界值都在0附近，就操作赋值，否则跳过 即操作悬空腿
  {
    if(! (v_near_zero(vlb_qpoases[i]) && v_near_zero(vub_qpoases[i]))) continue;
	
    double* c_row = &vA_qpoases[i*num_variables];//边界值都在0附近找到其对应的约束矩阵的行
	
    for(int j = 0; j < num_variables; j++)//遍历这一行的约束
    {
      if(v_near_one(c_row[j]))//如果约束参数接近1 即悬空足fz对应的约束
      {
        new_vars -= 3;//变量数-3	需要规划的三维足力少一组
        new_cons -= 5;//约束数-5	即悬空足所有约束无效
		
        int cs = (j*5)/3 -3;
		
        v_var_elim[j-2] = 1;//将悬空的置1
        v_var_elim[j-1] = 1;
        v_var_elim[j  ] = 1;
        v_con_elim[cs] = 1;//将悬空的置1
        v_con_elim[cs+1] = 1;
        v_con_elim[cs+2] = 1;
        v_con_elim[cs+3] = 1;
        v_con_elim[cs+4] = 1;
      }
    }
  }
  if(1==1)
  {
    int var_ind[new_vars];  //除去所有悬空腿后的支撑腿力 //储存不悬空足力序号
    int v_con_ind[new_cons];//除去所有悬空腿后的支撑腿约束 //储存不悬空足约束序号
	
    int vc = 0;
    for(int i = 0; i < num_variables; i++)//12*horizon
    {
      if(!v_var_elim[i])//如果为0 就是不悬空
      {
        if(!(vc<new_vars))
        {
          printf("BAD ERROR 1\n");
        }
        var_ind[vc] = i; //储存不悬空足力序号
        vc++;
      }
    }
    vc = 0;
    for(int i = 0; i < num_constraints; i++)
    {
      if(!v_con_elim[i])//如果为0 就是不悬空
      {
        if(!(vc<new_cons))
        {
          printf("BAD ERROR 1\n");
        }
        v_con_ind[vc] = i;//储存不悬空足约束序号
        vc++;
      }
    }
	
	
    for(int i = 0; i < new_vars; i++)//不悬空变量数
    {
      int olda = var_ind[i];
      vg_red[i] = vg_qpoases[olda];//不悬空足力序号来组成新的Qp问题梯度向量G
      for(int j = 0; j < new_vars; j++)
      {
        int oldb = var_ind[j];
		//不悬空足力序号来组成新的Qp问题（半）正定矩阵H 
		//12*horizon，12*horizon 但是不一定全用 下面同样
        vH_red[i*new_vars + j] = vH_qpoases[olda*num_variables + oldb]; 
      }
    }
  //同理如上 新约束矩阵
    for (int con = 0; con < new_cons; con++)
    {
      for(int st = 0; st < new_vars; st++)
      {
        float cval = vA_qpoases[(num_variables*v_con_ind[con]) + var_ind[st] ];
        vA_red[con*new_vars + st] = cval;
      }
    }
	//同理如上 新约束上下边界
    for(int i = 0; i < new_cons; i++)
    {
      int old = v_con_ind[i];
      vub_red[i] = vub_qpoases[old];
      vlb_red[i] = vlb_qpoases[old];
    }

    qpOASES::QProblem problem_red (new_vars, new_cons);//设置qp问题 https://blog.csdn.net/weixin_40709533/article/details/86064148
    qpOASES::Options op;
    op.setToMPC();
    op.printLevel = qpOASES::PL_NONE;
    problem_red.setOptions(op);

    int rval = problem_red.init(vH_red, vg_red, vA_red, NULL, NULL, vlb_red, vub_red, nWSR);//初始化所有内部数据结构
	//函数init会返回一个状态代码（类型为returnValue），表示初始化是否成功。可能的值是：
	//SUCCESSFUL_RETURN ：初始化成功（包括第一个QP的求解）。
	//RET_MAX_NWSR_REACHED：在给定的工作集重新计算数量内无法解决初始QP。
	//RET_INIT_FAILED（或更详细的错误代码）：初始化失败。
    (void)rval;
	//将最优原始解向量（dimension：nV）写入数组vq_red，该数组必须由用户分配（和释放）
	//会返回状态 ，如上
    int rval2 = problem_red.getPrimalSolution(vq_red);
	//输出求解状态
    if(rval2 != qpOASES::SUCCESSFUL_RETURN)
      printf("failed to solve!\n");

    // printf("solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);
//按格式处理输出量
    vc = 0;
    for(int i = 0; i < num_variables; i++)
    {
      if(v_var_elim[i])//输出悬空足力为零
      {
        vq_soln[i] = 0.0f;
      }
      else
      {
        vq_soln[i] = vq_red[vc];//输出足力
        vc++;
      }
    }
  }
}
