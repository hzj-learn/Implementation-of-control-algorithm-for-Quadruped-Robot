#include "SolverMPC.h"
#include "common_types.h"
#include "convexMPC_interface.h"
#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
//#include <unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>
#include <stdio.h>
#include <sys/time.h>
#include <Utilities/Timer.h>
#include <JCQP/QpProblem.h>

//#define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10

RobotState rs;
using std::cout;
using std::endl;
using Eigen::Dynamic;

//qpOASES::real_t a;

Matrix<fpt,Dynamic,13> A_qp;
Matrix<fpt,Dynamic,Dynamic> B_qp;
Matrix<fpt,13,12> Bdt;
Matrix<fpt,13,13> Adt;
Matrix<fpt,25,25> ABc,expmm;
Matrix<fpt,Dynamic,Dynamic> S;
Matrix<fpt,Dynamic,1> X_d;
Matrix<fpt,Dynamic,1> U_b;
Matrix<fpt,Dynamic,Dynamic> fmat;

Matrix<fpt,Dynamic,Dynamic> qH;
Matrix<fpt,Dynamic,1> qg;

Matrix<fpt,Dynamic,Dynamic> eye_12h;

qpOASES::real_t* H_qpoases;
qpOASES::real_t* g_qpoases;
qpOASES::real_t* A_qpoases;
qpOASES::real_t* lb_qpoases;
qpOASES::real_t* ub_qpoases;
qpOASES::real_t* q_soln;

qpOASES::real_t* H_red;
qpOASES::real_t* g_red;
qpOASES::real_t* A_red;
qpOASES::real_t* lb_red;
qpOASES::real_t* ub_red;
qpOASES::real_t* q_red;
u8 real_allocated = 0;


char var_elim[2000];
char con_elim[2000];

/**
 * 功能：获取结果
 */
mfp* get_q_soln()
{
  return q_soln;
}


/**
 * 功能：零附近归零
 */
s8 near_zero(fpt a)
{
  return (a < 0.01 && a > -.01) ;
}


/**
 * 功能：1附近归1
 */
s8 near_one(fpt a)
{
	
  return near_zero(a-1);
}


/**
 * 功能：矩阵格式转换成求解器格式
 */
void matrix_to_real(qpOASES::real_t* dst, Matrix<fpt,Dynamic,Dynamic> src, s16 rows, s16 cols)
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
 * 功能：离散化并转快速qp算法矩阵
 * 通过将状态变量表示为当前状态和输入序列的函数，可以将状态变量从优化问题的决策变量中剔除
 */
void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc,fpt dt,s16 horizon)
{
  ABc.setZero();
  ABc.block(0,0,13,13) = Ac;//式（25）
  ABc.block(0,13,13,12) = Bc;//式（25）
  //离散化公式 通过（25）使用线性定常系统其次离散化，[A,B;0,0]是A 离散化后A=e^(A*dt)
  ABc = dt*ABc;
  expmm = ABc.exp();//离散化公式
  Adt = expmm.block(0,0,13,13);
  Bdt = expmm.block(0,13,13,12);

  #ifdef K_PRINT_EVERYTHING
    cout<<"Adt: \n"<<Adt<<"\nBdt:\n"<<Bdt<<endl;
  #endif

  if(horizon > 19) 
  {
    throw std::runtime_error("horizon is too long!");
  }

  Matrix<fpt,13,13> powerMats[20];
  powerMats[0].setIdentity();             //单位矩阵13*13 
  for(int i = 1; i < horizon+1; i++) 
  {
    powerMats[i] = Adt * powerMats[i-1];//（Adt，Adt*Adt，Adt*Adt*Adt。。。）
  }

  for(s16 r = 0; r < horizon; r++)
  {
    A_qp.block(13*r,0,13,13) = powerMats[r+1];//Adt.pow(r+1); A_qp=[Adt,Adt*Adt,Adt*Adt*Adt,....]^T=13K*13
    for(s16 c = 0; c < horizon; c++)
    {
      if(r >= c)
      {
        s16 a_num = r-c;
        B_qp.block(13*r,12*c,13,12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
      }
    }
  }

  #ifdef K_PRINT_EVERYTHING
    cout<<"AQP:\n"<<A_qp<<"\nBQP:\n"<<B_qp<<endl;
  #endif
}


/**
 * 功能：重组qp矩阵，把状态方程转换成二次规划QP求解器的表达形式
 */
void resize_qp_mats(s16 horizon)
{
  int mcount = 0;
  int h2 = horizon*horizon;

  A_qp.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon*1;

  B_qp.resize(13*horizon, 12*horizon);
  mcount += 13*h2*12;

  S.resize(13*horizon, 13*horizon);
  mcount += 13*13*h2;

  X_d.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;

  U_b.resize(20*horizon, Eigen::NoChange);
  mcount += 20*horizon;

  fmat.resize(20*horizon, 12*horizon);
  mcount += 20*12*h2;

  qH.resize(12*horizon, 12*horizon);
  mcount += 12*12*h2;

  qg.resize(12*horizon, Eigen::NoChange);
  mcount += 12*horizon;

  eye_12h.resize(12*horizon, 12*horizon);
  mcount += 12*12*horizon;

  //printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;
//二次規劃的變量
  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  fmat.setZero();
  qH.setZero();
  eye_12h.setIdentity();

  //TODO: 在大小更改时使用realloc而不是free/malloc

  if(real_allocated)
  {
    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  H_qpoases = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_qpoases = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_qpoases = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  lb_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  ub_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  q_soln = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;

  H_red = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_red = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_red = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  lb_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  ub_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  q_red = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  real_allocated = 1;
  //printf("malloc'd %d floating point numbers.\n",mcount);
#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n",horizon);
#endif
}


/**
 * 功能：式16 I^-1[r]x表达式实现
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
 * 功能： 连续时间状态空间矩阵A、B
 */
void ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,4> r_feet, Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B, float x_drag)
{
  //A矩阵
  A.setZero();
  A(3,9) = 1.f;
  A(4,10) = 1.f;
  A(5,11) = 1.f;
  A(11,9) = x_drag;                       //z轴方向加速度受x轴方向速度的影响程度
  
  A(11,12) = 1.f;
  A.block(0,6,3,3) = R_yaw.transpose();

//B矩阵
  B.setZero();
  Matrix<fpt,3,3> I_inv = I_world.inverse();

  for(s16 b = 0; b < 4; b++)
  {
    B.block(6,b*3,3,3) = cross_mat(I_inv,r_feet.col(b));
    B.block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
  }
}



/**
 * 功能： 四元数转欧拉角
 */
void quat_to_rpy(Quaternionf q, Matrix<fpt,3,1>& rpy)
{
  fpt as = t_min(-2.*(q.x()*q.z()-q.w()*q.y()),.99999);
  rpy(0) = atan2(2.f*(q.x()*q.y()+q.w()*q.z()),sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f*(q.y()*q.z()+q.w()*q.x()),sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));
}


/**
 * 功能：打印问题配置
 */
void print_problem_setup(problem_setup* setup)
{
  printf("DT: %.3f\n",setup->dt);
  printf("Mu: %.3f\n",setup->mu);
  printf("F_Max: %.3f\n",setup->f_max);
  printf("Horizon: %d\n",setup->horizon);
}


/**
 * 功能：打印更新数据
 */
void print_update_data(update_data_t* update, s16 horizon)
{
  print_named_array("p",update->p,1,3);
  print_named_array("v",update->v,1,3);
  print_named_array("q",update->q,1,4);
  print_named_array("w",update->r,3,4);
  pnv("Yaw",update->yaw);
  print_named_array("weights",update->weights,1,12);
  print_named_array("trajectory",update->traj,horizon,12);
  pnv("Alpha",update->alpha);
  print_named_array("gait",update->gait,horizon,4);
}


Matrix<fpt,13,1> x_0;       //初始状态
Matrix<fpt,3,3> I_world;    //惯性张量 世界坐标系下
Matrix<fpt,13,13> A_ct;     //储存连续状态转移矩阵
Matrix<fpt,13,12> B_ct_r;   //储存连续控制矩阵

/**
 * 功能：求解MPC问题过程
 */
void solve_mpc(update_data_t* update, problem_setup* setup)
{
  //////////////////////*（1）设置机器人状态，将数组值转换成矩阵，方便计算连续状态空间方程*/////////////////
    // p_   ：位置
    // v_   ：速度
    // q_   ：角速度
    // w_   ：四元数
    // r_   ：四脚从COM指向足端向量
    //yaw_ ：偏航角
  rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw);

  #ifdef K_PRINT_EVERYTHING//打印输出MPC问题的数据，和机器人数据，调试的时候用，真正运行的时候不用的 

    printf("-----------------\n");
      printf("   PROBLEM DATA  \n");
      printf("-----------------\n");
      print_problem_setup(setup);     //打印问题配置

      printf("-----------------\n");
      printf("    ROBOT DATA   \n");
      printf("-----------------\n");
      rs.print();                     //rs代表机器人状态，机器人状态打印，包括位置、线速度、角速度、旋转角、偏航角、足端位置、身体惯量
      print_update_data(update,setup->horizon);
  #endif

  /////////////////////////*(2)设置初始状态形式 (13个状态表示)把写入状态变量x_0中*//////////////////////////
    //欧拉角（3x1）【rpy(2), rpy(1), rpy(0)】
    //机身位置（3x1）【rs.p】
    //机身角速度（3*1）【rs.w】
    //机身速度（3*1）【rs.v】
    //重力【-9.8f】
  Matrix<fpt,3,1> rpy;     //定义一个欧拉角表达三个方向  roll pitch yaw
  quat_to_rpy(rs.q,rpy);   //把四元数方向形式 转成 欧拉角方向形式
  x_0 << rpy(2), rpy(1), rpy(0), rs.p , rs.w, rs.v, -9.8f;        

  ////////////////////////*（3）求世界坐标系下惯性矩阵，式（15）*///////////////////////////////////////////
  I_world = rs.R_yaw * rs.I_body * rs.R_yaw.transpose();    

  ////////////////////////*（4）求连续状态空间矩阵方程 式（16）和式（17）*//////////////////////////////////////              
  ct_ss_mats(I_world,rs.m,rs.r_feet,rs.R_yaw,A_ct,B_ct_r, update->x_drag);


#ifdef K_PRINT_EVERYTHING//调试的时候，如果要打印
    cout<<"Initial state: \n"<<x_0<<endl;           //打印初状态
    cout<<"World Inertia: \n"<<I_world<<endl;       //世界坐标系下的惯量
    cout<<"A CT: \n"<<A_ct<<endl;                   //状态转移矩阵A
    cout<<"B CT (simplified): \n"<<B_ct_r<<endl;    //控制矩阵B
#endif

  ////////////////////////*（5）计算QP矩阵*////////////////////////////////////////////////////////////
  c2qp(A_ct,B_ct_r,setup->dt,setup->horizon);    //计算得到A Bqp矩阵，把上一步求得的两个矩阵带进去得到
  

  ///////////////////////*（6）定义并更新更新权重矩阵*//////////////////////////////////////////////////////
  Matrix<fpt,13,1> full_weight;                  //定义weights权重矩阵
  for(u8 i = 0; i < 12; i++)                     //更新权重矩阵
    full_weight(i) = update->weights[i];
  full_weight(12) = 0.f;

  //////////////////////*（7）计算L矩阵，S是L矩阵*//////////////////////////////////////////////////////////
  S.diagonal() = full_weight.replicate(setup->horizon,1);//复制horizon次 3horizon*13horizon

  //////////////////////*（8）更新参考轨迹 这一步中所有参考 horizon*/////////////////////////////////////////
  for(s16 i = 0; i < setup->horizon; i++)             
  {
    for(s16 j = 0; j < 12; j++)
      X_d(13*i+j,0) = update->traj[12*i+j];
  }
 //cout<<"XD:\n"<<X_d<<endl;

  ///////////////////////*（9）建立约束不等式*////////////////////////////////////////////////////////////////////
  //note - I'm not doing the shifting here.
  s16 k = 0;
  for(s16 i = 0; i < setup->horizon; i++) //输入力边界 ，x,y方向力无穷的上边界             
  {
    for(s16 j = 0; j < 4; j++)//4条腿上分别是5条约束不等式的上边界
    {
      U_b(5*k + 0) = BIG_NUMBER;//x
      U_b(5*k + 1) = BIG_NUMBER;//x
      U_b(5*k + 2) = BIG_NUMBER;//y
      U_b(5*k + 3) = BIG_NUMBER;//y，值可以很大，看成是一个无穷大的值
      U_b(5*k + 4) = update->gait[i*4 + j] * setup->f_max;//z上的约束，这里的这个值可能会设置的很小，其他四个变量的值都是很大的，这个值悬空是fz的值很变得很小，怎么知道是悬空呐？是根据状态机预测得到的
      k++;
    }
  }

///////////////////////*（10）添加摩擦约束Fix=mu/*Fiz 式（22）~（24）*///////////////////////////////////////////
  fpt mu = 1.f/setup->mu;
  Matrix<fpt,5,3> f_block;    //定义一个5*3矩阵块

  f_block <<  mu, 0,  1.f,    //容器用<<传入数据，构成一个Ci阵
    -mu, 0,  1.f,
    0,  mu, 1.f,
    0, -mu, 1.f,
    0,   0, 1.f;

////////////////////////*(11)把约束组合成为约束矩阵*////////////////////////////////////////////////////////////////////
  for(s16 i = 0; i < setup->horizon*4; i++)//组合成约束矩阵， 4>=c>=3  才有解
  {
    fmat.block(i*5,i*3,5,3) = f_block;//对角阵 size= horizon*20,horizon*12//合成C
  }

////////////////////////*(12)计算矩阵H、g式子31、21*////////////////////////////////////////////////////////////////////
  qH = 2*(B_qp.transpose()*S*B_qp + update->alpha*eye_12h);//式（31）
  qg = 2*B_qp.transpose()*S*(A_qp*x_0 - X_d);//式（32）

////////////////////////*(13)计算二次规划问题（QP）*////////////////////////////////////////////////////////////////////
  //（1）先定义一个QP问题
  QpProblem<double> jcqp(setup->horizon*12, setup->horizon*20);
  
  //（2）QP问题解法选择
  /*使用qp求解器解法*/
  if(update->use_jcqp == 1) 
  {
    jcqp.A = fmat.cast<double>();
    jcqp.P = qH.cast<double>();
    jcqp.q = qg.cast<double>();
    jcqp.u = U_b.cast<double>();
    for(s16 i = 0; i < 20*setup->horizon; i++)
      jcqp.l[i] = 0.;

    jcqp.settings.sigma = update->sigma;
    jcqp.settings.alpha = update->solver_alpha;
    jcqp.settings.terminate = update->terminate;
    jcqp.settings.rho = update->rho;
    jcqp.settings.maxIterations = update->max_iterations;
    jcqp.runFromDense(update->max_iterations, true, false);
  }
  /*不使用qp求解器解法*/
  else 
  {
    //(1)矩阵格式转换成QP求解器能够处理的格式
    matrix_to_real(H_qpoases,qH,setup->horizon*12, setup->horizon*12);  //式（31）H矩阵  
    matrix_to_real(g_qpoases,qg,setup->horizon*12, 1);                  //式（32）3*4 = 12 4条腿 每条腿有3个变量，力的3个分量
    matrix_to_real(A_qpoases,fmat,setup->horizon*20, setup->horizon*12);//所求量约束矩阵
    matrix_to_real(ub_qpoases,U_b,setup->horizon*20, 1);                //约束上边界

    for(s16 i = 0; i < 20*setup->horizon; i++)// 每条腿5个约束不等式，4*5 =20 4条腿
      lb_qpoases[i] = 0.0f;                    //约束下边界

    //（2）处理约束问题
    s16 num_constraints = 20*setup->horizon;    //约束数量
    s16 num_variables = 12*setup->horizon;      //变量数
    qpOASES::int_t nWSR = 100;                  //指定在初始同伦期间要执行的最大工作集重新计算次数（在输出时它包含实际执行的工作集重新计算的次数！）
    int new_vars = num_variables;               //不悬空变量数
    int new_cons = num_constraints;             //不悬空约束数
	
   //（3）清空，准备简化QP矩阵
    for(int i =0; i < num_constraints; i++)    //把上一次存的数据清零
      con_elim[i] = 0;

    for(int i = 0; i < num_variables; i++)     //变量数
      var_elim[i] = 0;


    for(int i = 0; i < num_constraints; i++)//约束数
    {
     //遍历所有约束上下边界 如果边界值都在0附近，就操作赋值，否则跳过 即操作悬空腿
      if(! (near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i]))) continue; //上下限

      double* c_row = &A_qpoases[i*num_variables];        //边界值都在0附近找到其对应的约束矩阵的行  //取地址，放到指针变量中

    //把悬空足给找出来，悬空足的Fz方向的maxf很小
      for(int j = 0; j < num_variables; j++)              //遍历这一行的约束，因为一行包含了变量数，所以这里j最大数就是变量个数
      {
        if(near_one(c_row[j]))//如果约束参数接近1 即悬空足fz对应的约束，C约束阵 ，前面把把接近0的约束的数，在C阵中找到对应的约束值保存起来，这里如果约束值接近于1
        {
          new_vars -= 3;      //变量数-3	需要规划的三维足力少一组 ，力是3个维度的，分x,  y ,z三个方向
          new_cons -= 5;      //约束数-5	即悬空足所有约束无效 ，5条约束不等式
          int cs = (j*5)/3 -3;//找到悬空约束数的起始位置   //j*5是表示用变量数*5，之后/3是取整，这样就得到了对应的约束不等式，-3
          var_elim[j-2] = 1;  //将悬空的置1，将悬空足的3个变量，即3个力的分量全部置1
          var_elim[j-1] = 1;
          var_elim[j  ] = 1;
          con_elim[cs] = 1;
          con_elim[cs+1] = 1; //将悬空的置1
          con_elim[cs+2] = 1;
          con_elim[cs+3] = 1;
          con_elim[cs+4] = 1;
        }
      }
    }

     ///（4）移动中应用mpc算力
     //要计算的是不悬空触地脚的力，移动中总有悬空的腿，力有三个分力，分别对应x,y,z，每条腿对应5个约束不等式
    if(1==1)//意思就是不是完全站立，而是有悬空的腿出现的状态
    {
      int var_ind[new_vars];        //除去所有悬空腿后的支撑腿力 //储存不悬空足力序号
      int con_ind[new_cons];        //除去所有悬空腿后的支撑腿约束 //储存不悬空足约束序号
      int vc = 0;
      for(int i = 0; i < num_variables; i++)//12*horizon
      {
        if(!var_elim[i])//如果为0 就是不悬空
        {
          if(!(vc<new_vars))
          {
            printf("BAD ERROR 1\n");
          }
          var_ind[vc] = i;//储存不悬空足力序号//把不悬空的足力序号保存在这个数组中，这个数组上面新建的
          vc++;
        }
      }
      vc = 0;//结束后置零

      for(int i = 0; i < num_constraints; i++)
      {
        if(!con_elim[i])
        {
          if(!(vc<new_cons))
          {
            printf("BAD ERROR 1\n");
          }
          con_ind[vc] = i;//储存不悬空足约束序号
          vc++;
        }
      }

      for(int i = 0; i < new_vars; i++)//不悬空变量数，new_vars是去除了悬空变量数之后的不悬空变量数
      {
        int olda = var_ind[i];//把不悬空的变量数一个一个取出来
        g_red[i] = g_qpoases[olda];//不悬空足力序号来组成新的Qp问题梯度向量G，所以g_red[]是一个新的g
        for(int j = 0; j < new_vars; j++)
        {//不悬空足力序号来组成新的Qp问题（半）正定矩阵H 
		//12*horizon，12*horizon 但是不一定全用 下面同样
          int oldb = var_ind[j];
          H_red[i*new_vars + j] = H_qpoases[olda*num_variables + oldb];//不悬空足力序号来组成新的Qp问题（半）正定矩阵H 
        }
      }

//同理如上 定义新约束矩阵
      for (int con = 0; con < new_cons; con++)
      {
        for(int st = 0; st < new_vars; st++)
        {
          float cval = A_qpoases[(num_variables*con_ind[con]) + var_ind[st] ];
          A_red[con*new_vars + st] = cval;//新的C矩阵
        }
      }
	  	//同理如上 定义新约束上下边界
      for(int i = 0; i < new_cons; i++)
      {
        int old = con_ind[i];
        ub_red[i] = ub_qpoases[old];
        lb_red[i] = lb_qpoases[old];//新的上下边界约束
      }

      if(update->use_jcqp == 0)  /*不使用jcqp解法*/
      {
		  //设置qp问题 https://blog.csdn.net/weixin_40709533/article/details/86064148
        Timer solve_timer;
        qpOASES::QProblem problem_red (new_vars, new_cons);//创建了一个problem_red的类
        qpOASES::Options op;
        op.setToMPC();
        op.printLevel = qpOASES::PL_NONE;
        problem_red.setOptions(op);
        //int_t nWSR = 50000;


        int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
        //函数init会返回一个状态代码（类型为returnValue），表示初始化是否成功。可能的值是：
	//SUCCESSFUL_RETURN ：初始化成功（包括第一个QP的求解）。
	//RET_MAX_NWSR_REACHED：在给定的工作集重新计算数量内无法解决初始QP。
	//RET_INIT_FAILED（或更详细的错误代码）：初始化失败。
		(void)rval;
		//将最优原始解向量（dimension：nV）写入数组vq_red，该数组必须由用户分配（和释放）
	//会返回状态 ，如上
        int rval2 = problem_red.getPrimalSolution(q_red);//求解后的结果会保存再q_red这个变量中
		//输出求解状态
        if(rval2 != qpOASES::SUCCESSFUL_RETURN)//如果返回这个值，说明解算成功
          printf("failed to solve!\n");

        // printf("solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);

//按格式处理输出量
        vc = 0;
        for(int i = 0; i < num_variables; i++)
        {
          if(var_elim[i])
          {
            q_soln[i] = 0.0f;//输出悬空足力为零
          }
          else
          {
            q_soln[i] = q_red[vc];//输出足力
            vc++;
          }
        }
      } 
	  else  /*使用jcqp解法*/
    { // use jcqp == 2
        QpProblem<double> reducedProblem(new_vars, new_cons);
        reducedProblem.A = DenseMatrix<double>(new_cons, new_vars);
        int i = 0;
        for(int r = 0; r < new_cons; r++) 
        {
          for(int c = 0; c < new_vars; c++) 
          {
            reducedProblem.A(r,c) = A_red[i++];
          }
        }

        reducedProblem.P = DenseMatrix<double>(new_vars, new_vars);
        i = 0;
        for(int r = 0; r < new_vars; r++) 
        {
          for(int c = 0; c < new_vars; c++) 
          {
            reducedProblem.P(r,c) = H_red[i++];
          }
        }

        reducedProblem.q = Vector<double>(new_vars);
        for(int r = 0; r < new_vars; r++) 
        {
          reducedProblem.q[r] = g_red[r];
        }

        reducedProblem.u = Vector<double>(new_cons);
        for(int r = 0; r < new_cons; r++) 
        {
          reducedProblem.u[r] = ub_red[r];
        }

        reducedProblem.l = Vector<double>(new_cons);
        for(int r = 0; r < new_cons; r++) 
        {
          reducedProblem.l[r] = lb_red[r];
        }

        reducedProblem.settings.sigma = update->sigma;
        reducedProblem.settings.alpha = update->solver_alpha;
        reducedProblem.settings.terminate = update->terminate;
        reducedProblem.settings.rho = update->rho;
        reducedProblem.settings.maxIterations = update->max_iterations;
        reducedProblem.runFromDense(update->max_iterations, true, false);
        vc = 0;
        for(int kk = 0; kk < num_variables; kk++)
        {
          if(var_elim[kk])
          {
            q_soln[kk] = 0.0f;
          }
          else
          {
            q_soln[kk] = reducedProblem.getSolution()[vc];
            vc++;
          }
        }
    }
   }
  }


//输出计算的到的足端反作用力
  if(update->use_jcqp == 1) 
  {
    for(int i = 0; i < 12 * setup->horizon; i++) 
    {
      q_soln[i] = jcqp.getSolution()[i];
    }
  }

#ifdef K_PRINT_EVERYTHING
  //cout<<"fmat:\n"<<fmat<<endl;
#endif
}
