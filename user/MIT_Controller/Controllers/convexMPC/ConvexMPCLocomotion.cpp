//这个非常重要
#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>

#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../../../../common/FootstepPlanner/GraphSearch.h"

#include "Gait.h"

//#define DRAW_DEBUG_SWINGS
//#define DRAW_DEBUG_PATH


////////////////////
//原理
// Controller
// 参考Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control
// 一个步态周期由horizonLength(10)个mpc周期组成 
// 步态按1KHz处理 mpc计数间隔为30左右 一毫秒计数一次来控制频率 即一个mpc周期为30ms
// 则步态周期为 10*30 =300ms 
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
  iterationsBetweenMPC(_iterations_between_mpc),//MPC的迭代次数，控制频率用
  horizonLength(10),                            //MPC的预测未来状态数量，预测未来的10个时间步，值越大MPC计算量越大，越大和越小都不行，电机40KHZ不支持
  dt(_dt),                                      //控制周期
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),//以下的格式都一样，是一个默认设置。第一个向量时四条腿的的偏移量（相位差），第二个向量是mpc支撑腿的中一步持续时长（支撑时长）
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(4,4,4,4),"Bounding"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  jumping(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(2,2,2,2), "Jumping"),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(4,4,4,4),"Galloping"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
  trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),
  random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
  random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
{
  //（1）定义用户参数，根据传进来的形参
  _parameters = parameters;            
  //（2）定义mpc运算周期，公式：mpc运算周期=迭代次数*控制周期                 
  dtMPC = dt * iterationsBetweenMPC;   
  //（3）定义MPC的迭代次数
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  //（4）二次规划器QP的参数配置，配置1）mpc运算周期、2）MPC的预测未来状态数量（分段数）、3）摩擦系数、4）最大力
  setup_problem(dtMPC, horizonLength, 0.4, 120);        
  //（5）初始化变量欧拉角数据为0
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;
  //（6）初始化四条腿的状态初次都是摆动状态
  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;
  //（7）初始化稀疏MPC
  initSparseMPC();
  //（8）初始化期望的位置、速度、加速度为0
   pBody_des.setZero();
   vBody_des.setZero();
   aBody_des.setZero();
}



/**
 * 功能：初始化MPC
 */
void ConvexMPCLocomotion::initialize()
{
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
}


/**
 * 功能：重新计算 MPC时间
 */
void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) 
{
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

/**
 * 功能：设置躯干一系列的期望值，包括躯干高度、躯干位姿【即躯干的位置和方向】
 */
void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> & data)
{
	/////////////////*（1）根据机型设置默认机体期望高度*//////////////////////////////////////////
  if(data._quadruped->_robotType == RobotType::MINI_CHEETAH)//若是MINI_CHEETAH机器人
  {
    _body_height = 0.29;
  }
  else if(data._quadruped->_robotType == RobotType::CHEETAH_3)//若是CHEETAH_3机器人
  {
    _body_height = 0.45;
  }
  else                                                        //若既不是MINI_CHEETAH机器人，也不是CHEETAH_3机器人，报错
  {
    assert(false);
  }


//////////////////*（2）使用或者不使用遥控手柄来设置XY\偏航的速度命令、步态命令、机身高度*////////
  float x_vel_cmd, y_vel_cmd;   //定义机体XY两个方向的速度命令
  float filter(0.1);            //定义一个过滤器的权重变量，值为0.1，这个值决定期望状态的变化幅度
  //若使用遥控手柄控制
  if(data.controlParameters->use_rc)    
  {
    const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand;//（1）获得遥控命令
    data.userParameters->cmpc_gait = rc_cmd->variable[0];                    //（2）获得步态类型命令
    _yaw_turn_rate = -rc_cmd->omega_des[2];                                  //（3）机身偏航方向速度
    x_vel_cmd = rc_cmd->v_des[0];                                            //（4）机身X方向速度命令
    y_vel_cmd = rc_cmd->v_des[1] * 0.5;                                      //（5）机身Y方向速度命令
    _body_height += rc_cmd->height_variation * 0.08;                         //（6）调整机身高度
  }

  //默认，若不使用遥控手柄控制
  else                                  
  {
    _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];    //（1）机身偏航方向速度
    x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];          //（2）机身X方向速度命令
    y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];          //（3）机身Y方向速度命令
  }
  


//////////////////*（3）通过上一次的期望数据和本次遥控手柄输入的期望数据，计算机体的期望位姿【即XY的期望位置、期望偏航角度】*/////

  /*（1）计算本次机体XY期望速度*/
  //设置本次X方向期望速度，注意：这里的计算使经过权重滤波的
  //公式：本次X方向期望速度=上一次X方向期望速度*（1-过滤器的权重变量）+遥控器机身X方向速度命令*过滤器的权重变量
  _x_vel_des = _x_vel_des*(1-filter) + x_vel_cmd*filter; 

  //计算本次Y方向期望速度 ，注意：这里的计算使经过权重滤波的
  //公式：本次Y方向期望速度=上一次Y方向期望速度*（1-过滤器的权重变量）+遥控器机身Y方向速度命令*过滤器的权重变量
  _y_vel_des = _y_vel_des*(1-filter) + y_vel_cmd*filter;    

  /*（2）计算本次机体期望PRY方向*/
   //计算本次期望偏航角度
  //公式：本次期望偏航角度=状态估计的偏航角度+（一般频率下时间间隔*遥控器机身偏航方向速度）
  _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;  //计算偏航角度
  _roll_des = 0.;   //默认横滚角=0
  _pitch_des = 0.;  //默认俯仰角=0
}



/**
 * 功能：MPC运行逻辑函数（重点）
 */
template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data) 
{
  /////////////////////*（1）设置机体高度、位姿和步态类型*//////////////////////////////////////////
  bool omniMode = false;                               //关闭全方位操控模式
  _SetupCommand(data);                                 //设置机体一系列的期望值，包括机体高度、机体位姿【即机体的位置和方向】
  gaitNumber = data.userParameters->cmpc_gait;         //获得步态类型
  if(gaitNumber >= 10)                                 //选择步态模式 并开启全方位
  {
    gaitNumber -= 10;
    omniMode = true;                                   //开启全方位操控模式
  }


//////////////////////*（2）根据状态估计返回值设置机体方向、位置和速度*////////////////////////////////////
//注意状态估计器使一直在周期任务管理器中运行的，所以随时可以直接获取状态估计的结果
  auto& seResult = data._stateEstimator->getResult();  

/////////////////////*（3）检查是否过渡到站立状态*/////////////////////////////////////////////////////
//判断条件：当前步态不是站立，但是设置的下一个步态枚举量4使代表站立，或者使第一次运行
//作用：再刚步态切换后，提供初始逻辑数据
  if(((gaitNumber == 4) && current_gait != 4) || firstRun)  
  {
    stand_traj[0] = seResult.position[0];        //1）设置当前机体质心X方向位置
    stand_traj[1] = seResult.position[1];        //2）设置当前机体质心Y方向位置
    stand_traj[2] = 0.21;                        //3）设置机体质心位置Z方向高度
    stand_traj[3] = 0;                           //4）设置机体横滚角度
    stand_traj[4] = 0;                           //5）设置机体俯仰角度
    stand_traj[5] = seResult.rpy[2];             //6）设置机体偏航角度
    world_position_desired[0] = stand_traj[0];   //7）设置世界坐标系下，当前机体质心位置x
    world_position_desired[1] = stand_traj[1];   //8）设置世界坐标系下，当前机体质心位置y
  }

  ////////////////////*（4）根据枚举变量选择步态类型*//////////////////////////////////////////////////
  Gait* gait = &trotting;    //声明一个类指针，并初始化为原地踏步步态
  if(gaitNumber == 1)        //colocation的构造函数中初始化列表中已经给各种步态赋值了，这根据gaitNumber的值确定用那种步态
    gait = &bounding;        //gait是一个指针类，放的是一个指向类的指针变量，&bounding是返回步态类的初地址，只是gait就指向了对应的步态了
  else if(gaitNumber == 2)
    gait = &pronking;
  else if(gaitNumber == 3)
    gait = &random;
  else if(gaitNumber == 4)
    gait = &standing;
  else if(gaitNumber == 5)
    gait = &trotRunning;
  else if(gaitNumber == 6)
    gait = &random2;
  else if(gaitNumber == 7)
    gait = &random2;
  else if(gaitNumber == 8)
    gait = &pacing;
  current_gait = gaitNumber;//当前步态，，gaitNumber就可以表示当前步态类型了，因为前面也是根据这个是，确定步态参数的

  ////////////////////*（5）步态周期及相位计算*///////////////////////////////////////////////////////////
  gait->setIterations(iterationsBetweenMPC, iterationCounter);  
  
  ////////////////////*（6）跳跃周期及相位计算*//////////////////////////////////////////////////////////
  jumping.setIterations(iterationsBetweenMPC, iterationCounter);
  jumping.setIterations(27/2, iterationCounter);                
  //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
  
  
  ///////////////////*（7）判断跳跃是否触发*//////////////////////////////////////////////////////
  jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
      data._desiredStateCommand->trigger_pressed); //跳跃相关//检查有没触发跳跃
  // bool too_high = seResult.position[2] > 0.29;
  
  
  ///////////////////*（8）判断是否需要跳跃*/ ////////////////////////////////////////////////////
  if(jump_state.should_jump(jumping.getCurrentGaitPhase())) //若需要跳跃
  {
    gait = &jumping;                          //1)指针给指出步态类型，步态实现切换
    recompute_timing(27/2);                   //2)重新计算 MPC时间
    _body_height = _body_height_jumping;      //3)重新设置机体的高度
    currently_jumping = true;                 //4)开启跳跃的标志位

  }
  else                                                      //若不需要跳跃 
  {
    recompute_timing(default_iterations_between_mpc);//1)重新计算 MPC时间
    currently_jumping = false;                       //2)关闭跳跃的标志位
  }

 ///////////////////*（9）机身高度限幅，不能太低*///////////////////////////////////////////////////
  if(_body_height < 0.02)     
  {
    _body_height = 0.29;
  }

  //////////////////*（10）把期望状态转换到世界坐标系下 因为mpc运算都在世界坐标系下*//////////////////
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);                 //（1）定义并赋值在机体坐标系下的期望vx vy vz速度
  Vec3<float> v_des_world = 
    omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;//（2）计算世界坐标下的机体期望速度，把机体期望速度转换到世界坐标下
  Vec3<float> v_robot = seResult.vWorld;                              //（3）定义并赋值当前机器人世界坐标系下速度

 //////////////////*（11）俯仰角、横滚角积分进行补偿、角度限幅，再转换到机器人世界坐标系下表达*///////////////
  //俯仰角、横滚角积分进行补偿
  if(fabs(v_robot[0]) > .2)   //判断一下v_robot[0]是否为0，避免除以0产生错误
  {
    rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];//俯仰角度积分
  }
  if(fabs(v_robot[1]) > 0.1)  //判断一下v_robot[1]是否为0，避免除以0产生错误
  {
    rpy_int[0] += dt*(_roll_des - seResult.rpy[0])/v_robot[1];//横滚角度积分
  }
  
  //横滚角、偏航角的初始角度限幅
  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);//rpy 中的r即横滚角
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);//rpy 中的y即偏航角
  //横滚角、偏航角转换到机器人世界坐标系下表达
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking

/////////////////*（12）把足端位置转换到世界坐标下 *//////////////////////////////////////////////////////
//公式：机身坐标  +  机身旋转矩阵^T  *  （髋关节在机身下坐标+腿在髋关节下坐标）
//备注：这个是运动学关系的转换
  for(int i = 0; i < 4; i++) 
  {
    pFoot[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);
  }

//////////////*（13）计算躯干的最终期望位置*/////////////////////////////////////////////////////////////////
//通过累加目标速度完成，公式：x=v*t
  if(gait != &standing) //在非站立步态时
  {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

//////////////*（14）第一次初始化时，确定质心的位姿（XY速度+偏航角度）和足端起点、高度、落点*/////////////////////////////
//作用是第一次初始化的起始速度给定
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0];       //当前机体质心位置X
    world_position_desired[1] = seResult.position[1];       //当前机体质心位置Y
    world_position_desired[2] = seResult.rpy[2];            //当前机体质心偏航角度y

    for(int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(0.05);             //抬腿高度
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);//摆动足轨迹初始点
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);  //摆动足轨迹结束点
    }
    firstRun = false;//第一次运行的标志位
  }


  //////////////*（15）获取摆动时间 */ ////////////////////////////////////////////////////////////
  for(int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  float side_sign[4] = {-1, 1, -1, 1};                //四条腿的编号 (-1表示右腿，+1表示左腿) RF LF RH LH
  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  float interleave_gain = -0.2;
  float v_abs = std::fabs(v_des_robot[0]);            //机体的x速度


   /////////////*（16）计算下一刻腿的足端落脚点位置坐标，传给贝塞尔生成器使用*////////////////////////
  for(int i = 0; i < 4; i++)
  {
    //（1）设置摆动腿时间
    if(firstSwing[i]) //腿若是第一次摆动，定义一个摆动计时器，获取初始的摆动腿时间
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else              //腿若是第二次摆动，摆动腿时间倒计时递减
    {
      swingTimeRemaining[i] -= dt;//摆动计时器倒计时
    }

    //（2）设置抬腿高度，0.06应该是6cm的意思，摆动腿足端轨迹生成相关
    footSwingTrajectories[i].setHeight(.06);
    
    //(3)计算行走状态下落足点  
	Vec3<float> offset(0, side_sign[i] * .065, 0);                          //定义关节偏置
  Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);//求得修正后i腿hip关节坐标在机身坐标系
  pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;            //落足点Y修正
	
  //（4）设置支撑腿时间
  float stance_time = gait->getCurrentStanceTime(dtMPC, i);

  //（5）修正偏航角
  Vec3<float> pYawCorrected = 
  coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;
 //上面的公式是WBC13式中的Ii


  //（6）机身坐标系下期望速度
    Vec3<float> des_vel;      //定义躯干期望速度
    des_vel[0] = _x_vel_des;  //设置躯干X方向的期望速度
    des_vel[1] = _y_vel_des;  //设置躯干X方向的期望速度
    des_vel[2] = 0.0;         //设置躯干Z方向的期望速度=0，默认为0
	
  //（7）计算下一腿足端坐标的位置
   //原理：世界坐标系下，以剩余摆动时间内匀速运动来估计下一腿足端坐标的位置
   //下一腿足端坐标的位置的计算公式为：
   //下一腿足端坐标的位置=腿部当前位置+变换矩阵*（修正的偏航位置+期望速度*剩余的摆动时间）
   //Pf存放的是下一腿足端坐标的位置
    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
          + des_vel * swingTimeRemaining[i]);

    float p_rel_max = 0.3f;


	//（8）在世界坐标系下，计算摆动腿的最终落脚点
  //备注：//分别是式14+15式的结果，公式中的k为.03f
    float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
      .03f*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);

    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
      .03f*(seResult.vWorld[1]-v_des_world[1]) +                                //0.03f是式子的k
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate); // seResult.vWorld[1]式中的v ，v_des_world[1]式中的vcmd
	  
	 //（9）参考的XY坐标点限幅
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);

	//（10）计算世界坐标系下的落足点 ，WBC论文中 最终的12式
    Pf[0] +=  pfx_rel;  //x坐标
    Pf[1] +=  pfy_rel;  //y坐标
    Pf[2] = -0.003;     //z坐标，一个极小值

   //（11）计算摆动腿的最终落脚点，下一时刻的落脚点，贝塞尔曲线的下一个点
    footSwingTrajectories[i].setFinalPosition(Pf);

  }

  //////////////*（17）设置步态KP\KD的增益*///////////////////////////////////////////////////////
  iterationCounter++;
  //  加载LCM腿摆动增益
  Kp << 700, 0, 0,
     0, 700, 0,
     0, 0, 150;
  Kp_stance = 0*Kp;

  Kd << 7, 0, 0,
     0, 7, 0,
     0, 0, 7;
  Kd_stance = Kd;

  //////////////*（18）获取当前是接触状态还是摆动状态*////////////////////////////////////////////
  //两个状态表示，摆动状态进行到哪了，触地状态进行到哪了
  Vec4<float> contactStates = gait->getContactState();  //获取接触状态 在整个支撑过程百分比(从0到1)  完成后为0
  Vec4<float> swingStates = gait->getSwingState();      //获取摆动状态 在整个摆动过程百分比(从0到1) 完成后为0

  /////////////*（19）迭代更新计算MPC，并解算足端反作用力*/////////////////////////////////////////
  int* mpcTable = gait->getMpcTable();        //1）为mpc准备足端接触信息 从当前时刻预测之后一个步态周期的接触信息，获取触地与摆动状态，方便mpc解算的化简。
  updateMPCIfNeeded(mpcTable, data, omniMode);//2）【重要！！！】更新迭代计算mpc期望值，并解算足力


#ifdef DRAW_DEBUG_PATH//与轨迹调试有关系，运行的时候不会调用的
  auto* trajectoryDebug = data.visualizationData->addPath();
  if(trajectoryDebug) 
  {
    trajectoryDebug->num_points = 10;
    trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
    for(int i = 0; i < 10; i++) {
      trajectoryDebug->position[i][0] = trajAll[12*i + 3];
      trajectoryDebug->position[i][1] = trajAll[12*i + 4];
      trajectoryDebug->position[i][2] = trajAll[12*i + 5];
      auto* ball = data.visualizationData->addSphere();
      ball->radius = 0.01;
      ball->position = trajectoryDebug->position[i];
      ball->color = {1.0, 0.2, 0.2, 0.5};
    }
  }
#endif

///////////////*（20）判断当前脚处于支撑状态还是摆动状态，并对两个状态进行控制*//////////////////////
  Vec4<float> se_contactState(0,0,0,0);//定义估计的接触状态
  for(int foot = 0; foot < 4; foot++)
  {
/*（1）判断当前脚处于支撑状态还是摆动状态*/
    float contactState = contactStates[foot];  //在整个支撑过程百分比(从0到1)
    float swingState = swingStates[foot];      //在整个摆动过程百分比(从0到1)
  
/*（2）【重要！！！】对摆动足进行控制*/
    if(swingState > 0)  
    {
      if(firstSwing[foot]) //（1）若刚从支撑相切换到摆动相
      {
        firstSwing[foot] = false;                                   //标志位置0
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);//初始化设置起始点为原点 
      }

#ifdef DRAW_DEBUG_SWINGS//与摆动调试有关系，运行的时候不会调用的
      auto* debugPath = data.visualizationData->addPath();
      if(debugPath) 
      {
        debugPath->num_points = 100;
        debugPath->color = {0.2,1,0.2,0.5};
        float step = (1.f - swingState) / 100.f;
        for(int i = 0; i < 100; i++) //贝塞尔曲线的轨迹，分100个点
        {
          footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
          debugPath->position[i] = footSwingTrajectories[foot].getPosition();
        }
      }

      auto* finalSphere = data.visualizationData->addSphere();
      if(finalSphere) 
      {
        finalSphere->position = footSwingTrajectories[foot].getPosition();
        finalSphere->radius = 0.02;
        finalSphere->color = {0.6, 0.6, 0.2, 0.7};
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);//【重点！！！】计算贝塞尔曲线
      auto* actualSphere = data.visualizationData->addSphere();
      auto* goalSphere = data.visualizationData->addSphere();
      goalSphere->position = footSwingTrajectories[foot].getPosition();
      actualSphere->position = pFoot[foot];
      goalSphere->radius = 0.02;
      actualSphere->radius = 0.02;
      goalSphere->color = {0.2, 1, 0.2, 0.7};
      actualSphere->color = {0.8, 0.2, 0.2, 0.7};
#endif


    //（2）使用贝塞尔曲线，计算腿部足端轨迹 
    //输入参数：当前在摆动相的位置，摆动相时间长度
	  footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);//计算贝塞尔曲线


    //（3）获得足端轨迹位置点和速度点
      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();  //获得轨迹位置点
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();  //获得轨迹速度点

	  //（4）足端轨迹位置点和速度点先转换到机体坐标系下，然后再转到hip关节坐标系下
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) //先转换到机体坐标系等同于到机体坐标系下
        - data._quadruped->getHipLocation(foot);
		
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);  //再转换到机体坐标系等同于到髋关节坐标系下

      //（5）【默认使用wbc控制】为wbc控制更新参数，参数包括腿部位置、腿部速度、腿部加速度
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
      
      //（6）判断是否使用WBC控制 
      if(!data.userParameters->use_wbc)//若不使用WBC,直接发送腿部控制命令到腿部控制器 
      {
        //不管WBIC的使用情况如何，更新腿部控制命令都是这几个
        //用腿部控制器控制腿部的运动
        data._legController->commands[foot].pDes = pDesLeg;   //腿部期望位置
        data._legController->commands[foot].vDes = vDesLeg;   //腿部期望速度
        data._legController->commands[foot].kpCartesian = Kp; //腿部KP增益
        data._legController->commands[foot].kdCartesian = Kd; //腿部KD增益
      }
    }
    

/*（3）【重要！！！】对支撑足进行控制*/    
    else 
    {
      firstSwing[foot] = true; //设置刚从摆动相切换到支撑相状态

#ifdef DRAW_DEBUG_SWINGS
      auto* actualSphere = data.visualizationData->addSphere();
      actualSphere->position = pFoot[foot];
      actualSphere->radius = 0.02;
      actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

//（1）根据运动学坐标转换，通过摆动腿的位置和速度计算，足端的位置和速度
      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition(); //获取摆动腿的位置
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity(); //获取摆动腿的速度
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);//计算腿部足端位置
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);                                          //计算腿部足端速度
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

//（2）若未使用WBC 发送数据到腿部控制器
      if(!data.userParameters->use_wbc)
      {
        data._legController->commands[foot].pDes = pDesLeg;                           //四条腿腿部期望位置命令
        data._legController->commands[foot].vDes = vDesLeg;                           //四条腿腿部期望速度命令
        data._legController->commands[foot].kpCartesian = Kp_stance;                  //四条腿腿部KP参数命令
        data._legController->commands[foot].kdCartesian = Kd_stance;                  //四条腿腿部KD参数命令
        data._legController->commands[foot].forceFeedForward = f_ff[foot];            //四条腿腿部反作用力命令
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;  //关节KD参数命令
      }
//（3）若使用WBC,直接发送数据到腿部控制器
      else                            
      { 
        data._legController->commands[foot].pDes = pDesLeg;                           //四条腿腿部期望位置命令
        data._legController->commands[foot].vDes = vDesLeg;                           //四条腿腿部期望速度命令
        data._legController->commands[foot].kpCartesian = 0.*Kp_stance;               //四条腿腿部KP参数命令
        data._legController->commands[foot].kdCartesian = Kd_stance;                  //四条腿腿部KD参数命令
      }
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
//（4）更新接触状态估计
      se_contactState[foot] = contactState; 

      //为WBC做好更新准备了
    }
  }

///////////////*（21）设置接触状态到估计器*///////////////////////////////////////////////////
  data._stateEstimator->setContactPhase(se_contactState); 

///////////////*（22）更新wbc的参数 ，这些参数都是wbc需要用到的状态量*/////////////////////////
  pBody_des[0] = world_position_desired[0]; //（1）世界坐标系下期望的X位置
  pBody_des[1] = world_position_desired[1]; //（2）世界坐标系下期望的Y位置
  pBody_des[2] = _body_height;              //（3）机体高度    

  vBody_des[0] = v_des_world[0];            //（4）世界坐标系下期望的X速度
  vBody_des[1] = v_des_world[1];            //（5）世界坐标系下期望的Y速度
  vBody_des[2] = 0.;                        //（6）世界坐标系下期望的Z速度=0

  aBody_des.setZero();                      //（7）世界坐标系下期望的加速度=0

  pBody_RPY_des[0] = 0.;                    //（8）世界坐标系下期望的横滚角=0
  pBody_RPY_des[1] = 0.;                    //（9）世界坐标系下期望的俯仰角=0
  pBody_RPY_des[2] = _yaw_des;              //（10）世界坐标系下期望的偏航角

  vBody_Ori_des[0] = 0.;                    //（11）世界坐标系下期望的横滚角速度=0
  vBody_Ori_des[1] = 0.;                    //（12）世界坐标系下期望的俯仰角速度=0
  vBody_Ori_des[2] = _yaw_turn_rate;        //（13）世界坐标系下期望的偏航角速度

  contact_state = gait->getContactState();  //（14）步态接触状态
  // END of WBC Update
}


/**
 * 功能：MPC运行过程逻辑函数（重点），重载函数
 */
template<>
void ConvexMPCLocomotion::run(ControlFSMData<double>& data) 
{
  (void)data;
  printf("call to old CMPC with double!\n");

}



/**
 * 功能：定时迭代更新计算MPC，并解算足端反作用力（重点）
 */
void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode) 
{
  //iterationsBetweenMPC = 30;                            //设置计算MPC的间隔时间
  if((iterationCounter % iterationsBetweenMPC) == 0)      //当iterationCounter等于计算MPC的间隔时间，就进行一次MPC计算，用于控制频率
  {
  ///////////*（1）先获取状态估计器的数据，并计算世界坐标系下的机体速度*////////////////////////////////////////////////////////////////////////////////////
    auto seResult = data._stateEstimator->getResult();                                            //1）先获取状态估计器的数据，包括躯干得位置、方向、速度
  	float* p = seResult.position.data();                                                          //2）定义并赋值在世界坐标系下的机体位置 
    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);                                            //3）定义并赋值身体坐标系下的机体速度
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;  //4）定义并赋值世界坐标系下的机体速度

  ///////////*（2）判断当前状态为站立状态还是非站立状态*//////////////////////////////////////////////////////////////
  //若当前为站定状态时
    if(current_gait == 4)
    {  
      //（1）初始化12个躯干轨迹参数
      float trajInitial[12] =   
      {
        _roll_des,            //1）期望横滚角
        _pitch_des            //2）期望俯仰角    /*-hw_i->state_estimator->se_ground_pitch*/,
        (float)stand_traj[5]  //3）期望偏航角    /*+(float)stateCommand->data.stateDes[11]*/,//Yaw
        (float)stand_traj[0]  //4）X位置        /*+(float)fsm->main_control_settings.p_des[0]*/,//X
        (float)stand_traj[1]  //5）Y位置        /*+(float)fsm->main_control_settings.p_des[1]*/,//Y
        (float)_body_height   //6）Z位置        /*fsm->main_control_settings.p_des[2]*/,//Z
        0,0,0,0,0,0           //7）其他参数设置为0    
      };
		
      //（2）变成mpc问题需要的格式
      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];//把12个躯干轨迹参数整理成MPC问题的矩阵格式
    }

  //（2）若当前为非站定状态时
    else                 
    {
      //(0)定义并赋值目标值：轨迹的X、Y参数，轨迹跟踪误差
      const float max_pos_error = .1;            //定义并赋值轨迹跟踪误差为0.1
      float xStart = world_position_desired[0];  //定义并赋值轨迹的X参数设定为期望的X位置
      float yStart = world_position_desired[1];  //定义并赋值轨迹的Y参数设定为期望的Y位置
	  
      //（1）把目标值限制在误差范围内
        //轨迹的X参数在误差范围外时，显示其在最大误差边界
      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
     
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;
        //轨迹的Y参数在误差范围外时，显示其在最大误差边界
      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
     
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;
	  
      //（2）机体轨迹参数初始化
      float trajInitial[12] = {
                (float)rpy_comp[0],  	                  	// 0
								(float)rpy_comp[1],   					          // 1
								_yaw_des,    							                // 2
								//yawStart,                               // 2
								xStart,                                   // 3
								yStart,                                   // 4
								(float)_body_height,      		      			// 5
								0,                                        // 6
								0,                                        // 7
								_yaw_turn_rate,  						              // 8
								v_des_world[0],                           // 9
								v_des_world[1],                           // 10
								0};                                       // 11

    //（3）变成mpc问题需要的格式
      //轨迹为当前时刻向后预测一个步态周期的轨迹 按匀速运动计算
      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) //从当前位置开始
        {
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }

    Timer solveTimer;
    
    //（4）判断解算离散MPC还是稠密MPC，得到足端反作用力
    if(_parameters->cmpc_use_sparse > 0.5) 
    {
      solveSparseMPC(mpcTable, data);
    } 
    else //一般都是计算稠密mpc，得到足端反作用力
    {
      solveDenseMPC(mpcTable, data);
    }
  }

}



/**
 * 功能：解算solveDenseMPC，得到足端反作用力
 */
void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) 
{
  ///////////////*（1）先通过状态估计器获取世界坐标下状态估计值，赋值给机体对应的数据*///////////////////////////////
  auto seResult = data._stateEstimator->getResult();      

  float Q[12] = {0.25, 0.25, 10,         //1）定义Q矩阵权重
                  2, 2, 50, 
                  0, 0, 0.3, 
                  0.2, 0.2, 0.1};
  float yaw = seResult.rpy[2];           //2）偏航角
  float* weights = Q;                    //3）权重Q矩阵
  float alpha = 4e-5;                    //4）alpha
  float* p = seResult.position.data();   //5）机体位置
  float* v = seResult.vWorld.data();     //6）机体速度
  float* w = seResult.omegaWorld.data(); //7）机体角速度
  float* q = seResult.orientation.data();//8）机体方向


  /////////////*（2）计算从质心指向足端的向量*///////////////////////////////////////////////////////////////////
  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];//质心指向足端的向量=足端坐标（x,y,z)减去质心的位置坐标（x,y,z)

  ////////////*(3)Alpha值限幅*//////////////////////////////////////////////////////////////////////////////
  if(alpha > 1e-4) 
  {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  ///////////*（4）计算质心高度上的偏差*///////////////////////////////////////////////////////////////////////////////
  float pz_err = p[2] - _body_height;


  //////////*（5）设置MPC的参数*/////////////////////////////////////////////////////////////////////////////////////
  Timer t1;                                                                       //1）定义第一个计时器
  Vec3<float> pxy_act(p[0], p[1], 0);                                             //2）定义从IMU状态估计器中获取的机体XY位置的中间变量
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);   //3）定义从世界坐标系中获取的期望机体XY位置的中间变量
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);                     //4）定义从IMU状态估计器中获取的机体XY速度的中间变量

  /*5）设置计算MPC时间间隔 */
  dtMPC = dt * iterationsBetweenMPC;              

  /*6）二次规划器QP的参数配置*/
  setup_problem(dtMPC,horizonLength,0.4,120);     
  
  /*7）z轴方向加速度受x轴方向速度的影响程度*/
  update_x_drag(x_comp_integral);                 
  
  /*8）机体XY速度过大/过小处理*/
  if(vxy[0] > 0.3 || vxy[0] < -0.3) 
  {
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }
  /*9）更新qp求解器设置*/
  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
      _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
//t1.stopPrint("Setup MPC");




  ////////*（6）解MPC的过程，得到足端反作用力*///////////////////////////////////////////////////////////////////////////////////// 
  //trajAll放的是期望状态轨迹
    Timer t2;   //定义第二个计时器，测量解一次MPC的时间需要多久
  //cout << "dtMPC: " << dtMPC << "\n";

  update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable); //【非常重要】更新QP问题参数和解，每次解算出的horizon个组力 只取第一组 convex_mpc 5节开头
  //t2.stopPrint("Run MPC");
  //printf("MPC Solve time %f ms\n", t2.getMs());

 /////////*（7）将世界坐标下力转换到机体坐标下*////////////////////////////////////////////////////////////////////
  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg*3 + axis);//获取索引号的结果
    f_ff[leg] = -seResult.rBody * f;       //将世界坐标下力转换到机体坐标下
    Fr_des[leg] = f;                       //给WBC更新力
  }
}


/**
 * 功能：解算SparseMPC，得到足端反作用力
 */
void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, ControlFSMData<float> &data)
{
  (void)mpcTable;
  (void)data;
  auto seResult = data._stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for(int i = 0; i < horizonLength; i++) 
  {
    contactStates.emplace_back(mpcTable[i*4 + 0], mpcTable[i*4 + 1], mpcTable[i*4 + 2], mpcTable[i*4 + 3]);
  }

  for(int i = 0; i < horizonLength; i++)
  {
    for(u32 j = 0; j < 12; j++) 
    {
      _sparseTrajectory[i][j] = trajAll[i*12 + j];
    }
  }

  Vec12<float> feet;
  for(u32 foot = 0; foot < 4; foot++) 
  {
    for(u32 axis = 0; axis < 3; axis++) 
    {
      feet[foot*3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for(u32 foot = 0; foot < 4; foot++) 
  {
    Vec3<float> force(resultForce[foot*3], resultForce[foot*3 + 1], resultForce[foot*3 + 2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}


/**
 * 功能：初始化稀疏MPC
 */
void ConvexMPCLocomotion::initSparseMPC() 
{
  Mat3<double> baseInertia;  //定义机体惯量
  baseInertia << 0.07, 0, 0,
              0, 0.26, 0,
              0, 0, 0.242;
  double mass = 9;          //定义质量
  double maxForce = 120;    //定义最大力矩

  std::vector<double> dtTraj;
  for(int i = 0; i < horizonLength; i++) 
  {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;    //定义权重
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);//设置机器人参数
  _sparseCMPC.setFriction(0.4);                               //设置摩擦系数
  _sparseCMPC.setWeights(weights, 4e-5);                      //设置权重
  _sparseCMPC.setDtTrajectory(dtTraj);                        //设置轨迹

  _sparseTrajectory.resize(horizonLength);
}

