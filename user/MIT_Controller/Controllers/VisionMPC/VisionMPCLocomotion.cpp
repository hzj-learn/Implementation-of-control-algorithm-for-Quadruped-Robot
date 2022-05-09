#include <iostream>
#include <Utilities/Utilities_print.h>

#include "VisionMPCLocomotion.h"
#include "VisionMPC_interface.h"


///////////////
//原理：
// GAIT  参考Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains·（1）
// 和 Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control·（2）
// 一个步态周期由horizonLength(10)个mpc周期组成 
// 步态按1KHz处理 mpc计数间隔为30左右 一毫秒计数一次来控制频率 即一个mpc周期为30ms
// 则步态周期为 10*30 =300ms 
// 一个步态周期被分为horizonLength(10)段 
// offsets  durations _offsetsFloat _durationsFloat 
// 是步态的相关相位参数，支撑时间参数 前两个用整数即分段来计算，后两个则是在0~1之间
///////////////

/**
 * 功能：视觉步态构造函数
 */
VisionGait::VisionGait(int nMPC_segments, Vec4<int> offsets, Vec4<int>  durations, const std::string& name="")：
    Vec4<int> durations, const std::string &name) :
  _offsets(offsets.array()),
  _durations(durations.array()),    //支撑时间
  _nIterations(nMPC_segments)       //步态周期分段数
{
  _mpc_table = new int[nMPC_segments * 4];                          // 脚是否悬空
  _offsetsFloat = offsets.cast<float>() / (float) nMPC_segments;    //对应步态的腿之间相位关系  应该？
  _durationsFloat = durations.cast<float>() / (float) nMPC_segments;//对应步态支撑持续时间在整个周期百分比 支撑相结束标志点 应该？
  std::cout << "VisionGait " << name << "\n";
  std::cout << "nMPC_segments    : " << _nIterations << "\n";
  std::cout << "offsets (int)    : " << _offsets.transpose() << "\n";
  std::cout << "durations (int)  : " << _durations.transpose() << "\n";
  std::cout << "offsets (float)  : " << _offsetsFloat.transpose() << "\n";
  std::cout << "durations (float): " << _durationsFloat.transpose() << "\n";
  std::cout << "\n\n";

  _stance = durations[0];                //支撑持续时间 用整个过程的分段计数
  _swing = nMPC_segments - durations[0]; //摆动持续时间 用整个过程的分段计数
}

/**
 * 功能：视觉步态析构函数
 */
VisionGait::~VisionGait() 
{
  delete[] _mpc_table;
}

/**
 * 功能：设置MPC迭代
 */
void VisionGait::setIterations(int iterationsPerMPC, int currentIteration) 
{
	//细分为 nMPC_segments（10）个时间步 参考（2） A.Experimental Setup
//当前在第几个步态分段中 0~9
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations; 
  //当前在整个步态周期百分比 一个步态周期为 nMPC_segments（10） 个mpc周期
  //               当前为站立			重复计数用									整个周期长度
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);
}



/**
 * 功能：获取接触状态
 * 支撑状态  非支撑为0 支撑时为当前时刻在支撑相中相位
 */
Vec4<float> VisionGait::getContactState() 
{
  Array4f progress = _phase - _offsetsFloat; //progress每条腿在整个步态周期的位置 offest是相位差补偿 
  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _durationsFloat[i])//相位大于支撑结束相位，非支撑状态
    {
      progress[i] = 0.;
    }
    else                                //相位小于支撑结束相位，支撑状态
    {
      progress[i] = progress[i] / _durationsFloat[i];//相位在支撑相中的百分比
    }
  }

  return progress.matrix();
}

/**
 * 功能：获取摆动状态
 */
Vec4<float> VisionGait::getSwingState() 
{
  Array4f swing_offset = _offsetsFloat + _durationsFloat;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;

  Array4f swing_duration = 1. - _durationsFloat;//摆动相结束标志点
  Array4f progress = _phase - swing_offset;
  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.;						           	//相位大于摆动结束相位，非摆动状态
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];//相位在摆动相中的百分比
    }
  }
  return progress.matrix();
}


/**
 * 功能：为mpc准备足端接触信息 从当前时刻预测之后一个步态周期的接触信息
 */
int* VisionGait::mpc_gait() 
{
  for(int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])//在接触时间内 
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;
    }
  }

  return _mpc_table;
}



/*!
 * Representation of a quadruped robot's physical properties. 四足机器人物理特性的表征
 *
 * When viewed from the top, the quadruped's legs are: 从上往下看，四足动物的腿是
 *
 * FRONT
 * 2 1   RIGHT
 * 4 3
 * BACK
 *
 */



////////////////////
//原理
// Controller
// 参考Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control
// 一个步态周期由horizonLength(10)个mpc周期组成 
// trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),//步态分段
// 步态按1KHz处理 mpc计数间隔为30左右 一毫秒计数一次来控制频率 即一个mpc周期为30ms
// 则步态周期为 10*30 =300ms 
////////////////////
/**
 * 功能：视觉MPC运动
 */
VisionMPCLocomotion::VisionMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
  iterationsBetweenMPC(_iterations_between_mpc),//控制频率用
  horizonLength(10),
  dt(_dt),
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),//mpc片段数，mpc段中的偏移量？，mpc中一步持续时长？，名称
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
  standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),
  trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(3,3,3,3),"Trot Running")
{
  _parameters = parameters;         //用户参数定义
  dtMPC = dt * iterationsBetweenMPC;//mpc计算用时 在状态空间方程离散化时用 iterationsBetweenMPC MPC迭代次数
  printf("[Vision MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n",dt, iterationsBetweenMPC, dtMPC);
  vision_setup_problem(dtMPC, horizonLength, 0.4, 120);  //为mpc相关赋值
  //初始化变量
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;
}

/**
 * 功能：初始化MPC变量为0
 */
void VisionMPCLocomotion::initialize()
{
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
  rpy_des.setZero();
  v_rpy_des.setZero();
}



/**
 * 功能：世界坐标系下,更新落脚点 
 */
void VisionMPCLocomotion::_UpdateFoothold(Vec3<float> & foot, const Vec3<float> & body_pos,const DMat<float> & height_map, const DMat<int> & idx_map)
{
  //_UpdateFoothold(Pf, seResult.position, height_map, idx_map); float grid_size = 0.015;
    Vec3<float> local_pf = foot - body_pos;//落脚点-com 世界坐标下 从com指向落脚点的向量
    int row_idx_half = height_map.rows()/2;//height_map行数的一半
    int col_idx_half = height_map.rows()/2;
  //height_map以com为中心的二维”地图“
  //获得当前落脚点在二维地图上的位置索引
    int x_idx = floor(local_pf[0]/grid_size) + row_idx_half;
    int y_idx = floor(local_pf[1]/grid_size) + col_idx_half;
  //中间变量
    int x_idx_selected = x_idx;
    int y_idx_selected = y_idx;

    _IdxMapChecking(x_idx, y_idx, x_idx_selected, y_idx_selected, idx_map);//查找当前索引点附近，返回满足要求的索引

    foot[0] = (x_idx_selected - row_idx_half)*grid_size + body_pos[0];//新的落足点x,y
    foot[1] = (y_idx_selected - col_idx_half)*grid_size + body_pos[1];
    foot[2] = height_map(x_idx_selected, y_idx_selected);//从高度地图中获取新的落足点高度

}


/**
 * 功能：检查IdxMap
 */
void VisionMPCLocomotion::_IdxMapChecking(int x_idx, int y_idx, int & x_idx_selected, int & y_idx_selected, 
    const DMat<int> & idx_map)
{
  if(idx_map(x_idx, y_idx) == 0){ // (0,0)
    x_idx_selected = x_idx;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx+1, y_idx) == 0){ // (1, 0)
    x_idx_selected = x_idx+1;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx+1, y_idx+1) == 0){ // (1, 1)
    x_idx_selected = x_idx+1;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx, y_idx+1) == 0){ // (0, 1)
    x_idx_selected = x_idx;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx-1, y_idx+1) == 0){ // (-1, 1)
    x_idx_selected = x_idx-1;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx-1, y_idx) == 0){ // (-1, 0)
    x_idx_selected = x_idx-1;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx-1, y_idx-1) == 0){ // (-1, -1)
    x_idx_selected = x_idx-1;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx, y_idx-1) == 0){ // (0, -1)
    x_idx_selected = x_idx;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx+1, y_idx-1) == 0){ // (1, -1)
    x_idx_selected = x_idx+1;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx+2, y_idx-1) == 0){ // (2, -1)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx-1;

  }else if(idx_map(x_idx+2, y_idx) == 0){ // (2, 0)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx;

  }else if(idx_map(x_idx+2, y_idx+1) == 0){ // (2, 1)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx+1;

  }else if(idx_map(x_idx+2, y_idx+2) == 0){ // (2, 2)
    x_idx_selected = x_idx+2;
    y_idx_selected = y_idx+2;

  }else{
    printf("no proper step location (%d, %d)\n", x_idx, y_idx);
    x_idx_selected = x_idx;
    y_idx_selected = y_idx;
  }
}


/**
 * 功能：运行视觉MPC运动， 用来更新数据等
 */
template<>
void VisionMPCLocomotion::run(ControlFSMData<float>& data, 
    const Vec3<float> & vel_cmd, const DMat<float> & height_map, const DMat<int> & idx_map) 
{
  (void)idx_map;
  if(data.controlParameters->use_rc )//插上手柄
  {
    data.userParameters->cmpc_gait = data._desiredStateCommand->rcCommand->variable[0];
  }
  
  gaitNumber = data.userParameters->cmpc_gait;            //步态选择参数 
  auto& seResult = data._stateEstimator->getResult();     //状态估计返回值

  if(((gaitNumber == 4) && current_gait != 4) || firstRun)//检查是否过渡到站立
  {
    stand_traj[0] = seResult.position[0];                 //机体质心位置
    stand_traj[1] = seResult.position[1];                 //机体质心位置
    stand_traj[2] = 0.21;                                 //机体质心位置高度
    stand_traj[3] = 0;                                    //机体角度
    stand_traj[4] = 0;                                    //机体角度
    stand_traj[5] = seResult.rpy[2];                      //机体角度
    world_position_desired[0] = stand_traj[0];            //机体质心位置x
    world_position_desired[1] = stand_traj[1];            //机体质心位置y
  }

  VisionGait* gait = &trotting; // pick gait 选择步态
  if(gaitNumber == 1)         gait = &bounding;
  else if(gaitNumber == 2)    gait = &pronking;
  else if(gaitNumber == 3)    gait = &galloping;
  else if(gaitNumber == 4)    gait = &standing;
  else if(gaitNumber == 5)    gait = &trotRunning;
  current_gait = gaitNumber;//当前步态

  // integrate position setpoint 整合期望点 世界坐标系下 mpc运算都在世界坐标系下
  v_des_world[0] = vel_cmd[0];    //期望vx
  v_des_world[1] = vel_cmd[1];    //期望vy
  v_des_world[2] = 0.;            //期望vz
  
  rpy_des[2] = seResult.rpy[2];         //是期望偏航角
  v_rpy_des[2] = vel_cmd[2];            //是期望偏航角速度
  Vec3<float> v_robot = seResult.vWorld;//当前机器人世界坐标系下速度
  //pretty_print(v_des_world, std::cout, "v des world");

  //pitche和roll 积分达到目标值 
  if(fabs(v_robot[0]) > .2)            //避免除以0
  {  
    rpy_int[1] += dt*(rpy_des[1] - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1) 
  {
    rpy_int[0] += dt*(rpy_des[0] - seResult.rpy[0])/v_robot[1];
  }
                                      //初始角度 限幅
  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //关闭内翻

  for(int i = 0; i < 4; i++) //足端位置在世界坐标下 机身坐标+机身旋转矩阵^T*（髋关节在机身下坐标+腿在髋关节下坐标）
  {
    pFoot[i] = seResult.position + 
      seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + 
          data._legController->datas[i].p);//struct LegControllerData
  }

  if(gait != &standing)      //非站定下目标位置 通过累加目标速度完成
  {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }
  if(firstRun)               //第一次初始化 设置初始目标 原地踏步
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];
    for(int i = 0; i < 4; i++)//四条腿
    {
      footSwingTrajectories[i].setHeight(0.05);             //抬腿高度
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);//摆动足轨迹初始点
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);  //摆动足轨迹结束点
    }
    firstRun = false;
  }
  
  // foot placement 脚落点 
  // 摆动时间 摆动腿  根据步态确定  
  swingTimes[0] = dtMPC * gait->_swing;  
  swingTimes[1] = dtMPC * gait->_swing;
  swingTimes[2] = dtMPC * gait->_swing;
  swingTimes[3] = dtMPC * gait->_swing;
  float side_sign[4] = {-1, 1, -1, 1};//(-1表示右腿，+1表示左腿) RF LF RH LH

  for(int i = 0; i < 4; i++)          //计算四条腿落足点
  {
    if(firstSwing[i])                       //i号腿是否第一次摆动
    {
      swingTimeRemaining[i] = swingTimes[i];//摆动计时器
    } 
    else 
    {
      swingTimeRemaining[i] -= dt;          //摆动计时器
    }

    footSwingTrajectories[i].setHeight(.06);//设置摆动足轨迹相关 设置抬腿高度
	
	//计算行走状态下落足点   
    Vec3<float> offset(0, side_sign[i] * .065, 0);                           //关节偏置
    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset); //求得修正后i腿hip关节坐标在机身坐标系
	//获得机身YAW旋转后hip坐标//支撑时间中，力不变，速度近似恒定 机身坐标下
    Vec3<float> pYawCorrected = coordinateRotation(CoordinateAxis::Z,  -v_rpy_des[2] * gait->_stance * dtMPC / 2) * pRobotFrame;
    Vec3<float> des_vel = seResult.rBody * v_des_world;                      //机身下速度
	//世界坐标系下hip坐标 以剩余摆动时间内匀速运动来估计
    Vec3<float> Pf = seResult.position +
      seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

    float p_rel_max = 0.3f;

    //使用估计的速度是修正后的 支撑周期gait->_stance * dtMPC 
	//计算paper MIT Cheetah3 式（6） 世界坐标系下
    float pfx_rel = seResult.vWorld[0] * .5 * gait->_stance * dtMPC +
      .03f*(seResult.vWorld[0]-v_des_world[0]) +
      (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*v_rpy_des[2]);

    float pfy_rel = seResult.vWorld[1] * .5 * gait->_stance * dtMPC +
      .03f*(seResult.vWorld[1]-v_des_world[1]) +
      (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*v_rpy_des[2]);
	  
	  //限幅
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
  	//落足点 世界坐标系下
    Pf[0] +=  pfx_rel;
    Pf[1] +=  pfy_rel;

   //更新落足点 适应地形
    _UpdateFoothold(Pf, seResult.position, height_map, idx_map);
    _fin_foot_loc[i] = Pf;
    footSwingTrajectories[i].setFinalPosition(Pf);
  }
  
  
  // calc gait 
  gait->setIterations(iterationsBetweenMPC, iterationCounter);//步态周期计算
  iterationCounter++;

  // load LCM leg swing gains 加载LCM腿摆动增益
  Kp << 700, 0, 0,
    0, 700, 0,
    0, 0, 150;
  Kp_stance = 0*Kp;


  Kd << 11, 0, 0,
    0, 11, 0,
    0, 0, 11;
  Kd_stance = Kd;
  
  // gait
  Vec4<float> contactStates = gait->getContactState();//获取接触状态 在整个支撑过程百分比(从0到1)  完成后为0
  Vec4<float> swingStates = gait->getSwingState();//获取摆动状态 在整个摆动过程百分比(从0到1) 完成后为0
  int* mpcTable = gait->mpc_gait();//支撑状态
  
  updateMPCIfNeeded(mpcTable, data);//更新mpc期望值并解算足力

  Vec4<float> se_contactState(0,0,0,0);

  for(int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot]; //在整个支撑过程百分比(从0到1)
    float swingState = swingStates[foot]; //在整个摆动过程百分比(从0到1)
	
    if(swingState > 0) // foot is in swing 对摆动足进行控制 
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;//即刚从支撑切换到摆动
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);//第一次运行 起始点为原点 即刚从支撑切换到摆动
      }
	  //设定轨迹高度 由之前heightmap修正后落脚点参数得
      footSwingTrajectories[foot].setHeight(_fin_foot_loc[foot][2]+0.04);
	  //计算轨迹 参数为当前在摆动相的位置，摆动相时间长度
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
	  //获得轨迹点
      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
	  //转换到机体坐标系下后转到hip关节下
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
	//转换到机体坐标系等同于到髋关节坐标系下
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);


      // Update for WBC 为wbc控制更新参数 
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();
		//选择是否使用WBC控制 不使用 直接发送到腿部控制器 
      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;

        //singularity barrier 奇点的障碍
        data._legController->commands[foot].tauFeedForward[2] = 50*(data._legController->datas[foot].q(2)<.1)*data._legController->datas[foot].q(2);
      }
    }
    else // foot is in stance 对支撑足进行控制
    {
      firstSwing[foot] = true;
		//获得轨迹点??????
      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();//0
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();//0
	  //转换到机体坐标系下后转到hip关节下
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
	  //转换到机体坐标系等同于到髋关节坐标系下
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
//选择是否使用WBC控制 不使用 直接发送到腿部控制器
      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
		//主要是这个
        data._legController->commands[foot].forceFeedForward = f_ff[foot];//返回足力发送到腿部控制器
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;
      }
      se_contactState[foot] = contactState;
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);
  
  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.; 
  pBody_RPY_des[2] = rpy_des[2];

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = v_rpy_des[2];

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // END of WBC Update
}


/**
 * 功能：更新MPC,如果需要的化
 */
void VisionMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data) 
{
  //iterationsBetweenMPC = 30;
  if((iterationCounter % iterationsBetweenMPC) == 0)//控制频率
  {
    auto seResult = data._stateEstimator->getResult();//StateEstimator.h 的struct StateEstimate
    float* p = seResult.position.data();

    if(current_gait == 4)    //当前为站立
    {
      float trajInitial[12] = {(float)rpy_des[0],    // Roll
                               (float)rpy_des[1],    // Pitch
                               (float)stand_traj[5], //Yaw
                               (float)stand_traj[0], //X
                               (float)stand_traj[1], //Y
                               (float)_body_height,  //Z
                               0,0,0,0,0,0};         //速度们

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];//变成mpc问题需要的格式
    }
   	else                    	//非站定状态
   {
      const float max_pos_error = .1;//轨迹跟踪误差
      float xStart = world_position_desired[0];//轨迹参数设定为期望位置
      float yStart = world_position_desired[1];
    //在误差范围内，更新目标值
      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;
    //保存在误差范围内的目标值
      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;
	  
    //机体初始参考轨迹
      float trajInitial[12] = {(float)rpy_comp[0],  					// 0
                               (float)rpy_comp[1],    				// 1
                               (float)rpy_des[2],    					// 2
                               xStart,                        // 3
                               yStart,                        // 4
                               (float)_body_height,    				// 5
                               0,                             // 6
                               0,                             // 7
                               (float)v_rpy_des[2],  					// 8
                               v_des_world[0],                // 9
                               v_des_world[1],                // 10
                               0};                            // 11

    //变成mpc问题需要的格式
    //轨迹为当前时刻向后预测一个步态周期的轨迹 按匀速运动计算
      for(int i = 0; i < horizonLength; i++) 
      {
        for(int j = 0; j < 12; j++)  trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // 从当前位置开始
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        } 
        else 
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * v_rpy_des[2];
        }
      }
    }
    solveDenseMPC(mpcTable, data);
  }
}


/**
 * 功能：解算足力
 */
void VisionMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) 
{
	//世界坐标下状态估计值
  auto seResult = data._stateEstimator->getResult();

  float Q[12] = {0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2};
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5;                           //最后进行设置
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for(int i = 0; i < 12; i++) r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];   //更新每条腿r向量 [xxxx,yyyy,zzzz]

  if(alpha > 1e-4) 
  {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  dtMPC = dt * iterationsBetweenMPC;                                              //设置时间间隔 间隔怎么定的？
  vision_setup_problem(dtMPC,horizonLength,0.4,120);                              //设置MPC参数
  vision_update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable);//解MPC

  for(int leg = 0; leg < 4; leg++)                                                //每次结算出的horizon组力 只取第一组 convex_mpc 5节开头
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = vision_get_solution(leg*3 + axis);

    f_ff[leg] = -seResult.rBody * f;//将世界坐标下力转换到机体坐标下
    Fr_des[leg] = f;                //给WBC更新力
  }
}

