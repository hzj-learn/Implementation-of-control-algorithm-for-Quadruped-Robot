/*============================= FSM State =============================*/
/**
*FSM状态基类
 */

#include "FSM_State.h"

/**
 * 功能：FSM状态类的构造函数，把所有切换数据清零
 * @param _controlFSMData  保存所有相关的控制数据
 * @param stateNameIn      枚举状态名
 * @param stateStringIn    当前FSM状态的字符串名称
 */
template <typename T>
FSM_State<T>::FSM_State(ControlFSMData<T> *_controlFSMData,
                        FSM_StateName stateNameIn, std::string stateStringIn)
    : _data(_controlFSMData),
      stateName(stateNameIn),
      stateString(stateStringIn)
{
  transitionData.zero();    //把所有切换数据清零
  std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn
            << std::endl;
}




/**
 * 功能：直角坐标下独立控制给定腿
 * @param leg      腿序号
 * @param qDes     期望关节角度
 * @param dqDes    期望关节速度
 * 备注：调试用的
 */
template <typename T>
void FSM_State<T>::jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes)
{
  //给定kp、kd矩阵
  kpMat << 80, 0, 0, 0, 80, 0, 0, 0, 80;
  kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  _data->_legController->commands[leg].kpJoint = kpMat; //设定kp反馈增益矩阵
  _data->_legController->commands[leg].kdJoint = kdMat; //设定kd反馈增益矩阵
  _data->_legController->commands[leg].qDes = qDes;     //设定期望位置
  _data->_legController->commands[leg].qdDes = qdDes;   //设定期望速度
}



/**
 * 直角坐标下独立控制给定腿
 * @param leg           腿序号
 * @param pDes          期望位置
 * @param vDes          期望速度
 * @param kp_cartesian  P增益
 * @param kd_cartesian  D增益
 * 备注：调试用的
 */
template <typename T>
void FSM_State<T>::cartesianImpedanceControl(int leg, Vec3<T> pDes,
                                             Vec3<T> vDes,
                                             Vec3<double> kp_cartesian,
                                             Vec3<double> kd_cartesian)
{
  _data->_legController->commands[leg].pDes = pDes;           //设定期望位置
  //创建笛卡尔P增益矩阵
  kpMat << kp_cartesian[0], 0, 0,
      0, kp_cartesian[1], 0,
      0, 0, kp_cartesian[2];
  _data->_legController->commands[leg].kpCartesian = kpMat;  //设定kp矩阵
  _data->_legController->commands[leg].vDes = vDes;          //设定期望速度

//创建笛卡尔D增益矩阵
  kdMat << kd_cartesian[0], 0, 0, 0, kd_cartesian[1], 0, 0, 0, kd_cartesian[2];
  _data->_legController->commands[leg].kdCartesian = kdMat;  //设定kd矩阵
}




/**
 * 功能：步态表达方式，即选择适当的（1）反作用力（GRF）  （2）步的位置 （3）以及将这些转换成腿控制器的理解值。
 */
template <typename T>
void FSM_State<T>::runControls()
{
  int CONTROLLER_OPTION = 1;                //控制模式选项，应该从用户界面或最终自主地设置此选项，例如位置控制模式
  footFeedForwardForces = Mat34<T>::Zero(); // 将力重置为0
  footstepLocations = Mat34<T>::Zero();     // 将步长重置为0

  //选择控制器运行，选择步骤位置和平衡力
  if (CONTROLLER_OPTION == 0)     //给定位置控制模式
  {
    for (int leg = 0; leg < 4; leg++)//四条腿
    {
      footFeedForwardForces.col(leg) << 0.0, 0.0, 0;                                    // 将前馈力置为0，因为是位置模式
      footstepLocations.col(leg) << 0.0, 0.0, -_data->_quadruped->_maxLegLength / 2;    //设定落脚位置是最大腿长的一半
    }
  }
  else if (CONTROLLER_OPTION == 1)//站立QP平衡控制模式
  {
    runBalanceController(); //QP平衡控制器
    for (int leg = 0; leg < 4; leg++)//摇摆脚落地位置计算启发式
    {
      footstepLocations.col(leg) << 0.0, 0.0,   //设定落脚位置是最大腿长的一半
          -_data->_quadruped->_maxLegLength / 2;
    } 
  }
  else if (CONTROLLER_OPTION == 2)//全身力控模式WBC
  {
    runWholeBodyController();  // WBC
  }
  else if (CONTROLLER_OPTION == 3)//模型预测控制cMPC模式
  {
    runConvexModelPredictiveController();  // cMPC
  }
  else if (CONTROLLER_OPTION == 4)//RPC模式
  {
    runRegularizedPredictiveController();   // RPC
  }
  else                            //如果没有选择控制器，则将命令归零
  {
    jointFeedForwardTorques =
        Mat34<float>::Zero();               // 前馈关节扭矩
    jointPositions = Mat34<float>::Zero();  // 关节角度位置 
    jointVelocities = Mat34<float>::Zero(); // 关节角速度
    footFeedForwardForces =
        Mat34<float>::Zero();             // 足部前向力
    footPositions = Mat34<float>::Zero(); // 笛卡尔脚位置
    footVelocities = Mat34<float>::Zero();
    //打印错误消息 
    std::cout << "[FSM_State] ERROR: No known controller was selected: "
              << CONTROLLER_OPTION << std::endl;
  }
}




/**
 * 功能：QP平衡控制器运行 ，
 * 备注，很重要
 * 步骤：
 * (1)通过步态调度器设置四条腿接触状态表
 * (2)根据接触状态来设置约束电机最大和最小的力矩
 * (3)从状态估计器中获取机身当前四元数，4个量
 *（4）从状态估计器中更新上面的信息
 *（5）获取脚相对于COM的位置
 *（6）设置平衡控制器的参数
 *（7）QP解足端每个关节的力
 *（8）通过LCM发布结果
 *（9）将结果复制到前馈力
 */
template <typename T>
void FSM_State<T>::runBalanceController()
{
  double contactStateScheduled[4]; //定义四条腿接触状态表= {1, 1, 1, 1};
  for (int i = 0; i < 4; i++)      //(1)通过步态调度器设置四条腿接触状态表
  {
    contactStateScheduled[i] =
        _data->_gaitScheduler->gaitData.contactStateScheduled(i);
  }
  
  double minForce = 25;            //定义电机最小的力矩参数
  double maxForce = 500;           //定义电机最大的力矩参数
  double minForces[4];             //定义电机最小的力矩 = {minForce, minForce, minForce, minForce};
  double maxForces[4];             //定义电机最大的力矩 = {maxForce, maxForce, maxForce, maxForce};
  for (int leg = 0; leg < 4; leg++)//(2)根据接触状态来设置约束电机最大和最小的力矩
  {
    minForces[leg] = contactStateScheduled[leg] * minForce;//contactStateScheduled[leg]的值非1即0，用于选择的
    maxForces[leg] = contactStateScheduled[leg] * maxForce;
  }


  double COM_weights_stance[3] = {1, 1, 10};
  double Base_weights_stance[3] = {20, 10, 10};
  double pFeet[12];     //腿的位置
  double p_des[3];      //每个关节的期望位置
  double p_act[3];      //每个关节的实际位置
  double v_des[3];      //每个关节的期望速度
  double v_act[3];      //每个关节的实际速度
  double O_err[3];      //误差
  double rpy[3];        //rpy
  double omegaDes[3];   //期望俯仰角速度
  double se_xfb[13];    //记录四元数灯信息
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];    //机体虚拟弹簧PD系数

  for (int i = 0; i < 4; i++)      //(3)从状态估计器中获取机身当前四元数，4个量
  {
    se_xfb[i] = (double)_data->_stateEstimator->getResult().orientation(i);    //从状态估计器中获取机身当前四元数
  }
  
  for (int i = 0; i < 3; i++)      //（4）从状态估计器中更新上面的信息
  {
    rpy[i] = 0; //(double)_data->_stateEstimator->getResult().rpy(i);
    p_des[i] = (double)_data->_stateEstimator->getResult().position(i);        //期望位置为当前位置
    p_act[i] = (double)_data->_stateEstimator->getResult().position(i);        //执行位置为当前位置
    omegaDes[i] =0; //(double)_data->_stateEstimator->getResult().omegaBody(i);//角速度为0
    v_act[i] = (double)_data->_stateEstimator->getResult().vBody(i);           //每个关节的实际速度
    v_des[i] = (double)_data->_stateEstimator->getResult().vBody(i);           //每个关节的期望速度

    se_xfb[4 + i] = (double)_data->_stateEstimator->getResult().position(i);   //位置信息
    se_xfb[7 + i] = (double)_data->_stateEstimator->getResult().omegaBody(i);  //躯干俯仰角速度信息
    se_xfb[10 + i] = (double)_data->_stateEstimator->getResult().vBody(i);     //躯干线速度信息

    //设置平移和方向增益
    kpCOM[i] = (double)_data->controlParameters->kpCOM(i);        //轨迹kp增益信息
    kdCOM[i] = (double)_data->controlParameters->kdCOM(i);        //轨迹kd增益信息
    kpBase[i] = (double)_data->controlParameters->kpBase(i);      //躯干kp增益信息
    kdBase[i] = (double)_data->controlParameters->kdBase(i);      //躯干kd增益信息
  }

  Vec3<T> pFeetVec;
  Vec3<T> pFeetVecCOM;
  for (int leg = 0; leg < 4; leg++) //（5）获取脚相对于COM的位置
  {

    computeLegJacobianAndPosition(**&_data->_quadruped,
                                  _data->_legController->datas[leg].q,
                                  (Mat3<T> *)nullptr, &pFeetVec, 1);
    pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  (_data->_quadruped->getHipLocation(leg) + _data->_legController->datas[leg].p);

    pFeet[leg * 3] = (double)pFeetVecCOM[0];
    pFeet[leg * 3 + 1] = (double)pFeetVecCOM[1];
    pFeet[leg * 3 + 2] = (double)pFeetVecCOM[2];
  }
  
  //（6）设置平衡控制器的参数
  balanceController.set_alpha_control(0.01);
  balanceController.set_friction(0.5);
  balanceController.set_mass(46.0);
  balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
  balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
  balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
  balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
  balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act, O_err, 0.0);

  double fOpt[12];//定义足端每个关节的力
  balanceController.solveQP_nonThreaded(fOpt);//（7）QP解足端每个关节的力

  balanceController.publish_data_lcm();//（8）通过LCM发布结果

  for (int leg = 0; leg < 4; leg++)//（9）将结果复制到前馈力
  {
    footFeedForwardForces.col(leg) << (T)fOpt[leg * 3], (T)fOpt[leg * 3 + 1],
        (T)fOpt[leg * 3 + 2];
  }
}


/*
 *功能：开启步态独立公式安全检查，从而选择合适的 
 （1）反作用力(GRF)
 （2）步长位置 
 （3）以及将它们转换为腿控制器可理解的值。 
 */
template <typename T>
void FSM_State<T>::turnOnAllSafetyChecks()
{
  checkSafeOrientation = true; //开启预控安全检查，检查横摇和纵摇 
  //岗位控制安全检查
  checkPDesFoot = true;         //检查期望的KD值不要和footsetps命令相差太大
  checkForceFeedForward = true; //检查前馈力矩不要和力矩命令相差太大  
  checkLegSingularity = true;   //检查不要让腿
}


/*
 *功能：关闭步态独立公式安全检查，从而选择合适的 
 （1）反作用力(GRF)
 （2）步长位置 
 （3）以及将它们转换为腿控制器可理解的值。 
 */
template <typename T>
void FSM_State<T>::turnOffAllSafetyChecks()
{
  checkSafeOrientation = false; //开启预控安全检查，检查横摇和纵摇  
  //岗位控制安全检查
  checkPDesFoot = false;         //检查期望的KD值不要和footsetps命令相差太大
  checkForceFeedForward = false; //检查前馈力矩不要和力矩命令相差太大  
  checkLegSingularity = false;   ///检查不要让腿
}

template class FSM_State<float>;
