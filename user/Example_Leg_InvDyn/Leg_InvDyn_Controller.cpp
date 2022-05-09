//设置并启动腿部逆动力学控制器C++文件

#include "Leg_InvDyn_Controller.hpp"
#include <iostream>


/*!
 * 功能：计算腿部关节你动力学解算，最终输出每个关节的力矩
 * 步骤：
 * （1）把状态估计的状态（躯干方向、躯干位置、躯干俯仰角速度、躯干线速度）赋值给躯干的状态
 * （2）把躯干位置、速度设置为0
 * （3）四条腿电机反馈的数据计算躯干的位置和速度，腿的位置和速度通先过关节控制器进行设置
 * （4）设置模型状态
 * （5）计算每个关节的期望轨迹
 * （6）计算腿部所需的集合
 * （7）赋值对角线Ab/ad左腿&右腿
 * （8）构造指令加速度
 * （9）运行RNEA反向动力学
 * （10）替代策略：组合运动方程
 * （11）确保它们匹配
 * （12）向腿部控制器发送关节力矩
 */
void Leg_InvDyn_Controller::runController()
{
  static int iter = 0;
  iter ++;
  FBModelState<float> state;

/////////////////////////////////////////////*（1）方法一:状态估计器数据+电机反馈数据+正逆动力学的方法*///////////////////////////////////////////////
 /*状态估计器数据（躯干方向、躯干位置、躯干俯仰角速度、躯干线速度）*/

  //（1）把状态估计的状态（躯干方向、躯干位置、躯干俯仰角速度、躯干线速度）赋值给躯干的状态
  state.bodyOrientation = _stateEstimate->orientation;    //把状态估计的方向，赋值为躯干的方向
  state.bodyPosition    = _stateEstimate->position;       //把状态估计的位置，赋值为躯干的位置
  state.bodyVelocity.head(3) = _stateEstimate->omegaBody; //把状态估计的俯仰角速度，赋值为躯干的俯仰角速度
  state.bodyVelocity.tail(3) = _stateEstimate->vBody;     //把状态估计的线速度，赋值为躯干的线速度


  /*关节电机数据（位置、速度）*/
  //（2）把腿的角度、角速度复位为0
  state.q.setZero(12);    //躯干位置设置为0
  state.qd.setZero(12);   //躯干速度设置为0
  //（3）使用四条腿的每个关机电机反馈的数据计算躯干的角度和角速度
  for (int i = 0; i < 4; ++i) 
  {
    //躯干的位置值，用每条腿的角度赋值
    state.q(3*i+0) = _legController->datas[i].q[0];
    state.q(3*i+1) = _legController->datas[i].q[1];
    state.q(3*i+2) = _legController->datas[i].q[2];
    //躯干的速度值，用每条腿的角速度赋值
    state.qd(3*i+0)= _legController->datas[i].qd[0];
    state.qd(3*i+1)= _legController->datas[i].qd[1];
    state.qd(3*i+2)= _legController->datas[i].qd[2];
  }
  //（4）设置模型状态
  _model->setState(state);    


  //（5）计算每个关节的期望轨迹（角度、角速度、角加速度），使用给定的方式，用正弦曲线拟合
  float t = _controlParameters->controller_dt*iter;
  //定义简单关节空间正弦轨迹的期望轨迹参数 
  float freq_Hz = 1;                          //频率
  float freq_rad = freq_Hz * 2* 3.14159;      //角频率 360度
  float amplitude = 3.1415/3;                 //正弦波曲线振幅
  Vec12<float> qDes, qdDes, qddDes;           //定义期望的角度，角速度和角加速度
  //先对每个运动关节使用的期望轨迹置0
  qDes.setZero();                             //期望的角度置0
  qdDes.setZero();                            //期望的角速度置0
  qddDes.setZero();                           //期望的角加速度置0
  //计算每个关节的期望轨迹（角度、角速度、角加速度），轨迹是一个正弦波
  float desired_angle = sin(t*freq_rad)*amplitude;                          //期望的角度=Asin(xt)
  float desired_rate = freq_rad*cos(t*freq_rad)*amplitude;                  //期望的角速度=Axcos(xt)    ,期望的角速度是期望的角度的导数
  float desired_acceleration = freq_rad*freq_rad*sin(t*freq_rad)*amplitude; //期望的角加速度=Axxsin(xt) ,期望的角加速度是期望的角速度的导数


  /*使用正动力学的方法计算反作用力*/
  //（6）进行移动的腿部所需的角度、角速度、角加速度的数值限幅
  for( int i = 0 ; i < (int) userParameters.num_moving_legs ; i++) 
  {
    qDes.segment(3*i,3).setConstant(desired_angle);         //期望的角度限幅 
    qdDes.segment(3*i,3).setConstant(desired_rate);         //期望的角速度限幅
    qddDes.segment(3*i,i).setConstant(desired_acceleration);//期望的角加速度限幅
  }

  //（7）控制逻辑，轨迹对应对角线左腿&右腿赋值
  qDes(0)*=-1;      //期望角度
  qDes(6)*=-1;      
  qdDes(0)*=-1;     //期望角速度
  qdDes(6)*=-1; 
  qddDes(0)*=-1;    //期望角加速度
  qddDes(6)*=-1;
  //（8）阻抗控制-构造躯干加速度指令
  FBModelStateDerivative<float> commandedAccleration;                               //定义加速度指令1
  commandedAccleration.dBodyVelocity.setZero();                                     //躯干线速度置零
  commandedAccleration.qdd = qddDes + 25*(qdDes - state.qd) + 150*(qDes - state.q); //阻抗控制输出力矩 （重要）     躯干的先加速度指令=期望角加速度+25*（期望的角速度-真实状态的角速度）+150*（期望的角度-真实状态的角度）
  //（9）正运动学-生成腿部力矩和关节力矩
  Vec18<float> generalizedForce = _model->inverseDynamics(commandedAccleration); //生成腿部力矩
  Vec12<float> jointTorques = generalizedForce.tail(12);                         //生成关节力矩 


////////////////////////////////////////////////////*（2）方法二：动力学模型的方法*////////////////////////////////////////////////////////////////////////////////////

  //（10）替代策略：组合运动方程
  Mat18<float> H;                               //定义质量矩阵（H）
  Vec18<float> Cqd, tau_grav;                   //定义广义科里奥利力、广义引力
  H = _model->massMatrix();                     //（1）计算逆动力学公式中的质量矩阵（H）
  Cqd = _model->generalizedCoriolisForce();     //（2）广义科里奥利力的计算
  tau_grav = _model->generalizedGravityForce(); //（3）计算逆动力学中的广义引力（G）

  Vec18<float> generalizedAcceleration;                                        //定义加速度指令2
  generalizedAcceleration.head(6)  = commandedAccleration.dBodyVelocity;       //构造速度指令
  generalizedAcceleration.tail(12) = commandedAccleration.qdd;                 //构造加速度指令
  Vec18<float> generalizedForce2 = H*generalizedAcceleration + Cqd + tau_grav; //（4）计算反作用力，即躯干加速度指令     用动力学简化的那条公式：（重要） 反作用力=质量矩阵（H）*加速度指令+广义科里奥利力+广义引力

  ////////////////////////////////////////////////*（3）确保两个方法所计算的关节力矩值相差不大*///////////////////////////////////////////////////////////////////////////
  Vec18<float> err = generalizedForce - generalizedForce2;
  assert( err.norm() < 1e-4 );//过大就抛出错误

 /////////////////////////////////////////////////（4）向腿部控制器发送关节力矩////////////////////////////////////////////////////////////////////////////////////////
  int dof = 0;
  for(int leg(0); leg<4; ++leg)
  {
    for(int jidx(0); jidx<3; ++jidx)
    {
      _legController->commands[leg].qDes[jidx] = 0;                             //关节角度置0
      _legController->commands[leg].qdDes[jidx] = 0.;                           //关节角速度置0
      _legController->commands[leg].tauFeedForward[jidx] = jointTorques(dof);   //关节力矩
      dof++;
    }
    _legController->commands[leg].kpJoint = Mat3<float>::Zero();                //关节KP增益置0
    _legController->commands[leg].kdJoint = Mat3<float>::Zero();                //关节KD增益置0
  }
}
