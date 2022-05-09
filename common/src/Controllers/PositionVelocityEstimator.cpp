/*! @file PositionVelocityEstimator.h
 *位置速度传感器应计算：
 *-世界/身体框架中的身体位置/速度
 *-身体/世界框架中的脚位置/速度
 */

#include "Controllers/PositionVelocityEstimator.h"

/*!
 * 功能：位置和速度估计器初始化配置函数
 * 主要是对卡尔曼用到的矩阵进行初始化
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::setup() 
{
  //（1）打印状态估计器数据、参数、和时间戳
  printf("beans 0x%lx\n", (uint64_t)this);
  printf("beans2 0x%lx\n", (uint64_t) & (this->_stateEstimatorData));
  printf("beans3 0x%lx\n", (uint64_t)(this->_stateEstimatorData.parameters));
  printf("beans4 0x%lx\n",
         (uint64_t) & (this->_stateEstimatorData.parameters->controller_dt));
  //（2）初始化线性卡尔曼状态估计器
  T dt = this->_stateEstimatorData.parameters->controller_dt;
  printf("Initialize LinearKF State Estimator with dt = %.3f\n", dt);
  _xhat.setZero();                                                          //状态估计值矩阵置零：世界坐标下[p v p1 p2 p3 p4]         
  _ps.setZero();                                                            //储存状态矩阵p置零
  _vs.setZero();                                                            //储存状态矩阵v置零

  _A.setZero();                                                             //状态转移阵置零
  _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();                //
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();           //
  _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();                //
  _A.block(6, 6, 12, 12) = Eigen::Matrix<T, 12, 12>::Identity();            //

  _B.setZero();                                                             //输入阵置零置零
  _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();           //

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);                //
  C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero(); //
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);                //
  C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity(); //
  _C.setZero();                                                             //观测阵置零
  _C.block(0, 0, 3, 6) = C1;                                                //
  _C.block(3, 0, 3, 6) = C1;                                                //
  _C.block(6, 0, 3, 6) = C1;                                                //    
  _C.block(9, 0, 3, 6) = C1;                                                //
  _C.block(0, 6, 12, 12) = T(-1) * Eigen::Matrix<T, 12, 12>::Identity();    //
  _C.block(12, 0, 3, 6) = C2;                                               //
  _C.block(15, 0, 3, 6) = C2;                                               //
  _C.block(18, 0, 3, 6) = C2;                                               //
  _C.block(21, 0, 3, 6) = C2;                                               //
  _C(27, 17) = T(1);                                                        //
  _C(26, 14) = T(1);                                                        //
  _C(25, 11) = T(1);                                                        //
  _C(24, 8) = T(1);                                                         //

  _P.setIdentity();                                                         //初始不确定性矩阵置零
  _P = T(100) * _P;                                                         //

  _Q0.setIdentity();                                                        //初始状态估计噪声矩阵置零
  _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity(); //
  _Q0.block(3, 3, 3, 3) =                                                   //
      (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();              //
  _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<T, 12, 12>::Identity();      //

  _R0.setIdentity();                                                        //初始观测噪声矩阵
}


/*!
 * 功能：
 */
template <typename T>
LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator() {}


/*!
 * 功能：状态轨迹，通过卡尔曼滤波器估计躯干的位置和线速度
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::run() 
{
  ///////////////////////////////////////////////*（1）定义卡尔曼滤波器的噪声*////////////////////////////////////////////////
  //(1)定义过程躯干IMU位置噪声
  T process_noise_pimu =
      this->_stateEstimatorData.parameters->imu_process_noise_position;   //imu_process_noise_position=0.02，放在了配置文件.yaml中
  //(2)定义过程躯干IMU速度噪声
  T process_noise_vimu =
      this->_stateEstimatorData.parameters->imu_process_noise_velocity;   //imu_process_noise_velocity=0.02，放在了配置文件.yaml中
  //(3)定义过程腿的位置噪声
  T process_noise_pfoot =
      this->_stateEstimatorData.parameters->foot_process_noise_position;  //foot_process_noise_position=0.002，放在了配置文件.yaml中
  //(4)定义观测关节电机位置噪声
  T sensor_noise_pimu_rel_foot =
      this->_stateEstimatorData.parameters->foot_sensor_noise_position;   //foot_sensor_noise_position=0.001，放在了配置文件.yaml中
  //(5)定义观测关节电机速度噪声
  T sensor_noise_vimu_rel_foot =
      this->_stateEstimatorData.parameters->foot_sensor_noise_velocity;   //foot_sensor_noise_velocity=0.1，放在了配置文件.yaml中
  //（6）定义关节电机高度噪声
  T sensor_noise_zfoot =
      this->_stateEstimatorData.parameters->foot_height_sensor_noise;     //foot_height_sensor_noise=0.001，放在了配置文件.yaml中

  //（7）初始化状态估计噪声矩阵，使用上面的过程噪声进行状态估计噪声矩阵计算
  Eigen::Matrix<T, 18, 18> Q = Eigen::Matrix<T, 18, 18>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
  Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

  //（8）初始化观测噪声矩阵，使用上面的观测噪声进行观测噪声矩阵计算
  Eigen::Matrix<T, 28, 28> R = Eigen::Matrix<T, 28, 28>::Identity();
  R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
  R.block(12, 12, 12, 12) =
      _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
  R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

  int qindex = 0;
  int rindex1 = 0;
  int rindex2 = 0;
  int rindex3 = 0;

  ////////////////////////////////////////////*（2）定义三维矩阵*///////////////////////////////////////////////////////////////////
  Vec3<T> g(0, 0, T(-9.81));                                            // 1）定义重力矩阵
  Mat3<T> Rbod = this->_stateEstimatorData.result->rBody.transpose();   // 2）定义身体坐标系下的旋转矩阵，通过四元素转换而来
  Vec3<T> a = this->_stateEstimatorData.result->aWorld +g;              // 3）定义世界坐标系下的加速度矩阵

  ///////////////////////////////////////////*（3）定义四维矩阵*/////////////////////////////////////////////////////////////////////
  Vec4<T> pzs = Vec4<T>::Zero();
  Vec4<T> trusts = Vec4<T>::Zero();

  //////////////////////////////////////////*（4）定义状态估计值矩阵：世界坐标下[p v p1 p2 p3 p4] *////////////////////////////////////
  Vec3<T> p0, v0;
  p0 << _xhat[0], _xhat[1], _xhat[2];
  v0 << _xhat[3], _xhat[4], _xhat[5];

  //////////////////////////////////////////*（5）卡尔曼滤波的状态预测部分,计算机器人躯干的位置、速度和触地状态 *//////////////////////////////////////
  for (int i = 0; i < 4; i++) //四条腿的数据都加载进来
  {
    
    int i1 = 3 * i;
    Quadruped<T>& quadruped = *(this->_stateEstimatorData.legControllerData->quadruped);    //设定机器人类型            cheetah3 or mini
    Vec3<T> ph = quadruped.getHipLocation(i);                                               //计算相对于CoM的髋部位置，即机器人坐标系中腿的臀部位置
    Vec3<T> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p;                  //计算世界坐标系下机器人真正的位置=     机器人坐标系中腿的臀部位置+腿的位置
    Vec3<T> dp_rel = this->_stateEstimatorData.legControllerData[i].v;                      //计算世界坐标系下机器人真正的速度=     状态估计器中每条的速度
    Vec3<T> p_f = Rbod * p_rel;                                                             //计算身体坐标系下机器人真正的位置=     身体坐标系下的旋转矩阵*世界坐标系下机器人真正的位置？
    Vec3<T> dp_f =Rbod *(this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);//计算身体坐标系下机器人真正的速度=     身体坐标系下的旋转矩阵*（躯干坐标系中的角速度正交阵+世界坐标系下机器人真正的速度）

    qindex = 6 + i1;
    rindex1 = i1;
    rindex2 = 12 + i1;
    rindex3 = 24 + i;

    T trust = T(1);                                                                     //定义触地标志位变量trust
    T phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), T(1));         //计算接触相序
    T trust_window = T(0.2);

    //通过相序偏移判断触地状态trust
    if (phase < trust_window) 
    {
      trust = phase / trust_window;
    } 
    else if (phase > (T(1) - trust_window)) 
    {
      trust = (T(1) - phase) / trust_window;
    }
   // printf("Trust %d: %.3f\n", i, trust);

   
    Q.block(qindex, qindex, 3, 3) =
        (T(1) + (T(1) - trust) * T(100)) * Q.block(qindex, qindex, 3, 3);
    R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
    R.block(rindex2, rindex2, 3, 3) =
        (T(1) + (T(1) - trust) * 100.0f) * R.block(rindex2, rindex2, 3, 3);
    R(rindex3, rindex3) =
        (T(1) + (T(1) - trust) * T(100)) * R(rindex3, rindex3);

    trusts(i) = trust;

    _ps.segment(i1, 3) = -p_f;
    _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
    pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
  }

  ////////////////////////////////////////////*(6)卡尔曼滤波的状态融合部分(套公式)*////////////////////////////////////////////////////////////////////
  Eigen::Matrix<T, 28, 1> y;
  y << _ps, _vs, pzs;
  _xhat = _A * _xhat + _B * a;
  Eigen::Matrix<T, 18, 18> At = _A.transpose();
  Eigen::Matrix<T, 18, 18> Pm = _A * _P * At + Q;
  Eigen::Matrix<T, 18, 28> Ct = _C.transpose();
  Eigen::Matrix<T, 28, 1> yModel = _C * _xhat;
  Eigen::Matrix<T, 28, 1> ey = y - yModel;
  Eigen::Matrix<T, 28, 28> S = _C * Pm * Ct + R;

  // todo compute LU only once
  Eigen::Matrix<T, 28, 1> S_ey = S.lu().solve(ey);
  _xhat += Pm * Ct * S_ey;

  Eigen::Matrix<T, 28, 18> S_C = S.lu().solve(_C);
  _P = (Eigen::Matrix<T, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

  Eigen::Matrix<T, 18, 18> Pt = _P.transpose();
  _P = (_P + Pt) / T(2);

  if (_P.block(0, 0, 2, 2).determinant() > T(0.000001)) 
  {
    _P.block(0, 2, 2, 16).setZero();
    _P.block(2, 0, 16, 2).setZero();
    _P.block(0, 0, 2, 2) /= T(10);
  }

//////////////////////////////////////////////*(7)输出卡尔曼估计结果*///////////////////////////////////////////////////////////////
  this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);     //1）输出卡尔曼滤波的躯干位置
  this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);       //2）输出卡尔曼滤波的躯干速度，基于世界坐标系下的
  this->_stateEstimatorData.result->vBody =                                 //3）输出卡尔曼滤波的躯干速度，基于身体坐标系下的
      this->_stateEstimatorData.result->rBody *
      this->_stateEstimatorData.result->vWorld;
}
template class LinearKFPositionVelocityEstimator<float>;
template class LinearKFPositionVelocityEstimator<double>;


/*!
 * 功能：仿真模式下，运行位置和速度估计器
 */
template <typename T>
void CheaterPositionVelocityEstimator<T>::run() 
{
  //（1）计算世界坐标系下躯干位置估计值     方法：把仿真状态下的位置赋值直接给状态估计器的位置结果
  this->_stateEstimatorData.result->position = this->_stateEstimatorData.cheaterState->position.template cast<T>();

  //（2）计算世界坐标系下躯干速度估计值     方法：躯干速度vWorld=坐标变换矩阵rBody转换后*vBody
  this->_stateEstimatorData.result->vWorld =
      this->_stateEstimatorData.result->rBody.transpose().template cast<T>() * this->_stateEstimatorData.cheaterState->vBody.template cast<T>();

  //（3）计算躯干坐标系下躯干速度估计值     方法：把仿真状态下的速度直接赋值给状态估计器的位置结果 
  this->_stateEstimatorData.result->vBody = this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
}
template class CheaterPositionVelocityEstimator<float>;
template class CheaterPositionVelocityEstimator<double>;
