/*!
 * @file StateEstimator.h
 * @brief 状态估计器接口的实现
 * 每个状态估计器对象包含许多估计器
 * 当状态估计器运行时，它运行所有的估计器。
 */

//这个文件是状态估计器的接口文件，因为状态估计器估计的量很多，无法在一个C++文件里面写，但是在这个文件把接口写清楚了

#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H

#include "ControlParameters/RobotParameters.h"
#include "Controllers/LegController.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/VisualizationData.h"
#include "state_estimator_lcmt.hpp"

/*!
 * 状态估计结果结构体
 */
template <typename T>
struct StateEstimate 
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<T> contactEstimate;      //接触估计
  Vec3<T> position;             //位置
  Vec3<T> vBody;                //线速度
  Quat<T> orientation;          //四元数
  Vec3<T> omegaBody;            //角速度
  RotMat<T> rBody;              //旋转矩阵
  Vec3<T> rpy;                  //欧拉角

  Vec3<T> omegaWorld;           //世界坐标角速度
  Vec3<T> vWorld;               //世界坐标速度
  Vec3<T> aBody, aWorld;        //加速度，世界坐标下加速度
  
  void setLcm(state_estimator_lcmt& lcm_data) //通过lcm发送
  {
    for(int i = 0; i < 3; i++) 
    {
      lcm_data.p[i] = position[i];
      lcm_data.vWorld[i] = vWorld[i];
      lcm_data.vBody[i] = vBody[i];
      lcm_data.rpy[i] = rpy[i];
      lcm_data.omegaBody[i] = omegaBody[i];
      lcm_data.omegaWorld[i] = omegaWorld[i];
    }
    for(int i = 0; i < 4; i++) 
    {
      lcm_data.quat[i] = orientation[i];
    }
  }
};



/*
 * 状态估计的输入
 * 如果机器人代码需要通知状态估计器一些东西，应该在这里加上
 * 您还应该使用setter方法来设置StateEstimatorContainer
 */
template <typename T>
struct StateEstimatorData 
{
  StateEstimate<T>* result;               //状态估计器输出
  VectorNavData* vectorNavData;           //imu数据
  CheaterState<double>* cheaterState;
  LegControllerData<T>* legControllerData;//腿部数据
  Vec4<T>* contactPhase;                  //接触状态 mpc控制器来
  RobotControlParameters* parameters;     //文件内参数
};



/*!
 *功能：所有的估计器都应该继承这个类
 */
template <typename T>
class GenericEstimator 
{
 public:
  virtual void run() = 0;
  virtual void setup() = 0;

  void setData(StateEstimatorData<T> data) { _stateEstimatorData = data; }

  virtual ~GenericEstimator() = default;
  StateEstimatorData<T> _stateEstimatorData;
};






/*!
 * 主状态估计器类
 * 包含所有通用估计器，并可以运行它们也更新可视化将估计器添加进去统一管理
 */
template <typename T>
class StateEstimatorContainer 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * 功能：构造一个新的状态估计器容器 通过容器向每个估计器传递参数地址，之后直接更新参数
   * fun(仿真模式、imu导航数据、腿部控制器数据、状态估计结构体、机器人控制参数)
   */
  StateEstimatorContainer(CheaterState<double>* cheaterState,
                          VectorNavData* vectorNavData,
                          LegControllerData<T>* legControllerData,
                          StateEstimate<T>* stateEstimate,
                          RobotControlParameters* parameters) 
  {
    _data.cheaterState = cheaterState;          //根据实际情况未使用 robotrunner
    _data.vectorNavData = vectorNavData;        //imu数据
    _data.legControllerData = legControllerData;//腿部数据
    _data.result = stateEstimate;               //估计结果
    _phase = Vec4<T>::Zero();                   //相位信息
    _data.contactPhase = &_phase;               //接触状态
    _data.parameters = parameters;              //参数
  }



  /*!
   * 功能：运行所有估计函数
   */
  void run(CheetahVisualization* visualization = nullptr) 
  {
    /*（1）遍历运行所有状态估计器*/
    //备注：状态估计器分为仿真模式下和真实模式下两类，其中我们运行的是真实的模式
    //1）方向估计器：void VectorNavOrientationEstimator<T>::run()             真实模式下：imu已经提供了方向,我们只需直接读取IMU数据并作相应的左边变换，返回读取的那个IMU的值
    //            ：CheaterOrientationEstimator<T>::run()                    仿真cheater模式下，运行orientation方向估计

    //2）位置速度估计器：void LinearKFPositionVelocityEstimator<T>::run()      真正模式下，通过卡尔曼滤波器估计躯干的位置和线速度
    //                ：CheaterPositionVelocityEstimator<T>::run()           仿真cheater模式下，运行位置和速度估计器

    //3）触地估计器： virtual void run()                                      其实没有做触地检测，仅仅通过相位反馈的周期信息作为触地信号
    for (auto estimator : _estimators) //遍历状态估计器组
    {
      estimator->run();//运行机器人所有的估计器，这里run()的函数真实的有两个，仿真的有两个
    }

   /*4）判断状态估计数据是否要可视化*/
    if (visualization) //若需要
    {
      visualization->quat = _data.result->orientation.template cast<float>(); //可视化获取的四元素
      visualization->p = _data.result->position.template cast<float>();       //可视化获取的位置
      // todo contact!
    }
  }



  /*!
   * 功能：获取状态估计结果函数，返回状态估计的数据
   */
  const StateEstimate<T>& getResult() { return *_data.result; }
  StateEstimate<T> * getResultHandle() { return _data.result; }



  /*!
   * 功能：获取接触状态函数【数据从mpc控制器来】 在整个支撑过程百分比(从0到1)  完成后为0
   */
  void setContactPhase(Vec4<T>& phase) 
  { 
    *_data.contactPhase = phase; 
  }



  /*!
   * 功能：构造给定类型的估计器函数
   * @tparam EstimatorToAdd
   */
  template <typename EstimatorToAdd>
  void addEstimator() 
  {
    auto* estimator = new EstimatorToAdd();//（1）实例化一个传入类型的估计器
    estimator->setData(_data);             //（2）给估计器传入数据
    estimator->setup();                    //（3）估计器初始化设置
    _estimators.push_back(estimator);      //（4）推入估计器数组
  }

  /*!
   * 功能：删除给定类型的估计器函数
   * @tparam EstimatorToRemove
   */
  template <typename EstimatorToRemove>
  void removeEstimator() 
  {
    int nRemoved = 0;
    _estimators.erase(
        std::remove_if(_estimators.begin(), _estimators.end(),
                       [&nRemoved](GenericEstimator<T>* e) 
                       {
                         if (dynamic_cast<EstimatorToRemove*>(e)) 
                         {
                           delete e;
                           nRemoved++;
                           return true;
                         }
                          else 
                         {
                           return false;
                         }
                       }),
        _estimators.end());
  }


  /*!
   * 功能：删除所有状态估计器函数
   */
  void removeAllEstimators() 
  {
    for (auto estimator : _estimators) 
    {
      delete estimator;
    }
    _estimators.clear();
  }


  /*!
   * 功能：构造一个新的状态估计器容器函数的析构函数
   */

  ~StateEstimatorContainer() 
  {
    for (auto estimator : _estimators) 
    {
      delete estimator;
    }
  }

 private:
  StateEstimatorData<T> _data;                   //状态估计数据 传入估计器用
  std::vector<GenericEstimator<T>*> _estimators;//定义估计器数组
  Vec4<T> _phase;
};

#endif  // PROJECT_STATEESTIMATOR_H
