/*四足机器人包含参数的数据结构

 *此文件包含生成四足动物的实用函数
 *猎豹3（最终是迷你猎豹）。有一个buildModel（）方法
 *它可以用来建立四足动物的浮基动力学模型。
 */

#ifndef LIBBIOMIMETICS_QUADRUPED_H
#define LIBBIOMIMETICS_QUADRUPED_H

#include "Dynamics/ActuatorModel.h"
#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/SpatialInertia.h"
#include <eigen3/Eigen/StdVector>
#include <vector>


//命名空间cheetah
namespace cheetah 
{
constexpr size_t num_act_joint = 12;    //定义cheetah有12个关节
constexpr size_t num_q = 19;
constexpr size_t dim_config = 18;
constexpr size_t num_leg = 4;           //定义cheetah有四条腿
constexpr size_t num_leg_joint = 3;     //定义cheetah每条腿有三个关节
constexpr float servo_rate = 0.001;     //定义cheetah的私服率为0.001
}


//命名空间linkID,定义每条腿下面两个电机的ID号
namespace linkID 
{
constexpr size_t FR = 9;   //右前脚
constexpr size_t FL = 11;  //左前脚 
constexpr size_t HR = 13;  //后右脚
constexpr size_t HL = 15;  //后腿左脚

constexpr size_t FR_abd = 2;  //右前外展
constexpr size_t FL_abd = 0;  //左前外展
constexpr size_t HR_abd = 3;  //右后外展
constexpr size_t HL_abd = 1;  //左后外展
}

using std::vector;

/*!
 * 四足机器人物理特性的表示。
 * 从上面看，四足动物的腿的ID分布是：
 * FRONT
 * 2 1   RIGHT
 * 4 3
 * BACK
 *
 */
template <typename T>
class Quadruped 
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW                                    //本征使对齐的算符成立(与内存相关)
  RobotType _robotType;                                              //定义一个机器人类型
  T _bodyLength, _bodyWidth, _bodyHeight, _bodyMass;                 //定义四足机器人的属性包括躯干长宽高和质量
  T _abadGearRatio, _hipGearRatio, _kneeGearRatio;                   //定义四足机器人的关节传动比，包括每个腿的三个关节
  T _abadLinkLength, _hipLinkLength, _kneeLinkLength, _maxLegLength; //定义四足机器人腿部连杆的长度
  T _motorKT, _motorR, _batteryV;                                    //定义四足机器人电机类型，和电池电压
  T _motorTauMax;                                                    //定义四足机器人电机最大转矩
  T _jointDamping, _jointDryFriction;                                //定义四足机器人关节阻尼，关节摩擦
  SpatialInertia<T> _abadInertia, _hipInertia, _kneeInertia, _abadRotorInertia,
      _hipRotorInertia, _kneeRotorInertia, _bodyInertia;             //定义四足机器人可达空间
  Vec3<T> _abadLocation, _abadRotorLocation, _hipLocation, _hipRotorLocation,
      _kneeLocation, _kneeRotorLocation;                             //定义四足机器人每个关节的的定位
  FloatingBaseModel<T> buildModel();                                 //定义四足机器人的浮基模型
  bool buildModel(FloatingBaseModel<T>& model);                      //定义四足机器人的浮基模型，重载
  std::vector<ActuatorModel<T>> buildActuatorModels();               //建立电机执行器模型函数

  static T getSideSign(int leg) 
  {
    const T sideSigns[4] = {-1, 1, -1, 1};
    assert(leg >= 0 && leg < 4);
    return sideSigns[leg];
  }

  /*!
   * 功能：获取机器人坐标系中给定腿的臀部位置
   */
  Vec3<T> getHipLocation(int leg) 
  {
    assert(leg >= 0 && leg < 4);
    Vec3<T> pHip((leg == 0 || leg == 1) ? _abadLocation(0) : -_abadLocation(0),
                 (leg == 1 || leg == 3) ? _abadLocation(1) : -_abadLocation(1),
                 _abadLocation(2));
    return pHip;
  }
};

template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2>& v, int legID);

#endif  // LIBBIOMIMETICS_QUADRUPED_H
